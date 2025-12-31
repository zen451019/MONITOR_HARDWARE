/**
 * @file main.cpp
 * @brief Modbus RTU Master Firmware over LoRaWAN (ESP32/TTGO).
 * @details This system manages RS485 communication with multiple slaves, 
 * schedules periodic readings, formats the data, and transmits it via LoRaWAN.
 * @date 2025-12-02
 */

#include <Arduino.h>
#include <SPI.h>
#include <hal/hal.h>
#include <lmic.h>
#include <vector>
#include <cstdint>      ///< Necesario para definiciones de tipos enteros de tamaño fijo (uint8_t, etc).
#include <map>          ///< Estructura de datos para mapeo de sensores.
#include <ctime>        ///< Utilizado para la generación de timestamps UNIX.
#include <cstring>      ///< Utilidades de memoria (memcpy).
#include <algorithm>
#include "ModbusClientRTU.h"
#include "ModbusAPI.h"

// =================================================================================================
// Forward Declarations
// =================================================================================================
bool discoverDeviceSensors(uint8_t deviceId);
static bool formatAndEnqueueSensorData(const ModbusApiResult& response, uint8_t slaveId, uint8_t sensorId);
void parseAndStoreDiscoveryResponse(const uint8_t* data, size_t length, uint8_t slaveId);
bool getSensorParams(uint8_t slaveId, uint8_t sensorID, uint16_t& startAddr, uint16_t& numRegs);

// =================================================================================================
// Modbus RTU Configuration
// =================================================================================================

/**
 * @def RX_PIN
 * @brief UART reception pin for RS485.
 * @ingroup group_modbus_discovery
 */
#define RX_PIN 13
/**
 * @def TX_PIN
 * @brief UART transmission pin for RS485.
 * @ingroup group_modbus_discovery
 */
#define TX_PIN 12

/**
 * @struct ModbusSensorParam
 * @brief Defines the configuration parameters for an individual sensor.
 * @details This structure stores how to read and interpret data from a specific sensor.
 * @ingroup group_modbus_discovery
 */
typedef struct {
    uint8_t sensorID;           ///< Unique sensor identifier.
    uint8_t numberOfChannels;   ///< Number of data channels for the sensor.
    uint16_t startAddress;      ///< Initial Modbus register address.
    uint16_t maxRegisters;      ///< Total number of registers to read.
    uint16_t samplingInterval;  ///< Base sampling interval in milliseconds.
    uint8_t dataType;           ///< Data type: 1=uint8, 2=uint16, 3=compressed bytes, 4=float16.
    uint8_t scale;              ///< Decimal scale factor (10^scale).
    uint8_t compressedBytes;    ///< Number of bytes per value if compression is used (dataType=3).
} ModbusSensorParam;

/**
 * @struct ModbusSlaveParam
 * @brief Represents a physical slave device on the RS485 bus.
 * @ingroup group_modbus_discovery
 */
struct ModbusSlaveParam {
    uint8_t slaveID;                        ///< Modbus address of the slave (1-247).
    std::vector<ModbusSensorParam> sensors; ///< Vector with the sensors associated with this slave.
    uint8_t consecutiveFails;               ///< Counter of consecutive failures for error handling.
};

///< Global vector that stores the configuration and state of all discovered Modbus slaves.
std::vector<ModbusSlaveParam> slaveList;

QueueHandle_t queueFragmentos;          ///< Queue for binary LoRa messages.
SemaphoreHandle_t semaforoEnvioCompleto; ///< Semaphore to synchronize the end of a sending cycle.

/**
 * @struct SensorSchedule
 * @brief Planning element for sensor sampling.
 * @ingroup group_modbus_discovery
 */
struct SensorSchedule {
    uint8_t slaveID;           ///< Slave ID.
    uint8_t sensorID;          ///< Sensor ID.
    uint16_t samplingInterval; ///< Calculated sampling interval in ms.
    uint32_t nextSampleTime;   ///< Timestamp (millis) for the next execution.
};

std::vector<SensorSchedule> scheduleList; ///< Master scheduling list.
SemaphoreHandle_t schedulerMutex;         ///< Mutex to protect concurrent access to scheduleList;
TaskHandle_t dataRequestSchedulerHandle = NULL; ///< Handle para la tarea del planificador.

/**
 * @brief Initializes or updates the scheduling list (Scheduler).
 * @details Iterates through `slaveList`, calculates effective intervals based on channels and registers,
 * and populates `scheduleList`. It is thread-safe using `schedulerMutex`.
 * @ingroup group_modbus_discovery
 */
void initScheduler() {
    // Take exclusive control of the lists
    if (xSemaphoreTake(schedulerMutex, portMAX_DELAY) == pdTRUE) {
        scheduleList.clear();
        for (const auto& slave : slaveList) {
            for (const auto& sensor : slave.sensors) {
                uint32_t calculatedInterval = sensor.samplingInterval; // Valor por defecto
                if (sensor.numberOfChannels > 0 && sensor.maxRegisters > 0) {
                    // Calcular el intervalo total para llenar el buffer de registros
                    uint16_t registersPerChannel = sensor.maxRegisters / sensor.numberOfChannels;
                    calculatedInterval = (uint32_t)sensor.samplingInterval * registersPerChannel;
                }

                scheduleList.push_back({
                    slave.slaveID,
                    sensor.sensorID,
                    (uint16_t)calculatedInterval, // Usar el intervalo calculado
                    millis() // Primer muestreo inmediato
                });
            }
        }

        // Imprimir el contenido de scheduleList
        Serial.println("Contenido de scheduleList (actualizado con cálculo de intervalo):");
        for (const auto& item : scheduleList) {
            Serial.printf("  SlaveID: %u, SensorID: %u, Intervalo Calculado: %u ms, NextSample: %u\n",
                item.slaveID, item.sensorID, item.samplingInterval, item.nextSampleTime);
        }

        // Devolver el control
        xSemaphoreGive(schedulerMutex);
    }
}

/**
 * @brief Procesa una solicitud de muestreo para un sensor específico.
 * @details Realiza la lectura Modbus, formatea los datos en caso de éxito,
 * o gestiona el contador de fallos en caso de error.
 * @param item El elemento del planificador a procesar.
 * @return true si el esclavo asociado sigue activo, false si fue eliminado por fallos.
 */
bool handleScheduledSensor(const SensorSchedule& item) {
    const uint8_t MAX_CONSECUTIVE_FAILS = 3;

    Serial.printf("Solicitando muestreo: SlaveID=%u, SensorID=%u\n", item.slaveID, item.sensorID);
    
    uint16_t startAddr, numRegs;
    if (!getSensorParams(item.slaveID, item.sensorID, startAddr, numRegs)) {
        Serial.printf("Error: No se encontraron parámetros para Esclavo %u, Sensor %u.\n", item.slaveID, item.sensorID);
        return true; // El esclavo sigue existiendo, aunque el sensor no se encontró.
    }

    ModbusApiResult result = modbus_api_read_registers(item.slaveID, READ_HOLD_REGISTER, startAddr, numRegs, 2000);

    auto slaveIt = std::find_if(slaveList.begin(), slaveList.end(), [&](const ModbusSlaveParam& s) { return s.slaveID == item.slaveID; });
    if (slaveIt == slaveList.end()) {
        // El esclavo fue eliminado por otra operación, no hay nada que hacer.
        return false;
    }

    if (result.error_code == ModbusApiError::SUCCESS) {
        formatAndEnqueueSensorData(result, item.slaveID, item.sensorID);
        slaveIt->consecutiveFails = 0;
    } else {
        Serial.printf("Error en muestreo para Esclavo %u, Sensor %u. Código: %u\n", item.slaveID, item.sensorID, static_cast<uint8_t>(result.error_code));
        slaveIt->consecutiveFails++;
        Serial.printf("Fallo consecutivo %u para esclavo %u.\n", slaveIt->consecutiveFails, item.slaveID);
        
        if (slaveIt->consecutiveFails >= MAX_CONSECUTIVE_FAILS) {
            Serial.printf("Esclavo %u ha alcanzado el máximo de fallos. Eliminando...\n", item.slaveID);
            slaveList.erase(slaveIt);
            return false; // Indica que el esclavo fue eliminado.
        }
    }
    return true; // El esclavo sigue activo.
}

std::vector<uint8_t> dispositivosAConsultar = {1, 2, 3}; ///< Predefined list of Modbus IDs to scan at startup.

/**
 * @brief One-shot task for the initial discovery of sensors.
 * @details Iterates over `dispositivosAConsultar`, launches discovery, and then self-deletes.
 * @ingroup group_modbus_discovery
 */
void initialDiscoveryTask(void *pvParameters) {
    Serial.println("--- Tarea de Descubrimiento Inicial: Iniciando ---");
    const TickType_t delayBetweenDevices = pdMS_TO_TICKS(50);

    for (uint8_t deviceId : dispositivosAConsultar) {
        if (discoverDeviceSensors(deviceId)) {
            Serial.printf("Descubrimiento exitoso para el dispositivo %u.\n", deviceId);
        } else {
            Serial.printf("Fallo en el descubrimiento para el dispositivo %u.\n", deviceId);
        }
        vTaskDelay(delayBetweenDevices);
    }

    Serial.println("--- Tarea de Descubrimiento Inicial: Finalizada. Inicializando Scheduler... ---");
    initScheduler(); // Ahora que tenemos la lista de sensores, inicializamos el scheduler.

    Serial.println("--- Tarea de Descubrimiento Inicial: Autodestruyendo. ---");
    vTaskDelete(NULL); // La tarea se elimina a sí misma.
}


/**
 * @brief Comparación segura con wrap-around de millis().
 */
static inline bool timeReached(uint32_t now, uint32_t target) {
    return static_cast<int32_t>(now - target) >= 0;
}

/**
 * @brief Main task of the Scheduler.
 * @details Periodically checks which sensors should be sampled and generates events for the EventManager.
 * @ingroup group_modbus_discovery
 */
void DataRequestScheduler(void *pvParameters) {

    while (true) {
        const uint32_t now = millis();
        TickType_t sleepTime = pdMS_TO_TICKS(1000); // Default si no hay nada
        bool schedulerNeedsRebuild = false;

        // 1) Bajo mutex: seleccionar "due", reprogramar y calcular próximo evento.
        std::vector<SensorSchedule> dueItems;
        uint32_t nextEventTime = UINT32_MAX;

        if (xSemaphoreTake(schedulerMutex, portMAX_DELAY) == pdTRUE) {

            if (!scheduleList.empty()) {
                dueItems.reserve(scheduleList.size());

                for (auto& item : scheduleList) {
                    if (timeReached(now, item.nextSampleTime)) {
                        // Copia del trabajo a ejecutar fuera del mutex
                        dueItems.push_back(item);

                        // Reprogramar inmediatamente (sin hacer I/O aquí)
                        item.nextSampleTime = now + item.samplingInterval;
                    }

                    if (item.nextSampleTime < nextEventTime) {
                        nextEventTime = item.nextSampleTime;
                    }
                }

                // Calcular cuánto dormir hasta el próximo evento
                const uint32_t now2 = millis();
                if (nextEventTime != UINT32_MAX && nextEventTime > now2) {
                    sleepTime = pdMS_TO_TICKS(nextEventTime - now2);
                } else if (!dueItems.empty()) {
                    // Si hubo trabajo "due", cede un poco para no saturar
                    sleepTime = pdMS_TO_TICKS(10);  
                } else {
                    sleepTime = pdMS_TO_TICKS(1000);
                }
            }

            xSemaphoreGive(schedulerMutex);
        }

        // 2) Fuera del mutex: ejecutar I/O (Modbus). Si algún esclavo cae, marcar rebuild.
        for (const auto& item : dueItems) {
            if (!handleScheduledSensor(item)) {
                schedulerNeedsRebuild = true;
                // No hace falta seguir: scheduleList quedó desfasada respecto a slaveList.
                break;
            }
        }

        // 3) Rebuild fuera del mutex (evita deadlock y reduce tiempo en sección crítica)
        if (schedulerNeedsRebuild) {
            initScheduler();
        }

        vTaskDelay(sleepTime);
    }
}

/**
 * @brief Gets the configuration parameters of a specific sensor.
 * @param slaveId Slave ID.
 * @param sensorID Sensor ID.
 * @param startAddr Reference to return the start address.
 * @param numRegs Reference to return the number of registers.
 * @return true if the sensor was found, false otherwise.
 * @ingroup group_modbus_discovery
 */
bool getSensorParams(uint8_t slaveId, uint8_t sensorID, uint16_t& startAddr, uint16_t& numRegs) {
    auto slaveIt = std::find_if(slaveList.begin(), slaveList.end(),
        [&](const ModbusSlaveParam& s) { return s.slaveID == slaveId; });

    if (slaveIt != slaveList.end()) {
        auto sensorIt = std::find_if(slaveIt->sensors.begin(), slaveIt->sensors.end(),
            [&](const ModbusSensorParam& sensor) { return sensor.sensorID == sensorID; });

        if (sensorIt != slaveIt->sensors.end()) {
            startAddr = sensorIt->startAddress;
            numRegs = sensorIt->maxRegisters;
            return true;
        }
    }
    return false;
}

/**
 * @brief Calculates the number of registers per channel for a sensor.
 * @return Number of registers or 0 if the sensor is not found.
 * @ingroup group_modbus_discovery
 */
uint8_t getRegistersPerChannel(uint8_t slaveId, uint8_t sensorID) {
    auto slaveIt = std::find_if(slaveList.begin(), slaveList.end(),
        [&](const ModbusSlaveParam& s) { return s.slaveID == slaveId; });

    if (slaveIt != slaveList.end()) {
        auto sensorIt = std::find_if(slaveIt->sensors.begin(), slaveIt->sensors.end(),
            [&](const ModbusSensorParam& sensor) { return sensor.sensorID == sensorID; });

        if (sensorIt != slaveIt->sensors.end() && sensorIt->numberOfChannels > 0) {
            return static_cast<uint8_t>(sensorIt->maxRegisters / sensorIt->numberOfChannels);
        }
    }
    return 0;
}

/**
 * @brief Parses the discovery response and updates the slave list.
 * @details Decodes the 8 parameter registers and creates or updates the entry in `slaveList`.
 * @param response Raw data received.
 * @param slaveId ID of the slave that responded.
 * @ingroup group_modbus_discovery
 */
void parseAndStoreDiscoveryResponse(const uint8_t* data, size_t length, uint8_t slaveId) {
    // Se esperan 8 registros, que son 16 bytes de datos.
    if (length < 16) {
        Serial.printf("Error: Respuesta de descubrimiento incompleta para esclavo %u. Se esperaban 16 bytes, se recibieron %u.\n", slaveId, length);
        return;
    }

    ModbusSensorParam newSensor;
    // Los datos de la API ya no tienen la cabecera Modbus, empiezan en el byte 0.
    newSensor.sensorID          = data[1];  // Reg 0: sensorID (byte bajo)
    newSensor.numberOfChannels  = data[3];  // Reg 1: numberOfChannels (byte bajo)
    newSensor.startAddress      = (data[4] << 8) | data[5];   // Reg 2: startAddress
    newSensor.maxRegisters      = (data[6] << 8) | data[7];   // Reg 3: maxRegisters
    newSensor.samplingInterval  = (data[8] << 8) | data[9];   // Reg 4: samplingInterval
    newSensor.dataType          = data[11]; // Reg 5: dataType (byte bajo)
    newSensor.scale             = data[13]; // Reg 6: scale (byte bajo)
    newSensor.compressedBytes   = data[15]; // Reg 7: compressedBytes (byte bajo)

    Serial.printf("Sensor descubierto en esclavo %u: ID=%u, Canales=%u, Addr=%u, Regs=%u, Intervalo=%u ms\n",
        slaveId, newSensor.sensorID, newSensor.numberOfChannels, newSensor.startAddress, newSensor.maxRegisters, newSensor.samplingInterval);

    // Buscar si el esclavo ya existe en la lista
    auto slaveIt = std::find_if(slaveList.begin(), slaveList.end(),
        [&](const ModbusSlaveParam& s) { return s.slaveID == slaveId; });

    if (slaveIt != slaveList.end()) {
        // El esclavo ya existe. Buscar si el sensor ya está registrado.
        auto sensorIt = std::find_if(slaveIt->sensors.begin(), slaveIt->sensors.end(),
            [&](const ModbusSensorParam& s) { return s.sensorID == newSensor.sensorID; });

        if (sensorIt != slaveIt->sensors.end()) {
            // El sensor ya existe, sobreescribir sus parámetros.
            *sensorIt = newSensor;
            Serial.printf("Parámetros del sensor %u actualizados para el esclavo %u.\n", newSensor.sensorID, slaveId);
        } else {
            // El sensor no existe, añadirlo a la lista de sensores del esclavo.
            slaveIt->sensors.push_back(newSensor);
            Serial.printf("Nuevo sensor %u añadido al esclavo %u.\n", newSensor.sensorID, slaveId);
        }
    } else {
        // El esclavo no existe, crearlo y añadir el sensor.
        ModbusSlaveParam newSlave;
        newSlave.slaveID = slaveId;
        newSlave.consecutiveFails = 0;
        newSlave.sensors.push_back(newSensor);
        slaveList.push_back(newSlave);
        Serial.printf("Nuevo esclavo %u añadido a la lista con sensor %u.\n", slaveId, newSensor.sensorID);
    }
}

/**
 * @brief Starts the discovery process for a specific device.
 * @param deviceId Modbus ID of the device to query.
 * @return true if the event was queued correctly, false otherwise.
 * @ingroup group_modbus_discovery
 */
bool discoverDeviceSensors(uint8_t deviceId) {
    Serial.printf("Iniciando descubrimiento para dispositivo %u...\n", deviceId);

    // Llamada síncrona a la API para leer los 8 registros de parámetros
    ModbusApiResult result = modbus_api_read_registers(deviceId, READ_HOLD_REGISTER, 0, 8, 2000);

    if (result.error_code == ModbusApiError::SUCCESS) {
        Serial.printf("Respuesta de descubrimiento recibida para esclavo %u.\n", deviceId);
        // La API ya quita la cabecera, pasamos los datos directamente
        parseAndStoreDiscoveryResponse(result.data, result.data_len, deviceId);
        return true;
    } else {
        Serial.printf("Error en descubrimiento para esclavo %u: Código %u\n", deviceId, static_cast<uint8_t>(result.error_code));
        // Aquí puedes añadir la lógica para contar fallos si lo deseas
        return false;
    }
}

// ==================== BIT PACKER ====================
/**
 * @struct BitPacker
 * @brief Utility for packing arbitrary bits into a byte stream.
 * @details Allows data compression when `compressedBytes` > 0.
 * @ingroup group_data_format
 */
struct BitPacker
{
    uint64_t buffer = 0; ///< Temporary accumulator of up to 64 bits.
    int bits_usados = 0; ///< Counter of valid bits in the buffer.

    void push(uint16_t valor, int nbits, std::vector<uint8_t> &out)
    {
        // desplazar buffer para dejar espacio
        buffer <<= nbits;
        // quedarnos con solo los bits válidos
        buffer |= (valor & ((1ULL << nbits) - 1));
        bits_usados += nbits;

        // mientras tengamos al menos 8 bits, sacar bytes
        while (bits_usados >= 8)
        {
            int shift = bits_usados - 8;
            uint8_t byte = (buffer >> shift) & 0xFF;
            out.push_back(byte);
            bits_usados -= 8;
            buffer &= (1 << bits_usados) - 1; // limpiar bits ya usados
        }
    }

    void flush(std::vector<uint8_t> &out)
    {
        if (bits_usados > 0)
        {
            uint8_t byte = buffer << (8 - bits_usados);
            out.push_back(byte);
            bits_usados = 0;
            buffer = 0;
        }
    }
};

/**
 * @def MAX_SENSOR_PAYLOAD
 * @brief Maximum size of an individual sensor payload.
 * @ingroup group_data_format
 */
#define MAX_SENSOR_PAYLOAD 128 ///< Maximum size of an individual sensor payload.

/**
 * @struct SensorDataPayload
 * @brief Container for processed sensor data, ready for aggregation.
 * @ingroup group_data_format
 */
struct SensorDataPayload {
    uint8_t slaveId;                  ///< Source slave ID.
    uint8_t sensorId;                 ///< Source sensor ID.
    uint8_t data[MAX_SENSOR_PAYLOAD]; ///< Fixed-size data array.
    size_t dataSize;                  ///< Number of valid bytes in data.
};

QueueHandle_t queueSensorDataPayload; ///< Queue for processed sensor payloads.

/**
 * @brief Extracts and formats data from a sampling Modbus response.
 * @param response Raw response received.
 * @param request Information from the original request.
 * @param values Output vector where the processed bytes will be stored.
 * @return true if the process was successful.
 * @ingroup group_data_format
 */
static bool formatAndEnqueueSensorData(const ModbusApiResult& response, uint8_t slaveId, uint8_t sensorId) {
    auto slaveIt = std::find_if(slaveList.begin(), slaveList.end(),
        [&](const ModbusSlaveParam& s) { return s.slaveID == slaveId; });
    if (slaveIt == slaveList.end()) {
        Serial.printf("Formato: no se encontró el esclavo %u.\n", slaveId);
        return false;
    }

    auto sensorIt = std::find_if(slaveIt->sensors.begin(), slaveIt->sensors.end(),
        [&](const ModbusSensorParam& sensor) { return sensor.sensorID == sensorId; });
    if (sensorIt == slaveIt->sensors.end()) {
        Serial.printf("Formato: no se encontró el sensor %u en esclavo %u.\n", sensorId, slaveId);
        return false;
    }

    const ModbusSensorParam& params = *sensorIt;
    std::vector<uint8_t> values;

    Serial.printf("Formato: esclavo %u sensor %u -> regs:%u tipo:%u escala:%u comp:%u\n",
                  slaveId, params.sensorID, params.maxRegisters,
                  params.dataType, params.scale, params.compressedBytes);

    // --- INICIO: Bloque de depuración para imprimir bytes HIGH y LOW ---
    Serial.print("  [Debug] Bytes HIGH/LOW recibidos: ");
    // Los datos de la API ya no tienen cabecera Modbus
    for (size_t i = 0; i < params.maxRegisters; ++i) {
        size_t offset = i * 2;
        if ((offset + 1) < response.data_len) {
            uint8_t high = response.data[offset];
            uint8_t low  = response.data[offset + 1];
            Serial.printf("[H:%u, L:%u] ", high, low);
        } else {
            break; // Salir si no hay suficientes datos para un par completo
        }
    }
    Serial.println();
    // --- FIN: Bloque de depuración ---

    uint8_t dataType = params.dataType; // 1=uint8, 2=uint16
    uint8_t scale = params.scale;
    uint8_t compressedBytes = params.compressedBytes;

    values.clear();
    if (compressedBytes > 0) {
        BitPacker packer;
        for (size_t i = 0; i < params.maxRegisters; ++i) {
            size_t offset = i * 2;
            if ((offset + 1) >= response.data_len) {
                break;
            }
            uint8_t high = response.data[offset];
            uint8_t low  = response.data[offset + 1];
            uint16_t raw = (static_cast<uint16_t>(high) << 8) | low;
            packer.push(raw, compressedBytes, values);
        }
        packer.flush(values);
    } else {
        for (size_t i = 0; i < params.maxRegisters; ++i) {
            size_t offset = i * 2;
            if ((offset + 1) >= response.data_len) {
                break;
            }
            uint8_t high = response.data[offset];
            uint8_t low  = response.data[offset + 1];

            if (dataType == 1) {            // uint8 -> solo byte bajo
                values.push_back(low);
            } else if (dataType == 2) {     // uint16 -> alto seguido del bajo
                values.push_back(high);
                values.push_back(low);
            } else {                        // por defecto mantener big-endian
                values.push_back(high);
                values.push_back(low);
            }
        }
    }
    
    // Encolar el payload formateado
    SensorDataPayload payload;
    payload.slaveId = slaveId;
    payload.sensorId = sensorId;

    // Copiar los datos de forma segura
    payload.dataSize = std::min(values.size(), (size_t)MAX_SENSOR_PAYLOAD);
    memcpy(payload.data, values.data(), payload.dataSize);

    if (xQueueSend(queueSensorDataPayload, &payload, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("Error: No se pudo encolar el payload de datos del sensor.");
        return false;
    } else {
        Serial.printf("Payload de datos del sensor encolado: Esclavo %u, Sensor %u, Bytes %u\n",
                      payload.slaveId, payload.sensorId, payload.dataSize);
    }

    return true;
}

/**
 * @brief Tarea temporal para imprimir los payloads de datos formateados.
 * @details Simula el consumidor final (que será la tarea LoRa) para propósitos de depuración.
 * @ingroup group_tasks_utils
 */
void DataPrinterTask(void *pvParameters) {
    SensorDataPayload payload;
    while (true) {
        // Esperar indefinidamente a que llegue un payload
        if (xQueueReceive(queueSensorDataPayload, &payload, portMAX_DELAY) == pdTRUE) {
            
            Serial.println();
            Serial.println("--- 🛰️ PAYLOAD DE SENSOR FORMATEADO ---");
            Serial.printf("  [Fuente] Esclavo: %u, Sensor: %u\n", payload.slaveId, payload.sensorId);
            Serial.printf("  [Datos]  Tamaño: %u bytes\n", payload.dataSize);
            Serial.print("  [Payload] ");
            
            // Imprimir los bytes en formato hexadecimal
            for (size_t i = 0; i < payload.dataSize; i++) {
                if (payload.data[i] < 0x10) Serial.print("0");
                Serial.print(payload.data[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
            Serial.println("------------------------------------------");
            Serial.println();
        }
    }
}

// --- Mapping of Sensor ID to Activate Byte Bits ---
// IMPORTANT! You must adjust these IDs to the ones used in your system.
// These are just examples based on the context.
const uint8_t SENSOR_ID_BATERIA = 0;   ///< Assigned to Bit 0 of the Activate Byte.
const uint8_t SENSOR_ID_VOLTAJE = 1;   ///< Assigned to Bit 1 of the Activate Byte.
const uint8_t SENSOR_ID_CORRIENTE = 2; ///< Assigned to Bit 2 of the Activate Byte.
const uint8_t SENSOR_ID_EXT_START = 3; ///< Start of IDs for external sensors (Bits 3-7).
const int MAX_SENSORES_EXTERNOS = 5;   ///< Number of remaining available bits (3 to 7).

/**
 * @def AGGREGATION_INTERVAL_MS
 * @brief Aggregation interval (ms).
 * @ingroup group_data_format
 */
#define AGGREGATION_INTERVAL_MS 6100 ///< Aggregation interval (6s + 100ms margin).

/**
 * @brief Builds a unified payload from a collection of sensor data.
 * @details Payload Structure: [ID_MSG][TIMESTAMP][ACTIVATE_BYTE][LEN_BYTES...][DATA_BLOCKS...]
 * @param id_mensaje The message ID byte (Header).
 * @param collectedPayloads The vector with the collected data.
 * @return std::vector<uint8_t> The binary payload ready to be sent.
 * @ingroup group_data_format
 */
std::vector<uint8_t> construirPayloadUnificado(
    uint8_t id_mensaje,
    const std::vector<SensorDataPayload>& collectedPayloads)
{
    std::vector<uint8_t> payload;

    // ================== 1. CABECERA (1 byte) ==================
    payload.push_back(id_mensaje);

    // ================== 2. TIMESTAMP (4 bytes) ==================
    // Usamos el timestamp UNIX actual. El código original usaba millis()
    // o un timestamp del buffer. time(nullptr) es el estándar C++.
    uint32_t ts_s = static_cast<uint32_t>(time(nullptr));
    payload.push_back((ts_s >> 24) & 0xFF);
    payload.push_back((ts_s >> 16) & 0xFF);
    payload.push_back((ts_s >> 8) & 0xFF);
    payload.push_back(ts_s & 0xFF);

    // Usamos un mapa para organizar los sensores presentes y manejar duplicados
    // (el último sensor con el mismo ID sobreescribe a los anteriores).
    std::map<uint8_t, const SensorDataPayload*> activeSensors;
    for (const auto& sensorData : collectedPayloads) {
        activeSensors[sensorData.sensorId] = &sensorData;
    }

    // ================== 3. ACTIVATE BYTE (1 byte) ==================
    // Construido dinámicamente basado en los SENSOR_ID presentes
    uint8_t activate_byte = 0;

    // Bits 0, 1, 2 (Batería, Voltaje, Corriente)
    if (activeSensors.count(SENSOR_ID_BATERIA))   activate_byte |= (1 << 0);
    if (activeSensors.count(SENSOR_ID_VOLTAJE))   activate_byte |= (1 << 1);
    if (activeSensors.count(SENSOR_ID_CORRIENTE)) activate_byte |= (1 << 2);
    
    // Bits 3+ (Otros sensores)
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        uint8_t current_sensor_id = SENSOR_ID_EXT_START + i;
        if (activeSensors.count(current_sensor_id)) {
            activate_byte |= (1 << (i + 3));
        }
    }
    payload.push_back(activate_byte);

    // ================== 4. DATA LENGTH BYTES (N bytes) ==================
    // Se añade un byte de longitud por CADA bit activo en activate_byte,
    // en el orden LSB a MSB (Batería, Voltaje, Corriente, Externos...).

    // --- Batería (Bit 0) ---
    if (activate_byte & (1 << 0)) {
        const auto& sensor = activeSensors.at(SENSOR_ID_BATERIA);
        //optener Data Length Bytes
        uint8_t len_data = getRegistersPerChannel(sensor->slaveId, sensor->sensorId);
        // Asumimos formato del ejemplo: No PKD, No 2BIT
        uint8_t len_byte = (len_data & 0x1F);
        payload.push_back(len_byte);
    }

    // --- Voltaje (Bit 1) ---
    if (activate_byte & (1 << 1)) {
        const auto& sensor = activeSensors.at(SENSOR_ID_VOLTAJE);
        //optener Data Length Bytes
        uint8_t len_data = getRegistersPerChannel(sensor->slaveId, sensor->sensorId);
        // Asumimos formato del ejemplo: No PKD, No 2BIT
        uint8_t len_byte = (len_data & 0x1F);
        payload.push_back(len_byte);
    }

    // --- Corriente (Bit 2) ---
    if (activate_byte & (1 << 2)) {
        const auto& sensor = activeSensors.at(SENSOR_ID_CORRIENTE);
        //optener Data Length Bytes
        uint8_t len_data = getRegistersPerChannel(sensor->slaveId, sensor->sensorId);
        // Asumimos formato del ejemplo: PKD (Bit 7), No 2BIT
        uint8_t len_byte = (len_data & 0x1F);
        payload.push_back(len_byte);
    }

    // --- Sensores Externos (Bits 3+) ---
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        if (activate_byte & (1 << (i + 3))) {
            uint8_t current_sensor_id = SENSOR_ID_EXT_START + i;
            const auto& sensor = activeSensors.at(current_sensor_id);
            //optener Data Length Bytes
            uint8_t len_data = getRegistersPerChannel(sensor->slaveId, sensor->sensorId);
            // Asumimos formato simple: No PKD, No 2BIT
            // (Debes cambiar esto si tus sensores externos usan packing)
            uint8_t len_byte = (len_data & 0x1F);
            payload.push_back(len_byte);
        }
    }

    // ================== 5. BLOQUES DE DATOS (Resto) ==================
    // Añadimos los datos de cada sensor activo, en el mismo orden.
    // Esta es la mayor simplificación: asumimos que `sensor->data`
    // ya contiene los bytes listos para enviar (ya procesados/empaquetados).

    // --- Batería (Bit 0) ---
    if (activate_byte & (1 << 0)) {
        const auto& sensor = activeSensors.at(SENSOR_ID_BATERIA);
        payload.insert(payload.end(), sensor->data, sensor->data + sensor->dataSize);
    }

    // --- Voltaje (Bit 1) ---
    if (activate_byte & (1 << 1)) {
        const auto& sensor = activeSensors.at(SENSOR_ID_VOLTAJE);
        payload.insert(payload.end(), sensor->data, sensor->data + sensor->dataSize);
    }
    
    // --- Corriente (Bit 2) ---
    if (activate_byte & (1 << 2)) {
        const auto& sensor = activeSensors.at(SENSOR_ID_CORRIENTE);
        payload.insert(payload.end(), sensor->data, sensor->data + sensor->dataSize);
    }

    // --- Sensores Externos (Bits 3+) ---
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        if (activate_byte & (1 << (i + 3))) {
            uint8_t current_sensor_id = SENSOR_ID_EXT_START + i;
            const auto& sensor = activeSensors.at(current_sensor_id);
            payload.insert(payload.end(), sensor->data, sensor->data + sensor->dataSize);
        }
    }

    return payload;
}


// ==================== LORA CONFIG ====================
// Disable RX receive window (if no downlinks are expected)
//#define DISABLE_INVERT_IQ_ON_RX 1
//#define DISABLE_RX 1
//#define CFG_sx1272_radio 1

/**
 * @brief LMIC configuration functions (placeholders).
 * @ingroup group_lorawan
 */
void os_getArtEui(u1_t *buf) { memset(buf, 0, 8); }
void os_getDevEui(u1_t *buf) { memset(buf, 0, 8); }
void os_getDevKey(u1_t *buf) { memset(buf, 0, 16); }

/**
 * @brief ABP keys and device address (DEMO).
 * @warning Keys in plaintext, replace in production.
 * @ingroup group_lorawan
 */
static u1_t NWKSKEY[16] = {
    0xC2, 0x5B, 0x0A, 0x78, 0xA8, 0x0A, 0x63, 0x1D,
    0x86, 0xC8, 0x1B, 0xA3, 0x3A, 0x9E, 0x36, 0xEF
};

static u1_t APPSKEY[16] = {
    0x42, 0x8F, 0x67, 0xFA, 0xD7, 0xD7, 0x4A, 0x85,
    0x3C, 0x10, 0x80, 0x5F, 0x10, 0x1A, 0x0E, 0x14
};

static const u4_t DEVADDR = 0x260C691F;

// LoRa Pin Configuration
///**
// * @brief Pin map for the SX127x radio of the TTGO LoRa32.
// * @details Defines NSS, DIO0-DIO2 and pins not used by LMIC.
// * @ingroup group_lorawan
// */
const lmic_pinmap lmic_pins = {.nss = 18,
                               .rxtx = LMIC_UNUSED_PIN,
                               .rst = LMIC_UNUSED_PIN,
                               .dio = {26, 33, 32}};

// Max payload for DR3
/**
 * @def LORA_PAYLOAD_MAX
 * @brief Maximum payload size for the DR used.
 * @ingroup group_lorawan
 */
constexpr size_t LORA_PAYLOAD_MAX = 220;

/**
 * @struct Fragmento
 * @brief Binary fragment ready for LoRaWAN transmission.
 * @details Contains the buffer and its effective length, produced by the aggregation task.
 * @ingroup group_lorawan
 */
struct Fragmento {
    uint8_t data[LORA_PAYLOAD_MAX]; ///< Data buffer.
    size_t len;                     ///< Length of the data.
};


// ==================== LORA CALLBACKS ====================
/**
 * @brief LMIC/LoRaWAN event callback.
 * @details Handles events from the LMIC stack and synchronizes the transmission flow using `semaforoEnvioCompleto`.
 * - EV_TXCOMPLETE: indicates end of TX and releases the semaphore.
 * - TXRX_ACK: reports if an ACK was received from the network.
 * @param ev Event reported by LMIC.
 * @ingroup group_lorawan
 */
void onEvent(ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        Serial.println("[LORA] TX completo.");
        // Libera el semáforo para indicar que el ciclo de transmisión ha terminado.
        xSemaphoreGive(semaforoEnvioCompleto);
        if (LMIC.txrxFlags & TXRX_ACK) {
            Serial.println("[LORA] ACK recibido.");
        }
    }
}

// ==================== LORA FUNCTIONS ====================
/**
 * @brief Initializes the LMIC stack and configures LoRaWAN (ABP, US915).
 * @details
 * - Calls `os_init()` and `LMIC_reset()`.
 * - Adjusts `LMIC_setClockError()` (1%) for crystal tolerance.
 * - Configures ABP session with `LMIC_setSession()` and US915 region (`LMIC_selectSubBand(7)`).
 * - Disables ADR and LinkCheck to simplify the flow.
 * @note Adjust sub-banda, DR y potencia según gateway/región.
 * @ingroup group_lorawan
 */
void initLoRa() {
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100); // 1% de tolerancia

    // Configuración específica para ABP y US915
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
    LMIC_selectSubBand(7); // Asegúrate que esta es la sub-banda correcta para tu gateway
    LMIC_setDrTxpow(US915_DR_SF7, 20); // DR3 = SF7BW125
    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);
}

// ==================== TAREA LORA ====================
/**
 * @brief Task dedicated to sending data via LoRaWAN.
 * @details
 * - Waits for fragments in `queueFragmentos`.
 * - Takes `semaforoEnvioCompleto` to serialize transmissions.
 * - Calls `LMIC_setTxData2()` with application port 1 and without confirmation (confirmed=0).
 * @ingroup group_lorawan
 */
void tareaLoRa(void *pvParameters) {
    Fragmento frag;
    while (true) {
        if (xQueueReceive(queueFragmentos, &frag, portMAX_DELAY) == pdTRUE) {
            // Espera semáforo antes de enviar
            xSemaphoreTake(semaforoEnvioCompleto, portMAX_DELAY);
            Serial.printf("[LORA] Enviando fragmento de %u bytes...\n", frag.len);
            Serial.println("[LORA] Datos:");
            for (size_t i = 0; i < frag.len; i++) {
                Serial.print("0x");
                if (frag.data[i] < 0x10) Serial.print("0");
                Serial.print(frag.data[i], HEX);
                Serial.print(",");
            }
            LMIC_setTxData2(1, frag.data, frag.len, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // sólo lógica propia, no runloop
    }
}

/**
 * @brief LMIC runloop task.
 * @details Executes `os_runloop_once()` in a loop to process timers and internal LMIC events.
 * @ingroup group_lorawan
 */
void tareaRunLoop(void *pvParameters) {
    while (true) {
        os_runloop_once();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

#define AGGREGATION_INTERVAL_MS 6100 ///< Aggregation interval (6s + 100ms margin).

/**
 * @brief Proactive task that collects and packages sensor data at a fixed rate.
 * @details Wakes up every AGGREGATION_INTERVAL_MS, empties the data queue, and sends a
 * single LoRaWAN payload if it has collected anything.
 * @ingroup group_data_format
 */
void DataAggregatorTask(void *pvParameters) {
    std::vector<SensorDataPayload> collectedPayloads;
    SensorDataPayload incomingPayload;
    uint8_t ID_MSG = 0x00;
    const TickType_t aggregationCycle = pdMS_TO_TICKS(AGGREGATION_INTERVAL_MS);

    while (true) {
        // 1. Esperar el ciclo de agregación completo.
        vTaskDelay(aggregationCycle);
        Serial.printf("\n[Agregador] Ciclo de agregación iniciado. Recolectando datos de la cola...\n");

        collectedPayloads.clear();

        // 2. Vaciar la cola: recolectar todos los payloads que hayan llegado.
        // Se usa un timeout de 0 para no bloquear y salir inmediatamente si la cola está vacía.
        while (xQueueReceive(queueSensorDataPayload, &incomingPayload, 0) == pdTRUE) {
            collectedPayloads.push_back(incomingPayload);
            Serial.printf("[Agregador] Recolectado payload de Slave %u, Sensor %u.\n", incomingPayload.slaveId, incomingPayload.sensorId);
        }

        // 3. Si se recolectaron datos, empaquetar y enviar.
        if (!collectedPayloads.empty()) {
            Serial.printf("[Agregador] Recolección finalizada. Empaquetando %u payloads para LoRa.\n", collectedPayloads.size());

            // Construir el payload unificado
            std::vector<uint8_t> unifiedPayload = construirPayloadUnificado(ID_MSG++, collectedPayloads);

            // Preparar el fragmento para la tarea LoRa
            Fragmento loraFragment;
            loraFragment.len = std::min(unifiedPayload.size(), LORA_PAYLOAD_MAX);
            memcpy(loraFragment.data, unifiedPayload.data(), loraFragment.len);

            // 4. Enviar el fragmento a la tarea LoRa
            if (loraFragment.len > 0) {
                if (xQueueSend(queueFragmentos, &loraFragment, pdMS_TO_TICKS(100)) == pdTRUE) {
                    Serial.printf("[Agregador] Fragmento LoRa de %u bytes enviado a la cola.\n", loraFragment.len);
                } else {
                    Serial.println("[Agregador] ERROR: No se pudo encolar el fragmento LoRa.");
                }
            }
        } else {
            // Si no se recolectó nada, simplemente se informa y se espera al siguiente ciclo.
            Serial.println("[Agregador] No se encontraron payloads en este ciclo. Esperando al siguiente.");
        }
    }
}

// ==================== SETUP AND LOOP ====================
void setup() {
    Serial.begin(115200); // 115200 es más estándar y estable que 921600
    while (!Serial); // Espera a que el puerto serie esté listo
    delay(1000);
    Serial.println("Iniciando sistema...");

    // Inicialización SPI
    SPI.begin();

    // Inicializar la API Modbus síncrona
    modbus_api_init(Serial2, RX_PIN, TX_PIN);

    // Crear colas y semáforos
    schedulerMutex = xSemaphoreCreateMutex();

    queueSensorDataPayload = xQueueCreate(10, sizeof(SensorDataPayload));
    
    xTaskCreatePinnedToCore(DataRequestScheduler, "Scheduler", 4096, NULL, 3, &dataRequestSchedulerHandle, 0);

    // --- INICIAR DESCUBRIMIENTO ---
    // Creamos una tarea solo para el descubrimiento. Se ejecutará una vez y se borrará.
    xTaskCreate(initialDiscoveryTask, "InitialDiscovery", 4096, NULL, 2, NULL);

    // Creación de cola para fragmentos
    queueFragmentos = xQueueCreate(10, sizeof(Fragmento));

    // Creación de semáforo binario para señalizar el fin de la transmisión
    semaforoEnvioCompleto = xSemaphoreCreateBinary();
    xSemaphoreGive(semaforoEnvioCompleto); // Lo dejamos disponible para el primer envío

    // Inicialización de LoRa
    initLoRa();

    xTaskCreatePinnedToCore(tareaRunLoop, "RunLoop", 2048, NULL, 2, NULL, 1);

    // Creación de tarea LoRa en el núcleo 1
    xTaskCreatePinnedToCore(tareaLoRa, "LoRaTask", 2048, NULL, 5, NULL, 1); // 4KB y prioridad 5

    xTaskCreatePinnedToCore(DataAggregatorTask, "DataAggregator", 4096, NULL, 3, NULL, 1);
}

void loop() {
    // The main loop can be left empty or used for low-priority tasks.
    // It's better that the loop task is not deleted and simply yields control.
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// ==================== CONTROL FUNCTIONS ====================

/**
 * @brief Función interna para añadir las tareas de un esclavo al planificador.
 * @warning ESTA FUNCIÓN DEBE SER LLAMADA ÚNICAMENTE MIENTRAS SE POSEE EL 'schedulerMutex'.
 * @param slave El esclavo cuyas tareas de sensor se añadirán.
 */
void _internal_addSlaveToScheduler(const ModbusSlaveParam& slave) {
    for (const auto& sensor : slave.sensors) {
        uint32_t calculatedInterval = sensor.samplingInterval;
        if (sensor.numberOfChannels > 0 && sensor.maxRegisters > 0) {
            uint16_t registersPerChannel = sensor.maxRegisters / sensor.numberOfChannels;
            calculatedInterval = (uint32_t)sensor.samplingInterval * registersPerChannel;
        }

        scheduleList.push_back({
            slave.slaveID,
            sensor.sensorID,
            (uint16_t)calculatedInterval,
            millis() // Muestreo inmediato para el nuevo sensor
        });
        Serial.printf("  [Control] Tarea para SensorID %u añadida al planificador.\n", sensor.sensorID);
    }
}

bool _internal_removeSlave(uint8_t slaveId) {
    // 1. Eliminar el esclavo de slaveList
    auto slaveIt = std::remove_if(slaveList.begin(), slaveList.end(), 
        [slaveId](const ModbusSlaveParam& s) { return s.slaveID == slaveId; });

    if (slaveIt == slaveList.end()) {
        // El esclavo no fue encontrado
        return false;
    }
    
    slaveList.erase(slaveIt, slaveList.end());
    Serial.printf("[Control] Esclavo %u eliminado de slaveList.\n", slaveId);

    // 2. Eliminar las entradas correspondientes de scheduleList
    auto scheduleIt = std::remove_if(scheduleList.begin(), scheduleList.end(),
        [slaveId](const SensorSchedule& item) { return item.slaveID == slaveId; });
    
    scheduleList.erase(scheduleIt, scheduleList.end());
    Serial.printf("[Control] Tareas del esclavo %u eliminadas del planificador.\n", slaveId);

    return true;
}

/**
 * @brief Pausa la ejecución de la tarea DataRequestScheduler.
 * @details Utiliza el handle de la tarea para suspenderla. Es seguro llamarla múltiples veces.
 */
void pauseScheduler() {
    if (dataRequestSchedulerHandle != NULL) {
        vTaskSuspend(dataRequestSchedulerHandle);
        Serial.println("[Control] Planificador pausado.");
    }
}

/**
 * @brief Reanuda la ejecución de la tarea DataRequestScheduler.
 * @details Utiliza el handle de la tarea para reanudarla si estaba suspendida.
 */
void resumeScheduler() {
    if (dataRequestSchedulerHandle != NULL) {
        vTaskResume(dataRequestSchedulerHandle);
        Serial.println("[Control] Planificador reanudado.");
    }
}

/**
 * @brief Intenta descubrir y registrar un nuevo esclavo dinámicamente.
 * @details Realiza una consulta de descubrimiento al ID del esclavo. Si tiene éxito,
 *          el esclavo se añade a la lista global y se reconstruye el planificador.
 *          La función es segura para ser llamada en cualquier momento.
 * @param slaveId El ID del esclavo a descubrir y registrar.
 * @return true si el esclavo respondió y fue registrado, false en caso contrario.
 */
bool registerSlave(uint8_t slaveId) {
    Serial.printf("[Control] Intentando registrar esclavo con ID %u...\n", slaveId);

    bool success = discoverDeviceSensors(slaveId);

    if (success) {
        Serial.printf("[Control] Esclavo %u respondió. Actualizando planificador...\n", slaveId);
        
        if (xSemaphoreTake(schedulerMutex, portMAX_DELAY) == pdTRUE) {
            // Encontrar el esclavo que acabamos de añadir a slaveList
            auto slaveIt = std::find_if(slaveList.begin(), slaveList.end(),
                [&](const ModbusSlaveParam& s) { return s.slaveID == slaveId; });

            if (slaveIt != slaveList.end()) {
                _internal_addSlaveToScheduler(*slaveIt);
            }
            xSemaphoreGive(schedulerMutex);
        }
        
    } else {
        Serial.printf("[Control] FALLO: El esclavo %u no respondió.\n", slaveId);
    }

    return success;
}

/**
 * @brief Elimina un esclavo de la lista y actualiza el planificador.
 * @param slaveId El ID del esclavo a eliminar.
 */
void unregisterSlave(uint8_t slaveId) {
    if (xSemaphoreTake(schedulerMutex, portMAX_DELAY) == pdTRUE) {
        if (!_internal_removeSlave(slaveId)) {
            Serial.printf("[Control] No se encontró el esclavo %u para eliminar.\n", slaveId);
        }
        xSemaphoreGive(schedulerMutex);
    }
}