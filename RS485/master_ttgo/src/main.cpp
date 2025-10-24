#include <Arduino.h>
#include <SPI.h>
#include <hal/hal.h>
#include <lmic.h>
#include <vector>
#include <cstdint>      // Para uint8_t, uint32_t
#include <map>          // Para std::map (manejo de sensores)
#include <ctime>        // Para time() (generar timestamp)
#include <cstring>      // Para memcpy (aunque usamos insert)
#include <algorithm>
#include "ModbusClientRTU.h"

// =================================================================================================
// Configuración Modbus RTU
// =================================================================================================

#define RX_PIN 12
#define TX_PIN 13

ModbusClientRTU MB;

typedef struct {
    uint8_t sensorID;
    uint8_t numberOfChannels;
    uint16_t startAddress;
    uint16_t maxRegisters;
    uint16_t samplingInterval;     // ms
    uint8_t dataType;              // 1=uint8, 2=uint16, 3=compressed bytes, 4=float16
    uint8_t scale;                 // 10^scale
    uint8_t compressedBytes;       // Solo si dataType= 3
} ModbusSensorParam;

struct ModbusSlaveParam {
    uint8_t slaveID;
    std::vector<ModbusSensorParam> sensors;
    uint8_t consecutiveFails;
};

// Lista de esclavos Modbus
std::vector<ModbusSlaveParam> slaveList;

enum DiscoveryOrder {
    DISCOVERY_GET_COUNT = 1,
    DISCOVERY_GET_DATA_OFFSET = 255,
    DISCOVERY_READ_SENSOR_PARAM = 8   // Nueva orden: leer regs 0..7 (8 regs)
};

enum RequestType {
    REQUEST_UNKNOWN,
    REQUEST_DISCOVERY,
    REQUEST_SAMPLING
};

// Gestor de solicitudes Modbus RTU
struct ModbusRequestInfo {
    uint32_t token;           // Token de la solicitud
    uint8_t slaveId;          // Esclavo al que fue enviada
    uint8_t sensorId;        // ID del sensor
    uint8_t functionCode;     // Código de función Modbus
    RequestType type;         // Propósito de la solicitud (descubrimiento o muestreo)
};

constexpr size_t MAX_REQUESTS = 16;
ModbusRequestInfo requestBuffer[MAX_REQUESTS];
size_t requestHead = 0; // Próxima posición para guardar

void addRequest(uint32_t token, uint8_t slaveId, uint8_t sensorId, uint8_t functionCode, RequestType type) {
    requestBuffer[requestHead] = ModbusRequestInfo{token, slaveId, sensorId, functionCode, type};
    requestHead = (requestHead + 1) % MAX_REQUESTS;
}

ModbusRequestInfo* findRequestByToken(uint32_t token) {
    for (size_t i = 0; i < MAX_REQUESTS; ++i) {
        if (requestBuffer[i].token == token) {
            return &requestBuffer[i];
        }
    }
    return nullptr;
}

// =======================================================================
// CALLBACK para manejar respuestas de datos exitosas
// =======================================================================

#define MAX_MODBUS_RESPONSE_LENGTH 256 // O el máximo que esperes

typedef struct {
    uint8_t data[MAX_MODBUS_RESPONSE_LENGTH];
    size_t length; // número de bytes válidos en data
    uint8_t deviceId; // Opcional: saber a quién corresponde la respuesta
    uint8_t order;    // Opcional: para correlacionar con la solicitud
} ResponseFormat;

QueueHandle_t queueEventos_Peripheral;
QueueHandle_t queueEventos_Scheduler;
QueueHandle_t queueRespuestas;

// Estructura para el scheduler de muestreo
struct SensorSchedule {
    uint8_t slaveID;
    uint8_t sensorID;
    uint16_t samplingInterval; // ms
    uint32_t nextSampleTime;   // millis() en el que toca muestrear
};

// Lista de sensores a muestrear
std::vector<SensorSchedule> scheduleList;
SemaphoreHandle_t schedulerMutex;

void initScheduler() {
    // Tomar el control exclusivo de las listas
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

void handleData(ModbusMessage response, uint32_t token) {
    ResponseFormat resp;
    resp.length = response.size();
    resp.deviceId = response.getServerID();
    resp.order = token; // Puedes usar el token para correlacionar la respuesta con la solicitud

    // Copia los datos recibidos
    size_t len = resp.length < MAX_MODBUS_RESPONSE_LENGTH ? resp.length : MAX_MODBUS_RESPONSE_LENGTH;
    for (size_t i = 0; i < len; ++i) {
        resp.data[i] = response[i];
    }
    resp.length = len;

    // Enviar la respuesta a la cola
    xQueueSend(queueRespuestas, &resp, pdMS_TO_TICKS(10));
}

void handleError(Error error, uint32_t token) {
    ModbusError me(error);
    Serial.printf("\nError en respuesta para Token %u: %02X - %s\n", token, (int)me, (const char *)me);

    // Solo nos interesa gestionar los timeouts
    //0xE0 = TIMEOUT
    if ((const char *)me == "TIMEOUT") {
        ModbusRequestInfo* request = findRequestByToken(token);
        if (request) {
            uint8_t failedSlaveId = request->slaveId;
            Serial.printf("Timeout detectado para esclavo %u.\n", failedSlaveId);

            // Buscar el esclavo en la lista principal
            auto slaveIt = std::find_if(slaveList.begin(), slaveList.end(),
                [&](const ModbusSlaveParam& s) { return s.slaveID == failedSlaveId; });

            if (slaveIt != slaveList.end()) {
                slaveIt->consecutiveFails++;
                Serial.printf("Esclavo %u ahora tiene %u fallos consecutivos.\n", failedSlaveId, slaveIt->consecutiveFails);

                if (slaveIt->consecutiveFails >= 3) {
                    Serial.printf("Esclavo %u ha alcanzado 3 fallos. Eliminando de la lista de activos.\n", failedSlaveId);
                    
                    // Eliminar el esclavo de la lista principal
                    slaveList.erase(slaveIt);

                    // Actualizar la lista del scheduler para que deje de pedirle datos
                    // Usamos un mutex para proteger el acceso a scheduleList
                    if (xSemaphoreTake(schedulerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        scheduleList.erase(
                            std::remove_if(scheduleList.begin(), scheduleList.end(),
                                [&](const SensorSchedule& s) { return s.slaveID == failedSlaveId; }),
                            scheduleList.end()
                        );
                        xSemaphoreGive(schedulerMutex);
                        initScheduler(); // Re-inicializar el scheduler con la nueva lista
                        Serial.printf("Esclavo %u eliminado del scheduler.\n", failedSlaveId);
                    } else {
                        Serial.println("Error: No se pudo tomar el mutex del scheduler para eliminar el esclavo.");
                    }
                }
            }
            // Invalidar el token para no procesarlo de nuevo
            request->token = 0; 
        } else {
            Serial.printf("Error: No se encontró la solicitud para el token %u.\n", token);
        }
    }
}

std::vector<uint8_t> dispositivosAConsultar = {1, 2, 3};

struct EventManagerFormat {
    uint8_t slaveId;
    uint8_t sensorID;
    uint16_t order;
};

// Realiza el proceso de descubrimiento para un solo dispositivo.
// Esta función se ejecuta en el contexto de la tarea que la llama.
bool discoverDeviceSensors(uint8_t deviceId) {
    Serial.printf("Iniciando descubrimiento simple para dispositivo %u...\n", deviceId);

    // Enviar una sola lectura: FC03 desde addr=0, qty=8
    EventManagerFormat ev = {deviceId, 0, DISCOVERY_READ_SENSOR_PARAM};
    if (xQueueSend(queueEventos_Peripheral, &ev, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("Error: No se pudo encolar el evento de descubrimiento (lectura 0..7).");
        return false;
    }
    return true;
}

// TAREA TEMPORAL para el descubrimiento inicial
void initialDiscoveryTask(void *pvParameters) {
    Serial.println("--- Tarea de Descubrimiento Inicial: Iniciando ---");
    const TickType_t delayBetweenDevices = pdMS_TO_TICKS(50);

    for (uint8_t deviceId : dispositivosAConsultar) {
        if (discoverDeviceSensors(deviceId)) {
            Serial.printf("Mensaje de descubrimiento enviado exitosamente para el dispositivo %u.\n", deviceId);
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



// Tarea del scheduler
void DataRequestScheduler(void *pvParameters) {
    // La inicialización se hace desde la tarea de descubrimiento ahora
    // initScheduler(); 

    while (true) {
        uint32_t now = millis();
        TickType_t sleepTime = pdMS_TO_TICKS(1000); // Por defecto, esperar 1s si no hay nada que hacer

        // Tomar el control para leer la lista de planificación
        if (xSemaphoreTake(schedulerMutex, portMAX_DELAY) == pdTRUE) {
            
            if (!scheduleList.empty()) {
                uint32_t nextEventTime = UINT32_MAX;

                for (auto& item : scheduleList) {
                    if (now >= item.nextSampleTime) {
                        // Es hora de enviar el evento
                        EventManagerFormat event = {item.slaveID, item.sensorID, 1};
                        Serial.printf("Enviando solicitud de muestreo: SlaveID=%u, SensorID=%u, Intervalo=%u ms\n", item.slaveID, item.sensorID, item.samplingInterval);
                        xQueueSend(queueEventos_Scheduler, &event, pdMS_TO_TICKS(10));
                        
                        // Actualizar el próximo tiempo de muestreo
                        item.nextSampleTime = now + item.samplingInterval;
                    }
                    // Buscar el próximo evento más cercano en el futuro
                    if (item.nextSampleTime < nextEventTime) {
                        nextEventTime = item.nextSampleTime;
                    }
                }

                // Calcular cuánto dormir hasta el próximo evento
                now = millis(); // Actualizar 'now' por si el bucle tardó mucho
                if (nextEventTime > now) {
                    sleepTime = pdMS_TO_TICKS(nextEventTime - now);
                } else {
                    // Si todos los eventos ya pasaron, hacer una pequeña pausa para no saturar
                    sleepTime = pdMS_TO_TICKS(10); 
                }
            }
            // Si la lista está vacía, sleepTime mantiene su valor por defecto (1000 ms)
            
            // Devolver el control
            xSemaphoreGive(schedulerMutex);
        }

        // Dormir la cantidad de tiempo calculada
        vTaskDelay(sleepTime);
    }
}

// Devuelve true si encuentra el sensor, y asigna startAddr y numRegs
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

// Devuelve el número de registros por canal para un sensor dado.
// Si no encuentra el sensor, retorna 0.
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

// Procesa la respuesta de descubrimiento, crea/actualiza el esclavo y su sensor.
void parseAndStoreDiscoveryResponse(const ResponseFormat& response, uint8_t slaveId) {
    // Se esperan 8 registros, que son 16 bytes de datos.
    if (response.length < 16) {
        Serial.printf("Error: Respuesta de descubrimiento incompleta para esclavo %u. Se esperaban 16 bytes, se recibieron %u.\n", slaveId, response.length);
        return;
    }

    ModbusSensorParam newSensor;
    // Asumimos que cada registro (2 bytes) corresponde a un campo de la estructura.
    // Los datos de Modbus vienen en formato Big Endian.
    // El offset correcto para los datos de registros Modbus es 3 (después del encabezado Modbus RTU)
    // Cada registro son 2 bytes (big-endian)
    newSensor.sensorID          = response.data[3 + 1];  // Reg 0: sensorID (byte bajo)
    newSensor.numberOfChannels  = response.data[3 + 3];  // Reg 1: numberOfChannels (byte bajo)
    newSensor.startAddress      = (response.data[3 + 4] << 8) | response.data[3 + 5];   // Reg 2: startAddress
    newSensor.maxRegisters      = (response.data[3 + 6] << 8) | response.data[3 + 7];   // Reg 3: maxRegisters
    newSensor.samplingInterval  = (response.data[3 + 8] << 8) | response.data[3 + 9];   // Reg 4: samplingInterval
    newSensor.dataType          = response.data[3 + 11]; // Reg 5: dataType (byte bajo)
    newSensor.scale             = response.data[3 + 13]; // Reg 6: scale (byte bajo)
    newSensor.compressedBytes   = response.data[3 + 15]; // Reg 7: compressedBytes (byte bajo)

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

void EventManager(void *pvParameters) {
    uint32_t requestToken = 0;
    while (true) {
        EventManagerFormat event;
        bool requestSent = false;
        uint8_t functionCode = 0;
        Error err = Error::SUCCESS;
        RequestType reqType = REQUEST_UNKNOWN;

        // Prioridad: primero scheduler, luego peripheral
        if (xQueueReceive(queueEventos_Scheduler, &event, 0) == pdTRUE) {
            uint16_t startAddr, numRegs;
            if (getSensorParams(event.slaveId, event.sensorID, startAddr, numRegs)) {
                functionCode = READ_HOLD_REGISTER;
                reqType = REQUEST_SAMPLING;
                err = MB.addRequest(++requestToken, event.slaveId, functionCode, startAddr, numRegs);
                requestSent = true;
            } else {
                Serial.printf("Parámetros no encontrados para SlaveID %u, SensorID %u\n", event.slaveId, event.sensorID);
            }
        } else if (xQueueReceive(queueEventos_Peripheral, &event, 0) == pdTRUE) {
            reqType = REQUEST_DISCOVERY; // Todas las solicitudes de periféricos son de descubrimiento
            if (event.order == DISCOVERY_READ_SENSOR_PARAM) {
                functionCode = READ_HOLD_REGISTER;          // FC03
                err = MB.addRequest(++requestToken, event.slaveId, functionCode, 0, 8); // addr=0, qty=8
                requestSent = true;
            } else if (event.order == DISCOVERY_GET_COUNT) {
                functionCode = READ_HOLD_REGISTER;
                err = MB.addRequest(++requestToken, event.slaveId, functionCode, 0, event.order);
                requestSent = true;
            } else if (event.order >= DISCOVERY_GET_DATA_OFFSET) {
                uint16_t writeValue = event.order - DISCOVERY_GET_DATA_OFFSET;
                functionCode = READ_HOLD_REGISTER; // Asumiendo que es la misma función
                err = MB.addRequest(++requestToken, event.slaveId, functionCode, DISCOVERY_GET_COUNT, writeValue);
                requestSent = true;
            }
        }

        if (requestSent && err == Error::SUCCESS) {
            addRequest(requestToken, event.slaveId, event.sensorID, functionCode, reqType);
        } else if (requestSent) {
            Serial.printf("Error al encolar solicitud Modbus para token %u\n", requestToken);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeño delay para evitar saturar la CPU
    }
}

// ==================== EMPAQUETADOR DE BITS ====================
struct BitPacker
{
    uint64_t buffer = 0; // acumulador de hasta 64 bits
    int bits_usados = 0; // cuántos bits válidos hay en el buffer

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

static bool formatAndEnqueueSensorData(const ResponseFormat& response, const ModbusRequestInfo& request, std::vector<uint8_t>& values) {
    uint8_t slaveId = request.slaveId;

    auto slaveIt = std::find_if(slaveList.begin(), slaveList.end(),
        [&](const ModbusSlaveParam& s) { return s.slaveID == slaveId; });
    if (slaveIt == slaveList.end()) {
        Serial.printf("Formato: no se encontró el esclavo %u.\n", slaveId);
        return false;
    }

    uint8_t sensorID = request.sensorId;

    auto sensorIt = std::find_if(slaveIt->sensors.begin(), slaveIt->sensors.end(),
        [&](const ModbusSensorParam& sensor) { return sensor.sensorID == sensorID; });
    if (sensorIt == slaveIt->sensors.end()) {
        Serial.printf("Formato: no se encontró el sensor %u en esclavo %u.\n", sensorID, slaveId);
        return false;
    }

    const ModbusSensorParam& params = *sensorIt;
    Serial.printf("Formato: esclavo %u sensor %u -> regs:%u tipo:%u escala:%u comp:%u\n",
                  slaveId, params.sensorID, params.maxRegisters,
                  params.dataType, params.scale, params.compressedBytes);

    uint8_t dataType = params.dataType; // 1=uint8, 2=uint16
    uint8_t scale = params.scale;
    uint8_t compressedBytes = params.compressedBytes;

    values.clear();
    if (compressedBytes > 0) {
        BitPacker packer;
        for (size_t i = 0; i < params.maxRegisters; ++i) {
            size_t offset = 3 + i * 2;
            if ((offset + 1) >= response.length) {
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
            size_t offset = 3 + i * 2;
            if ((offset + 1) >= response.length) {
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
    // TODO: decodificar según params.dataType, aplicar escala, manejar compresión si params.dataType == 3
    // TODO: encolar payload formateado para LoRa

    return true;
}

#define MAX_SENSOR_PAYLOAD 128

struct SensorDataPayload {
    uint8_t slaveId;
    uint8_t sensorId;
    uint8_t data[MAX_SENSOR_PAYLOAD]; // Array de tamaño fijo
    size_t dataSize;                  // Para saber cuántos bytes son válidos
};

QueueHandle_t queueSensorDataPayload;

void DataFormatter (void *pvParameters) {
    while (true) {
        ResponseFormat response;
        // Esperar indefinidamente por una respuesta en la cola
        if (xQueueReceive(queueRespuestas, &response, portMAX_DELAY) == pdTRUE) {
            
            // Usar el token de la respuesta para encontrar la solicitud original
            ModbusRequestInfo* request = findRequestByToken(response.order);

            if (request && request->token != 0) {
                Serial.printf("Respuesta recibida para Token %u (Esclavo %u)\n", request->token, request->slaveId);
                std::vector<uint8_t> values;
                // Diferenciar el tratamiento según el tipo de solicitud
                switch (request->type) {
                    case REQUEST_DISCOVERY:
                        Serial.println("-> Procesando respuesta de DESCUBRIMIENTO.");
                        parseAndStoreDiscoveryResponse(response, request->slaveId);
                        //initScheduler(); // Re-inicializar el scheduler con la nueva lista de sensores
                        break;

                    case REQUEST_SAMPLING:
                        Serial.println("-> Procesando respuesta de MUESTREO de datos.");
                        
                        if (!formatAndEnqueueSensorData(response, *request, values)) {
                            Serial.println("Formato: fallo al procesar datos de muestreo.");
                        }
                        else {
                            // Encolar el payload formateado
                            SensorDataPayload payload;
                            payload.slaveId = request->slaveId;
                            payload.sensorId = request->sensorId;


                            // Copiar los datos de forma segura
                            payload.dataSize = std::min(values.size(), (size_t)MAX_SENSOR_PAYLOAD);
                            memcpy(payload.data, values.data(), payload.dataSize);

                            if (xQueueSend(queueSensorDataPayload, &payload, pdMS_TO_TICKS(10)) != pdTRUE) {
                                Serial.println("Error: No se pudo encolar el payload de datos del sensor.");
                            } else {
                                Serial.printf("Payload de datos del sensor encolado: Esclavo %u, Sensor %u, Bytes %u\n",
                                              payload.slaveId, payload.sensorId, payload.dataSize);
                            }
                        }
                        break;

                    default:
                        Serial.printf("Tipo de solicitud desconocido para Token %u\n", request->token);
                        break;
                }

                // Invalidar el token para que no se procese de nuevo (importante si hay reintentos o errores)
                request->token = 0;

            } else {
                Serial.printf("No se encontró información de solicitud para el token %u, o ya fue procesado.\n", response.order);
            }
        }
    }
}

/**
 * @brief Tarea temporal para imprimir los payloads de datos formateados.
 * Esto simula el consumidor final (que será la tarea LoRa).
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

// --- Mapeo de Sensor ID a Bits del Activate Byte ---
// ¡IMPORTANTE! Debes ajustar estos IDs a los que uses en tu sistema.
// Estos son solo ejemplos basados en el contexto.
const uint8_t SENSOR_ID_BATERIA = 0;   // Asignado al Bit 0
const uint8_t SENSOR_ID_VOLTAJE = 1;   // Asignado al Bit 1
const uint8_t SENSOR_ID_CORRIENTE = 2; // Asignado al Bit 2
const uint8_t SENSOR_ID_EXT_START = 3; // IDs 3, 4, 5, 6, 7 (Bits 3-7)
const int MAX_SENSORES_EXTERNOS = 5;   // 5 bits restantes (3 al 7)

/**
 * @brief Construye un payload unificado a partir de una colección de datos de sensores.
 * * Sigue el formato: [ID_MSG][TIMESTAMP][ACTIVATE_BYTE][LEN_BYTES...][DATA_BLOCKS...]
 * * @param id_mensaje El byte de ID del mensaje (Cabecera).
 * @param collectedPayloads El vector con los datos recolectados.
 * @return std::vector<uint8_t> El payload binario listo para enviar.
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
        uint8_t len_byte = 0x80 | (len_data & 0x1F);
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
// Deshabilitar ventana RX de recepción (si no se esperan downlinks)
//#define DISABLE_INVERT_IQ_ON_RX 1
//#define DISABLE_RX 1
//#define CFG_sx1272_radio 1

// Funciones de configuración LoRa (placeholders)
void os_getArtEui(u1_t *buf) { memset(buf, 0, 8); }
void os_getDevEui(u1_t *buf) { memset(buf, 0, 8); }
void os_getDevKey(u1_t *buf) { memset(buf, 0, 16); }

// Claves de red y dispositivo (reemplaza con tus valores reales)
static u1_t NWKSKEY[16] = {0x49, 0x78, 0xCB, 0x8E, 0x7F, 0xFB, 0xD4, 0x6B,
                           0xC5, 0x70, 0xFE, 0x11, 0xF1, 0x7F, 0xA5, 0x6E};
static u1_t APPSKEY[16] = {0x53, 0xC0, 0x20, 0x84, 0x14, 0x86, 0x26, 0x39,
                           0x81, 0xFA, 0x77, 0x35, 0x5D, 0x27, 0x87, 0x62};
static const u4_t DEVADDR = 0x260CB229;

// Configuración de pines LoRa
const lmic_pinmap lmic_pins = {.nss = 18,
                               .rxtx = LMIC_UNUSED_PIN,
                               .rst = LMIC_UNUSED_PIN,
                               .dio = {26, 33, 32}};

// Payload máximo para DR3
constexpr size_t LORA_PAYLOAD_MAX = 220;

// Estructura para fragmentos de datos LoRa
struct Fragmento {
    uint8_t data[LORA_PAYLOAD_MAX];
    size_t len;
};

// Cola para mensajes binarios LoRa
QueueHandle_t queueFragmentos;

// Semáforo para controlar el fin de un ciclo de envío
SemaphoreHandle_t semaforoEnvioCompleto;

// ==================== LORA CALLBACKS ====================
// Callback de eventos LoRa
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

// ==================== FUNCIONES LORA ====================
// Inicialización de LoRa
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
// Tarea dedicada para el envío de datos LoRaWAN
void tareaLoRa(void *pvParameters) {
    Fragmento frag;
    while (true) {
        if (xQueueReceive(queueFragmentos, &frag, portMAX_DELAY) == pdTRUE) {
            // Espera semáforo antes de enviar
            //xSemaphoreTake(semaforoEnvioCompleto, portMAX_DELAY);
            Serial.printf("[LORA] Enviando fragmento de %u bytes...\n", frag.len);
            Serial.println("[LORA] Datos:");
            for (size_t i = 0; i < frag.len; i++) {
                Serial.print("0x");
                if (frag.data[i] < 0x10) Serial.print("0");
                Serial.print(frag.data[i], HEX);
                Serial.print(",");
            }
            //LMIC_setTxData2(1, frag.data, frag.len, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // sólo lógica propia, no runloop
    }
}

// Tarea dedicada solo al runloop
void tareaRunLoop(void *pvParameters) {
    while (true) {
        os_runloop_once();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

#define AGGREGATION_WINDOW_MS 300

/**
 * @brief Tarea que recolecta datos de sensores durante una ventana de tiempo
 * y los empaqueta en un solo payload para LoRaWAN.
 */
void DataAggregatorTask(void *pvParameters) {
    std::vector<SensorDataPayload> collectedPayloads;
    SensorDataPayload incomingPayload;
    uint8_t ID_MSG = 0x00;
    while (true) {
        // 1. Esperar indefinidamente por el PRIMER paquete de datos
        if (xQueueReceive(queueSensorDataPayload, &incomingPayload, portMAX_DELAY) == pdTRUE) {
            
            collectedPayloads.clear();
            collectedPayloads.push_back(incomingPayload);
            Serial.printf("[Agregador] Recibido primer payload (Slave %u). Iniciando ventana de %d ms.\n", incomingPayload.slaveId, AGGREGATION_WINDOW_MS);

            // 2. Iniciar la ventana de tiempo para recolectar más paquetes
            uint32_t windowStartTime = millis();
            uint32_t remainingTime = AGGREGATION_WINDOW_MS;

            while (remainingTime > 0) {
                // Esperar por más paquetes, pero solo por el tiempo restante
                if (xQueueReceive(queueSensorDataPayload, &incomingPayload, pdMS_TO_TICKS(remainingTime)) == pdTRUE) {
                    collectedPayloads.push_back(incomingPayload);
                    Serial.printf("[Agregador] Recibido payload adicional (Slave %u) dentro de la ventana.\n", incomingPayload.slaveId);
                }
                
                // Actualizar el tiempo restante
                uint32_t elapsedTime = millis() - windowStartTime;
                if (elapsedTime >= AGGREGATION_WINDOW_MS) {
                    remainingTime = 0;
                } else {
                    remainingTime = AGGREGATION_WINDOW_MS - elapsedTime;
                }
            }

            // 3. La ventana de tiempo ha terminado. Empaquetar y enviar.
            Serial.printf("[Agregador] Ventana cerrada. Empaquetando %u payloads para LoRa.\n", collectedPayloads.size());
            
            // --- NUEVO: Construir el payload unificado ---
            std::vector<uint8_t> unifiedPayload = construirPayloadUnificado(ID_MSG++, collectedPayloads); // 0x01 = ejemplo de ID_MSG

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
        }
    }
}

// ==================== SETUP Y LOOP ====================
void setup() {
    Serial.begin(115200); // 115200 es más estándar y estable que 921600
    while (!Serial); // Espera a que el puerto serie esté listo
    delay(1000);
    Serial.println("Iniciando sistema...");

    // Inicialización SPI
    SPI.begin();

    RTUutils::prepareHardwareSerial(Serial2);
    Serial2.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);

    // Configurar el cliente Modbus
    MB.onDataHandler(&handleData);      // Asignar callback para datos
    MB.onErrorHandler(&handleError);    // Asignar callback para errores
    MB.setTimeout(2000);                // Timeout de 2 segundos por petición
    MB.begin(Serial2);                  // Iniciar cliente (la tarea de fondo se inicia aquí)

    // Crear colas y semáforos
    schedulerMutex = xSemaphoreCreateMutex();
    queueEventos_Peripheral = xQueueCreate(10, sizeof(EventManagerFormat));
    queueEventos_Scheduler = xQueueCreate(10, sizeof(EventManagerFormat));
    queueRespuestas = xQueueCreate(10, sizeof(ResponseFormat));

    // ⬇️ 1. LÍNEA FALTANTE (AHORA CORREGIDA CON EL TAMAÑO FIJO)
    queueSensorDataPayload = xQueueCreate(10, sizeof(SensorDataPayload));

    xTaskCreatePinnedToCore(EventManager, "EventManager", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(DataRequestScheduler, "Scheduler", 4096, NULL, 3, NULL, 0);

    xTaskCreatePinnedToCore(DataFormatter, "DataFormatter", 4096, NULL, 2, NULL, 0);

    //xTaskCreatePinnedToCore(DataPrinterTask, "DataPrinter", 4096, NULL, 2, NULL, 0);

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
    // El loop principal puede quedar vacío o usarse para tareas de baja prioridad.
    // Es mejor que la tarea del loop no se elimine y simplemente ceda el control.
    vTaskDelay(pdMS_TO_TICKS(1000));
}