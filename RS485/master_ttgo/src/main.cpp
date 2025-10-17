#include <Arduino.h>
#include <SPI.h>
#include <hal/hal.h>
#include <lmic.h>

#include <vector>
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
    uint8_t compressedBytes;       // Solo si dataType=3
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
    DISCOVERY_GET_DATA_OFFSET = 255
};

// Gestor de solicitudes Modbus RTU
struct ModbusRequestInfo {
    uint32_t token;           // Token de la solicitud
    uint8_t slaveId;          // Esclavo al que fue enviada
    uint8_t functionCode;     // Código de función Modbus
};

constexpr size_t MAX_REQUESTS = 16;
ModbusRequestInfo requestBuffer[MAX_REQUESTS];
size_t requestHead = 0; // Próxima posición para guardar

void addRequest(uint32_t token, uint8_t slaveId, uint8_t functionCode) {
    requestBuffer[requestHead] = ModbusRequestInfo{token, slaveId, functionCode};
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

// NUEVA FUNCIÓN: Realiza el proceso de descubrimiento para un solo dispositivo.
// Esta función se ejecuta en el contexto de la tarea que la llama.
bool discoverDeviceSensors(uint8_t deviceId) {
    Serial.printf("Iniciando descubrimiento para dispositivo %u...\n", deviceId);
    const TickType_t responseTimeout = pdMS_TO_TICKS(200);

    // --- FASE 1: Preguntar cuántos registros de configuración hay ---
    // La orden 'DISCOVERY_GET_COUNT' es para solicitar el número de registros.
    EventManagerFormat event_p1 = {deviceId, 0, DISCOVERY_GET_COUNT}; // sensorID=0 es un placeholder
    if (xQueueSend(queueEventos_Peripheral, &event_p1, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("Error: No se pudo encolar el evento de descubrimiento (Fase 1).");
        return false;
    }

    ResponseFormat response_p1;
    if (xQueueReceive(queueRespuestas, &response_p1, responseTimeout) != pdTRUE) {
        Serial.printf("Timeout (Fase 1) esperando respuesta de %u\n", deviceId);
        return false;
    }

    // Validar que la respuesta es para nosotros
    if (response_p1.deviceId != deviceId) {
        Serial.printf("Respuesta de Fase 1 era para otro dispositivo (%u)\n", response_p1.deviceId);
        // NOTA: Aquí podrías devolver el mensaje a la cola si es para otra tarea.
        return false;
    }

    uint8_t numRegistersToRead = 0;
    memcpy(&numRegistersToRead, response_p1.data, sizeof(uint8_t));
    Serial.printf("Dispositivo %u informa que tiene %u registros de configuración.\n", deviceId, numRegistersToRead);

    if (numRegistersToRead == 0) {
        return true; // Éxito, no hay sensores que registrar.
    }

    // --- FASE 2: Pedir los registros de configuración ---
    uint16_t instruccion = DISCOVERY_GET_DATA_OFFSET + numRegistersToRead;
    EventManagerFormat event_p2 = {deviceId, DISCOVERY_GET_DATA_OFFSET, instruccion};
    if (xQueueSend(queueEventos_Peripheral, &event_p2, pdMS_TO_TICKS(10)) != pdTRUE) {
        Serial.println("Error: No se pudo encolar el evento de descubrimiento (Fase 2).");
        return false;
    }

    ResponseFormat response_p2;
    if (xQueueReceive(queueRespuestas, &response_p2, responseTimeout) != pdTRUE) {
        Serial.printf("Timeout (Fase 2) esperando respuesta de %u\n", deviceId);
        return false;
    }

    // TODO: Aquí procesarías response_p2.data para llenar tu estructura slaveList.
    Serial.printf("Recibidos %u bytes de configuración del dispositivo %u. Procesamiento pendiente.\n", response_p2.length, deviceId);
    // parseAndStoreSensorConfig(response_p2.data, response_p2.length);

    return true;
}

// TAREA TEMPORAL para el descubrimiento inicial
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
    //initScheduler(); // Ahora que tenemos la lista de sensores, inicializamos el scheduler.

    Serial.println("--- Tarea de Descubrimiento Inicial: Autodestruyendo. ---");
    vTaskDelete(NULL); // La tarea se elimina a sí misma.
}


void initScheduler() {
    // Tomar el control exclusivo de las listas
    if (xSemaphoreTake(schedulerMutex, portMAX_DELAY) == pdTRUE) {
        scheduleList.clear();
        for (const auto& slave : slaveList) {
            for (const auto& sensor : slave.sensors) {
                scheduleList.push_back({
                    slave.slaveID,
                    sensor.sensorID,
                    sensor.samplingInterval,
                    millis() // Primer muestreo inmediato
                });
            }
        }
        // Devolver el control
        xSemaphoreGive(schedulerMutex);
    }
}

// Tarea del scheduler
void DataRequestScheduler(void *pvParameters) {
    // La inicialización se hace desde la tarea de descubrimiento ahora
    // initScheduler(); 

    while (true) {
        uint32_t now = millis();
        TickType_t sleepTime = portMAX_DELAY; // Por defecto, dormir para siempre si no hay nada que hacer

        // Tomar el control para leer la lista de planificación
        if (xSemaphoreTake(schedulerMutex, portMAX_DELAY) == pdTRUE) {
            
            if (!scheduleList.empty()) {
                uint32_t nextEventTime = UINT32_MAX;

                for (auto& item : scheduleList) {
                    if (now >= item.nextSampleTime) {
                        // Es hora de enviar el evento
                        EventManagerFormat event = {item.slaveID, item.sensorID, 1};
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

void EventManager(void *pvParameters) {
    uint32_t requestToken = 0;
    while (true) {
        EventManagerFormat event;
        bool requestSent = false;
        uint8_t functionCode = 0;
        Error err = Error::SUCCESS;

        // Prioridad: primero scheduler, luego peripheral
        if (xQueueReceive(queueEventos_Scheduler, &event, 0) == pdTRUE) {
            uint16_t startAddr, numRegs;
            if (getSensorParams(event.slaveId, event.sensorID, startAddr, numRegs)) {
                functionCode = READ_HOLD_REGISTER;
                err = MB.addRequest(++requestToken, event.slaveId, functionCode, startAddr, numRegs);
                requestSent = true;
            } else {
                Serial.printf("Parámetros no encontrados para SlaveID %u, SensorID %u\n", event.slaveId, event.sensorID);
            }
        } else if (xQueueReceive(queueEventos_Peripheral, &event, 0) == pdTRUE) {
            if (event.order == DISCOVERY_GET_COUNT) {
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
            addRequest(requestToken, event.slaveId, functionCode);
        } else if (requestSent) {
            Serial.printf("Error al encolar solicitud Modbus para token %u\n", requestToken);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeño delay para evitar saturar la CPU
    }
}

void DataFormatter (void *pvParameters) {
    while (true) {


    }
}

// ==================== LORA CONFIG ====================
// Deshabilitar ventana RX de recepción (si no se esperan downlinks)
#define DISABLE_INVERT_IQ_ON_RX 1
#define DISABLE_RX 1
#define CFG_sx1272_radio 1

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
            xSemaphoreTake(semaforoEnvioCompleto, portMAX_DELAY);
            LMIC_setTxData2(1, frag.data, frag.len, 0);
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

    schedulerMutex = xSemaphoreCreateMutex();

    queueEventos_Peripheral = xQueueCreate(10, sizeof(EventManagerFormat));
    queueEventos_Scheduler = xQueueCreate(10, sizeof(EventManagerFormat));
    queueRespuestas = xQueueCreate(10, sizeof(ResponseFormat));

    xTaskCreatePinnedToCore(EventManager, "EventManager", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(DataRequestScheduler, "Scheduler", 4096, NULL, 3, NULL, 0);
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

    // Ejemplo: Enviar un paquete de prueba después de 5 segundos
    delay(5000);
    Fragmento testFrag;
    const char* mensaje = "Hola LoRa , 128qedsdsadasdasdsadsadsadsadsadsafsdfsdffs!";
    testFrag.len = strlen(mensaje);
    memcpy(testFrag.data, mensaje, testFrag.len);
    if (xQueueSend(queueFragmentos, &testFrag, pdMS_TO_TICKS(100)) == pdPASS) {
        Serial.println("Fragmento de prueba enviado a la cola.");
    }
}

void loop() {
    // El loop principal puede quedar vacío o usarse para tareas de baja prioridad.
    // Es mejor que la tarea del loop no se elimine y simplemente ceda el control.
    vTaskDelay(pdMS_TO_TICKS(1000));
}