/**
 * @file main.cpp
 * @brief Modbus RTU Master -> MQTT (WiFi) - variante sin LoRaWAN.
 * @details Lectura Modbus por BLOQUES (varias senales en una sola peticion) +
 *          API sincrona delgada. La tarea de polling decodifica valores y los
 *          entrega a la tarea MQTT por cola. MQTT (WiFi + JSON) en pasos siguientes.
 */

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <cstdint>
#include <ctime>
#include "ModbusAPI.h"
#include "ModbusConfig.h"
#include "netconfig.h"
#include "Log.h"

// =================================================================================================
// Data structures
// =================================================================================================

#define MQTT_MAX_SENSORS   8
#define MQTT_MAX_CHANNELS  4

struct SensorValues {
    uint8_t sensorId;
    uint8_t channels;
    float   value[MQTT_MAX_CHANNELS];
};

struct MeasureFrame {
    uint8_t      id;
    uint32_t     ts;
    uint8_t      count;
    SensorValues sensors[MQTT_MAX_SENSORS];
};

// Acumulador por tipo de sensor durante un ciclo de polling.
struct SensorAcc {
    bool    present;
    bool    failed;
    uint8_t channels;
    float   value[MQTT_MAX_CHANNELS];
};

// =================================================================================================
// Runtime state
// =================================================================================================

volatile uint32_t g_pollIntervalMs = POLL_INTERVAL_MS;
static TaskHandle_t g_pollTaskHandle = NULL;

QueueHandle_t queueMqtt;

// Estado de red (lo usara MQTT en el Paso 3).
volatile bool g_wifiReady = false;
volatile uint8_t g_wifiReason = 0;

// =================================================================================================
// Main polling task - lee bloques, decodifica senales, encola un MeasureFrame
// =================================================================================================

void mainPollingTask(void *pvParameters) {
    uint8_t msgId = 0;

    while (true) {
        SensorAcc acc[8];
        for (auto& a : acc) { a = SensorAcc{}; }

        LOG_I("--- Ciclo de consulta (msgId=%u, %zu bloques) ---", msgId, kBlockCount);

        for (size_t bi = 0; bi < kBlockCount; ++bi) {
            const ModbusBlock& blk = kBlocks[bi];
            uint32_t timeout = lookupTimeout(blk.slaveID);

            ModbusApiResult r = modbus_api_read_registers(
                blk.slaveID, blk.functionCode, blk.startAddr, blk.numRegs, timeout);

            if (r.error_code != ModbusApiError::SUCCESS) {
                LOG_W("Bloque %zu (0x%04X x%u) fallo: err=%u exc=0x%02X",
                      bi, blk.startAddr, blk.numRegs,
                      (uint8_t)r.error_code, r.exception_code);
                for (size_t si = 0; si < kSignalCount; ++si) {
                    if (kSignals[si].blockIndex == bi) {
                        acc[kSignals[si].sensorType].failed = true;
                    }
                }
                continue;
            }

            LOG_D("Bloque %zu (0x%04X x%u) OK: %zu bytes",
                  bi, blk.startAddr, blk.numRegs, r.data_len);

            for (size_t si = 0; si < kSignalCount; ++si) {
                const ModbusSignal& s = kSignals[si];
                if (s.blockIndex != bi) continue;

                SensorAcc& a = acc[s.sensorType];
                a.present = true;
                if ((uint8_t)(s.channel + 1) > a.channels) a.channels = (uint8_t)(s.channel + 1);

                a.value[s.channel] = s.decode(r.data, r.data_len);
            }

            vTaskDelay(pdMS_TO_TICKS(10));   // margen RS485 entre bloques
        }

        MeasureFrame frame{};
        frame.id    = msgId;
        frame.ts    = (uint32_t)time(nullptr);
        frame.count = 0;

        for (uint8_t st = 0; st < 8; ++st) {
            if (acc[st].failed) {
                LOG_W("SensorType %u: descartado (fallo de bloque)", st);
                continue;
            }
            if (!acc[st].present) continue;
            if (frame.count >= MQTT_MAX_SENSORS) break;

            SensorValues& sv = frame.sensors[frame.count++];
            sv.sensorId = st;
            sv.channels = acc[st].channels;
            for (uint8_t c = 0; c < MQTT_MAX_CHANNELS; ++c) {
                sv.value[c] = (c < acc[st].channels) ? acc[st].value[c] : 0.0f;
            }
        }

        if (frame.count > 0) {
            LOG_I("Encolando frame MQTT (id=%u, %u sensores)", frame.id, frame.count);
            if (xQueueSend(queueMqtt, &frame, pdMS_TO_TICKS(100)) != pdTRUE) {
                LOG_W("Cola MQTT llena; frame descartado.");
            }
        }

        ++msgId;

        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(g_pollIntervalMs));
    }
}

// =================================================================================================
// Network task - WiFi (Paso 2). AsyncMqttClient + JSON se agregan en los pasos siguientes.
// =================================================================================================

// Handler de eventos: solo banderas (NO loguear aqui: corre en el task de WiFi).
static void wifi_event(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            g_wifiReady = true;
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            g_wifiReady = false;
            g_wifiReason = info.wifi_sta_disconnected.reason;
            break;
        default:
            break;
    }
}

void tareaRed(void *pvParameters) {
    MeasureFrame frame;
    bool     prevReady = false;
    uint32_t discSince = 0;

    while (true) {
        const bool ready = (WiFi.status() == WL_CONNECTED);

        if (ready && !prevReady) {
            LOG_I("WiFi: IP %s (RSSI %d dBm)",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
        } else if (!ready && prevReady) {
            LOG_W("WiFi: desconectado, reason=%u", (unsigned)g_wifiReason);
        }
        prevReady = ready;

        if (ready) {
            discSince = 0;
        } else {
            if (discSince == 0) {
                discSince = millis();
            } else if (millis() - discSince > 10000) {
                LOG_W("WiFi: 10 s sin conexion, re-lanzando begin()...");
                WiFi.begin(WIFI_SSID, WIFI_PASS);
                discSince = millis();
            }
        }

        if (xQueueReceive(queueMqtt, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            LOG_I("Net[stub]: frame id=%u ts=%lu sensores=%u (MQTT pendiente)",
                  frame.id, (unsigned long)frame.ts, frame.count);
            for (uint8_t i = 0; i < frame.count; ++i) {
                const SensorValues& sv = frame.sensors[i];
                LOG_D("  sensor=%u ch=%u v0=%.2f v1=%.2f v2=%.2f v3=%.2f",
                      sv.sensorId, sv.channels,
                      sv.value[0], sv.value[1], sv.value[2], sv.value[3]);
            }
        }
    }
}

// =================================================================================================
// Setup / Loop
// =================================================================================================

void setup() {
    Serial.begin(115200);
    {
        const uint32_t t0 = millis();
        while (!Serial && (millis() - t0) < 3000) { delay(10); }
    }
    Serial.println("Iniciando modbus_master_mqtt (variante MQTT)...");

    SPI.begin();

    modbus_api_init(Serial2, kBusCfg.rxPin, kBusCfg.txPin,
                    kBusCfg.baudRate, kBusCfg.uartConfig,
                    kBusCfg.defaultTimeoutMs);

    queueMqtt = xQueueCreate(3, sizeof(MeasureFrame));

    // WiFi (Paso 2)
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(MQTT_CLIENT);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.onEvent(wifi_event, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(wifi_event, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    LOG_I("WiFi: conectando a '%s'...", WIFI_SSID);

    xTaskCreatePinnedToCore(mainPollingTask, "MainPoll", 8192, NULL, 3, &g_pollTaskHandle, 0);
    xTaskCreatePinnedToCore(tareaRed, "NetTask", 8192, NULL, 5, NULL, 1);

    Serial.printf("Configurado: bus a %lu baud, %zu bloques, intervalo %lu ms\n",
                  kBusCfg.baudRate, kBlockCount, (unsigned long)g_pollIntervalMs);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
