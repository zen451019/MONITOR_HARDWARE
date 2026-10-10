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
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
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

// Estado MQTT (Paso 3).
AsyncMqttClient mqttClient;
volatile bool g_mqttReady = false;
volatile bool g_mqttConnecting = false;

// Buffers persistentes (setWill/setServer guardan el puntero, no copian).
static char g_statusTopic[128];
static char g_dataTopic[128];

// Nombres de sensor (indice = sensorId), espejo de codec.js.
static const char* const SENSOR_NAMES[8] = {
    "energy", "voltage", "current", "real_power",
    "apparent_power", "reactive_power", "power_factor", "frequency"
};

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

// Handler de eventos WiFi: solo banderas (NO loguear aqui: corre en el task de WiFi).
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

// Callbacks MQTT: solo banderas + birth (NO loguear aqui: task AsyncTCP).
static void mqtt_onConnect(bool sessionPresent) {
    (void)sessionPresent;
    g_mqttConnecting = false;
    g_mqttReady = true;
    mqttClient.publish(g_statusTopic, 0, true, "online");   // birth (retained)
}

static void mqtt_onDisconnect(AsyncMqttClientDisconnectReason reason) {
    (void)reason;
    g_mqttConnecting = false;
    g_mqttReady = false;
}

static void mqtt_setup() {
    // Topic estilo ChirpStack: application/<app>/device/<dev>/event/<event>
    snprintf(g_statusTopic, sizeof(g_statusTopic),
             "application/%s/device/%s/event/status", MQTT_APP, MQTT_DEVICE);
    snprintf(g_dataTopic, sizeof(g_dataTopic),
             "application/%s/device/%s/event/up", MQTT_APP, MQTT_DEVICE);

    mqttClient.setClientId(MQTT_CLIENT);
    mqttClient.setKeepAlive(15);
    mqttClient.setCleanSession(true);
    if (MQTT_USER[0] != '\0') {
        mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
    }
    mqttClient.setWill(g_statusTopic, 0, true, "offline");   // LWT (retained)
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.onConnect(mqtt_onConnect);
    mqttClient.onDisconnect(mqtt_onDisconnect);
}

// Construye el JSON del frame y lo publica en <base>/data (QoS 0, no retained).
static void publish_frame(const MeasureFrame& frame) {
    JsonDocument doc;
    doc["id"] = frame.id;
    doc["ts"] = frame.ts;

    JsonObject meas = doc["measurements"].to<JsonObject>();
    for (uint8_t i = 0; i < frame.count; ++i) {
        const SensorValues& sv = frame.sensors[i];
        const char* name = (sv.sensorId < 8) ? SENSOR_NAMES[sv.sensorId] : "unknown";

        JsonObject chans = meas[name].to<JsonObject>();
        for (uint8_t c = 0; c < sv.channels; ++c) {
            char key[4];
            snprintf(key, sizeof(key), "ch%u", (unsigned)(c + 1));
            chans[key] = sv.value[c];
        }
    }

    char buf[768];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    if (n > 0 && n < sizeof(buf)) {
        mqttClient.publish(g_dataTopic, 0, false, buf, n);
        LOG_I("MQTT: data publicado (%zu bytes)", n);
    } else {
        LOG_W("MQTT: JSON no cabe (%zu bytes)", n);
    }
}

void tareaRed(void *pvParameters) {
    MeasureFrame frame;
    bool     prevWifi = false;
    bool     prevMqtt = false;
    uint32_t wifiDiscSince = 0;
    uint32_t mqttDiscSince = 0;
    uint32_t mqttBackoff   = 2000;

    while (true) {
        const bool wifi = (WiFi.status() == WL_CONNECTED);
        const bool mqtt = g_mqttReady;

        // --- WiFi: transiciones + reconexion suave ---
        if (wifi && !prevWifi) {
            LOG_I("WiFi: IP %s (RSSI %d dBm)",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
        } else if (!wifi && prevWifi) {
            LOG_W("WiFi: desconectado, reason=%u", (unsigned)g_wifiReason);
        }
        prevWifi = wifi;

        if (wifi) {
            wifiDiscSince = 0;
        } else {
            mqttDiscSince = 0;
            mqttBackoff = 2000;
            if (wifiDiscSince == 0) {
                wifiDiscSince = millis();
            } else if (millis() - wifiDiscSince > 10000) {
                LOG_W("WiFi: 10 s sin conexion, re-lanzando begin()...");
                WiFi.begin(WIFI_SSID, WIFI_PASS);
                wifiDiscSince = millis();
            }
        }

        // --- MQTT: transiciones + reconexion con backoff ---
        if (mqtt && !prevMqtt) {
            LOG_I("MQTT: conectado a %s:%u", MQTT_HOST, (unsigned)MQTT_PORT);
        } else if (!mqtt && prevMqtt) {
            LOG_W("MQTT: desconectado, se reintentara.");
        }
        prevMqtt = mqtt;

        if (wifi && !mqtt && !g_mqttConnecting) {
            if (mqttDiscSince == 0) {
                mqttDiscSince = millis();
            } else if (millis() - mqttDiscSince >= mqttBackoff) {
                LOG_I("MQTT: conectando a %s:%u...", MQTT_HOST, (unsigned)MQTT_PORT);
                g_mqttConnecting = true;
                mqttClient.connect();
                mqttDiscSince = millis();
                mqttBackoff = (mqttBackoff * 2 > 30000) ? 30000 : mqttBackoff * 2;
            }
        } else if (mqtt) {
            mqttDiscSince = 0;
            mqttBackoff = 2000;
        }

        // --- Drenar la cola de datos: publicar JSON si hay MQTT ---
        if (xQueueReceive(queueMqtt, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (mqtt) {
                publish_frame(frame);
            } else {
                LOG_D("Net: frame id=%u descartado (MQTT no conectado)", frame.id);
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

    // MQTT (Paso 3)
    mqtt_setup();
    LOG_I("MQTT: broker %s:%u, up topic '%s'", MQTT_HOST, (unsigned)MQTT_PORT, g_dataTopic);

    xTaskCreatePinnedToCore(mainPollingTask, "MainPoll", 8192, NULL, 3, &g_pollTaskHandle, 0);
    xTaskCreatePinnedToCore(tareaRed, "NetTask", 8192, NULL, 5, NULL, 1);

    Serial.printf("Configurado: bus a %lu baud, %zu bloques, intervalo %lu ms\n",
                  kBusCfg.baudRate, kBlockCount, (unsigned long)g_pollIntervalMs);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
