/**
 * @file main.cpp
 * @brief Modbus RTU Master -> Sparkplug B (WiFi/MQTT).
 * @details Lectura Modbus por bloques, un MeasureFrame por Device, y publicacion
 *          Sparkplug B Device-level con nanopb.
 */

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>

#include <cstdint>
#include <cstring>
#include <ctime>
#include <time.h>
#include <esp_timer.h>

#include "ModbusAPI.h"
#include "ModbusConfig.h"
#include "DevicesConfig.h"
#include "SparkplugConfig.h"
#include "SparkplugB.h"
#include "MeasureFrame.h"
#include "netconfig.h"
#include "Log.h"

// =================================================================================================
// Runtime state
// =================================================================================================

volatile uint32_t g_pollIntervalMs = POLL_INTERVAL_MS;
static TaskHandle_t g_pollTaskHandle = NULL;

QueueHandle_t queueMqtt;

volatile bool g_wifiReady = false;
volatile uint8_t g_wifiReason = 0;

AsyncMqttClient mqttClient;
volatile bool g_mqttReady = false;
volatile bool g_mqttConnecting = false;
volatile bool g_needBirth = false;
volatile bool g_birthDone = false;
volatile bool g_ncmdSubscribed = false;

static uint32_t g_subscribeSentAt = 0;

static const spb::Config kSpbCfg = {
    SPB_GROUP_ID,
    SPB_EDGE_NODE,
    SPB_NTP1,
    SPB_NTP2,
};

// NCMD puede llegar fragmentado; se acumula en el callback (tarea AsyncTCP).
static uint8_t g_ncmdBuf[512];
static size_t  g_ncmdExpected = 0;

static volatile bool g_lastCycleOk = true;

static char g_clientId[40];

struct SensorAcc {
    uint8_t channels;
    uint8_t validMask;
    float   value[MQTT_MAX_CHANNELS];
};

static spb::NodeHealth makeHealth(bool scanOk) {
    spb::NodeHealth h;
    h.uptimeMs      = (uint64_t)(esp_timer_get_time() / 1000);
    h.rssiDbm       = (int32_t)WiFi.RSSI();
    h.freeHeapBytes = (uint32_t)ESP.getFreeHeap();
    h.scanOk        = scanOk;
    return h;
}

// =================================================================================================
// Main polling task - recorre Devices, lee bloques y encola un frame por Device
// =================================================================================================

void mainPollingTask(void *pvParameters) {
    (void)pvParameters;

    uint8_t  failStreak[kMaxDevices] = {};
    uint32_t cycleId = 0;

    while (true) {
        const uint64_t ts = spb::nowMs();
        bool cycleOk = true;

        for (uint8_t d = 0; d < kDeviceCount; ++d) {
            const DeviceConfig& dev = kDevices[d];

            SensorAcc acc[MQTT_MAX_SENSORS] = {};
            bool anyOk = false;
            bool allOk = true;

            for (size_t b = 0; b < dev.blockCount; ++b) {
                const ModbusBlock& blk = dev.blocks[b];

                ModbusApiResult r = modbus_api_read_registers(
                    dev.slaveId, blk.functionCode, blk.startAddr,
                    blk.numRegs, dev.timeoutMs);

                if (r.error_code != ModbusApiError::SUCCESS) {
                    LOG_W("Device %s bloque %zu (0x%04X x%u) fallo: err=%u exc=0x%02X",
                          dev.deviceId, b, blk.startAddr, blk.numRegs,
                          (uint8_t)r.error_code, r.exception_code);
                    allOk = false;
                    continue;
                }

                anyOk = true;

                for (size_t si = 0; si < dev.signalCount; ++si) {
                    const ModbusSignal& s = dev.signals[si];
                    if (s.blockIndex != b || s.channel >= MQTT_MAX_CHANNELS) continue;

                    SensorAcc& a = acc[s.sensorType];
                    a.validMask |= (uint8_t)(1u << s.channel);
                    if ((uint8_t)(s.channel + 1) > a.channels) {
                        a.channels = (uint8_t)(s.channel + 1);
                    }
                    a.value[s.channel] = s.decode(r.data, r.data_len);
                }

                vTaskDelay(pdMS_TO_TICKS(10));   // margen RS485 entre bloques
            }

            if (!allOk) cycleOk = false;
            if (anyOk) {
                failStreak[d] = 0;
            } else if (failStreak[d] < 255) {
                failStreak[d]++;
            }

            MeasureFrame frame{};
            frame.deviceIndex = d;
            frame.cycleId     = cycleId;
            frame.tsMs        = ts;
            frame.scanOk      = allOk;
            frame.failStreak  = failStreak[d];
            frame.cycleEnd    = (d == (uint8_t)(kDeviceCount - 1));
            frame.cycleOk     = frame.cycleEnd ? cycleOk : false;

            for (uint8_t st = 0; st < MQTT_MAX_SENSORS; ++st) {
                if (acc[st].channels == 0) continue;
                if (frame.count >= MQTT_MAX_SENSORS) break;

                SensorValues& sv = frame.sensors[frame.count++];
                sv.sensorId  = st;
                sv.channels  = acc[st].channels;
                sv.validMask = acc[st].validMask;
                for (uint8_t c = 0; c < MQTT_MAX_CHANNELS; ++c) {
                    sv.value[c] = (c < acc[st].channels) ? acc[st].value[c] : 0.0f;
                }
            }

            if (xQueueSend(queueMqtt, &frame, pdMS_TO_TICKS(100)) != pdTRUE) {
                LOG_W("Cola MQTT llena; frame de %s descartado", dev.deviceId);
            }
        }

        ++cycleId;
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(g_pollIntervalMs));
    }
}

// =================================================================================================
// Network task - WiFi + MQTT + Sparkplug lifecycle
// =================================================================================================

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

static void mqtt_onConnect(bool sessionPresent) {
    (void)sessionPresent;
    g_mqttConnecting = false;
    g_mqttReady      = true;
    g_ncmdSubscribed = false;
    g_needBirth      = true;
    g_birthDone      = false;
}

static void mqtt_onDisconnect(AsyncMqttClientDisconnectReason reason) {
    (void)reason;
    g_mqttConnecting = false;
    g_mqttReady      = false;
    g_needBirth      = false;
    g_birthDone      = false;
    g_ncmdSubscribed = false;
}

static void mqtt_onSubscribe(uint16_t packetId, uint8_t qos) {
    (void)packetId;
    (void)qos;
    g_ncmdSubscribed = true;
}

static void mqtt_onMessage(char* topic, char* payload,
                           AsyncMqttClientMessageProperties properties,
                           size_t len, size_t index, size_t total) {
    (void)properties;

    if (strcmp(topic, spb::ncmdTopic()) != 0) return;

    if (index == 0) g_ncmdExpected = total;
    if (total > sizeof(g_ncmdBuf) || g_ncmdExpected != total) {
        g_ncmdExpected = 0;
        return;
    }
    if (index + len > sizeof(g_ncmdBuf)) return;

    memcpy(g_ncmdBuf + index, payload, len);

    if (index + len == total) {
        spb::handleCommand(g_ncmdBuf, total);
        g_ncmdExpected = 0;
    }
}

static void mqtt_setup() {
    mqttClient.setClientId(g_clientId);
    mqttClient.setKeepAlive(15);
    mqttClient.setCleanSession(true);

    if (MQTT_USER[0] != '\0') {
        mqttClient.setCredentials(MQTT_USER, MQTT_PASS);
    }

    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.onConnect(mqtt_onConnect);
    mqttClient.onDisconnect(mqtt_onDisconnect);
    mqttClient.onSubscribe(mqtt_onSubscribe);
    mqttClient.onMessage(mqtt_onMessage);
}

static bool publishBirthIfReady(const MeasureFrame* lastFrames,
                                const bool* hasLast) {
    if (!g_mqttReady || !g_needBirth || g_birthDone) return false;

    // Suscribir NCMD antes del NBIRTH. No publicar birth hasta tener SUBACK.
    if (!g_ncmdSubscribed) {
        if (g_subscribeSentAt == 0 || (millis() - g_subscribeSentAt) >= 2000) {
            g_subscribeSentAt = millis();
            mqttClient.subscribe(spb::ncmdTopic(), 1);
            LOG_I("Sparkplug: suscribiendo NCMD...");
        }
        return false;
    }

    // Sparkplug exige timestamps UTC en todos los birth/data. No publicar sin NTP.
    if (!spb::timeReady()) {
        static uint32_t lastLog = 0;
        if (lastLog == 0 || (millis() - lastLog) > 10000) {
            lastLog = millis();
            LOG_W("Sparkplug: esperando NTP para publicar birth...");
        }
        return false;
    }

    spb::NodeHealth health = makeHealth(g_lastCycleOk);

    if (spb::publishBirth(mqttClient, health, lastFrames, hasLast)) {
        g_birthDone = true;
        g_needBirth = false;
        LOG_I("Sparkplug: NBIRTH + DBIRTH publicados");
        return true;
    }

    LOG_W("Sparkplug: fallo al publicar birth; se reintenta");
    return false;
}

void tareaRed(void *pvParameters) {
    (void)pvParameters;

    MeasureFrame lastFrames[kMaxDevices] = {};
    bool         hasLast[kMaxDevices]    = {};
    bool         deviceOnline[kMaxDevices] = {};

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
                  WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
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
            g_subscribeSentAt = 0;
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
                spb::prepareDeath();
                mqttClient.setWill(spb::deathTopic(), 1, false,
                                   reinterpret_cast<const char*>(spb::deathPayload()),
                                   spb::deathPayloadLen());
                mqttClient.connect();

                mqttDiscSince = millis();
                mqttBackoff = (mqttBackoff * 2 > 30000) ? 30000 : mqttBackoff * 2;
            }
        } else if (mqtt) {
            mqttDiscSince = 0;
            mqttBackoff = 2000;
        }

        // --- Birth / Rebirth ---
        if (!mqtt) {
            g_birthDone = false;
            g_needBirth = false;
            g_ncmdSubscribed = false;
        } else {
            if (g_birthDone && spb::rebirthPending()) {
                spb::clearRebirth();
                g_birthDone = false;
                g_needBirth = true;
                LOG_I("Sparkplug: rebirth solicitado");
            }

            if (publishBirthIfReady(lastFrames, hasLast)) {
                for (uint8_t d = 0; d < kDeviceCount; ++d) deviceOnline[d] = true;
            }
        }

        // --- Drenar cola ---
        MeasureFrame frame;
        if (xQueueReceive(queueMqtt, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (frame.deviceIndex < kMaxDevices) {
                lastFrames[frame.deviceIndex] = frame;
                hasLast[frame.deviceIndex] = true;
            }

            if (frame.cycleEnd) g_lastCycleOk = frame.cycleOk;

            if (mqtt && g_birthDone && frame.deviceIndex < kDeviceCount) {
                const uint8_t d = frame.deviceIndex;
                const bool recovered = (frame.failStreak == 0);

                if (deviceOnline[d] && frame.failStreak >= SPB_DEVICE_DEATH_CYCLES) {
                    spb::publishDeviceDeath(mqttClient, d, frame.tsMs);
                    deviceOnline[d] = false;
                    LOG_W("Sparkplug: %s DDEATH (sin respuesta)", kDevices[d].deviceId);
                } else if (!deviceOnline[d] && recovered) {
                    if (spb::publishDeviceBirth(mqttClient, d, &frame)) {
                        deviceOnline[d] = true;
                        LOG_I("Sparkplug: %s DBIRTH (recuperado)", kDevices[d].deviceId);
                    }
                }

                if (deviceOnline[d]) {
                    const bool scanOk = frame.cycleEnd ? frame.cycleOk : true;
                    spb::publishData(mqttClient, frame, makeHealth(scanOk));
                } else if (frame.cycleEnd) {
                    // El ultimo Device del ciclo esta offline; aun asi publicar NDATA.
                    spb::publishNodeData(mqttClient, makeHealth(frame.cycleOk), frame.tsMs);
                }
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
    Serial.println("Iniciando modbus_master_mqtt (Sparkplug B)...");

    SPI.begin();

    modbus_api_init(Serial2, kBusCfg.rxPin, kBusCfg.txPin,
                    kBusCfg.baudRate, kBusCfg.uartConfig,
                    kBusCfg.defaultTimeoutMs);

    queueMqtt = xQueueCreate(2 * kMaxDevices, sizeof(MeasureFrame));

    spb::begin(kSpbCfg);

    snprintf(g_clientId, sizeof(g_clientId), "%s-%06llx", MQTT_CLIENT,
             (unsigned long long)(ESP.getEfuseMac() & 0xFFFFFFULL));
    LOG_I("MQTT: client id '%s'", g_clientId);

    WiFi.mode(WIFI_STA);
    WiFi.setHostname(g_clientId);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.onEvent(wifi_event, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(wifi_event, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    LOG_I("WiFi: conectando a '%s'...", WIFI_SSID);

    mqtt_setup();
    LOG_I("MQTT: broker %s:%u, grupo Sparkplug '%s', nodo '%s'",
          MQTT_HOST, (unsigned)MQTT_PORT, SPB_GROUP_ID, SPB_EDGE_NODE);

    xTaskCreatePinnedToCore(mainPollingTask, "MainPoll", 8192, NULL, 3, &g_pollTaskHandle, 0);
    xTaskCreatePinnedToCore(tareaRed, "NetTask", 12288, NULL, 5, NULL, 1);

    Serial.printf("Configurado: bus %lu baud, %u devices, intervalo %lu ms\n",
                  kBusCfg.baudRate, (unsigned)kDeviceCount, (unsigned long)g_pollIntervalMs);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
