/**
 * @file main.cpp
 * @brief Modbus RTU Master -> MQTT (WiFi) - variante sin LoRaWAN.
 * @details Lectura Modbus por BLOQUES (varias senales en una sola peticion) +
 *          API sincrona delgada. La tarea de polling decodifica valores y los
 *          entrega a la tarea MQTT por cola. MQTT (WiFi + JSON) en pasos siguientes.
 */

#include <Arduino.h>
#include <SPI.h>
#include <cstdint>
#include <ctime>
#include "ModbusAPI.h"
#include "ModbusConfig.h"
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
// MQTT task - stub. WiFi + AsyncMqttClient + JSON se implementan en los pasos siguientes.
// =================================================================================================

void tareaMqtt(void *pvParameters) {
    MeasureFrame frame;

    while (true) {
        if (xQueueReceive(queueMqtt, &frame, portMAX_DELAY) == pdTRUE) {
            LOG_I("MQTT[stub]: frame id=%u ts=%lu sensores=%u (WiFi/MQTT pendiente)",
                  frame.id, (unsigned long)frame.ts, frame.count);
            for (uint8_t i = 0; i < frame.count; ++i) {
                const SensorValues& sv = frame.sensors[i];
                LOG_D("  sensor=%u ch=%u v0=%.2f v1=%.2f v2=%.2f",
                      sv.sensorId, sv.channels, sv.value[0], sv.value[1], sv.value[2]);
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

    xTaskCreatePinnedToCore(mainPollingTask, "MainPoll", 8192, NULL, 3, &g_pollTaskHandle, 0);
    xTaskCreatePinnedToCore(tareaMqtt, "MqttTask", 8192, NULL, 5, NULL, 1);

    Serial.printf("Configurado: bus a %lu baud, %zu bloques, intervalo %lu ms\n",
                  kBusCfg.baudRate, kBlockCount, (unsigned long)g_pollIntervalMs);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
