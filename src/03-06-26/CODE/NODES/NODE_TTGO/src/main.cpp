/**
 * @file main.cpp
 * @brief Phantom Master Node — floods LoRaWAN with synthetic sensor data.
 * @details Generates fake payloads identical in wire format to the real Modbus master.
 *          No RS485 bus, no Modbus — pure LoRaWAN stress-test tool.
 *          Configure sensors, modes and values in PhantomConfig.h.
 * @date 2026-06-13
 */

#include <Arduino.h>
#include <SPI.h>
#include <hal/hal.h>
#include <lmic.h>
#include <vector>
#include <cstdint>
#include <map>
#include <set>
#include <ctime>
#include <cstring>
#include <cmath>
#ifdef TBEAM_V1
#include <Wire.h>
#endif
#include "loraconfig.h"
#include "SensorRegistry.h"
#include "PhantomConfig.h"
#include "Log.h"

// =================================================================================================
// Data Structures (identical to real master)
// =================================================================================================

#define MAX_SENSOR_PAYLOAD 128

struct SensorDataPayload {
    uint8_t slaveId;
    uint8_t sensorId;
    uint8_t data[MAX_SENSOR_PAYLOAD];
    size_t  dataSize;
    uint8_t regsPerChannel;
};

// =================================================================================================
// Forward Declarations
// =================================================================================================
std::vector<uint8_t> construirPayloadUnificado(uint8_t id_mensaje,
    const std::vector<SensorDataPayload>& collectedPayloads);

// =================================================================================================
// LoRa Globals
// =================================================================================================

QueueHandle_t    queueFragmentos;
SemaphoreHandle_t semaforoEnvioCompleto;

#define LORA_PAYLOAD_MAX 220

struct Fragmento {
    uint8_t data[LORA_PAYLOAD_MAX];
    size_t  len;
};

// =================================================================================================
// Helper: pack float to IEEE 754 big-endian bytes
// =================================================================================================

static inline uint32_t floatToRawU32(float f) {
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

// =================================================================================================
// Helper: generate bytes for a single phantom sensor
// =================================================================================================

static void generatePhantomData(const PhantomSensorDef& sensor, uint32_t t_ms,
                                 std::vector<uint8_t>& bytes) {
    uint32_t raw = 0;

    if (sensor.mode == PHANTOM_FIXED) {
        raw = sensor.fixedRaw;
    } else {
        float angle = 6.283185307f * (float)(t_ms % sensor.sinePeriodMs) / (float)sensor.sinePeriodMs;
        float value = sensor.sineCenter + sensor.sineAmplitude * sinf(angle);

        if (sensor.numRegs == 1) {
            uint16_t v = (uint16_t)(value < 0 ? 0 : (value > 65535 ? 65535 : value));
            raw = v;  // upper 16 bits zero → only lower 16 used for 1-reg
        } else {
            raw = floatToRawU32(value);
        }
    }

    // Pack big-endian
    if (sensor.numRegs == 2) {
        bytes.push_back((uint8_t)((raw >> 24) & 0xFF));
        bytes.push_back((uint8_t)((raw >> 16) & 0xFF));
        bytes.push_back((uint8_t)((raw >> 8)  & 0xFF));
        bytes.push_back((uint8_t)(raw & 0xFF));
    } else {
        bytes.push_back((uint8_t)((raw >> 8) & 0xFF));
        bytes.push_back((uint8_t)(raw & 0xFF));
    }
}

// =================================================================================================
// Payload Builder (IDENTICAL to real master — do not modify wire format)
// =================================================================================================

std::vector<uint8_t> construirPayloadUnificado(
    uint8_t id_mensaje,
    const std::vector<SensorDataPayload>& collectedPayloads)
{
    std::vector<uint8_t> payload;

    // 1. Header
    payload.push_back(id_mensaje);

    // 2. Timestamp (4 bytes, big-endian UNIX)
    uint32_t ts_s = static_cast<uint32_t>(time(nullptr));
    payload.push_back((ts_s >> 24) & 0xFF);
    payload.push_back((ts_s >> 16) & 0xFF);
    payload.push_back((ts_s >> 8)  & 0xFF);
    payload.push_back(ts_s & 0xFF);

    // Map: sensorType -> data (last one wins per type, but we send one per type)
    std::map<uint8_t, const SensorDataPayload*> activeSensors;
    for (const auto& sd : collectedPayloads) {
        activeSensors[sd.sensorId] = &sd;
    }

    // 3. Activate Byte
    uint8_t activate_byte = 0;
    if (activeSensors.count(SENSOR_ID_BATERIA))   activate_byte |= (1 << 0);
    if (activeSensors.count(SENSOR_ID_VOLTAJE))   activate_byte |= (1 << 1);
    if (activeSensors.count(SENSOR_ID_CORRIENTE)) activate_byte |= (1 << 2);
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        uint8_t sid = SENSOR_ID_EXT_START + i;
        if (activeSensors.count(sid)) activate_byte |= (1 << (i + 3));
    }
    payload.push_back(activate_byte);

    // 4. Len Bytes (one per active bit, in LSB→MSB order)
    if (activate_byte & (1 << 0)) {
        payload.push_back(activeSensors.at(SENSOR_ID_BATERIA)->regsPerChannel & 0x1F);
    }
    if (activate_byte & (1 << 1)) {
        payload.push_back(activeSensors.at(SENSOR_ID_VOLTAJE)->regsPerChannel & 0x1F);
    }
    if (activate_byte & (1 << 2)) {
        payload.push_back(activeSensors.at(SENSOR_ID_CORRIENTE)->regsPerChannel & 0x1F);
    }
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        if (activate_byte & (1 << (i + 3))) {
            uint8_t sid = SENSOR_ID_EXT_START + i;
            payload.push_back(activeSensors.at(sid)->regsPerChannel & 0x1F);
        }
    }

    // 5. Data Blocks (same order)
    if (activate_byte & (1 << 0)) {
        const auto& s = activeSensors.at(SENSOR_ID_BATERIA);
        payload.insert(payload.end(), s->data, s->data + s->dataSize);
    }
    if (activate_byte & (1 << 1)) {
        const auto& s = activeSensors.at(SENSOR_ID_VOLTAJE);
        payload.insert(payload.end(), s->data, s->data + s->dataSize);
    }
    if (activate_byte & (1 << 2)) {
        const auto& s = activeSensors.at(SENSOR_ID_CORRIENTE);
        payload.insert(payload.end(), s->data, s->data + s->dataSize);
    }
    for (int i = 0; i < MAX_SENSORES_EXTERNOS; ++i) {
        if (activate_byte & (1 << (i + 3))) {
            uint8_t sid = SENSOR_ID_EXT_START + i;
            const auto& s = activeSensors.at(sid);
            payload.insert(payload.end(), s->data, s->data + s->dataSize);
        }
    }

    return payload;
}

// =================================================================================================
// Phantom Task — generates fake sensor data, builds payload, queues for LoRa
// =================================================================================================

void phantomTask(void *pvParameters) {
    uint8_t msgId = 0;

    while (true) {
        uint32_t t_ms = millis();

        std::map<uint8_t, std::vector<uint8_t>> groups;
        std::map<uint8_t, uint8_t> regsPerChannel;

        LOG_I("--- Ciclo fantasma (msgId=%u) ---", msgId);

        for (size_t i = 0; i < kPhantomSensorCount; ++i) {
            const auto& sensor = kPhantomSensors[i];

            std::vector<uint8_t> bytes;
            generatePhantomData(sensor, t_ms, bytes);

            auto& group = groups[sensor.sensorType];
            group.insert(group.end(), bytes.begin(), bytes.end());
            regsPerChannel[sensor.sensorType] = sensor.numRegs;

            LOG_D("  Sensor %u: %u bytes generados", sensor.sensorType, bytes.size());
        }

        if (!groups.empty()) {
            std::vector<SensorDataPayload> payloads;
            for (const auto& kv : groups) {
                uint8_t sensorType = kv.first;
                const auto& data   = kv.second;
                if (data.empty()) continue;

                SensorDataPayload p{};
                p.slaveId        = 0;  // phantom has no physical slave
                p.sensorId       = sensorType;
                p.dataSize       = std::min(data.size(), (size_t)MAX_SENSOR_PAYLOAD);
                p.regsPerChannel = regsPerChannel[sensorType];
                memcpy(p.data, data.data(), p.dataSize);

                payloads.push_back(p);
            }

            std::vector<uint8_t> unified = construirPayloadUnificado(msgId, payloads);

            Fragmento frag;
            frag.len = std::min(unified.size(), (size_t)LORA_PAYLOAD_MAX);
            memcpy(frag.data, unified.data(), frag.len);

            LOG_I("Phantom enviando %u bytes por LoRa (%zu sensores)",
                  frag.len, payloads.size());

            xQueueSend(queueFragmentos, &frag, pdMS_TO_TICKS(100));
        }

        ++msgId;
        vTaskDelay(pdMS_TO_TICKS(PHANTOM_INTERVAL_MS));
    }
}

// =================================================================================================
// LoRa (unchanged from real master)
// =================================================================================================

const lmic_pinmap lmic_pins = {
    .nss  = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = LMIC_UNUSED_PIN,
    .dio  = {26, 33, 32}
};

void onEvent(ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        LOG_I("LoRa: TX completo.");
        xSemaphoreGive(semaforoEnvioCompleto);
        if (LMIC.txrxFlags & TXRX_ACK) {
            LOG_I("LoRa: ACK recibido.");
        }
    }
}

void initLoRa() {
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
    LMIC_selectSubBand(7);
    LMIC_setDrTxpow(US915_DR_SF7, 20);
    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);
}

void tareaLoRa(void *pvParameters) {
    Fragmento frag;
    while (true) {
        if (xQueueReceive(queueFragmentos, &frag, portMAX_DELAY) == pdTRUE) {
            xSemaphoreTake(semaforoEnvioCompleto, portMAX_DELAY);
            LOG_I("LoRa: enviando fragmento de %u bytes", frag.len);
            if (LOG_LEVEL >= 3) {
                Serial.print("[I] Payload: ");
                for (size_t i = 0; i < frag.len; i++) {
                    if (i > 0) Serial.print(",");
                    Serial.print("0x");
                    if (frag.data[i] < 0x10) Serial.print("0");
                    Serial.print(frag.data[i], HEX);
                }
                Serial.println();
            }
            LMIC_setTxData2(1, frag.data, frag.len, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void tareaRunLoop(void *pvParameters) {
    while (true) {
        os_runloop_once();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// =================================================================================================
// Setup and Loop
// =================================================================================================

void setup() {
    Serial.begin(115200);
    {
        const uint32_t t0 = millis();
        while (!Serial && (millis() - t0) < 3000) { delay(10); }
    }
    Serial.println("Iniciando PHANTOM node (sin Modbus)...");

#ifdef TBEAM_V1
    // ── T-Beam V1.2: power on SX1276 via AXP192 LDO2 ─────────────────────
    {
        Wire.begin(21, 22);
        // Set LDO2 voltage to ~3.3 V (register 0x28, bits 7:4 = 0xF)
        Wire.beginTransmission(0x34);
        Wire.write(0x28);
        Wire.write(0xF0);
        Wire.endTransmission();
        // Enable LDO2 output (register 0x12, bit 2)
        Wire.beginTransmission(0x34);
        Wire.write(0x12);
        Wire.endTransmission(false);
        Wire.requestFrom(0x34, 1);
        if (Wire.available()) {
            uint8_t reg12 = Wire.read();
            Wire.beginTransmission(0x34);
            Wire.write(0x12);
            Wire.write(reg12 | 0x04);
            Wire.endTransmission();
        }
        delay(10);
    }
    Serial.println("T-Beam: AXP192 LDO2 habilitado");
#endif

    SPI.begin();

    queueFragmentos       = xQueueCreate(10, sizeof(Fragmento));
    semaforoEnvioCompleto = xSemaphoreCreateBinary();
    xSemaphoreGive(semaforoEnvioCompleto);

    initLoRa();

    xTaskCreatePinnedToCore(tareaRunLoop, "RunLoop",   2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(tareaLoRa,    "LoRaTask",  2048, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(phantomTask,  "Phantom",   4096, NULL, 3, NULL, 0);

    Serial.printf("PHANTOM: DEVADDR=0x%08lX, intervalo=%lu ms, %zu sensores\n",
                  (unsigned long)DEVADDR, (unsigned long)PHANTOM_INTERVAL_MS,
                  kPhantomSensorCount);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
