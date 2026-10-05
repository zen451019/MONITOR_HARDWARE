/**
 * @file main.cpp
 * @brief Modbus RTU Master over LoRaWAN (ESP32/TTGO) — Polling Edition.
 * @details Table-driven system that reads all Modbus sensors in batch every
 *          POLL_INTERVAL_MS, groups results by sensor type, and transmits
 *          via LoRaWAN. No auto-discovery, no dynamic slave removal.
 * @date 2026-06-05
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
#include <algorithm>
#include "ModbusClientRTU.h"
#include "ModbusAPI.h"
#include "ModbusConfig.h"
#include "loraconfig.h"
#include "SensorRegistry.h"
#include "Log.h"

// =================================================================================================
// Data Structures
// =================================================================================================

#define MAX_SENSOR_PAYLOAD 128

struct SensorDataPayload {
    uint8_t slaveId;
    uint8_t sensorId;
    uint8_t data[MAX_SENSOR_PAYLOAD];
    size_t  dataSize;
    uint8_t  regsPerChannel;   // registers per channel (for the len byte in the payload)
};

// =================================================================================================
// Forward Declarations
// =================================================================================================
std::vector<uint8_t> construirPayloadUnificado(uint8_t id_mensaje,
    const std::vector<SensorDataPayload>& collectedPayloads);
static void flushUartRx(HardwareSerial& s);

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
// UART Helper
// =================================================================================================

static void flushUartRx(HardwareSerial& s) {
    while (s.available() > 0) { (void)s.read(); }
}

// =================================================================================================
// Payload Builder (wire format IDENTICAL to previous version)
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
    // Uses sensor->regsPerChannel instead of the old getRegistersPerChannel()
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
// Main Polling Task — reads all requests in batch, groups by sensorType, sends via LoRa
// =================================================================================================

void mainPollingTask(void *pvParameters) {
    uint8_t msgId = 0;

    while (true) {
        // Group accumulated data:  sensorType → concatenated bytes
        std::map<uint8_t, std::vector<uint8_t>> groups;
        // Registers per channel for each sensorType (for the len byte)
        std::map<uint8_t, uint8_t> regsPerChannel;
        // SensorTypes that had at least one failure — excluded from payload
        std::set<uint8_t> failedTypes;

        LOG_I("--- Ciclo de consulta (msgId=%u) ---", msgId);

        for (size_t i = 0; i < kRequestCount; ++i) {
            const auto& req = kRequests[i];
            uint8_t  fnCode    = lookupFunctionCode(req.slaveID);
            uint32_t timeout   = lookupTimeout(req.slaveID);
            bool     swapWords = lookupSwapWords(req.slaveID);

            LOG_D("Solicitando Slave=%u, Addr=0x%04X, Regs=%u, FC=0x%02X",
                  req.slaveID, req.startAddr, req.numRegs, fnCode);

            flushUartRx(Serial2);
            ModbusApiResult result = modbus_api_read_registers(
                req.slaveID, fnCode, req.startAddr, req.numRegs, timeout);

            if (result.error_code == ModbusApiError::SUCCESS) {
                // Extract register bytes respecting endianness
                std::vector<uint8_t> bytes;
                bytes.reserve(req.numRegs * 2);
                for (size_t r = 0; r < req.numRegs; ++r) {
                    size_t off = r * 2;
                    if ((off + 1) >= result.data_len) break;
                    uint8_t hi = result.data[off];
                    uint8_t lo = result.data[off + 1];
                    if (swapWords) {
                        bytes.push_back(lo);
                        bytes.push_back(hi);
                    } else {
                        bytes.push_back(hi);
                        bytes.push_back(lo);
                    }
                }

                // Pad per-request to 4-byte boundary (each channel = 32-bit block)
                while (bytes.size() % 4 != 0) {
                    bytes.insert(bytes.begin(), 0x00);
                }

                // Word swap for 32-bit values when device sends LOW word first
                if (req.swapWordOrder) {
                    uint8_t t0 = bytes[0]; bytes[0] = bytes[2]; bytes[2] = t0;
                    uint8_t t1 = bytes[1]; bytes[1] = bytes[3]; bytes[3] = t1;
                }

                auto& group = groups[req.sensorType];
                group.insert(group.end(), bytes.begin(), bytes.end());
                LOG_D("  -> OK: %u bytes", bytes.size());
            } else {
                LOG_W("  -> Error %u: Slave=%u, Addr=0x%04X",
                      static_cast<uint8_t>(result.error_code),
                      req.slaveID, req.startAddr);
                failedTypes.insert(req.sensorType);
            }
        }

        // Safety: pad each group to multiple of 4 bytes (should already be aligned)
        for (auto& kv : groups) {
            while (kv.second.size() % 4 != 0) {
                kv.second.insert(kv.second.begin(), 0x00);
            }
            regsPerChannel[kv.first] = (uint8_t)(kv.second.size() / 4);
        }

        // Assemble payloads and send
        if (!groups.empty()) {
            std::vector<SensorDataPayload> payloads;
            for (const auto& kv : groups) {
                uint8_t sensorType = kv.first;
                const auto& data   = kv.second;
                if (data.empty()) continue;
                if (failedTypes.count(sensorType)) {
                    LOG_W("SensorType %u: descartado (fallo parcial en al menos un canal)", sensorType);
                    continue;
                }

                SensorDataPayload p{};
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

            LOG_I("Enviando %u bytes por LoRa (%zu grupos de sensores)",
                  frag.len, payloads.size());

            xQueueSend(queueFragmentos, &frag, pdMS_TO_TICKS(100));
        }

        ++msgId;
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

// =================================================================================================
// LoRa — unchanged
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
    Serial.println("Iniciando sistema (modo tabla)...");

    SPI.begin();

    // Modbus init with configurable bus parameters
    modbus_api_init(Serial2, kBusCfg.rxPin, kBusCfg.txPin,
                    kBusCfg.baudRate, kBusCfg.uartConfig);

    // LoRa queues and semaphore
    queueFragmentos       = xQueueCreate(10, sizeof(Fragmento));
    semaforoEnvioCompleto = xSemaphoreCreateBinary();
    xSemaphoreGive(semaforoEnvioCompleto);

    initLoRa();

    xTaskCreatePinnedToCore(tareaRunLoop, "RunLoop",  2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(tareaLoRa,    "LoRaTask", 2048, NULL, 5, NULL, 1);

    // Single main polling task — replaces all scheduler/aggregator complexity
    xTaskCreatePinnedToCore(mainPollingTask, "MainPoll", 8192, NULL, 3, NULL, 0);

    Serial.printf("Configurado: bus a %lu baud, %zu requests, intervalo %lu ms\n",
                  kBusCfg.baudRate, kRequestCount, POLL_INTERVAL_MS);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
