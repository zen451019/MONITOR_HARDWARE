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
#include <RadioLib.h>
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
#include "SensorSimulator.h"
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
// LoRaWAN configuration
// =================================================================================================

// RadioLib sub-band numbering starts at 1.
// Legacy LMIC used sub-band 7 (channels 56-63); its RadioLib equivalent is sub-band 8.
static const uint8_t LORAWAN_SUB_BAND = 8;

// Application ports.
static const uint8_t FPORT_DATA = 1;   // uplink: sensor payload
static const uint8_t FPORT_CMD  = 2;   // downlink: control commands

// TTGO LoRa32 v2.1 pinout: NSS=18, DIO0=26, RST=23, DIO1=33.
static SX1276 radio = new Module(18, 26, 23, 33);
static LoRaWANNode node(&radio, &US915, LORAWAN_SUB_BAND);

// =================================================================================================
// Runtime-mutable state (control via downlink)
// =================================================================================================

volatile uint32_t g_pollIntervalMs = POLL_INTERVAL_MS;
static TaskHandle_t g_pollTaskHandle = NULL;

#if SIMULATION_MODE
static uint32_t g_simLastCycleMs = 0;
#endif

// =================================================================================================
// Forward Declarations
// =================================================================================================
std::vector<uint8_t> construirPayloadUnificado(uint8_t id_mensaje,
    const std::vector<SensorDataPayload>& collectedPayloads);
static void flushUartRx(HardwareSerial& s);
static void handleDownlink(const uint8_t* data, size_t len, const LoRaWANEvent_t& ev);

// =================================================================================================
// LoRa Globals
// =================================================================================================

QueueHandle_t queueFragmentos;

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

#if SIMULATION_MODE
        {
            const uint32_t nowMs = millis();
            const uint32_t dtMs  = (g_simLastCycleMs == 0)
                                       ? g_pollIntervalMs
                                       : (nowMs - g_simLastCycleMs);
            g_simLastCycleMs = nowMs;
            sim_begin_cycle(dtMs);
        }
#endif

        for (size_t i = 0; i < kRequestCount; ++i) {
            const auto& req = kRequests[i];
            uint8_t  fnCode    = lookupFunctionCode(req.slaveID);
            uint32_t timeout   = lookupTimeout(req.slaveID);
            bool     swapWords = lookupSwapWords(req.slaveID);
#if SIMULATION_MODE
            (void)timeout;
#endif

            LOG_D("Solicitando Slave=%u, Addr=0x%04X, Regs=%u, FC=0x%02X",
                  req.slaveID, req.startAddr, req.numRegs, fnCode);

#if SIMULATION_MODE
            ModbusApiResult result = sim_read_registers(
                req.slaveID, fnCode, req.startAddr, req.numRegs);
#else
            flushUartRx(Serial2);
            ModbusApiResult result = modbus_api_read_registers(
                req.slaveID, fnCode, req.startAddr, req.numRegs, timeout);
#endif

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

            // Inter-frame delay: gives the RS485 bus/slave time to recover between frames.
            vTaskDelay(pdMS_TO_TICKS(10));
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

        // Wait for the configured interval, unless a downlink asked to force a cycle now.
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(g_pollIntervalMs)) > 0) {
            LOG_I("Consulta forzada por downlink.");
        }
    }
}

// =================================================================================================
// LoRaWAN (RadioLib) — ABP activation
// =================================================================================================
// Fixed session (ABP), same behavior as the legacy LMIC version: no join, no DevNonce and
// no session persistence. The frame counter starts at 0 on every boot, so the network
// server must be configured to accept an ABP frame-counter reset.

// =================================================================================================
// Downlink command handling (FPort FPORT_CMD)
// =================================================================================================
// [0] = command
//   0x01 SET_POLL_INTERVAL : [1..4] uint32 BE, milliseconds
//   0x02 FORCE_POLL        : trigger the next polling cycle immediately
//   0x03 SET_TX_POWER      : [1] int8 dBm
//   0x04 SET_DATARATE      : [1] uint8 data rate (only meaningful with ADR off)
//   0x05 REBOOT            : restart the MCU
//   0x06 REJOIN            : not applicable in ABP (ignored)
//   0x07 SET_ADR           : [1] 0 = off, 1 = on

static void handleDownlink(const uint8_t* data, size_t len, const LoRaWANEvent_t& ev) {
    if (ev.fPort != FPORT_CMD) {
        LOG_I("Downlink en FPort %u ignorado (%u bytes).", ev.fPort, (unsigned)len);
        return;
    }
    if (len == 0) return;

    switch (data[0]) {
        case 0x01: {
            if (len < 5) { LOG_W("CMD SET_POLL_INTERVAL incompleto."); break; }
            uint32_t value = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                             ((uint32_t)data[3] << 8)  | (uint32_t)data[4];
            if (value < 1000) value = 1000;
            g_pollIntervalMs = value;
            LOG_I("CMD: intervalo de consulta = %lu ms", (unsigned long)value);
            break;
        }
        case 0x02:
            LOG_I("CMD: forzar consulta inmediata.");
            if (g_pollTaskHandle != NULL) {
                xTaskNotifyGive(g_pollTaskHandle);
            }
            break;
        case 0x03:
            if (len < 2) { LOG_W("CMD SET_TX_POWER incompleto."); break; }
            node.setTxPower((int8_t)data[1]);
            LOG_I("CMD: TX power = %d dBm", (int)data[1]);
            break;
        case 0x04:
            if (len < 2) { LOG_W("CMD SET_DATARATE incompleto."); break; }
            node.setDatarate(data[1]);
            LOG_I("CMD: datarate = %u", data[1]);
            break;
        case 0x05:
            LOG_I("CMD: reinicio solicitado.");
            delay(100);
            ESP.restart();
            break;
        case 0x06:
            LOG_W("CMD: rejoin no aplica en modo ABP (ignorado).");
            break;
        case 0x07:
            if (len < 2) { LOG_W("CMD SET_ADR incompleto."); break; }
            node.setADR(data[1] != 0);
            LOG_I("CMD: ADR %s", data[1] ? "ON" : "OFF");
            break;
        default:
            LOG_W("Downlink: comando 0x%02X desconocido.", data[0]);
            break;
    }
}

void initLoRa() {
    int16_t state = radio.begin();
    if (state != RADIOLIB_ERR_NONE) {
        LOG_E("LoRaWAN: radio.begin() falló (%d).", state);
    }

    // LoRaWAN 1.0.x ABP: pass NULL for the 1.1 forwarding/serving keys.
    state = node.beginABP(DEVADDR, nullptr, nullptr, NWKSKEY, APPSKEY);
    if (state != RADIOLIB_ERR_NONE) {
        LOG_E("LoRaWAN: beginABP() falló (%d).", state);
    }

    int16_t act = node.activateABP();
    if (act == RADIOLIB_LORAWAN_NEW_SESSION ||
        act == RADIOLIB_LORAWAN_SESSION_RESTORED ||
        act == RADIOLIB_ERR_NONE) {
        LOG_I("LoRaWAN: ABP activo (DevAddr=0x%08lX, max payload %u B).",
              (unsigned long)DEVADDR, (unsigned)node.getMaxPayloadLen());
    } else {
        LOG_E("LoRaWAN: activateABP() falló (%d).", act);
    }

    // Same settings as the legacy LMIC version.
    node.setADR(false);
    node.setDatarate(3);       // US915 DR3 = SF7/125 kHz
    node.setTxPower(20);
}

void tareaLoRa(void *pvParameters) {
    Fragmento frag;
    uint8_t downBuf[RADIOLIB_LORAWAN_MAX_PAYLOAD_SIZE];

    while (true) {
        if (xQueueReceive(queueFragmentos, &frag, portMAX_DELAY) == pdTRUE) {
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

            size_t downLen = 0;
            LoRaWANEvent_t evUp;
            LoRaWANEvent_t evDown;
            int16_t state = node.sendReceive(frag.data, frag.len, FPORT_DATA,
                                             downBuf, &downLen, false, &evUp, &evDown);

            if (state < RADIOLIB_ERR_NONE) {
                LOG_E("LoRa: sendReceive() error %d", state);
            } else {
                if (state > 0) {
                    LOG_I("LoRa: TX completo (DR=%u, freq=%.1f MHz, cnt=%lu) + downlink de %u bytes (FPort %u).",
                          evUp.datarate, evUp.freq, (unsigned long)evUp.fCnt,
                          (unsigned)downLen, evDown.fPort);
                    if (downLen > 0) {
                        handleDownlink(downBuf, downLen, evDown);
                    }
                } else {
                    LOG_I("LoRa: TX completo (DR=%u, freq=%.1f MHz, cnt=%lu), sin downlink.",
                          evUp.datarate, evUp.freq, (unsigned long)evUp.fCnt);
                }
            }
        }
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

#if SIMULATION_MODE
    // Simulation build: no physical RS485 traffic, no Modbus master init.
    sim_init();
    LOG_I("Modo SIMULACION activo: no se realizan lecturas Modbus reales.");
#else
    // Modbus init with configurable bus parameters
    modbus_api_init(Serial2, kBusCfg.rxPin, kBusCfg.txPin,
                    kBusCfg.baudRate, kBusCfg.uartConfig);
#endif

    // LoRa queue
    queueFragmentos = xQueueCreate(10, sizeof(Fragmento));

    initLoRa();

    xTaskCreatePinnedToCore(tareaLoRa, "LoRaTask", 8192, NULL, 5, NULL, 1);

    // Single main polling task — replaces all scheduler/aggregator complexity
    xTaskCreatePinnedToCore(mainPollingTask, "MainPoll", 8192, NULL, 3, &g_pollTaskHandle, 0);

#if SIMULATION_MODE
    Serial.printf("SIMULACION activa: intervalo %lu ms, %zu requests\n",
                  (unsigned long)g_pollIntervalMs, kRequestCount);
#else
    Serial.printf("Configurado: bus a %lu baud, %zu requests, intervalo %lu ms\n",
                  kBusCfg.baudRate, kRequestCount, (unsigned long)g_pollIntervalMs);
#endif
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
