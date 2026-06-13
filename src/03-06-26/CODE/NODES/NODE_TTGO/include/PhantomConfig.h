#ifndef PHANTOM_CONFIG_H
#define PHANTOM_CONFIG_H

#include <cstdint>
#include "SensorRegistry.h"

// =================================================================================================
// Phantom Master Node — bombards LoRaWAN server with synthetic sensor data
// =================================================================================================
// Edit DEVADDR & INTERVAL here. Tweak each sensor's mode (FIXED / SINE) and its values.
// Wire format is IDENTICAL to the real master — the backend cannot tell them apart.
// =================================================================================================

// ── LoRaWAN identity ────────────────────────────────────────────────────────────────────────────
#define PHANTOM_DEVADDR 0xC0FFEE01

// ── Bombardment interval (ms) ───────────────────────────────────────────────────────────────────
// 0 = as fast as LoRa duty cycle permits (TX_COMPLETE-gated)
#define PHANTOM_INTERVAL_MS 15000

// ── Data generation mode ────────────────────────────────────────────────────────────────────────
enum PhantomMode : uint8_t {
    PHANTOM_FIXED = 0,   // static value, uses fixedRaw
    PHANTOM_SINE  = 1    // oscillates between center-amp and center+amp
};

// ── Sensor definition ───────────────────────────────────────────────────────────────────────────
//  numRegs = 1 → uint16  (2 bytes on the wire, big-endian)
//  numRegs = 2 → float32 (4 bytes on the wire, IEEE 754 big-endian)
//
//  FIXED mode: fixedRaw contains the exact bytes to send every cycle.
//    • 1-reg sensor → fixedRaw holds the uint16 value (upper 16 bits ignored)
//    • 2-reg sensor → fixedRaw holds the float32 bit pattern
//
//  SINE mode: value = sineCenter + sineAmplitude * sin(2π·t / sinePeriodMs)
//    where t = millis().  For 1-reg sensors the float result is cast to uint16.
//
struct PhantomSensorDef {
    uint8_t      sensorType;      // SensorRegistry ID
    uint8_t      numRegs;         // 1 = uint16, 2 = float32
    PhantomMode  mode;
    uint32_t     fixedRaw;        // FIXED: raw bytes (big-endian)
    float        sineCenter;      // SINE: center value
    float        sineAmplitude;   // SINE: amplitude
    uint32_t     sinePeriodMs;    // SINE: period in ms
};

// ═════════════════════════════════════════════════════════════════════════════════════════════════
// PHANTOM SENSOR TABLE — same 8 sensors as the real master
// ═════════════════════════════════════════════════════════════════════════════════════════════════
//
// Real master mapping (for reference):
//   SLAVE 10 (SDM630MCT): V_L1, I_L1, kWh, W_L1, PF_L1  (all 32-bit float)
//   SLAVE 5  (Analog):    Pressure (centi-PSI), mV        (uint16)
//   SLAVE 1  (Lux):       Illuminance                     (32-bit float)
//
// Activate-byte bit layout:
//   bit 0 = BATERIA (kWh)
//   bit 1 = VOLTAJE
//   bit 2 = CORRIENTE
//   bit 3 = EXT+0 (Pressure)
//   bit 4 = EXT+1 (mV)
//   bit 5 = EXT+2 (Lux)
//   bit 6 = EXT+3 (Active Power)
//   bit 7 = EXT+4 (Power Factor)

const PhantomSensorDef kPhantomSensors[] = {

    // ── SDM630MCT energy meter (2 registers = 32-bit float IEEE 754) ──

    // Voltage L1: 120 V ± 10 V, 30 s period
    {SENSOR_ID_VOLTAJE,        2, PHANTOM_SINE,  0, 120.0f,  10.0f,  30000},

    // Current L1: 500 A ± 50 A, 30 s period
    {SENSOR_ID_CORRIENTE,      2, PHANTOM_SINE,  0, 500.0f,  50.0f,  30000},

    // kWh total: 10000 kWh fixed
    {SENSOR_ID_BATERIA,        2, PHANTOM_FIXED, 0x461C4000, 0, 0, 0},  // IEEE 754 for 10000.0

    // Active power: 10000 W ± 1000 W, 30 s period
    {SENSOR_ID_EXT_START + 3,  2, PHANTOM_SINE,  0, 10000.0f, 1000.0f, 30000},

    // Power factor: 0.95 ± 0.05, 30 s period
    {SENSOR_ID_EXT_START + 4,  2, PHANTOM_SINE,  0, 0.95f,   0.05f,  30000},

    // ── Analog module (1 register = uint16) ──

    // Pressure: ~5000 centi-PSI ± 3000 (≈ 50 PSI ± 30), 30 s period
    {SENSOR_ID_EXT_START + 0,  1, PHANTOM_SINE,  0, 5000.0f, 3000.0f, 30000},

    // Voltage mV: ~12000 mV ± 500 (≈ 12 V ± 0.5), 30 s period
    {SENSOR_ID_EXT_START + 1,  1, PHANTOM_SINE,  0, 12000.0f, 500.0f,  30000},

    // ── Lux meter (2 registers = 32-bit float) ──

    // Illuminance: 1000 lux ± 200, 30 s period
    {SENSOR_ID_EXT_START + 2,  2, PHANTOM_SINE,  0, 1000.0f, 200.0f,  30000},

};

constexpr size_t kPhantomSensorCount = sizeof(kPhantomSensors) / sizeof(kPhantomSensors[0]);

#endif // PHANTOM_CONFIG_H
