#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <cstdint>
#include <Arduino.h>
#include "SensorRegistry.h"

// =================================================================================================
// RS485 Bus configuration (global, all slaves on the same bus share these parameters)
// =================================================================================================
struct ModbusBusConfig {
    unsigned long baudRate;
    uint32_t      uartConfig;       // SERIAL_8N1, SERIAL_8E1, SERIAL_8O1, etc.
    int           rxPin;
    int           txPin;
    uint32_t      defaultTimeoutMs;
};

const ModbusBusConfig kBusCfg = {
    9600,            // baudRate
    SERIAL_8N1,      // uartConfig
    13,              // rxPin
    12,              // txPin
    2000             // defaultTimeoutMs
};

// =================================================================================================
// Per-device overrides (optional - only needed when a device deviates from defaults)
// =================================================================================================
struct ModbusDeviceCfg {
    uint8_t  slaveID;
    uint8_t  functionCode;   // 0x03 = holding registers, 0x04 = input registers
    bool     swapWords;      // true = low-byte-first, false = hi/lo (big-endian)
    uint32_t timeoutMs;      // per-device timeout override
};

#define DEV_TRIFASICO_NUEVO 1   // Medidor Trifasico: low-byte-first (little-endian por registro)

const ModbusDeviceCfg kDeviceCfg[] = {
    {DEV_TRIFASICO_NUEVO, 0x04, true, 2000},
};

constexpr size_t kDeviceCfgCount = sizeof(kDeviceCfg) / sizeof(kDeviceCfg[0]);

// =================================================================================================
// Decode helpers - el dispositivo manda little-endian (byte bajo primero)
// =================================================================================================
// Se usan dentro de las funciones decode de cada senal.

// u16 en el registro 'reg' (bytes [lo, hi] en el aire).
static inline uint16_t mb_u16(const uint8_t* d, size_t len, uint16_t reg) {
    size_t o = (size_t)reg * 2;
    if (o + 1 >= len) return 0;
    return (uint16_t)(d[o] | (d[o + 1] << 8));
}

// int32 a partir del registro 'reg' (palabra baja primero, little-endian).
static inline int32_t mb_i32(const uint8_t* d, size_t len, uint16_t reg) {
    size_t o = (size_t)reg * 2;
    if (o + 3 >= len) return 0;
    uint32_t u = (uint32_t)d[o] | ((uint32_t)d[o + 1] << 8) |
                 ((uint32_t)d[o + 2] << 16) | ((uint32_t)d[o + 3] << 24);
    return (int32_t)u;
}

// Byte crudo en la posicion 'i' del buffer.
static inline uint8_t mb_byte(const uint8_t* d, size_t len, uint16_t i) {
    return (i < len) ? d[i] : (uint8_t)0;
}

// =================================================================================================
// Read blocks - UNA entrada por cada lectura Modbus (registros contiguos en una sola peticion)
// =================================================================================================
struct ModbusBlock {
    uint8_t  slaveID;
    uint8_t  functionCode;
    uint16_t startAddr;
    uint16_t numRegs;
};

const ModbusBlock kBlocks[] = {
    // Bloque 0: V A/B/C (0x0000-0x0002) + I A/B/C (0x0003-0x0005) + Frecuencia A/B/C (0x0006-0x0008)
    {DEV_TRIFASICO_NUEVO, 0x04, 0x0000, 9},
    // Bloque 1: Potencias totales (int32) Activa 0x0020 / Reactiva 0x0022 / Aparente 0x0024
    {DEV_TRIFASICO_NUEVO, 0x04, 0x0020, 6},
    // Bloque 2: Factor de potencia (0x0026: A|B, 0x0027: C|Combinado) - bytes empaquetados
    {DEV_TRIFASICO_NUEVO, 0x04, 0x0026, 2},
    // Bloque 3: Energia activa total 0x003A (int32)
    {DEV_TRIFASICO_NUEVO, 0x04, 0x003A, 2},
};

constexpr size_t kBlockCount = sizeof(kBlocks) / sizeof(kBlocks[0]);

// =================================================================================================
// Signal map - una entrada por valor; cada una lleva SU funcion de decodificacion
// =================================================================================================
// La funcion recibe los bytes crudos del bloque (data/len) y devuelve el valor final.
// Es el equivalente al "lambda" por sensor de ESPHome: tu decides como acomodar los datos.
struct ModbusSignal {
    uint8_t sensorType;
    uint8_t channel;
    uint8_t blockIndex;
    float (*decode)(const uint8_t* data, size_t len);
};

const ModbusSignal kSignals[] = {
    // ---- Voltaje Fase A/B/C (u16, x0.1) ----
    {SENSOR_ID_VOLTAJE, 0, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 0) * 0.1f; }},
    {SENSOR_ID_VOLTAJE, 1, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 1) * 0.1f; }},
    {SENSOR_ID_VOLTAJE, 2, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 2) * 0.1f; }},

    // ---- Corriente Fase A/B/C (u16, x0.01) ----
    {SENSOR_ID_CORRIENTE, 0, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 3) * 0.01f; }},
    {SENSOR_ID_CORRIENTE, 1, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 4) * 0.01f; }},
    {SENSOR_ID_CORRIENTE, 2, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 5) * 0.01f; }},

    // ---- Frecuencia Fase A/B/C (u16, x0.01) ----
    {SENSOR_ID_FRECUENCIA, 0, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 6) * 0.01f; }},
    {SENSOR_ID_FRECUENCIA, 1, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 7) * 0.01f; }},
    {SENSOR_ID_FRECUENCIA, 2, 0, [](const uint8_t* d, size_t n) { return mb_u16(d, n, 8) * 0.01f; }},

    // ---- Potencia Activa Total (int32, x0.1) @0x0020 ----
    {SENSOR_ID_POTENCIA_ACTIVA, 0, 1, [](const uint8_t* d, size_t n) { return mb_i32(d, n, 0) * 0.1f; }},
    // ---- Potencia Reactiva Total (int32, x0.1) @0x0022 ----
    {SENSOR_ID_POTENCIA_REACTIVA, 0, 1, [](const uint8_t* d, size_t n) { return mb_i32(d, n, 2) * 0.1f; }},
    // ---- Potencia Aparente Total (int32, x0.1) @0x0024 ----
    {SENSOR_ID_POTENCIA_APARENTE, 0, 1, [](const uint8_t* d, size_t n) { return mb_i32(d, n, 4) * 0.1f; }},

    // ---- Factor de Potencia (bytes empaquetados, x0.01) ----
    // Aire: 0x0026 = [B, A], 0x0027 = [Combinado, C]  (byte bajo primero)
    {SENSOR_ID_FACTOR_POTENCIA, 0, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 1) * 0.01f; }}, // Fase A
    {SENSOR_ID_FACTOR_POTENCIA, 1, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 0) * 0.01f; }}, // Fase B
    {SENSOR_ID_FACTOR_POTENCIA, 2, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 3) * 0.01f; }}, // Fase C
    {SENSOR_ID_FACTOR_POTENCIA, 3, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 2) * 0.01f; }}, // Combinado

    // ---- Energia Activa Total (int32, x0.1) @0x003A ----
    {SENSOR_ID_BATERIA, 0, 3, [](const uint8_t* d, size_t n) { return mb_i32(d, n, 0) * 0.1f; }},
};

constexpr size_t kSignalCount = sizeof(kSignals) / sizeof(kSignals[0]);

// =================================================================================================
// Timing
// =================================================================================================
constexpr unsigned long POLL_INTERVAL_MS = 30000;   // Cada 30 s se consulta todo el bus

// =================================================================================================
// Lookup helpers (inline to avoid ODR violations)
// =================================================================================================
inline uint8_t lookupFunctionCode(uint8_t slaveID) {
    for (size_t i = 0; i < kDeviceCfgCount; ++i) {
        if (kDeviceCfg[i].slaveID == slaveID) return kDeviceCfg[i].functionCode;
    }
    return 0x03;
}

inline uint32_t lookupTimeout(uint8_t slaveID) {
    for (size_t i = 0; i < kDeviceCfgCount; ++i) {
        if (kDeviceCfg[i].slaveID == slaveID) return kDeviceCfg[i].timeoutMs;
    }
    return kBusCfg.defaultTimeoutMs;
}

inline bool lookupSwapWords(uint8_t slaveID) {
    for (size_t i = 0; i < kDeviceCfgCount; ++i) {
        if (kDeviceCfg[i].slaveID == slaveID) return kDeviceCfg[i].swapWords;
    }
    return false;
}

#endif // MODBUS_CONFIG_H
