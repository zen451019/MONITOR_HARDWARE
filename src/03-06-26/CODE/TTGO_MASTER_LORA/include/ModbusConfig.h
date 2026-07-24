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
// Per-device overrides (optional — only needed when a device deviates from defaults)
// =================================================================================================
// Defaults: functionCode=0x03 (read holding registers), swapWords=false, timeoutMs=kBusCfg.defaultTimeoutMs
//
// Example entry for a device that uses input registers (0x04) and needs word swapping:
//   {1, 0x04, true, 3000},
struct ModbusDeviceCfg {
    uint8_t  slaveID;
    uint8_t  functionCode;   // 0x03 = holding registers, 0x04 = input registers
    bool     swapWords;       // true = lo/hi byte order, false = hi/lo (big-endian)
    uint32_t timeoutMs;       // per-device timeout override
};

#define DEV_TRIFASICO_NUEVO 1   // Medidor Trifásico (manual_modbus_rtu_clean): low-byte-first

const ModbusDeviceCfg kDeviceCfg[] = {
    {DEV_TRIFASICO_NUEVO, 0x04, true, 2000},   // input regs, low-byte-first (little-endian per register)
};

constexpr size_t kDeviceCfgCount = sizeof(kDeviceCfg) / sizeof(kDeviceCfg[0]);

// =================================================================================================
// Request table — UNA entrada por cada lectura Modbus individual
// =================================================================================================
// - functionCode se resuelve desde kDeviceCfg o usa 0x03 por defecto
// - channelIndex ordena dentro del mismo sensorType (0=L1, 1=L2, 2=L3...)
// - sensorType debe coincidir con los IDs de SensorRegistry.h

struct ModbusRequest {
    uint8_t  slaveID;
    uint16_t startAddr;
    uint16_t numRegs;
    uint8_t  channelIndex;     // sub-índice dentro del tipo de sensor
    uint8_t  sensorType;       // SENSOR_ID_VOLTAJE, SENSOR_ID_CORRIENTE, etc.
    bool     swapWordOrder;    // intercambia palabra ALTA/BAJA (para 32-bit con LOW word primero)
};

const ModbusRequest kRequests[] = {
    // --- Esclavo 1 (DEV_TRIFASICO_NUEVO): FC 0x04, low-byte-first per register ---
    // swapWordOrder=true  → valores 32-bit donde el dispositivo manda LOW word primero
    // swapWordOrder=false → valores 16-bit (1 solo registro, no hay segundo word que intercambiar)

    // BATERIA (bit 0): Energía Activa Total → 32-bit, LOW word first
    {DEV_TRIFASICO_NUEVO, 0x003A, 2, 0, SENSOR_ID_BATERIA,              true},

    // VOLTAJE (bit 1): V Fase A, B, C → 16-bit c/u (LSB = 0.1 V)
    {DEV_TRIFASICO_NUEVO, 0x0000, 1, 0, SENSOR_ID_VOLTAJE,             false},
    {DEV_TRIFASICO_NUEVO, 0x0001, 1, 1, SENSOR_ID_VOLTAJE,             false},
    {DEV_TRIFASICO_NUEVO, 0x0002, 1, 2, SENSOR_ID_VOLTAJE,             false},

    // CORRIENTE (bit 2): I Fase A, B, C → 16-bit c/u (LSB = 0.01 A)
    {DEV_TRIFASICO_NUEVO, 0x0003, 1, 0, SENSOR_ID_CORRIENTE,           false},
    {DEV_TRIFASICO_NUEVO, 0x0004, 1, 1, SENSOR_ID_CORRIENTE,           false},
    {DEV_TRIFASICO_NUEVO, 0x0005, 1, 2, SENSOR_ID_CORRIENTE,           false},

    // EXT+0 (bit 3): Potencia Activa Total → 32-bit signed, LOW word first
    {DEV_TRIFASICO_NUEVO, 0x0020, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 0),  true},

    // EXT+1 (bit 4): Potencia Aparente Total → 32-bit signed, LOW word first
    {DEV_TRIFASICO_NUEVO, 0x0024, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 1),  true},

    // EXT+2 (bit 5): Potencia Reactiva Total → 32-bit signed, LOW word first
    {DEV_TRIFASICO_NUEVO, 0x0022, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 2),  true},

    // EXT+3 (bit 6): Factor de Potencia Combinado → 1 reg, backend extrae byte bajo
    {DEV_TRIFASICO_NUEVO, 0x0027, 1, 0, (uint8_t)(SENSOR_ID_EXT_START + 3), false},

    // EXT+4 (bit 7): Frecuencia Fase A → 16-bit (LSB = 0.01 Hz)
    {DEV_TRIFASICO_NUEVO, 0x0006, 1, 0, (uint8_t)(SENSOR_ID_EXT_START + 4), false},
};

constexpr size_t kRequestCount = sizeof(kRequests) / sizeof(kRequests[0]);

// =================================================================================================
// Timing
// =================================================================================================
constexpr unsigned long POLL_INTERVAL_MS = 30000;   // Cada 30 segundos se consulta todo el bus

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
