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

#define DEV_VCC   10
#define DEV_PANEL 2
#define DEV_LUX   1

const ModbusDeviceCfg kDeviceCfg[] = {
    // El luxómetro usa 0x03 (default), no necesita override.
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
    uint8_t  channelIndex;   // sub-índice dentro del tipo de sensor
    uint8_t  sensorType;     // SENSOR_ID_VOLTAJE, SENSOR_ID_CORRIENTE, etc.
};

const ModbusRequest kRequests[] = {
    // --- Esclavo 10 (DEV_VCC): Voltajes (3 líneas, 1 int16 c/u) ---
    {DEV_VCC,   0x0000, 1, 0, SENSOR_ID_VOLTAJE},
    {DEV_VCC,   0x0002, 1, 1, SENSOR_ID_VOLTAJE},
    {DEV_VCC,   0x0004, 1, 2, SENSOR_ID_VOLTAJE},
    // --- Esclavo 10: Corrientes (3 líneas, 1 int16 c/u) ---
    {DEV_VCC,   0x0006, 1, 0, SENSOR_ID_CORRIENTE},
    {DEV_VCC,   0x0008, 1, 1, SENSOR_ID_CORRIENTE},
    {DEV_VCC,   0x000A, 1, 2, SENSOR_ID_CORRIENTE},
    // --- Esclavo 2: Batería ---
    {DEV_PANEL, 0x0100, 2, 0, SENSOR_ID_BATERIA},
    // --- Esclavo 10: Analógicos (cada uno con su propia bandera) ---
    {DEV_VCC,   0x000C, 1, 0, (uint8_t)(SENSOR_ID_EXT_START + 0)},  // Presión (bit 3)
    {DEV_VCC,   0x000D, 1, 0, (uint8_t)(SENSOR_ID_EXT_START + 1)},  // Voltaje mV (bit 4)
    // --- Esclavo 1 (DEV_LUX): Luxómetro industrial, 32-bit (2 regs) ---
    {DEV_LUX,   0x0002, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 2)},  // Iluminación (bit 5)
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
