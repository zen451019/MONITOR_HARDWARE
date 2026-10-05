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

// #define DEV_VCC   5       // Comentado: prueba con nuevo dispositivo
#define DEV_SDM   10         // Eastron SDM630MCT Smart Meter
// #define DEV_LUX   1       // Comentado: prueba con nuevo dispositivo
// #define DEV_TRIFASICO   2   // Reemplazado por SDM630

const ModbusDeviceCfg kDeviceCfg[] = {
    {DEV_SDM, 0x04, false, 2000},       // SDM630MCT: input regs, MSB register first, hi-byte first
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
    // --- Esclavo 10 (DEV_SDM): Eastron SDM630MCT, FC 0x04, 32-bit IEEE 754 float (MSB register first) ---
    // BATERIA (bit 0): Total kWh                          → 1 canal
    {DEV_SDM, 0x0156, 2, 0, SENSOR_ID_BATERIA,                    false},  // 30343: Total kWh (float)
    // VOLTAJE (bit 1): V L1-L2, V L2-L3, V L3-L1           → 3 canales
    {DEV_SDM, 0x00C8, 2, 0, SENSOR_ID_VOLTAJE,                   false},  // 30201: V L1-L2 (float)
    {DEV_SDM, 0x00CA, 2, 1, SENSOR_ID_VOLTAJE,                   false},  // 30203: V L2-L3 (float)
    {DEV_SDM, 0x00CC, 2, 2, SENSOR_ID_VOLTAJE,                   false},  // 30205: V L3-L1 (float)
    // CORRIENTE (bit 2): I L1, I L2, I L3                   → 3 canales
    {DEV_SDM, 0x0006, 2, 0, SENSOR_ID_CORRIENTE,                 false},  // 30007: I L1 (float)
    {DEV_SDM, 0x0008, 2, 1, SENSOR_ID_CORRIENTE,                 false},  // 30009: I L2 (float)
    {DEV_SDM, 0x000A, 2, 2, SENSOR_ID_CORRIENTE,                 false},  // 30011: I L3 (float)
    // EXT+0 (bit 3): Potencia activa total                  → 1 canal
    {DEV_SDM, 0x0034, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 0),  false},  // 30053: P activa total (float)
    // EXT+1 (bit 4): Potencia aparente total                → 1 canal
    {DEV_SDM, 0x0038, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 1),  false},  // 30057: S aparente total (float)
    // EXT+2 (bit 5): Potencia reactiva total                → 1 canal
    {DEV_SDM, 0x003C, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 2),  false},  // 30061: Q reactiva total (float)
    // EXT+3 (bit 6): Factor de potencia total               → 1 canal
    {DEV_SDM, 0x003E, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 3),  false},  // 30063: PF total (float)
    // EXT+4 (bit 7): Frecuencia                             → 1 canal
    {DEV_SDM, 0x0046, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 4),  false},  // 30071: Frecuencia (float)
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
