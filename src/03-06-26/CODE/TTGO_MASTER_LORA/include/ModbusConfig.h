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
// #define DEV_SDM   10      // Comentado: prueba con nuevo dispositivo
// #define DEV_LUX   1       // Comentado: prueba con nuevo dispositivo
#define DEV_TRIFASICO   2     // Nuevo Medidor de Energía Trifásico

const ModbusDeviceCfg kDeviceCfg[] = {
    {DEV_TRIFASICO, 0x04, true, 2000},  // Trifásico: input regs, low byte first → swapWords
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
    // --- Esclavo 2 (DEV_TRIFASICO): Medidor de Energía Trifásico, FC 0x04, uint16/int32, low byte first ---
    {DEV_TRIFASICO, 0x0000, 1, 0, SENSOR_ID_VOLTAJE,                    false},  // V Fase A, uint16 (0.1V)   → bit 1
    {DEV_TRIFASICO, 0x0003, 1, 0, SENSOR_ID_CORRIENTE,                  false},  // I Fase A, uint16 (0.01A)  → bit 2
    {DEV_TRIFASICO, 0x000E, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 3),   true},   // P Activa A, int32 (0.1W)  → bit 6, LOW word first
    {DEV_TRIFASICO, 0x003A, 2, 0, SENSOR_ID_BATERIA,                    true},   // Energía Total, uint32 (0.1kWh) → bit 0, LOW word first
    {DEV_TRIFASICO, 0x0026, 2, 0, (uint8_t)(SENSOR_ID_EXT_START + 4),   false},  // PF A,B,C,Total (packed uint8×4, ×0.01) → bit 7
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
