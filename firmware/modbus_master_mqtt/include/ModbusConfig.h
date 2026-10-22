#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <Arduino.h>
#include <cstdint>
#include <cstddef>

// =================================================================================================
// RS485 bus configuration (global, all slaves on the same bus share these parameters)
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
// Decode helpers - el dispositivo manda little-endian (byte bajo primero)
// =================================================================================================
// reg = offset en registros dentro del bloque.

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
// Timing
// =================================================================================================
constexpr unsigned long POLL_INTERVAL_MS = 30000;   // Cada 30 s se consulta todo el bus

#endif // MODBUS_CONFIG_H
