#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <Arduino.h>
#include <cstdint>
#include <cstddef>

#include "ModbusDecode.h"

// =================================================================================================
// Timing
// =================================================================================================
constexpr uint32_t MODBUS_DEFAULT_TIMEOUT_MS = 500;  // timeout por bloque; ajustar tras medir
constexpr uint32_t MODBUS_BLOCK_GAP_MS       = 10;   // margen RS485 entre bloques
constexpr unsigned long POLL_INTERVAL_MS     = 30000; // cada 30 s se consulta todo el bus

// Presupuesto peor caso (1 device, 4 bloques): 4*500 + 3*10 = 2030 ms.
// Con kMaxDevices=8: ~16.3 s, por debajo de POLL_INTERVAL_MS.

// =================================================================================================
// RS485 bus configuration (global, all slaves on the same bus share these parameters)
// =================================================================================================
// Hardware notes:
// - The RS485 transceiver is auto-direction; no DE/RE GPIO is driven.
// - TX is GPIO12 (MTDI strapping): verify reliable cold boot.
// - RX is GPIO13, which is SD_CS on ttgo-lora32-v21new: do not use an SD card.
struct ModbusBusConfig {
    unsigned long baudRate;
    uint32_t      uartConfig;       // SERIAL_8N1, SERIAL_8E1, SERIAL_8O1, etc.
    int           rxPin;
    int           txPin;
    uint32_t      defaultTimeoutMs;
    int8_t        deRePin;          // -1 = transceptor auto-direccion
};

const ModbusBusConfig kBusCfg = {
    9600,            // baudRate
    SERIAL_8N1,      // uartConfig
    13,              // rxPin
    12,              // txPin
    MODBUS_DEFAULT_TIMEOUT_MS, // defaultTimeoutMs
    -1               // deRePin (auto-direccion)
};

// Decode helpers live in ModbusDecode.h (pure C++ for host tests).

#endif // MODBUS_CONFIG_H
