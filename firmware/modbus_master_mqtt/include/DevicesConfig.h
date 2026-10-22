#ifndef DEVICES_CONFIG_H
#define DEVICES_CONFIG_H

#include <cstdint>
#include <cstddef>

#include "ModbusConfig.h"
#include "SensorRegistry.h"
#include "SparkplugConfig.h"

// Una peticion Modbus = un bloque de registros contiguos de UN Device.
struct ModbusBlock {
    uint8_t  functionCode;   // 0x03 = holding, 0x04 = input
    uint16_t startAddr;
    uint16_t numRegs;
};

// Una senal decodificada dentro de un bloque del Device.
struct ModbusSignal {
    uint8_t sensorType;      // SENSOR_ID_*
    uint8_t channel;
    uint8_t blockIndex;      // indice DENTRO de DeviceConfig.blocks
    float (*decode)(const uint8_t* data, size_t len);
};

// Mapeo de una senal Modbus a una metrica Sparkplug del Device.
struct SpbMetricDef {
    uint8_t     sensorType;
    uint8_t     channel;
    uint32_t    datatype;    // SPB_DT_*
    uint16_t    offset;      // alias = device.aliasBase + offset
    const char* name;
};

struct DeviceConfig {
    const char* deviceId;    // Sparkplug Device ID
    uint8_t     slaveId;     // Modbus slave address
    uint16_t    aliasBase;   // primer alias del Device
    uint32_t    timeoutMs;

    const ModbusBlock*  blocks;
    size_t              blockCount;

    const ModbusSignal* signals;
    size_t              signalCount;

    const SpbMetricDef* metrics;
    size_t              metricCount;
};

// =================================================================================================
// Modelo "medidor trifasico" (reutilizable por varios Devices)
// =================================================================================================
constexpr ModbusBlock kMeterBlocks[] = {
    // Bloque 0: V A/B/C (0x0000-0x0002) + I A/B/C (0x0003-0x0005) + Frecuencia A/B/C (0x0006-0x0008)
    {0x04, 0x0000, 9},
    // Bloque 1: Potencias totales (int32): Activa 0x0020 / Reactiva 0x0022 / Aparente 0x0024
    {0x04, 0x0020, 6},
    // Bloque 2: Factor de potencia (0x0026: A|B, 0x0027: C|Combinado)
    {0x04, 0x0026, 2},
    // Bloque 3: Energia activa total 0x003A (int32)
    {0x04, 0x003A, 2},
};

const ModbusSignal kMeterSignals[] = {
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
    {SENSOR_ID_FACTOR_POTENCIA, 0, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 1) * 0.01f; }}, // Fase A
    {SENSOR_ID_FACTOR_POTENCIA, 1, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 0) * 0.01f; }}, // Fase B
    {SENSOR_ID_FACTOR_POTENCIA, 2, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 3) * 0.01f; }}, // Fase C
    {SENSOR_ID_FACTOR_POTENCIA, 3, 2, [](const uint8_t* d, size_t n) { return mb_byte(d, n, 2) * 0.01f; }}, // Combinado

    // ---- Energia Activa Total (int32, x0.1) @0x003A ----
    {SENSOR_ID_BATERIA, 0, 3, [](const uint8_t* d, size_t n) { return mb_i32(d, n, 0) * 0.1f; }},
};

constexpr SpbMetricDef kMeterMetrics[] = {
    {SENSOR_ID_VOLTAJE,          0, SPB_DT_FLOAT,  0, "Voltage_L1"},
    {SENSOR_ID_VOLTAJE,          1, SPB_DT_FLOAT,  1, "Voltage_L2"},
    {SENSOR_ID_VOLTAJE,          2, SPB_DT_FLOAT,  2, "Voltage_L3"},
    {SENSOR_ID_CORRIENTE,        0, SPB_DT_FLOAT,  3, "Current_L1"},
    {SENSOR_ID_CORRIENTE,        1, SPB_DT_FLOAT,  4, "Current_L2"},
    {SENSOR_ID_CORRIENTE,        2, SPB_DT_FLOAT,  5, "Current_L3"},
    {SENSOR_ID_FRECUENCIA,       0, SPB_DT_FLOAT,  6, "Frequency_L1"},
    {SENSOR_ID_FRECUENCIA,       1, SPB_DT_FLOAT,  7, "Frequency_L2"},
    {SENSOR_ID_FRECUENCIA,       2, SPB_DT_FLOAT,  8, "Frequency_L3"},
    {SENSOR_ID_POTENCIA_ACTIVA,  0, SPB_DT_FLOAT,  9, "ActivePower_Total"},
    {SENSOR_ID_POTENCIA_REACTIVA,0, SPB_DT_FLOAT, 10, "ReactivePower_Total"},
    {SENSOR_ID_POTENCIA_APARENTE,0, SPB_DT_FLOAT, 11, "ApparentPower_Total"},
    {SENSOR_ID_FACTOR_POTENCIA,  0, SPB_DT_FLOAT, 12, "PowerFactor_L1"},
    {SENSOR_ID_FACTOR_POTENCIA,  1, SPB_DT_FLOAT, 13, "PowerFactor_L2"},
    {SENSOR_ID_FACTOR_POTENCIA,  2, SPB_DT_FLOAT, 14, "PowerFactor_L3"},
    {SENSOR_ID_FACTOR_POTENCIA,  3, SPB_DT_FLOAT, 15, "PowerFactor_Total"},
    {SENSOR_ID_BATERIA,          0, SPB_DT_FLOAT, 16, "ActiveEnergy_Total"},
};

template <typename T, size_t N>
constexpr size_t arrayCount(const T (&)[N]) { return N; }

// =================================================================================================
// Tabla de Devices. Agregar un medidor nuevo = agregar una entrada aqui.
// =================================================================================================
constexpr DeviceConfig kDevices[] = {
    {
        "medidor_trifasico_1",     // deviceId
        1,                         // slaveId
        10,                        // aliasBase (10..26)
        2000,                      // timeoutMs
        kMeterBlocks,  arrayCount(kMeterBlocks),
        kMeterSignals, arrayCount(kMeterSignals),
        kMeterMetrics, arrayCount(kMeterMetrics),
    },
};

constexpr size_t  kDeviceCount = arrayCount(kDevices);
constexpr uint8_t kMaxDevices  = 8;

static_assert(kDeviceCount <= kMaxDevices, "Demasiados Devices para kMaxDevices");
static_assert(kDeviceCount > 0, "Debe existir al menos un Device");
static_assert(kDevices[0].aliasBase > SPB_NODE_ALIAS_END,
              "aliasBase del primer Device debe ser mayor que los alias del nodo");
static_assert(arrayCount(kMeterMetrics) <= SPB_MAX_METRICS,
              "kMeterMetrics excede SPB_MAX_METRICS");

#endif // DEVICES_CONFIG_H
