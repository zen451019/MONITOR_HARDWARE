#ifndef SPARKPLUG_CONFIG_H
#define SPARKPLUG_CONFIG_H

#include <cstdint>
#include <cstddef>

// =================================================================================================
// Sparkplug B datatypes (org.eclipse.tahu.protobuf.DataType)
// =================================================================================================
constexpr uint32_t SPB_DT_INT32   = 3;
constexpr uint32_t SPB_DT_INT64   = 4;
constexpr uint32_t SPB_DT_UINT32  = 7;
constexpr uint32_t SPB_DT_UINT64  = 8;
constexpr uint32_t SPB_DT_FLOAT   = 9;
constexpr uint32_t SPB_DT_DOUBLE  = 10;
constexpr uint32_t SPB_DT_BOOLEAN = 11;

// =================================================================================================
// Node metric names (reserved / health)
// =================================================================================================
constexpr const char* SPB_METRIC_BDSEQ     = "bdSeq";
constexpr const char* SPB_METRIC_REBIRTH   = "Node Control/Rebirth";
constexpr const char* SPB_METRIC_UPTIME    = "Uptime_ms";
constexpr const char* SPB_METRIC_RSSI      = "RSSI_dBm";
constexpr const char* SPB_METRIC_FREE_HEAP = "FreeHeap_bytes";
constexpr const char* SPB_METRIC_SCAN_OK   = "Modbus_ScanOK";

// =================================================================================================
// Alias ranges. Aliases MUST be unique across the whole Edge Node (node + all devices).
// =================================================================================================
constexpr uint16_t SPB_ALIAS_NODE_UPTIME    = 1;
constexpr uint16_t SPB_ALIAS_NODE_RSSI      = 2;
constexpr uint16_t SPB_ALIAS_NODE_FREE_HEAP = 3;
constexpr uint16_t SPB_ALIAS_NODE_SCAN_OK   = 4;
constexpr uint16_t SPB_ALIAS_NODE_BDSEQ     = 5;
constexpr uint16_t SPB_ALIAS_NODE_REBIRTH   = 6;
constexpr uint16_t SPB_NODE_ALIAS_END       = 6;

constexpr uint16_t SPB_DEVICE_ALIAS_STEP    = 20;   // deja espacio para futuras metricas

// =================================================================================================
// MQTT QoS policy
// =================================================================================================
constexpr uint8_t SPB_QOS_BIRTH  = 1;  // NBIRTH / DBIRTH: broker PUBACK + esp-mqtt retry
constexpr uint8_t SPB_QOS_DATA   = 0;  // NDATA / DDATA: telemetry, loss is acceptable
constexpr uint8_t SPB_QOS_DDEATH = 0;
constexpr uint8_t SPB_QOS_NDEATH = 1;  // LWT

// =================================================================================================
// Limits / timing
// =================================================================================================
constexpr size_t   SPB_MAX_METRICS        = 24;     // debe coincidir con sparkplug_b.options
constexpr size_t   SPB_MAX_PAYLOAD_BYTES  = 1536;
constexpr uint8_t  SPB_DEVICE_DEATH_CYCLES = 3;     // DDEATH tras 3 ciclos sin respuesta
constexpr uint32_t SPB_REBIRTH_DEBOUNCE_MS = 5000;

#endif // SPARKPLUG_CONFIG_H
