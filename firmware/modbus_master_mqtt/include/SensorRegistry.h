#ifndef SENSOR_REGISTRY_H
#define SENSOR_REGISTRY_H

#include <cstdint>
#include <array>

// =================================================================================================
// Logical sensor IDs.
// =================================================================================================
// These IDs are indexes used by ModbusSignal::sensorType and
// SpbMetricDef::sensorType in this firmware. They are not a LoRa wire format.
// Keep existing values stable; DevicesConfig.h groups signals and metrics by them.

/** @brief Logical type for energy/battery. */
constexpr uint8_t SENSOR_ID_BATERIA = 0;

/** @brief Logical type for voltage. */
constexpr uint8_t SENSOR_ID_VOLTAJE = 1;

/** @brief Logical type for current. */
constexpr uint8_t SENSOR_ID_CORRIENTE = 2;

/** @brief First logical type available for external sensor groups. */
constexpr uint8_t SENSOR_ID_EXT_START = 3;

/** @brief Number of available logical slots for external groups (3..7). */
constexpr int MAX_SENSORES_EXTERNOS = 5;

// =================================================================================================
// Named IDs for the external sensor groups (aliases of SENSOR_ID_EXT_START + n).
// =================================================================================================
// The order is a firmware convention; it is not tied to a LoRa codec.

/** @brief Active power total. Logical type 3. */
constexpr uint8_t SENSOR_ID_POTENCIA_ACTIVA   = SENSOR_ID_EXT_START + 0;

/** @brief Apparent power total. Logical type 4. */
constexpr uint8_t SENSOR_ID_POTENCIA_APARENTE = SENSOR_ID_EXT_START + 1;

/** @brief Reactive power total. Logical type 5. */
constexpr uint8_t SENSOR_ID_POTENCIA_REACTIVA = SENSOR_ID_EXT_START + 2;

/** @brief Combined power factor. Logical type 6. */
constexpr uint8_t SENSOR_ID_FACTOR_POTENCIA   = SENSOR_ID_EXT_START + 3;

/** @brief Frequency phase A. Logical type 7. */
constexpr uint8_t SENSOR_ID_FRECUENCIA        = SENSOR_ID_EXT_START + 4;

// =================================================================================================
// Legacy priority list (LoRa variant). Not used by this firmware.
// =================================================================================================
constexpr std::array<uint8_t, 2> DEFINED_PRIORITY_IDS = {
    SENSOR_ID_VOLTAJE,
    SENSOR_ID_CORRIENTE
};

#endif // SENSOR_REGISTRY_H
