#ifndef SENSOR_REGISTRY_H
#define SENSOR_REGISTRY_H

#include <cstdint>
#include <array>

// =================================================================================================
// Symbolic sensor IDs.
// =================================================================================================
// IMPORTANT! These IDs are part of the wire format of the LoRa payload
// (see construirPayloadUnificado). They must NOT be reordered or reassigned
// without coordinating with the gateway/codec owner.

/**
 * @def SENSOR_ID_BATERIA
 * @brief Bit 0 of the Activate Byte.
 */
constexpr uint8_t SENSOR_ID_BATERIA = 0;

/**
 * @def SENSOR_ID_VOLTAJE
 * @brief Bit 1 of the Activate Byte.
 */
constexpr uint8_t SENSOR_ID_VOLTAJE = 1;

/**
 * @def SENSOR_ID_CORRIENTE
 * @brief Bit 2 of the Activate Byte.
 */
constexpr uint8_t SENSOR_ID_CORRIENTE = 2;

/**
 * @def SENSOR_ID_EXT_START
 * @brief Start of IDs for external sensors (Bits 3-7).
 */
constexpr uint8_t SENSOR_ID_EXT_START = 3;

/**
 * @def MAX_SENSORES_EXTERNOS
 * @brief Number of remaining available bits in the Activate Byte (3 to 7).
 */
constexpr int MAX_SENSORES_EXTERNOS = 5;

// =================================================================================================
// Named IDs for the external sensors (aliases of SENSOR_ID_EXT_START + n).
// =================================================================================================
// El orden coincide con los bits 3..7 del Activate Byte y con el codec del gateway:
//   EXT+0 = real_power, EXT+1 = apparent_power, EXT+2 = reactive_power,
//   EXT+3 = power_factor, EXT+4 = frequency.

/** @brief Potencia Activa Total. Bit 3 (EXT+0). */
constexpr uint8_t SENSOR_ID_POTENCIA_ACTIVA   = SENSOR_ID_EXT_START + 0;

/** @brief Potencia Aparente Total. Bit 4 (EXT+1). */
constexpr uint8_t SENSOR_ID_POTENCIA_APARENTE = SENSOR_ID_EXT_START + 1;

/** @brief Potencia Reactiva Total. Bit 5 (EXT+2). */
constexpr uint8_t SENSOR_ID_POTENCIA_REACTIVA = SENSOR_ID_EXT_START + 2;

/** @brief Factor de Potencia Combinado. Bit 6 (EXT+3). */
constexpr uint8_t SENSOR_ID_FACTOR_POTENCIA   = SENSOR_ID_EXT_START + 3;

/** @brief Frecuencia Fase A. Bit 7 (EXT+4). */
constexpr uint8_t SENSOR_ID_FRECUENCIA        = SENSOR_ID_EXT_START + 4;

// =================================================================================================
// Priority configuration.
// =================================================================================================
// Sensors listed here trigger immediate transmission in the aggregator task.
// Add or remove IDs here without touching the rest of the logic.

/**
 * @brief Sensors that act as "Bus Drivers" (i.e. trigger the send).
 */
constexpr std::array<uint8_t, 2> DEFINED_PRIORITY_IDS = {
    SENSOR_ID_VOLTAJE,
    SENSOR_ID_CORRIENTE
};

#endif // SENSOR_REGISTRY_H
