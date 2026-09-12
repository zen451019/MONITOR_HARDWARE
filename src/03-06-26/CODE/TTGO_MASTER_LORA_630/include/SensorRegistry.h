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
