#ifndef SENSOR_SIMULATOR_H
#define SENSOR_SIMULATOR_H

#include <cstdint>
#include "ModbusAPI.h"
#include "SimulationConfig.h"

// =================================================================================================
// Modbus register simulator.
// =================================================================================================
// Generates raw register bytes with the exact wire encoding of the three-phase meter
// (low byte first per register, low word first for 32-bit values). The existing polling
// pipeline (swapWords, padding, swapWordOrder) then produces the same payload as real reads.

/**
 * @brief Initializes the simulated electrical state.
 */
void sim_init();

/**
 * @brief Advances the physical model by one polling cycle.
 * @param dtMs Elapsed milliseconds since the previous cycle (used for energy integration).
 */
void sim_begin_cycle(uint32_t dtMs);

/**
 * @brief Returns simulated register bytes for a Modbus read request.
 * @details The values are shared by all requests within the same cycle, so voltage,
 *          current, power, PF and energy stay coherent with each other.
 */
ModbusApiResult sim_read_registers(uint8_t slaveId, uint8_t functionCode,
                                   uint16_t startAddr, uint16_t numRegs);

#endif // SENSOR_SIMULATOR_H
