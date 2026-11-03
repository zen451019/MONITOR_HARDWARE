#ifndef SIMULATION_CONFIG_H
#define SIMULATION_CONFIG_H

// =================================================================================================
// Simulation mode switch.
// =================================================================================================
// Enabled only by the PlatformIO env `ttgo-lora32-v21-sim` (-DSIMULATION_MODE=1).
// Production builds keep doing real Modbus RTU reads.

#ifndef SIMULATION_MODE
#define SIMULATION_MODE 0
#endif

// =================================================================================================
// Nominal electrical values (three-phase meter model).
// =================================================================================================

constexpr float SIM_NOMINAL_VOLTAGE = 127.0f;   // V phase-neutral
constexpr float SIM_NOMINAL_CURRENT = 10.0f;    // A per phase
constexpr float SIM_NOMINAL_FREQ    = 60.0f;    // Hz

// =================================================================================================
// Alert thresholds used by the injected faults (must match the backend expectations).
// =================================================================================================

constexpr float SIM_VOLTAGE_NORMAL_MIN = 115.0f;
constexpr float SIM_VOLTAGE_NORMAL_MAX = 140.0f;
constexpr float SIM_CURRENT_ALERT      = 20.0f;   // A
constexpr float SIM_FREQ_NORMAL_MIN    = 59.5f;
constexpr float SIM_FREQ_NORMAL_MAX    = 60.5f;
constexpr float SIM_PF_ALERT           = 0.85f;

// =================================================================================================
// Fault scheduling.
// =================================================================================================

constexpr int SIM_FAULT_INTERVAL_MIN  = 8;    // cycles between faults
constexpr int SIM_FAULT_INTERVAL_MAX  = 15;
constexpr int SIM_FAULT_MIN_CYCLES    = 1;    // fault duration
constexpr int SIM_FAULT_MAX_CYCLES    = 3;

// Initial active energy counter (kWh).
constexpr float SIM_ENERGY_START_KWH = 12345.6f;

#endif // SIMULATION_CONFIG_H
