#include "SensorSimulator.h"

#if SIMULATION_MODE

#include <Arduino.h>
#include <cmath>
#include "Log.h"

// =================================================================================================
// Internal state
// =================================================================================================

enum class SimFault : uint8_t {
    NONE = 0,
    SAG,
    SWELL,
    OVERCURRENT,
    PF_DROP,
    FREQ_DRIFT,
    OVERLOAD
};

static const float kVoltageBase[3] = {127.0f, 127.3f, 126.8f};
static const float kCurrentBase[3] = { 9.8f,  10.2f,   9.9f};
static const float kPfBase[3]      = { 0.95f,  0.96f,   0.94f};

static float   s_voltage[3];
static float   s_current[3];
static float   s_pf[3];
static float   s_freq;
static double  s_energyKwh;

static float   s_pTotal;
static float   s_qTotal;
static float   s_sTotal;
static float   s_pfTotal;

static SimFault s_fault = SimFault::NONE;
static uint8_t  s_faultCyclesLeft = 0;
static uint16_t s_cyclesUntilFault = 0;

// =================================================================================================
// Helpers
// =================================================================================================

static float clampf(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static float randRange(float lo, float hi) {
    return lo + (hi - lo) * ((float)random(0, 10001) / 10000.0f);
}

static const char* faultName(SimFault fault) {
    switch (fault) {
        case SimFault::SAG:         return "SAG";
        case SimFault::SWELL:       return "SWELL";
        case SimFault::OVERCURRENT: return "OVERCURRENT";
        case SimFault::PF_DROP:     return "PF_DROP";
        case SimFault::FREQ_DRIFT:  return "FREQ_DRIFT";
        case SimFault::OVERLOAD:    return "OVERLOAD";
        default:                    return "NONE";
    }
}

static void putReg16(ModbusApiResult& r, size_t offset, uint16_t value) {
    r.data[offset]     = (uint8_t)(value & 0xFF);
    r.data[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

// The meter sends 32-bit values low-word first, and each word low-byte first.
static void putReg32(ModbusApiResult& r, size_t offset, int32_t value) {
    const uint32_t u  = (uint32_t)value;
    const uint16_t lo = (uint16_t)(u & 0xFFFF);
    const uint16_t hi = (uint16_t)(u >> 16);
    putReg16(r, offset,     lo);
    putReg16(r, offset + 2, hi);
}

static uint16_t toU16(float value, float scale) {
    float scaled = value * scale;
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 65535.0f) scaled = 65535.0f;
    return (uint16_t)std::lroundf(scaled);
}

// =================================================================================================
// Physical model
// =================================================================================================

static void applyNormalWalk() {
    for (int k = 0; k < 3; ++k) {
        s_voltage[k] = clampf(s_voltage[k] + randRange(-0.20f, 0.20f),
                              kVoltageBase[k] - 4.0f, kVoltageBase[k] + 4.0f);
        s_current[k] = clampf(s_current[k] + randRange(-0.10f, 0.10f),
                              kCurrentBase[k] * 0.70f, kCurrentBase[k] * 1.30f);
        s_pf[k]      = clampf(s_pf[k] + randRange(-0.004f, 0.004f), 0.88f, 0.99f);
    }

    s_freq = clampf(s_freq + randRange(-0.015f, 0.015f),
                    SIM_FREQ_NORMAL_MIN + 0.2f, SIM_FREQ_NORMAL_MAX - 0.2f);
}

static void applyFaultEffect() {
    switch (s_fault) {
        case SimFault::SAG:
            for (int k = 0; k < 3; ++k) {
                s_voltage[k] = randRange(106.0f, 112.0f);
            }
            break;

        case SimFault::SWELL:
            for (int k = 0; k < 3; ++k) {
                s_voltage[k] = randRange(143.0f, 149.0f);
            }
            break;

        case SimFault::OVERCURRENT: {
            const int phase = (int)random(0, 3);
            s_current[phase] = randRange(SIM_CURRENT_ALERT + 1.0f,
                                         SIM_CURRENT_ALERT + 6.0f);
            break;
        }

        case SimFault::PF_DROP:
            for (int k = 0; k < 3; ++k) {
                s_pf[k] = randRange(0.55f, 0.75f);
            }
            break;

        case SimFault::FREQ_DRIFT:
            s_freq = (random(0, 2) == 0) ? randRange(58.0f, 59.2f)
                                         : randRange(60.8f, 62.0f);
            break;

        case SimFault::OVERLOAD:
            for (int k = 0; k < 3; ++k) {
                s_current[k] = randRange(18.0f, 24.0f);
                s_voltage[k] = randRange(118.0f, 124.0f);
            }
            break;

        default:
            break;
    }
}

static void recoverAfterFault() {
    for (int k = 0; k < 3; ++k) {
        s_voltage[k] = kVoltageBase[k] + randRange(-0.3f, 0.3f);
        s_current[k] = kCurrentBase[k] + randRange(-0.2f, 0.2f);
        s_pf[k]      = clampf(kPfBase[k] + randRange(-0.005f, 0.005f), 0.88f, 0.99f);
    }
    s_freq = SIM_NOMINAL_FREQ + randRange(-0.02f, 0.02f);
}

static void computeDerived() {
    float pTotal = 0.0f;
    float qTotal = 0.0f;
    float sTotal = 0.0f;

    for (int k = 0; k < 3; ++k) {
        const float pf      = clampf(s_pf[k], 0.0f, 0.999f);
        const float apparent = s_voltage[k] * s_current[k];
        const float active   = apparent * pf;
        const float reactive = apparent * sqrtf(1.0f - pf * pf);

        pTotal += active;
        qTotal += reactive;
        sTotal += apparent;
    }

    s_pTotal  = pTotal;
    s_qTotal  = qTotal;
    s_sTotal  = sTotal;
    s_pfTotal = (sTotal > 0.0f) ? (pTotal / sTotal) : 0.0f;
}

static void startFault() {
    s_fault = (SimFault)(1 + (int)random(0, 6));   // SAG..OVERLOAD
    s_faultCyclesLeft = (uint8_t)random(SIM_FAULT_MIN_CYCLES, SIM_FAULT_MAX_CYCLES + 1);
    LOG_W("[SIM] Anomalia activada: %s (%u ciclos)",
          faultName(s_fault), (unsigned)s_faultCyclesLeft);
}

// =================================================================================================
// Public API
// =================================================================================================

void sim_init() {
    randomSeed((unsigned long)(micros() ^ (uint32_t)millis()));

    for (int k = 0; k < 3; ++k) {
        s_voltage[k] = kVoltageBase[k] + randRange(-0.4f, 0.4f);
        s_current[k] = kCurrentBase[k] + randRange(-0.3f, 0.3f);
        s_pf[k]      = clampf(kPfBase[k] + randRange(-0.005f, 0.005f), 0.88f, 0.99f);
    }
    s_freq      = SIM_NOMINAL_FREQ + randRange(-0.03f, 0.03f);
    s_energyKwh = SIM_ENERGY_START_KWH;

    s_fault            = SimFault::NONE;
    s_faultCyclesLeft  = 0;
    s_cyclesUntilFault = (uint16_t)random(SIM_FAULT_INTERVAL_MIN, SIM_FAULT_INTERVAL_MAX + 1);

    computeDerived();

    LOG_I("[SIM] Estado inicial: V=%.1f/%.1f/%.1f V, I=%.2f/%.2f/%.2f A, PF=%.2f, F=%.2f Hz, E=%.1f kWh",
          s_voltage[0], s_voltage[1], s_voltage[2],
          s_current[0], s_current[1], s_current[2],
          s_pfTotal, s_freq, s_energyKwh);
}

void sim_begin_cycle(uint32_t dtMs) {
    if (dtMs > 3600000UL) dtMs = 3600000UL;   // clamp absurd deltas

    if (s_fault == SimFault::NONE) {
        applyNormalWalk();

        if (s_cyclesUntilFault > 0) --s_cyclesUntilFault;
        if (s_cyclesUntilFault == 0) {
            startFault();
            applyFaultEffect();
        }
    } else {
        applyNormalWalk();
        applyFaultEffect();

        if (s_faultCyclesLeft > 0 && --s_faultCyclesLeft == 0) {
            s_fault = SimFault::NONE;
            recoverAfterFault();
            s_cyclesUntilFault =
                (uint16_t)random(SIM_FAULT_INTERVAL_MIN, SIM_FAULT_INTERVAL_MAX + 1);
            LOG_I("[SIM] Anomalia finalizada, valores recuperados.");
        }
    }

    computeDerived();

    // Energy integration: W * s / 3.6e6 = kWh
    s_energyKwh += (double)s_pTotal * ((double)dtMs / 1000.0) / 3600000.0;

    LOG_I("[SIM] V=%.1f/%.1f/%.1f V I=%.2f/%.2f/%.2f A PF=%.3f P=%.0f W Q=%.0f var S=%.0f VA F=%.2f Hz E=%.1f kWh [%s]",
          s_voltage[0], s_voltage[1], s_voltage[2],
          s_current[0], s_current[1], s_current[2],
          s_pfTotal, s_pTotal, s_qTotal, s_sTotal,
          s_freq, s_energyKwh, faultName(s_fault));
}

ModbusApiResult sim_read_registers(uint8_t slaveId, uint8_t functionCode,
                                   uint16_t startAddr, uint16_t numRegs) {
    (void)functionCode;

    ModbusApiResult r{};
    r.slave_id   = slaveId;
    r.error_code = ModbusApiError::SUCCESS;
    r.data_len   = (size_t)numRegs * 2;

    if (r.data_len > MODBUS_API_MAX_DATA_SIZE) {
        r.error_code = ModbusApiError::ERROR_INVALID_PARAMS;
        r.data_len   = 0;
        return r;
    }

    const int32_t pRaw  = (int32_t)std::lroundf(s_pTotal * 10.0f);
    const int32_t qRaw  = (int32_t)std::lroundf(s_qTotal * 10.0f);
    const int32_t sRaw  = (int32_t)std::lroundf(s_sTotal * 10.0f);
    const int32_t eRaw  = (int32_t)std::lroundf((float)(s_energyKwh * 10.0));

    switch (startAddr) {
        // Voltaje Fase A/B/C (u16, 0.1 V)
        case 0x0000:
            putReg16(r, 0, toU16(s_voltage[0], 10.0f));
            break;
        case 0x0001:
            putReg16(r, 0, toU16(s_voltage[1], 10.0f));
            break;
        case 0x0002:
            putReg16(r, 0, toU16(s_voltage[2], 10.0f));
            break;

        // Corriente Fase A/B/C (u16, 0.01 A)
        case 0x0003:
            putReg16(r, 0, toU16(s_current[0], 100.0f));
            break;
        case 0x0004:
            putReg16(r, 0, toU16(s_current[1], 100.0f));
            break;
        case 0x0005:
            putReg16(r, 0, toU16(s_current[2], 100.0f));
            break;

        // Frecuencia Fase A (u16, 0.01 Hz)
        case 0x0006:
            putReg16(r, 0, toU16(s_freq, 100.0f));
            break;

        // Potencia activa total (int32, 0.1 W, low word first)
        case 0x0020:
            putReg32(r, 0, pRaw);
            break;

        // Potencia reactiva total (int32, 0.1 var, low word first)
        case 0x0022:
            putReg32(r, 0, qRaw);
            break;

        // Potencia aparente total (int32, 0.1 VA, low word first)
        case 0x0024:
            putReg32(r, 0, sRaw);
            break;

        // Factor de potencia: High byte = Phase C, Low byte = Combined (0.01)
        case 0x0027: {
            const uint16_t pfCombined = (uint16_t)clampf(std::lroundf(s_pfTotal * 100.0f), 0.0f, 100.0f);
            const uint16_t pfPhaseC   = (uint16_t)clampf(std::lroundf(s_pf[2] * 100.0f), 0.0f, 100.0f);
            putReg16(r, 0, (uint16_t)((pfPhaseC << 8) | pfCombined));
            break;
        }

        // Energía activa total (int32, 0.1 kWh, low word first)
        case 0x003A:
            putReg32(r, 0, eRaw);
            break;

        default:
            r.error_code = ModbusApiError::ERROR_INVALID_PARAMS;
            r.data_len   = 0;
            LOG_W("[SIM] Registro simulado no soportado: 0x%04X", startAddr);
            break;
    }

    return r;
}

#endif // SIMULATION_MODE
