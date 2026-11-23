#include "MultiDeviceSimulator.h"

#include <Arduino.h>
#include <cmath>
#include <cstring>
#include <ctime>

#include "PhantomMultiConfig.h"

enum SimFault : uint8_t {
    SIM_FAULT_NONE = 0,
    SIM_FAULT_SAG,
    SIM_FAULT_SWELL,
    SIM_FAULT_OVERCURRENT,
    SIM_FAULT_PF_DROP,
    SIM_FAULT_FREQ_DRIFT,
    SIM_FAULT_OVERLOAD
};

static float clampf(float value, float lo, float hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static uint32_t rngNext(uint32_t& state) {
    state = state * 1664525u + 1013904223u;
    return state;
}

static float randRange(uint32_t& state, float lo, float hi) {
    const float unit = (float)((rngNext(state) >> 8) & 0x00FFFFFFu) / 16777215.0f;
    return lo + (hi - lo) * unit;
}

static int randInt(uint32_t& state, int lo, int hi) {
    const uint32_t span = (uint32_t)(hi - lo + 1);
    return lo + (int)(rngNext(state) % span);
}

static void putU32BE(uint8_t* out, uint32_t value) {
    out[0] = (uint8_t)((value >> 24) & 0xFFu);
    out[1] = (uint8_t)((value >> 16) & 0xFFu);
    out[2] = (uint8_t)((value >> 8) & 0xFFu);
    out[3] = (uint8_t)(value & 0xFFu);
}

static void putI32BE(uint8_t* out, int32_t value) {
    putU32BE(out, (uint32_t)value);
}

static int32_t scaled(float value, float scale) {
    const float limit = 2147483000.0f / scale;
    if (value < 0.0f) value = 0.0f;
    if (value > limit) value = limit;
    return (int32_t)lroundf(value * scale);
}

static int32_t scaledDouble(double value, double scale) {
    const double limit = 2147483000.0 / scale;
    if (value < 0.0) value = 0.0;
    if (value > limit) value = limit;
    return (int32_t)llround(value * scale);
}

static void applyNormalWalk(PhantomDeviceState& st) {
    for (int k = 0; k < 3; ++k) {
        const float lo = fmaxf(st.vBase[k] - PHANTOM_VOLTAGE_WALK,
                               PHANTOM_VOLTAGE_NORMAL_MIN);
        const float hi = fminf(st.vBase[k] + PHANTOM_VOLTAGE_WALK,
                               PHANTOM_VOLTAGE_NORMAL_MAX);
        st.v[k] = clampf(st.v[k] + randRange(st.rng, -PHANTOM_VOLTAGE_STEP,
                                             PHANTOM_VOLTAGE_STEP),
                         lo, hi);
        st.i[k] = clampf(st.i[k] + randRange(st.rng, -0.10f, 0.10f),
                         st.iBase[k] * 0.70f, st.iBase[k] * 1.30f);
        st.pf[k] = clampf(st.pf[k] + randRange(st.rng, -0.004f, 0.004f), 0.88f, 0.99f);
    }

    st.freq = clampf(st.freq + randRange(st.rng, -0.015f, 0.015f),
                     SIM_FREQ_NORMAL_MIN + 0.2f, SIM_FREQ_NORMAL_MAX - 0.2f);
}

static void applyFaultEffect(PhantomDeviceState& st) {
    switch (st.fault) {
        case SIM_FAULT_SAG:
            for (int k = 0; k < 3; ++k) {
                st.v[k] = randRange(st.rng, PHANTOM_VOLTAGE_SAG_MIN,
                                    PHANTOM_VOLTAGE_SAG_MAX);
            }
            break;

        case SIM_FAULT_SWELL:
            for (int k = 0; k < 3; ++k) {
                st.v[k] = randRange(st.rng, PHANTOM_VOLTAGE_SWELL_MIN,
                                    PHANTOM_VOLTAGE_SWELL_MAX);
            }
            break;

        case SIM_FAULT_OVERCURRENT: {
            const int phase = randInt(st.rng, 0, 2);
            float base = st.iBase[phase] * 1.8f;
            if (base < SIM_CURRENT_ALERT + 1.0f) base = SIM_CURRENT_ALERT + 1.0f;
            st.i[phase] = randRange(st.rng, base, base + 6.0f);
            break;
        }

        case SIM_FAULT_PF_DROP:
            for (int k = 0; k < 3; ++k) {
                st.pf[k] = randRange(st.rng, 0.55f, 0.75f);
            }
            break;

        case SIM_FAULT_FREQ_DRIFT:
            st.freq = (randInt(st.rng, 0, 1) == 0) ? randRange(st.rng, 58.0f, 59.2f)
                                                   : randRange(st.rng, 60.8f, 62.0f);
            break;

        case SIM_FAULT_OVERLOAD:
            for (int k = 0; k < 3; ++k) {
                float base = st.iBase[k] * 1.8f;
                if (base < 18.0f) base = 18.0f;
                st.i[k] = randRange(st.rng, base, base + 6.0f);
                st.v[k] = randRange(st.rng, PHANTOM_VOLTAGE_OVERLOAD_MIN,
                                    PHANTOM_VOLTAGE_OVERLOAD_MAX);
            }
            break;

        default:
            break;
    }
}

static void recoverAfterFault(PhantomDeviceState& st) {
    for (int k = 0; k < 3; ++k) {
        st.v[k] = st.vBase[k] + randRange(st.rng, -0.3f, 0.3f);
        st.i[k] = st.iBase[k] + randRange(st.rng, -0.2f, 0.2f);
        st.pf[k] = clampf(st.pfBase[k] + randRange(st.rng, -0.005f, 0.005f), 0.88f, 0.99f);
    }
    st.freq = SIM_NOMINAL_FREQ + randRange(st.rng, -0.02f, 0.02f);
}

static void computeDerived(PhantomDeviceState& st) {
    float pTotal = 0.0f;
    float qTotal = 0.0f;
    float sTotal = 0.0f;

    for (int k = 0; k < 3; ++k) {
        const float pf = clampf(st.pf[k], 0.0f, 0.999f);
        const float apparent = st.v[k] * st.i[k];
        const float active = apparent * pf;
        const float reactive = apparent * sqrtf(1.0f - pf * pf);

        pTotal += active;
        qTotal += reactive;
        sTotal += apparent;
    }

    st.pTotal = pTotal;
    st.qTotal = qTotal;
    st.sTotal = sTotal;
    st.pfTotal = (sTotal > 0.0f) ? (pTotal / sTotal) : 0.0f;
}

static void startFault(PhantomDeviceState& st) {
    st.fault = (uint8_t)(SIM_FAULT_SAG + randInt(st.rng, 0, 5));
    st.faultCyclesLeft = (uint8_t)randInt(st.rng, SIM_FAULT_MIN_CYCLES, SIM_FAULT_MAX_CYCLES);
}

void simInitAll(PhantomDeviceState* states, size_t count, uint32_t nowMs) {
    if (!states || count == 0) return;

    for (size_t idx = 0; idx < count; ++idx) {
        PhantomDeviceState& st = states[idx];
        memset(&st, 0, sizeof(st));

        st.rng = 0x9E3779B9u ^ ((uint32_t)idx * 2654435761u) ^ 0x1234567u;

        const float load = 5.0f + (float)(idx % 20) * 2.0f;

        for (int k = 0; k < 3; ++k) {
            st.vBase[k] = PHANTOM_VOLTAGE_NOMINAL
                        + randRange(st.rng, -PHANTOM_VOLTAGE_PHASE_SPREAD,
                                    PHANTOM_VOLTAGE_PHASE_SPREAD);
            st.iBase[k] = load + randRange(st.rng, -0.8f, 0.8f);
            st.pfBase[k] = 0.90f + randRange(st.rng, 0.0f, 0.08f);

            st.v[k] = st.vBase[k] + randRange(st.rng, -PHANTOM_VOLTAGE_INITIAL_SPREAD,
                                              PHANTOM_VOLTAGE_INITIAL_SPREAD);
            st.i[k] = st.iBase[k] + randRange(st.rng, -0.3f, 0.3f);
            st.pf[k] = clampf(st.pfBase[k] + randRange(st.rng, -0.005f, 0.005f), 0.88f, 0.99f);
        }

        st.freq = SIM_NOMINAL_FREQ + randRange(st.rng, -0.03f, 0.03f);
        st.energyKwh = 1000.0 + (double)idx * 137.5 + (double)randRange(st.rng, 0.0f, 100.0f);
        st.fault = SIM_FAULT_NONE;
        st.faultCyclesLeft = 0;
        st.cyclesUntilFault = (uint16_t)randInt(st.rng, SIM_FAULT_INTERVAL_MIN, SIM_FAULT_INTERVAL_MAX);
        st.msgId = (uint8_t)(idx * 7u);

        const int32_t jitter = randInt(st.rng, -(int)PHANTOM_INTERVAL_JITTER_MS,
                                       (int)PHANTOM_INTERVAL_JITTER_MS);
        st.intervalMs = (uint32_t)((int32_t)PHANTOM_DEVICE_INTERVAL_MS + jitter);
        if (st.intervalMs < 5000u) st.intervalMs = 5000u;

        st.lastCycleMs = nowMs - st.intervalMs;
        st.nextDueMs = nowMs + (uint32_t)(((uint64_t)idx * st.intervalMs) / count)
                     + (uint32_t)randInt(st.rng, 0, 2000);

        computeDerived(st);
    }
}

void simAdvance(PhantomDeviceState& st, uint32_t dtMs) {
    if (dtMs > 3600000u) dtMs = 3600000u;

    if (st.fault == SIM_FAULT_NONE) {
        applyNormalWalk(st);

        if (st.cyclesUntilFault > 0) --st.cyclesUntilFault;
        if (st.cyclesUntilFault == 0) {
            startFault(st);
            applyFaultEffect(st);
        }
    } else {
        applyNormalWalk(st);
        applyFaultEffect(st);

        if (st.faultCyclesLeft > 0 && --st.faultCyclesLeft == 0) {
            st.fault = SIM_FAULT_NONE;
            recoverAfterFault(st);
            st.cyclesUntilFault =
                (uint16_t)randInt(st.rng, SIM_FAULT_INTERVAL_MIN, SIM_FAULT_INTERVAL_MAX);
        }
    }

    computeDerived(st);
    st.energyKwh += (double)st.pTotal * ((double)dtMs / 1000.0) / 3600000.0;
}

size_t simBuildPayload(const PhantomDeviceState& st, uint8_t* out, size_t maxLen) {
    if (!out || maxLen < 62) return 0;

    size_t pos = 0;

    out[pos++] = st.msgId;

    uint32_t ts = (uint32_t)time(nullptr);
    if (ts == 0u || ts == 0xFFFFFFFFu) {
        ts = PHANTOM_TIMESTAMP_BASE + (uint32_t)(millis() / 1000UL);
    }
    putU32BE(out + pos, ts);
    pos += 4;

    out[pos++] = 0xFFu;

    static const uint8_t lens[8] = {1, 3, 3, 1, 1, 1, 1, 1};
    memcpy(out + pos, lens, sizeof(lens));
    pos += sizeof(lens);

    putI32BE(out + pos, scaledDouble(st.energyKwh, 10.0));
    pos += 4;

    for (int k = 0; k < 3; ++k) {
        putI32BE(out + pos, scaled(st.v[k], 10.0f));
        pos += 4;
    }

    for (int k = 0; k < 3; ++k) {
        putI32BE(out + pos, scaled(st.i[k], 100.0f));
        pos += 4;
    }

    putI32BE(out + pos, scaled(st.pTotal, 10.0f));
    pos += 4;

    putI32BE(out + pos, scaled(st.sTotal, 10.0f));
    pos += 4;

    putI32BE(out + pos, scaled(st.qTotal, 10.0f));
    pos += 4;

    const uint16_t combined =
        (uint16_t)clampf((float)lroundf(st.pfTotal * 100.0f), 0.0f, 100.0f);
    const uint16_t phaseC =
        (uint16_t)clampf((float)lroundf(st.pf[2] * 100.0f), 0.0f, 100.0f);
    out[pos++] = 0x00u;
    out[pos++] = 0x00u;
    out[pos++] = (uint8_t)(phaseC & 0xFFu);
    out[pos++] = (uint8_t)(combined & 0xFFu);

    putI32BE(out + pos, scaled(st.freq, 100.0f));
    pos += 4;

    if (pos > maxLen) return 0;
    return pos;
}
