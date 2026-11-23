#ifndef MULTI_DEVICE_SIMULATOR_H
#define MULTI_DEVICE_SIMULATOR_H

#include <cstddef>
#include <cstdint>

struct PhantomDeviceState {
    uint32_t rng;
    float vBase[3];
    float iBase[3];
    float pfBase[3];
    float v[3];
    float i[3];
    float pf[3];
    float freq;
    double energyKwh;
    float pTotal;
    float qTotal;
    float sTotal;
    float pfTotal;
    uint8_t fault;
    uint8_t faultCyclesLeft;
    uint16_t cyclesUntilFault;
    uint8_t msgId;
    uint32_t lastCycleMs;
    uint32_t nextDueMs;
    uint32_t intervalMs;
};

void simInitAll(PhantomDeviceState* states, size_t count, uint32_t nowMs);
void simAdvance(PhantomDeviceState& st, uint32_t dtMs);
size_t simBuildPayload(const PhantomDeviceState& st, uint8_t* out, size_t maxLen);

#endif // MULTI_DEVICE_SIMULATOR_H
