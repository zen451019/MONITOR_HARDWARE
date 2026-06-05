#include "loraconfig.h"

// =================================================================================================
// LoRaWAN ABP credentials (default values for the demo).
// Override at build time by defining these symbols in a separate compilation unit
// (e.g. a `loraconfig.local.cpp` excluded from git).
// =================================================================================================

u1_t NWKSKEY[16] = {
    0xC2, 0x5B, 0x0A, 0x78, 0xA8, 0x0A, 0x63, 0x1D,
    0x86, 0xC8, 0x1B, 0xA3, 0x3A, 0x9E, 0x36, 0xEF
};

u1_t APPSKEY[16] = {
    0x42, 0x8F, 0x67, 0xFA, 0xD7, 0xD7, 0x4A, 0x85,
    0x3C, 0x10, 0x80, 0x5F, 0x10, 0x1A, 0x0E, 0x14
};

const u4_t DEVADDR = 0x260C691F;

// =================================================================================================
// LMIC placeholder callbacks. Real OTAA values come from `os_getArtEui` etc.
// For ABP-only setups these can stay zeroed.
// =================================================================================================

void os_getArtEui(u1_t *buf) { memset(buf, 0, 8); }
void os_getDevEui(u1_t *buf) { memset(buf, 0, 8); }
void os_getDevKey(u1_t *buf) { memset(buf, 0, 16); }
