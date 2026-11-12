#ifndef MODBUS_DECODE_H
#define MODBUS_DECODE_H

#include <cstdint>
#include <cstddef>

// Decode helpers for the meter's little-endian register payload.
// Pure C++ on purpose: they are unit-tested on the host (env:native).
// reg = offset in registers inside the block.

static inline uint16_t mb_u16(const uint8_t* d, size_t len, uint16_t reg) {
    size_t o = (size_t)reg * 2;
    if (o + 1 >= len) return 0;
    return (uint16_t)(d[o] | (d[o + 1] << 8));
}

// int32 from register 'reg' (low word first, little-endian).
static inline int32_t mb_i32(const uint8_t* d, size_t len, uint16_t reg) {
    size_t o = (size_t)reg * 2;
    if (o + 3 >= len) return 0;
    uint32_t u = (uint32_t)d[o] | ((uint32_t)d[o + 1] << 8) |
                 ((uint32_t)d[o + 2] << 16) | ((uint32_t)d[o + 3] << 24);
    return (int32_t)u;
}

// Raw byte at position 'i' of the buffer.
static inline uint8_t mb_byte(const uint8_t* d, size_t len, uint16_t i) {
    return (i < len) ? d[i] : (uint8_t)0;
}

#endif // MODBUS_DECODE_H
