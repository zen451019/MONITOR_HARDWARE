#include <unity.h>

#include <cstdint>

#include "ModbusDecode.h"

void setUp(void) {}
void tearDown(void) {}

void test_mb_u16_little_endian(void) {
    const uint8_t d[] = {0x34, 0x12};
    TEST_ASSERT_EQUAL_UINT16(0x1234, mb_u16(d, sizeof(d), 0));
}

void test_mb_u16_register_offset(void) {
    const uint8_t d[] = {0x00, 0x11, 0x22, 0x33};
    TEST_ASSERT_EQUAL_UINT16(0x3322, mb_u16(d, sizeof(d), 1));
}

void test_mb_u16_bounds(void) {
    const uint8_t d[] = {0x34};
    TEST_ASSERT_EQUAL_UINT16(0, mb_u16(d, sizeof(d), 0));

    const uint8_t d2[] = {0x34, 0x12};
    TEST_ASSERT_EQUAL_UINT16(0, mb_u16(d2, sizeof(d2), 1));
}

void test_mb_i32_positive(void) {
    const uint8_t d[] = {0x78, 0x56, 0x34, 0x12};
    TEST_ASSERT_EQUAL_INT32(0x12345678, mb_i32(d, sizeof(d), 0));
}

void test_mb_i32_negative(void) {
    const uint8_t d[] = {0xFE, 0xFF, 0xFF, 0xFF};
    TEST_ASSERT_EQUAL_INT32(-2, mb_i32(d, sizeof(d), 0));
}

void test_mb_i32_min_max(void) {
    const uint8_t minv[] = {0x00, 0x00, 0x00, 0x80};
    const uint8_t maxv[] = {0xFF, 0xFF, 0xFF, 0x7F};
    TEST_ASSERT_EQUAL_INT32(INT32_MIN, mb_i32(minv, sizeof(minv), 0));
    TEST_ASSERT_EQUAL_INT32(INT32_MAX, mb_i32(maxv, sizeof(maxv), 0));
}

void test_mb_i32_word_order(void) {
    const uint8_t d[] = {0x00, 0x00, 0x01, 0x00};  // 0x00010000
    TEST_ASSERT_EQUAL_INT32(65536, mb_i32(d, sizeof(d), 0));
}

void test_mb_i32_bounds(void) {
    const uint8_t d[] = {0x01, 0x02, 0x03};
    TEST_ASSERT_EQUAL_INT32(0, mb_i32(d, sizeof(d), 0));
}

void test_mb_byte_in_bounds(void) {
    const uint8_t d[] = {0x10, 0x20, 0x30};
    TEST_ASSERT_EQUAL_UINT8(0x20, mb_byte(d, sizeof(d), 1));
}

void test_mb_byte_out_of_bounds(void) {
    const uint8_t d[] = {0x10, 0x20};
    TEST_ASSERT_EQUAL_UINT8(0, mb_byte(d, sizeof(d), sizeof(d)));
}

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(test_mb_u16_little_endian);
    RUN_TEST(test_mb_u16_register_offset);
    RUN_TEST(test_mb_u16_bounds);
    RUN_TEST(test_mb_i32_positive);
    RUN_TEST(test_mb_i32_negative);
    RUN_TEST(test_mb_i32_min_max);
    RUN_TEST(test_mb_i32_word_order);
    RUN_TEST(test_mb_i32_bounds);
    RUN_TEST(test_mb_byte_in_bounds);
    RUN_TEST(test_mb_byte_out_of_bounds);
    return UNITY_END();
}
