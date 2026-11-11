#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdint.h>

#include "drivers/bmb3y/bmb3y_temps.h"

// The Tesla firmware's table and algorithm, as extracted: clamp at the ends,
// binary-search the bracket, interpolate.
typedef struct { uint16_t raw; int16_t dC; } ntc_point_t;

static const ntc_point_t FIRMWARE_TABLE[] = {
    {     0, -600 }, {   439, -400 }, {   572, -350 }, {   737, -300 },
    {   942, -250 }, {  1193, -200 }, {  1498, -150 }, {  1863, -100 },
    {  2295,  -50 }, {  2800,    0 }, {  3385,   50 }, {  4051,  100 },
    {  4801,  150 }, {  5633,  200 }, {  6542,  250 }, {  7521,  300 },
    {  8560,  350 }, {  9646,  400 }, { 10765,  450 }, { 11901,  500 },
    { 13038,  550 }, { 14163,  600 }, { 15261,  650 }, { 16320,  700 },
    { 17332,  750 }, { 18289,  800 }, { 19186,  850 }, { 20023,  900 },
    { 20795,  950 }, { 21507, 1000 }, { 22158, 1050 }, { 22752, 1100 },
    { 23291, 1150 }, { 65535, 3000 },
};
#define FIRMWARE_TABLE_LEN (int)(sizeof(FIRMWARE_TABLE) / sizeof(FIRMWARE_TABLE[0]))

static int16_t firmware_raw_to_dC(uint16_t raw) {
    const ntc_point_t *t = FIRMWARE_TABLE;
    if(raw <= t[0].raw) return t[0].dC;
    if(raw >= t[FIRMWARE_TABLE_LEN - 1].raw) return t[FIRMWARE_TABLE_LEN - 1].dC;
    int lo = 0, hi = FIRMWARE_TABLE_LEN - 1;
    while(hi - lo > 1) {
        int mid = (lo + hi) >> 1;
        if(raw < t[mid].raw) hi = mid; else lo = mid;
    }
    int32_t dx = (int32_t)t[hi].raw - t[lo].raw;
    int32_t dy = (int32_t)t[hi].dC - t[lo].dC;
    return (int16_t)(t[lo].dC + (dy * (int32_t)(raw - t[lo].raw)) / dx);
}

static void test_matches_firmware_table_for_every_code(void **state) {
    (void) state;
    for(uint32_t raw = 0; raw <= 0xFFFF; raw++) {
        int16_t expected = firmware_raw_to_dC((uint16_t)raw);
        int16_t actual = bmb3y_ntc_raw_to_dC((uint16_t)raw);
        if(expected != actual) {
            fail_msg("raw %u: expected %d dC, got %d dC", raw, expected, actual);
        }
    }
}

static void test_breakpoints_and_ends(void **state) {
    (void) state;
    assert_int_equal(bmb3y_ntc_raw_to_dC(0), -600);
    assert_int_equal(bmb3y_ntc_raw_to_dC(439), -400);
    assert_int_equal(bmb3y_ntc_raw_to_dC(2800), 0);
    assert_int_equal(bmb3y_ntc_raw_to_dC(6542), 250);
    assert_int_equal(bmb3y_ntc_raw_to_dC(23291), 1150);
    // Open/absent thermistor
    assert_int_equal(bmb3y_ntc_raw_to_dC(0xFFFF), 3000);
}

// Captured 0x0D frames (thermistor A and B on three BMBs). The firmware
// truncates, so e.g. 6482 (24.67 C) reads 246.
static void test_captured_readings(void **state) {
    (void) state;
    assert_int_equal(bmb3y_ntc_raw_to_dC(5840), 211);
    assert_int_equal(bmb3y_ntc_raw_to_dC(6370), 240);
    assert_int_equal(bmb3y_ntc_raw_to_dC(5726), 205);
    assert_int_equal(bmb3y_ntc_raw_to_dC(6482), 246);
    assert_int_equal(bmb3y_ntc_raw_to_dC(5595), 197);
    assert_int_equal(bmb3y_ntc_raw_to_dC(5862), 212);
}

static void test_ntc_valid_range(void **state) {
    (void) state;
    assert_false(bmb3y_ntc_raw_valid(0));
    assert_false(bmb3y_ntc_raw_valid(438));
    assert_true(bmb3y_ntc_raw_valid(439));     // -40.0C
    assert_true(bmb3y_ntc_raw_valid(23291));   // 115.0C
    assert_false(bmb3y_ntc_raw_valid(23292));
    assert_false(bmb3y_ntc_raw_valid(0xFFFF)); // open
}

// Captured 0x0E frames, from the handoff
static void test_die_temperature(void **state) {
    (void) state;
    assert_int_equal(bmb3y_die_raw_to_dC(1371), 254); // 25.4C
    assert_int_equal(bmb3y_die_raw_to_dC(1378), 270);
    assert_int_equal(bmb3y_die_raw_to_dC(1393), 302);
    assert_int_equal(bmb3y_die_raw_to_dC(0), -2731);
}

static void test_thermistor_index(void **state) {
    (void) state;
    // nmc_96s31p_sr_t8-style: none on BMB 0, both on 1 and 2, B only on 3
    const uint8_t presence[] = { 0x0, 0x3, 0x3, 0x2 };
    assert_int_equal(bmb3y_thermistor_index(presence, 0, 0), -1);
    assert_int_equal(bmb3y_thermistor_index(presence, 0, 1), -1);
    assert_int_equal(bmb3y_thermistor_index(presence, 1, 0), 0);
    assert_int_equal(bmb3y_thermistor_index(presence, 1, 1), 1);
    assert_int_equal(bmb3y_thermistor_index(presence, 2, 0), 2);
    assert_int_equal(bmb3y_thermistor_index(presence, 2, 1), 3);
    assert_int_equal(bmb3y_thermistor_index(presence, 3, 0), -1);
    assert_int_equal(bmb3y_thermistor_index(presence, 3, 1), 4);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_matches_firmware_table_for_every_code),
        cmocka_unit_test(test_breakpoints_and_ends),
        cmocka_unit_test(test_captured_readings),
        cmocka_unit_test(test_ntc_valid_range),
        cmocka_unit_test(test_die_temperature),
        cmocka_unit_test(test_thermistor_index),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
