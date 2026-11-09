#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "app/isolation.h"
#include "app/model.h"
#include "config/limits.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    (void)format;
}

millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;

#define VOLTS_PER_COUNT_PER_RATIO (4.096f / 32768.0f)
#define ZERO ISOLATION_ZERO_DEFAULT_COUNTS

// Channel counts for a floating pack of battery_V with dividers of the given
// ratio (bottom 24k), and a leak of leak_ohms from one terminal to chassis
// (INFINITY for none; leak_on_neg selects the terminal).
static void channels(float battery_V, float ratio, float leak_ohms, bool leak_on_neg, float *pos, float *neg) {
    float rd = ratio * ISOLATION_DIVIDER_BOTTOM_OHMS;
    float leaked = isinf(leak_ohms) ? rd : rd * leak_ohms / (rd + leak_ohms);
    float r_pos = leak_on_neg ? rd : leaked;
    float r_neg = leak_on_neg ? leaked : rd;
    // Pack+ to chassis across r_pos, chassis to pack- across r_neg
    float v_pos = battery_V * r_pos / (r_pos + r_neg);
    float v_neg = -battery_V * r_neg / (r_pos + r_neg);
    float volts_per_count = ratio * VOLTS_PER_COUNT_PER_RATIO;
    *pos = ZERO + v_pos / volts_per_count;
    *neg = ZERO + v_neg / volts_per_count;
}

static isolation_reading_t reading(float battery_V, float ratio, float leak_ohms, bool leak_on_neg) {
    float pos, neg;
    channels(battery_V, ratio, leak_ohms, leak_on_neg, &pos, &neg);
    return isolation_compute(pos, neg, ZERO, battery_V);
}

static int setup(void **state) {
    (void)state;
    stored_millis = 5000;
    stored_millis64 = 5000;
    for (int i = 0; i < ERR_HIGHEST; i++) {
        bms_event_slots[i] = (bms_event_slot_t){0};
    }
    extern uint16_t highest_level;
    highest_level = LEVEL_NONE;
    return 0;
}

static void test_balanced_pack_reads_the_cap(void **state) {
    (void)state;
    isolation_reading_t r = reading(400.0f, 501.0f, INFINITY, false);
    assert_float_equal(r.pos_to_chassis_V, 200.0f, 0.01f);
    assert_float_equal(r.neg_to_chassis_V, -200.0f, 0.01f);
    assert_float_equal(r.resistance_ohms, ISOLATION_RESISTANCE_CAP_OHMS, 1.0f);
}

static void test_terminal_voltages_follow_a_leak(void **state) {
    (void)state;
    // 100k on pack- with 12M dividers: pack- is pulled most of the way to
    // chassis, and the two still span the battery voltage
    float rd = 501.0f * ISOLATION_DIVIDER_BOTTOM_OHMS;
    float leaked = rd * 100e3f / (rd + 100e3f);
    float expect_neg = -400.0f * leaked / (rd + leaked);
    isolation_reading_t r = reading(400.0f, 501.0f, 100e3f, true);
    assert_float_equal(r.neg_to_chassis_V, expect_neg, 0.01f);
    assert_float_equal(r.pos_to_chassis_V, 400.0f + expect_neg, 0.01f);
    assert_true(r.neg_to_chassis_V > -10.0f);
    // And the other way round
    r = reading(400.0f, 501.0f, 100e3f, false);
    assert_float_equal(r.pos_to_chassis_V, -expect_neg, 0.01f);
    assert_float_equal(r.neg_to_chassis_V, -400.0f - expect_neg, 0.01f);
}

static void test_leak_is_recovered_on_either_side(void **state) {
    (void)state;
    const float leaks[] = {50e3f, 200e3f, 1e6f, 10e6f, 50e6f};
    const float ratios[] = {501.0f, 416.0f}; // Rev1, Rev0
    for (int ri = 0; ri < 2; ri++) {
        for (int i = 0; i < 5; i++) {
            isolation_reading_t neg = reading(400.0f, ratios[ri], leaks[i], true);
            assert_true(neg.pos_to_chassis_V > -neg.neg_to_chassis_V);
            assert_float_equal(neg.resistance_ohms, leaks[i], leaks[i] * 0.01f);

            isolation_reading_t pos = reading(56.0f, ratios[ri], leaks[i], false);
            assert_true(pos.pos_to_chassis_V < -pos.neg_to_chassis_V);
            assert_float_equal(pos.resistance_ohms, leaks[i], leaks[i] * 0.01f);
        }
    }
}

static void test_terminal_at_chassis_is_a_short(void **state) {
    (void)state;
    float span = 400.0f / (501.0f * VOLTS_PER_COUNT_PER_RATIO);
    // Pack- at chassis, and slightly past it (noise)
    isolation_reading_t r = isolation_compute(ZERO + span, ZERO, ZERO, 400.0f);
    assert_float_equal(r.pos_to_chassis_V, 400.0f, 0.01f);
    assert_float_equal(r.neg_to_chassis_V, 0.0f, 0.01f);
    assert_float_equal(r.resistance_ohms, 0.0f, 1e-3f);
    r = isolation_compute(ZERO + span + 5.0f, ZERO + 5.0f, ZERO, 400.0f);
    assert_true(r.neg_to_chassis_V > 0.0f);
    assert_float_equal(r.resistance_ohms, 0.0f, 1e-3f);
    // Pack+ at chassis
    r = isolation_compute(ZERO, ZERO - span, ZERO, 400.0f);
    assert_float_equal(r.pos_to_chassis_V, 0.0f, 0.01f);
    assert_float_equal(r.neg_to_chassis_V, -400.0f, 0.01f);
    assert_float_equal(r.resistance_ohms, 0.0f, 1e-3f);
}

static void test_unusable_inputs_are_nan(void **state) {
    (void)state;
    isolation_reading_t r = isolation_compute(ZERO + 100.0f, ZERO - 100.0f, ZERO, ISOLATION_MIN_BATTERY_V - 1.0f);
    assert_true(isnan(r.pos_to_chassis_V) && isnan(r.neg_to_chassis_V) && isnan(r.resistance_ohms));
    r = isolation_compute(ZERO, ZERO, ZERO, 400.0f); // no span
    assert_true(isnan(r.pos_to_chassis_V) && isnan(r.neg_to_chassis_V) && isnan(r.resistance_ohms));
    r = isolation_compute(ZERO + 100.0f, ZERO - 100.0f, ZERO, NAN);
    assert_true(isnan(r.pos_to_chassis_V) && isnan(r.neg_to_chassis_V) && isnan(r.resistance_ohms));
}

static void test_low_threshold_scales_with_voltage(void **state) {
    (void)state;
    // 500 ohm/V: 200k at 400 V
    assert_true(isolation_is_low(199e3f, 400.0f));
    assert_false(isolation_is_low(201e3f, 400.0f));
    assert_false(isolation_is_low(ISOLATION_RESISTANCE_CAP_OHMS, 400.0f));
}

// ---------------------------------------------------------------------------
// isolation_update

static bms_model_t m;

static void update(float leak_ohms, bool leak_on_neg, bool open_settled) {
    float pos, neg;
    channels(m.high_voltages.battery, 501.0f, leak_ohms, leak_on_neg, &pos, &neg);
    isolation_update(&m, pos, neg, open_settled);
}

static void model_setup(bool zeroed) {
    m = (bms_model_t){0};
    m.high_voltages.battery = 400.0f;
    m.isolation_zero_counts = zeroed ? ZERO : 0.0f;
}

static void test_closed_only_updates_the_live_voltages(void **state) {
    (void)state;
    model_setup(true);
    update(100e3f, true, false);
    assert_true(m.isolation_pos_to_chassis_V > 390.0f);
    assert_true(m.isolation_neg_to_chassis_V > -10.0f);
    assert_int_equal(m.isolation_open_millis, 0);
    assert_int_equal(get_event_level(ERR_ISOLATION_LOW), LEVEL_NONE);
}

static void test_zeroed_low_isolation_raises_and_clears(void **state) {
    (void)state;
    model_setup(true);
    update(100e3f, true, true);
    assert_int_equal(m.isolation_open_millis, 5000);
    assert_float_equal(m.isolation_open_resistance_ohms, 100e3f, 1e3f);
    assert_int_equal(get_event_level(ERR_ISOLATION_LOW), LEVEL_WARNING);

    // Closing holds the open reading and the event
    stored_millis = 9000;
    update(INFINITY, false, false);
    assert_float_equal(m.isolation_pos_to_chassis_V, 200.0f, 0.1f);
    assert_float_equal(m.isolation_neg_to_chassis_V, -200.0f, 0.1f);
    assert_int_equal(m.isolation_open_millis, 5000);
    assert_float_equal(m.isolation_open_resistance_ohms, 100e3f, 1e3f);
    assert_int_equal(get_event_level(ERR_ISOLATION_LOW), LEVEL_WARNING);

    // A healthy open reading clears it
    update(INFINITY, false, true);
    assert_int_equal(m.isolation_open_millis, 9000);
    assert_int_equal(get_event_level(ERR_ISOLATION_LOW), LEVEL_NONE);
}

static void test_unzeroed_measures_but_never_raises(void **state) {
    (void)state;
    model_setup(false);
    update(10e3f, false, true);
    assert_true(m.isolation_pos_to_chassis_V < 10.0f);
    assert_float_equal(m.isolation_open_resistance_ohms, 10e3f, 100.0f);
    assert_int_equal(get_event_level(ERR_ISOLATION_LOW), LEVEL_NONE);
}

static void test_unusable_reading_keeps_the_open_values(void **state) {
    (void)state;
    model_setup(true);
    update(1e6f, true, true);
    float held = m.isolation_open_resistance_ohms;
    stored_millis = 9000;
    m.high_voltages.battery = 5.0f; // below ISOLATION_MIN_BATTERY_V
    isolation_update(&m, ZERO + 10.0f, ZERO - 10.0f, true);
    assert_true(isnan(m.isolation_pos_to_chassis_V) && isnan(m.isolation_neg_to_chassis_V));
    assert_int_equal(m.isolation_open_millis, 5000);
    assert_float_equal(m.isolation_open_resistance_ohms, held, 1.0f);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_balanced_pack_reads_the_cap),
        cmocka_unit_test(test_terminal_voltages_follow_a_leak),
        cmocka_unit_test(test_leak_is_recovered_on_either_side),
        cmocka_unit_test(test_terminal_at_chassis_is_a_short),
        cmocka_unit_test(test_unusable_inputs_are_nan),
        cmocka_unit_test(test_low_threshold_scales_with_voltage),
        cmocka_unit_test_setup(test_closed_only_updates_the_live_voltages, setup),
        cmocka_unit_test_setup(test_zeroed_low_isolation_raises_and_clears, setup),
        cmocka_unit_test_setup(test_unzeroed_measures_but_never_raises, setup),
        cmocka_unit_test_setup(test_unusable_reading_keeps_the_open_values, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
