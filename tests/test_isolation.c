#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "app/isolation.h"
#include "config/limits.h"

#define COUNT_V (4.096f / 32768.0f)

// Raw channel counts for a pack floating around `zero`, with `pos_share` of the
// pack voltage on the + side, through a divider of `ratio`.
static void make_counts(float zero, float battery_V, float ratio, float pos_share,
                        float *pos, float *neg) {
    float span = battery_V / (ratio * COUNT_V);
    *pos = zero + span * pos_share;
    *neg = zero - span * (1.0f - pos_share);
}

static void test_zero_default_and_drift(void **state) {
    (void)state;
    float at_cal = isolation_zero_counts(0.0f, (int32_t)ISOLATION_ZERO_CAL_TEMP_DC);
    assert_float_equal(at_cal, ISOLATION_ZERO_DEFAULT_COUNTS, 0.01f);

    float warmer = isolation_zero_counts(0.0f, (int32_t)ISOLATION_ZERO_CAL_TEMP_DC + 100);
    assert_float_equal(warmer - at_cal, 100.0f * ISOLATION_ZERO_TEMPCO_COUNTS_PER_DC, 0.01f);
}

static void test_zero_calibration_round_trip(void **state) {
    (void)state;
    float ref = isolation_zero_ref_from_counts(15000.0f, 337);
    assert_float_equal(isolation_zero_counts(ref, 337), 15000.0f, 0.01f);
    // A stored calibration replaces the board default
    assert_float_equal(isolation_zero_counts(ref, (int32_t)ISOLATION_ZERO_CAL_TEMP_DC), ref, 0.01f);
}

static void test_floating_pack_is_symmetric_and_healthy(void **state) {
    (void)state;
    float pos, neg;
    make_counts(13317.0f, 383.3f, 501.0f, 0.5f, &pos, &neg);
    isolation_result_t r = isolation_compute(pos, neg, 13317.0f, 383.3f, true);
    assert_float_equal(r.pos_to_chassis_V, 191.65f, 0.05f);
    assert_float_equal(r.neg_to_chassis_V, -191.65f, 0.05f);
    assert_float_equal(r.resistance_ohms, ISOLATION_RESISTANCE_CAP_OHMS, 1.0f);
}

static void test_closed_contactors_give_voltages_but_no_resistance(void **state) {
    (void)state;
    // ck0 reading with the contactors closed to the (faulted) inverter
    isolation_result_t r = isolation_compute(17788.0f, 11620.0f, 13317.0f, 383.3f, false);
    assert_float_equal(r.pos_to_chassis_V, 277.8f, 0.2f);
    assert_float_equal(r.neg_to_chassis_V, -105.5f, 0.2f);
    assert_float_equal(r.pos_to_chassis_V - r.neg_to_chassis_V, 383.3f, 0.01f);
    assert_true(isnan(r.resistance_ohms));
}

static void test_single_sided_leaks(void **state) {
    (void)state;
    const float ratio = 501.0f;
    const float rd = ratio * ISOLATION_DIVIDER_BOTTOM_OHMS;
    const float leak = 1e6f;
    const float rd_par = rd * leak / (rd + leak);
    float pos, neg;

    // 1 Mohm from pack+ to chassis: the + side carries rd||leak against rd
    make_counts(13317.0f, 380.0f, ratio, rd_par / (rd_par + rd), &pos, &neg);
    isolation_result_t r = isolation_compute(pos, neg, 13317.0f, 380.0f, true);
    assert_float_equal(r.resistance_ohms, leak, leak * 0.01f);

    // 1 Mohm from pack- to chassis
    make_counts(13317.0f, 380.0f, ratio, rd / (rd_par + rd), &pos, &neg);
    r = isolation_compute(pos, neg, 13317.0f, 380.0f, true);
    assert_float_equal(r.resistance_ohms, leak, leak * 0.01f);
}

static void test_hard_short_reads_zero(void **state) {
    (void)state;
    // pack+ at chassis: the + channel sits on the zero
    isolation_result_t r = isolation_compute(13317.0f, 13317.0f - 6000.0f, 13317.0f, 380.0f, true);
    assert_float_equal(r.resistance_ohms, 0.0f, 0.001f);
}

static void test_unusable_inputs_give_nan(void **state) {
    (void)state;
    isolation_result_t r = isolation_compute(13320.0f, 13310.0f, 13317.0f, 380.0f, true);
    assert_true(isnan(r.pos_to_chassis_V));
    assert_true(isnan(r.neg_to_chassis_V));
    assert_true(isnan(r.resistance_ohms));

    r = isolation_compute(17788.0f, 11620.0f, 13317.0f, 0.0f, true);
    assert_true(isnan(r.pos_to_chassis_V));
}


// Feeds a constant reading for `ms` at 100 ms steps; returns the last status
static isolation_autozero_status_t run_autozero(isolation_autozero_t *az, uint32_t *now, uint32_t ms,
                                                bool usable, bool window_over, float pos, float neg,
                                                float zero, float *new_zero, float *mid) {
    isolation_autozero_status_t st = ISOLATION_AUTOZERO_PENDING;
    for (uint32_t t = 0; t < ms && (st == ISOLATION_AUTOZERO_PENDING); t += 100) {
        *now += 100;
        st = isolation_autozero_step(az, *now, usable, window_over, pos, neg, zero, new_zero, mid);
    }
    return st;
}

static void test_autozero_blends_small_drift(void **state) {
    (void)state;
    isolation_autozero_t az = {0};
    uint32_t now = 1000;
    float new_zero = 0, mid = 0;
    // midpoint 13377, 60 counts above the zero: accepted, half blended in
    isolation_autozero_status_t st = run_autozero(&az, &now, 10000, true, false, 16465.0f, 10289.0f,
                                                  13317.0f, &new_zero, &mid);
    assert_int_equal(st, ISOLATION_AUTOZERO_ACCEPTED);
    assert_float_equal(mid, 13377.0f, 0.01f);
    assert_float_equal(new_zero, 13317.0f + 60.0f * ISOLATION_AUTOZERO_BLEND, 0.01f);
    // only once per boot
    assert_int_equal(isolation_autozero_step(&az, now + 100, true, false, 16465.0f, 10289.0f, 13317.0f,
                                             &new_zero, &mid), ISOLATION_AUTOZERO_DONE);
}

static void test_autozero_rejects_large_offset(void **state) {
    (void)state;
    isolation_autozero_t az = {0};
    uint32_t now = 1000;
    float new_zero = -1, mid = 0;
    // midpoint 400 counts off: treated as a possible leak, zero untouched
    isolation_autozero_status_t st = run_autozero(&az, &now, 10000, true, false, 16805.0f, 10629.0f,
                                                  13317.0f, &new_zero, &mid);
    assert_int_equal(st, ISOLATION_AUTOZERO_REJECTED);
    assert_float_equal(mid, 13717.0f, 0.01f);
    assert_float_equal(new_zero, -1.0f, 0.0f);
}

static void test_autozero_waits_for_settling_and_aborts_when_window_ends(void **state) {
    (void)state;
    isolation_autozero_t az = {0};
    uint32_t now = 1000;
    float new_zero = 0, mid = 0;
    // not long enough to settle + average
    isolation_autozero_status_t st = run_autozero(&az, &now, ISOLATION_AUTOZERO_SETTLE_MS, true, false,
                                                  16405.0f, 10229.0f, 13317.0f, &new_zero, &mid);
    assert_int_equal(st, ISOLATION_AUTOZERO_PENDING);
    // contactors closed / system operating before it finished
    st = isolation_autozero_step(&az, now + 100, false, true, 16405.0f, 10229.0f, 13317.0f, &new_zero, &mid);
    assert_int_equal(st, ISOLATION_AUTOZERO_ABORTED);
}

static void test_autozero_restarts_on_noise(void **state) {
    (void)state;
    isolation_autozero_t az = {0};
    uint32_t now = 1000;
    float new_zero = 0, mid = 0;
    isolation_autozero_status_t st = ISOLATION_AUTOZERO_PENDING;
    // alternating midpoints 100 counts apart never give a stable average
    for (int i = 0; i < 200 && st == ISOLATION_AUTOZERO_PENDING; i++) {
        now += 100;
        float off = (i & 1) ? 50.0f : -50.0f;
        st = isolation_autozero_step(&az, now, true, false, 16405.0f + off, 10229.0f + off, 13317.0f,
                                     &new_zero, &mid);
    }
    assert_int_equal(st, ISOLATION_AUTOZERO_PENDING);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_zero_default_and_drift),
        cmocka_unit_test(test_zero_calibration_round_trip),
        cmocka_unit_test(test_floating_pack_is_symmetric_and_healthy),
        cmocka_unit_test(test_closed_contactors_give_voltages_but_no_resistance),
        cmocka_unit_test(test_single_sided_leaks),
        cmocka_unit_test(test_hard_short_reads_zero),
        cmocka_unit_test(test_unusable_inputs_give_nan),
        cmocka_unit_test(test_autozero_blends_small_drift),
        cmocka_unit_test(test_autozero_rejects_large_offset),
        cmocka_unit_test(test_autozero_waits_for_settling_and_aborts_when_window_ends),
        cmocka_unit_test(test_autozero_restarts_on_noise),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
