#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "app/model.h"
#include "app/estimators/lifetime.h"

bms_model_t model;

#define PACK_V 360.0

// Run at a steady measured current, 1s at a time
static void run(uint32_t seconds, double current_A, double bias_A) {
    for (uint32_t s = 0; s < seconds; s++) {
        lifetime_accumulate(&model, current_A / 3600.0, 1.0, bias_A, PACK_V);
    }
}

// Roughly normal noise, mean zero (sum of 12 uniforms)
static double noise(double sd) {
    double sum = 0.0;
    for (int i = 0; i < 12; i++) sum += (double)rand() / RAND_MAX;
    return (sum - 6.0) * sd;
}

static int setup(void **state) {
    (void)state;
    memset(&model, 0, sizeof(model));
    srand(1);
    return 0;
}

static void test_steady_charge_and_discharge(void **state) {
    (void)state;
    run(3601, 10.0, 0.0); // 10002.8mAh, 3601Wh
    assert_int_equal(model.lifetime_charged_mAh, 10002);
    assert_int_equal(model.lifetime_charged_Wh, 3601);
    assert_int_equal(model.lifetime_discharged_mAh, 0);
    assert_int_equal(model.lifetime_discharged_Wh, 0);

    // The leftover charge offsets the start of the discharge, by under a unit
    run(1801, -20.0, 0.0); // 10005.6mAh, 3602Wh
    assert_int_equal(model.lifetime_charged_mAh, 10002);
    assert_uint_in_range(model.lifetime_discharged_mAh, 10004, 10005);
    assert_uint_in_range(model.lifetime_discharged_Wh, 3601, 3602);
}

// Continues from the loaded totals
static void test_adds_to_stored_totals(void **state) {
    (void)state;
    model.lifetime_charged_mAh = 1000000;
    model.lifetime_charged_Wh = 360000;
    run(361, 10.0, 0.0); // 1002.8mAh, 361Wh
    assert_int_equal(model.lifetime_charged_mAh, 1001002);
    assert_int_equal(model.lifetime_charged_Wh, 360361);
}

// A month of idle noise like ck0's (~4mA rms per 1s sample). Split by sign every
// second it would put ~1.6mA average into each total: ~1.2Ah and ~400Wh each.
static void test_idle_noise_rejected(void **state) {
    (void)state;
    for (int s = 0; s < 30 * 86400; s++) {
        lifetime_accumulate(&model, noise(0.004) / 3600.0, 1.0, 0.0, PACK_V);
    }
    // A random walk of ~1.1e-3mAh (4e-4Wh) a step crosses 1mAh in ~10 days (1Wh in ~70)
    assert_true(model.lifetime_charged_mAh <= 5);
    assert_true(model.lifetime_discharged_mAh <= 5);
    assert_true(model.lifetime_charged_Wh <= 1);
    assert_true(model.lifetime_discharged_Wh <= 1);
}

// A real small load below any deadband still adds up
static void test_small_load_counted(void **state) {
    (void)state;
    run(101 * 3600 + 1800, -0.010, 0.0); // 10mA for 101.5h = 1015mAh, 365.4Wh
    assert_uint_in_range(model.lifetime_discharged_mAh, 1014, 1015);
    assert_int_equal(model.lifetime_discharged_Wh, 365);
    assert_int_equal(model.lifetime_charged_mAh, 0);
    assert_int_equal(model.lifetime_charged_Wh, 0);
}

// What the sensor bias explains isn't counted
static void test_bias_removed(void **state) {
    (void)state;
    run(100 * 3600, 0.050, 0.050);
    assert_int_equal(model.lifetime_charged_mAh, 0);
    assert_int_equal(model.lifetime_discharged_mAh, 0);

    run(3601, 10.050, 0.050);
    assert_int_equal(model.lifetime_charged_mAh, 10002);
    assert_int_equal(model.lifetime_charged_Wh, 3601);

    // Over a short step (as before the EKF starts) only that long of it is taken out
    memset(&model.lifetime, 0, sizeof(model.lifetime));
    for (int i = 0; i < 80; i++) lifetime_accumulate(&model, 0.0, 0.0625, 0.36, PACK_V);
    assert_float_equal(model.lifetime.acc_mAh, -0.5, 1e-9); // 0.36A for 5s
}

// Back-and-forth below a unit nets out; above it, both directions count.
// Steps are powers of two so the sums are exact.
static void test_reversals(void **state) {
    (void)state;
    for (int i = 0; i < 100; i++) {
        lifetime_accumulate(&model, 1.0 / 2048, 1.0, 0.0, PACK_V);  // 0.49mAh, 0.18Wh
        lifetime_accumulate(&model, -1.0 / 2048, 1.0, 0.0, PACK_V);
    }
    assert_int_equal(model.lifetime_charged_mAh, 0);
    assert_int_equal(model.lifetime_discharged_mAh, 0);
    assert_int_equal(model.lifetime_charged_Wh, 0);
    assert_int_equal(model.lifetime_discharged_Wh, 0);

    for (int i = 0; i < 100; i++) {
        lifetime_accumulate(&model, 1.0 / 256, 1.0, 0.0, PACK_V);   // 3.9mAh, 1.4Wh
        lifetime_accumulate(&model, -1.0 / 256, 1.0, 0.0, PACK_V);
    }
    assert_int_equal(model.lifetime_charged_mAh, 300);
    assert_int_equal(model.lifetime_discharged_mAh, 300);
    assert_int_equal(model.lifetime_charged_Wh, 100);
    assert_int_equal(model.lifetime_discharged_Wh, 100);
    assert_float_equal(model.lifetime.acc_mAh, 0.0, 1e-12);
}

// A large step moves in one go
static void test_large_step(void **state) {
    (void)state;
    lifetime_accumulate(&model, -1000.0, 1.0, 0.0, PACK_V);
    assert_int_equal(model.lifetime_discharged_mAh, 1000000);
    assert_int_equal(model.lifetime_discharged_Wh, 360000);
    assert_float_equal(model.lifetime.acc_mAh, 0.0, 1e-6);
}

// The totals wrap, so the difference between readings stays right across it
static void test_wraps(void **state) {
    (void)state;
    uint32_t before = UINT32_MAX - 1;
    model.lifetime_charged_Wh = before;
    run(3601, 10.0, 0.0); // 3601Wh
    assert_int_equal(model.lifetime_charged_Wh, 3599);
    assert_int_equal((uint32_t)(model.lifetime_charged_Wh - before), 3601);

    memset(&model.lifetime, 0, sizeof(model.lifetime)); // the charge above left ~0.8mAh
    model.lifetime_discharged_mAh = UINT32_MAX - 5;
    lifetime_accumulate(&model, -1000.0, 1.0, 0.0, PACK_V);
    assert_int_equal(model.lifetime_discharged_mAh, 1000000 - 6);
}

// A NaN or impossible step is dropped rather than poisoning the accumulator
static void test_bad_step_dropped(void **state) {
    (void)state;
    lifetime_accumulate(&model, NAN, 1.0, 0.0, PACK_V);
    lifetime_accumulate(&model, 1e12, 1.0, 0.0, PACK_V);
    assert_int_equal(model.lifetime_charged_mAh, 0);
    assert_int_equal(model.lifetime_charged_Wh, 0);
    run(3601, 10.0, 0.0);
    assert_int_equal(model.lifetime_charged_mAh, 10002);
    assert_int_equal(model.lifetime_charged_Wh, 3601);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_steady_charge_and_discharge, setup),
        cmocka_unit_test_setup(test_adds_to_stored_totals, setup),
        cmocka_unit_test_setup(test_idle_noise_rejected, setup),
        cmocka_unit_test_setup(test_small_load_counted, setup),
        cmocka_unit_test_setup(test_bias_removed, setup),
        cmocka_unit_test_setup(test_reversals, setup),
        cmocka_unit_test_setup(test_large_step, setup),
        cmocka_unit_test_setup(test_wraps, setup),
        cmocka_unit_test_setup(test_bad_step_dropped, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
