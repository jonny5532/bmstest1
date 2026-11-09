#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app/model.h"
#include "app/estimators/ekf.h"
#include "app/estimators/estimators.h"
#include "drivers/sensors/ina228.h"
#include "sys/time/time.h"

// The basic coulomb count against a simulated pack, 1s at a time. The count
// keeps static state, so the tests run in order as one story.

millis64_t stored_millis64 = 0;
millis_t stored_millis = 0;
uint32_t stored_timestep = 1000; // past the start-up wait

bms_model_t model;

// The profile's working limits, and a voltage well below the max for the rest
// of the range
#define WORKING_MIN_mV DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV
#define WORKING_MAX_mV DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV
#define MID_mV (WORKING_MAX_mV - 200)

// A straight-line factory table with the working limits at 25% and 75%: half
// of a 200Ah pack, so 100Ah between them, and each 1Ah is 1%
#define CAPACITY_AH 200
const float *ekf_factory_ocv_table(void) { return NULL; }
float table_soc(const float *table, float ocv) {
    (void)table;
    float soc = 0.25f + 0.5f * (ocv * 1000.0f - WORKING_MIN_mV) / (WORKING_MAX_mV - WORKING_MIN_mV);
    return soc < 0.0f ? 0.0f : soc > 1.0f ? 1.0f : soc;
}

static uint16_t soc;

// Run for a while at a steady current and max cell voltage
static void run(uint32_t seconds, int32_t current_mA, int16_t cell_max_mV) {
    for (uint32_t s = 0; s < seconds; s++) {
        stored_millis += 1000;
        stored_millis64 += 1000;
        stored_timestep += 50;
        model.current_mA = current_mA;
        model.cell_voltage_max_mV = cell_max_mV;
        model.charge_raw += (int64_t)current_mA * 4 * 1000000; // 0.25mA LSBs x us
        soc = basic_count_soc_estimate(&model);
    }
}

static int setup(void **state) {
    (void)state;
    memset(&model, 0, sizeof(model));
    model.actual_capacity_mC = CAPACITY_AH * 3600000u;
    // Starts halfway between the working limits
    model.high_voltages.battery = (WORKING_MIN_mV + WORKING_MAX_mV) / 2000.0f * NUM_CELLS;
    return 0;
}

static void test_starts_from_the_voltage_and_counts(void **state) {
    (void)state;

    run(1, 0, MID_mV);
    assert_int_equal(soc, 5000);

    // 10A for an hour is 10%
    run(3600, -10000, MID_mV);
    assert_int_equal(soc, 4000);
}

static void test_small_currents_count(void **state) {
    (void)state;

    // 50mA for 20 hours is 1Ah, 1%: under a float counter, each step's charge
    // was below its resolution and rounded away
    run(20 * 3600, -50, MID_mV);
    assert_int_equal(soc, 3900);
}

static void test_resets_once_settled_at_the_top(void **state) {
    (void)state;

    // Charge 30Ah: the count says 69%, but the pack is really full
    run(3 * 3600, 10000, MID_mV);
    assert_int_equal(soc, 6900);

    // At the top with the current swinging +-1.5A about zero. The average
    // falls under 1A ~140s after charging stops, and it's full 10 minutes later.
    for (int i = 0; i < 700; i++) {
        run(1, (i % 2) ? 1500 : -1500, WORKING_MAX_mV - 5);
    }
    assert_int_equal(soc, 6900);
    for (int i = 0; i < 100; i++) {
        run(1, (i % 2) ? 1500 : -1500, WORKING_MAX_mV - 5);
    }
    assert_int_equal(soc, 10000);
}

static void test_stays_full_while_floating(void **state) {
    (void)state;

    // Still settled at the top: a small discharge is topped back up
    run(1800, -300, WORKING_MAX_mV - 5);
    assert_int_equal(soc, 10000);

    // Charge counted past full doesn't go over 100%, or bank anything
    run(600, 2000, WORKING_MAX_mV);
    assert_int_equal(soc, 10000);
    run(360, -10000, MID_mV);
    assert_int_equal(soc, 9900);
}

static void test_no_reset_while_still_charging(void **state) {
    (void)state;

    // Held at the top but still taking 3A (the averaged current never settles)
    run(1800, -10000, MID_mV); // down to 94%
    run(3600, 3000, WORKING_MAX_mV);
    assert_int_equal(soc, 9700);
}

static void test_no_reset_below_the_margin(void **state) {
    (void)state;

    // Settled, but further below the working max than BASIC_COUNT_FULL_MARGIN_mV
    run(3600, -10000, MID_mV); // down to 87%
    run(3600, 0, WORKING_MAX_mV - BASIC_COUNT_FULL_MARGIN_mV - 1);
    assert_int_equal(soc, 8700);

    // ...and resets just inside it
    run(660, 0, WORKING_MAX_mV - BASIC_COUNT_FULL_MARGIN_mV);
    assert_int_equal(soc, 10000);
}

static void test_scaled_to_the_working_limits(void **state) {
    (void)state;

    // Full from the last test; raising the working min to halfway leaves 50Ah
    // between the limits, so 10Ah is 20%
    model.cell_voltage_working_min_mV = (WORKING_MIN_mV + WORKING_MAX_mV) / 2;
    run(3600, -10000, MID_mV);
    assert_int_equal(soc, 8000);
    model.cell_voltage_working_min_mV = 0;
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_starts_from_the_voltage_and_counts),
        cmocka_unit_test(test_small_currents_count),
        cmocka_unit_test(test_resets_once_settled_at_the_top),
        cmocka_unit_test(test_stays_full_while_floating),
        cmocka_unit_test(test_no_reset_while_still_charging),
        cmocka_unit_test(test_no_reset_below_the_margin),
        cmocka_unit_test(test_scaled_to_the_working_limits),
    };
    return cmocka_run_group_tests(tests, setup, NULL);
}
