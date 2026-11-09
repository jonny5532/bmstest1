#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "app/model.h"
#include "app/calibration/calibration.h"
#include "drivers/sensors/ads1115.h"
#include "drivers/sensors/ina228.h"

// Drives the calibration state machine through each sequence, with fake ADC
// and current sensor readings, to check a sequence only keeps its new
// calibration values if every step succeeds.

// Mock globals
millis64_t stored_millis64 = 0;
millis_t stored_millis = 0;

#include "sys/logging/logging.h"
void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    (void)format;
}

// ============================================================================
// Fake drivers
// ============================================================================

ina228_t ina228_dev;

static bool adc_finished;
static int32_t adc_calibration[ADS1115_CHANNEL_COUNT];

void ads1115_start_calibration(uint16_t num_samples) {
    (void)num_samples;
    adc_finished = false;
}

bool ads1115_calibration_finished(void) {
    return adc_finished;
}

int32_t ads1115_get_calibration(int channel) {
    return adc_calibration[channel];
}

static int nvm_saves;

bool nvm_save_persistent_slow(bms_model_t *model) {
    (void)model;
    nvm_saves++;
    return true;
}

void ekf_reset_bias(bms_model_t *model) {
    (void)model;
}

// ============================================================================
// Helpers
// ============================================================================

#define PACK_V 350.0f
#define NEW_MUL 0.013f

// Calibration values before each test, to check they are restored
#define PRIOR_BATTERY_MUL 0.0121f
#define PRIOR_OUTPUT_MUL 0.0122f
#define PRIOR_NEG_MUL 0.0123f
#define PRIOR_NEG_OFFSET_MV 42.0f
#define PRIOR_POS_MUL 0.0124f
#define PRIOR_CURRENT_OFFSET 7
#define PRIOR_POS_OFFSET_MV 43.0f
#define PRIOR_LINK_MUL 0.0125f

static bms_model_t m;

static int setup(void **state) {
    (void)state;
    memset(&m, 0, sizeof(m));
    m.cell_voltage_total_mV = (int32_t)(PACK_V * 1000);
    m.battery_voltage_mul = PRIOR_BATTERY_MUL;
    m.output_voltage_mul = PRIOR_OUTPUT_MUL;
    m.neg_contactor_mul = PRIOR_NEG_MUL;
    m.neg_contactor_offset_mV = PRIOR_NEG_OFFSET_MV;
    m.pos_contactor_mul = PRIOR_POS_MUL;
    m.current_offset = PRIOR_CURRENT_OFFSET;
    m.pos_contactor_offset_mV = PRIOR_POS_OFFSET_MV;
    m.link_voltage_mul = PRIOR_LINK_MUL;
    m.calibration_sm.name = "calibration";
    nvm_saves = 0;
    adc_finished = false;
    memset(adc_calibration, 0, sizeof(adc_calibration));
    stored_millis64 = 1000;
    return 0;
}

static void advance_ms(uint32_t ms) {
    stored_millis64 += ms;
    stored_millis += ms;
}

// Raw accumulator values over n samples
static int32_t rail_raw(float n) { return (int32_t)(PACK_V * n / NEW_MUL); }
static int32_t closed_contactor_raw(float n) { return (int32_t)(5 * n); }
static int32_t bad_contactor_raw(float n) { return (int32_t)(5000 * n); }

// Finish the ADC calibration with a good reading on every channel, and a
// current sensor offset of 3
static void finish_adc(float n) {
    adc_calibration[0] = rail_raw(n);
    adc_calibration[1] = rail_raw(n);
    adc_calibration[2] = closed_contactor_raw(n);
    adc_calibration[3] = rail_raw(n);
#if IS_BMS_BOARD(REV1)
    // Direct pos contactor (closed), and the negated link voltage
    adc_calibration[6] = closed_contactor_raw(n);
    adc_calibration[7] = -rail_raw(n);
#endif
    ina228_dev.null_accumulator = 30;
    ina228_dev.null_counter = 10;
    adc_finished = true;
}

static void assert_float_near(float actual, float expected) {
    assert_true(fabsf(actual - expected) < 1e-6f);
}

static void assert_calibration_unchanged(void) {
    assert_float_near(m.battery_voltage_mul, PRIOR_BATTERY_MUL);
    assert_float_near(m.output_voltage_mul, PRIOR_OUTPUT_MUL);
    assert_float_near(m.neg_contactor_mul, PRIOR_NEG_MUL);
    assert_float_near(m.neg_contactor_offset_mV, PRIOR_NEG_OFFSET_MV);
    assert_float_near(m.pos_contactor_mul, PRIOR_POS_MUL);
    assert_int_equal(m.current_offset, PRIOR_CURRENT_OFFSET);
    assert_float_near(m.pos_contactor_offset_mV, PRIOR_POS_OFFSET_MV);
    assert_float_near(m.link_voltage_mul, PRIOR_LINK_MUL);
    assert_int_equal(nvm_saves, 0);
}

// Run the long sequence up to the point where the ADC is measuring
static void start_long(void) {
    m.calibration_req = CALIBRATION_REQUEST_START_OFFLINE_LONG;
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_LONG_WAIT_BOTH);
    m.contactor_sm.state = CONTACTORS_STATE_CALIBRATING_CLOSED;
    calibration_sm_tick(&m);
    advance_ms(2000);
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_LONG_MEASURING);
}

// Run the short sequence up to the point where the first stage is measuring
static void start_short(void) {
    m.contactor_sm.state = CONTACTORS_STATE_OPEN;
    m.calibration_req = CALIBRATION_REQUEST_START_OFFLINE_SHORT;
    calibration_sm_tick(&m);
    calibration_sm_tick(&m);
    advance_ms(2000);
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_SHORT_MEASURING_OPEN);
}

// Complete the short sequence's first stage (battery voltage and current)
static void complete_short_first_stage(void) {
    finish_adc(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_SHORT_WAIT_NEG);
    // The first stage's values are in use, but not saved yet
    assert_float_near(m.battery_voltage_mul, NEW_MUL);
    assert_int_equal(m.current_offset, 3);
    assert_int_equal(nvm_saves, 0);
}

// ============================================================================
// Tests
// ============================================================================

static void test_long_success_saves_all(void **state) {
    (void)state;
    start_long();
    finish_adc(VOLTAGE_CALIBRATION_SAMPLES);
    calibration_sm_tick(&m);

    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_float_near(m.battery_voltage_mul, NEW_MUL);
    assert_float_near(m.output_voltage_mul, NEW_MUL);
    assert_float_near(m.neg_contactor_mul, NEW_MUL);
    assert_float_near(m.pos_contactor_mul, NEW_MUL);
    assert_int_equal(m.current_offset, 3);
#if IS_BMS_BOARD(REV1)
    assert_float_near(m.link_voltage_mul, NEW_MUL);
    assert_true(fabsf(m.pos_contactor_offset_mV - -(5 * NEW_MUL * 1000.0f)) < 1e-3f);
#endif
    assert_int_equal(nvm_saves, 1);
}

static void test_long_failure_restores_all(void **state) {
    (void)state;
    start_long();
    // Every step succeeds except the neg contactor
    finish_adc(VOLTAGE_CALIBRATION_SAMPLES);
    adc_calibration[2] = bad_contactor_raw(VOLTAGE_CALIBRATION_SAMPLES);
    calibration_sm_tick(&m);

    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_calibration_unchanged();
}

static void test_long_timeout_restores_all(void **state) {
    (void)state;
    m.calibration_req = CALIBRATION_REQUEST_START_OFFLINE_LONG;
    calibration_sm_tick(&m);
    advance_ms(10000);
    calibration_sm_tick(&m);

    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_calibration_unchanged();
}

static void test_short_success_saves_all(void **state) {
    (void)state;
    start_short();
    complete_short_first_stage();

    m.contactor_sm.state = CONTACTORS_STATE_CALIBRATING_ONLY_NEG;
    calibration_sm_tick(&m);
    advance_ms(2000);
    calibration_sm_tick(&m);
    finish_adc(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
    calibration_sm_tick(&m);

    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_float_near(m.battery_voltage_mul, NEW_MUL);
    assert_int_equal(m.current_offset, 3);
    // Derived from the new battery and existing output multipliers
    float neg_mul = (NEW_MUL + PRIOR_OUTPUT_MUL) / 2.0f;
    assert_float_near(m.neg_contactor_mul, neg_mul);
    assert_true(fabsf(m.neg_contactor_offset_mV - -(5 * neg_mul * 1000.0f)) < 1e-3f);
    assert_int_equal(nvm_saves, 1);
}

static void test_short_neg_failure_restores_first_stage(void **state) {
    (void)state;
    start_short();
    complete_short_first_stage();

    m.contactor_sm.state = CONTACTORS_STATE_CALIBRATING_ONLY_NEG;
    calibration_sm_tick(&m);
    advance_ms(2000);
    calibration_sm_tick(&m);
    finish_adc(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
    adc_calibration[2] = bad_contactor_raw(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
    calibration_sm_tick(&m);

    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_calibration_unchanged();
}

static void test_short_neg_timeout_restores_first_stage(void **state) {
    (void)state;
    start_short();
    complete_short_first_stage();

    // The neg contactor never closes
    advance_ms(10000);
    calibration_sm_tick(&m);

    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_calibration_unchanged();
}

static void test_online_closed_failure_restores_all(void **state) {
    (void)state;
    m.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    m.calibration_req = CALIBRATION_REQUEST_START_ONLINE;
    calibration_sm_tick(&m);
    advance_ms(2000);
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_ONLINE_MEASURING);

    // Output voltage succeeds, then the neg contactor fails
    finish_adc(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
    adc_calibration[2] = bad_contactor_raw(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
    calibration_sm_tick(&m);

    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_calibration_unchanged();
}

#if IS_BMS_BOARD(REV1)
static void test_long_pos_failure_restores_link(void **state) {
    (void)state;
    start_long();
    // Link calibration succeeds, then the pos contactor that follows it fails
    finish_adc(VOLTAGE_CALIBRATION_SAMPLES);
    adc_calibration[6] = bad_contactor_raw(VOLTAGE_CALIBRATION_SAMPLES);
    calibration_sm_tick(&m);

    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_calibration_unchanged();
}
#endif

static void test_other_settings_survive_failure(void **state) {
    (void)state;
    start_short();
    complete_short_first_stage();

    // A setting outside the calibration values, changed mid-sequence
    m.user_charge_current_limit_dA = 123;
    advance_ms(10000);
    calibration_sm_tick(&m);

    assert_calibration_unchanged();
    assert_int_equal(m.user_charge_current_limit_dA, 123);
}

// Isolation zero: the midpoint of channels 4/5, with the contactors open
#define PRIOR_ISOLATION_ZERO 13000.0f

static void start_isolation(void) {
    m.isolation_zero_counts = PRIOR_ISOLATION_ZERO;
    m.contactor_sm.state = CONTACTORS_STATE_OPEN;
    m.calibration_req = CALIBRATION_REQUEST_START_ISOLATION;
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_ISOLATION_STABILIZING);
    advance_ms(ISOLATION_SETTLE_MS);
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_ISOLATION_MEASURING);
}

static void finish_isolation_adc(float pos_mean, float neg_mean) {
    float n = SHORT_VOLTAGE_CALIBRATION_SAMPLES;
    adc_calibration[4] = (int32_t)(pos_mean * n);
    adc_calibration[5] = (int32_t)(neg_mean * n);
    adc_finished = true;
}

static void test_isolation_zero_saved(void **state) {
    (void)state;
    start_isolation();
    finish_isolation_adc(13500.0f, 13100.0f);
    calibration_sm_tick(&m);

    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_float_near(m.isolation_zero_counts, 13300.0f);
    assert_int_equal(nvm_saves, 1);
    // Only the zero changes
    m.isolation_zero_counts = PRIOR_ISOLATION_ZERO;
    nvm_saves = 0;
    assert_calibration_unchanged();
}

static void test_isolation_needs_open_contactors(void **state) {
    (void)state;
    m.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    m.calibration_req = CALIBRATION_REQUEST_START_ISOLATION;
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_int_equal(m.calibration_req, CALIBRATION_REQUEST_NULL);

    // Contactors leaving OPEN mid-measurement abandon it
    start_isolation();
    m.contactor_sm.state = CONTACTORS_STATE_PRECHARGING;
    finish_isolation_adc(13500.0f, 13100.0f);
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_float_near(m.isolation_zero_counts, PRIOR_ISOLATION_ZERO);
    assert_int_equal(nvm_saves, 0);
}

static void test_isolation_absent_adc_keeps_zero(void **state) {
    (void)state;
    start_isolation();
    finish_isolation_adc(0.0f, 0.0f);
    calibration_sm_tick(&m);
    assert_int_equal(m.calibration_sm.state, CALIBRATION_STATE_IDLE);
    assert_float_near(m.isolation_zero_counts, PRIOR_ISOLATION_ZERO);
    assert_int_equal(nvm_saves, 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_long_success_saves_all, setup),
        cmocka_unit_test_setup(test_long_failure_restores_all, setup),
        cmocka_unit_test_setup(test_long_timeout_restores_all, setup),
        cmocka_unit_test_setup(test_short_success_saves_all, setup),
        cmocka_unit_test_setup(test_short_neg_failure_restores_first_stage, setup),
        cmocka_unit_test_setup(test_short_neg_timeout_restores_first_stage, setup),
        cmocka_unit_test_setup(test_online_closed_failure_restores_all, setup),
#if IS_BMS_BOARD(REV1)
        cmocka_unit_test_setup(test_long_pos_failure_restores_link, setup),
#endif
        cmocka_unit_test_setup(test_other_settings_survive_failure, setup),
        cmocka_unit_test_setup(test_isolation_zero_saved, setup),
        cmocka_unit_test_setup(test_isolation_needs_open_contactors, setup),
        cmocka_unit_test_setup(test_isolation_absent_adc_keeps_zero, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
