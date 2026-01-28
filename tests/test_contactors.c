#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app/model.h"
#include "app/state_machines/contactors.h"
#include "sys/events/events.h"
#include "drivers/contactors/contactors.h"
#include "sys/time/time.h"

#define CONTACTORS_TEST_WAIT_MS 1000

// Mock globals
extern millis_t stored_millis = 0;
extern millis64_t stored_millis64 = 0;

// Mock functions
//millis_t millis() { return stored_millis; }
//millis64_t millis64() { return stored_millis64; }

void contactors_set_pos_pre_neg(bool pos, bool pre, bool neg) {
    check_expected(pos);
    check_expected(pre);
    check_expected(neg);
}

void contactors_test_pre(bool pre) {
    check_expected(pre);
}

// Test helper to tick the SM multiple times if needed and advance time
void tick_sm(bms_model_t *model, uint32_t ms) {
    stored_millis += ms;
    stored_millis64 += ms;

    model->battery_voltage_millis = stored_millis;
    model->output_voltage_millis = stored_millis;
    model->pos_contactor_voltage_millis = stored_millis;
    model->neg_contactor_voltage_millis = stored_millis;
    model->current_millis = stored_millis;

    contactor_sm_tick(model);
}

static void test_initial_state(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    // Initial state should be OPEN
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    
    // Expect contactors to be set to open
    expect_uint_value(contactors_set_pos_pre_neg, pos, false);
    expect_uint_value(contactors_set_pos_pre_neg, pre, false);
    expect_uint_value(contactors_set_pos_pre_neg, neg, false);
    
    contactor_sm_tick(&model);
}

static void test_close_request_to_testing_sequence(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    model.contactor_sm.state = CONTACTORS_STATE_OPEN;
    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    
    // 1. Transition to TESTING_PRE_CLOSED
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 0);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_PRE_CLOSED);

    // 2. TESTING_PRE_CLOSED
    model.precharge_closed = true;
    expect_value(contactors_test_pre, pre, true);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_NEG_OPEN);

    // 3. TESTING_NEG_OPEN
    model.pos_contactor_voltage_mV = 10000; // Above threshold (open)
    model.neg_contactor_voltage_mV = 10000; // Above threshold (open)
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_NEG_CLOSED);

    // 4. TESTING_NEG_CLOSED
    model.neg_contactor_voltage_mV = 100; // Below threshold (closed)
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_POS_OPEN);

    // 5. TESTING_POS_OPEN
    model.neg_contactor_voltage_mV = 10000; // Above threshold (open)
    model.pos_contactor_voltage_mV = 10000; // Above threshold (open)
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_POS_CLOSED);

    // 6. TESTING_POS_CLOSED
    model.pos_contactor_voltage_mV = 100; // Below threshold (closed)
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_PRECHARGING_NEG);
}

static void test_precharge_success(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    // Force state to PRECHARGING
    model.contactor_sm.state = CONTACTORS_STATE_PRECHARGING;
    model.contactor_sm.last_transition_time = stored_millis;

    // Set up model for successful precharge
    model.battery_voltage_mV = 400000;
    model.battery_voltage_millis = stored_millis;
    model.output_voltage_mV = 398000; // 2V diff
    model.output_voltage_millis = stored_millis;
    model.neg_contactor_voltage_mV = 100; // < threshold
    model.neg_contactor_voltage_millis = stored_millis;
    model.current_mA = 300; // < PRECHARGE_SUCCESS_MAX_MA
    model.current_millis = stored_millis;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    
    tick_sm(&model, 1100);

    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CLOSED);
}

static void test_precharge_failure_timeout(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    model.contactor_sm.state = CONTACTORS_STATE_PRECHARGING;
    model.contactor_sm.last_transition_time = stored_millis;

    // Output voltage never rises
    model.battery_voltage_mV = 400000;
    model.output_voltage_mV = 0;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    
    tick_sm(&model, 10001);

    assert_int_equal(get_event_count(ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH), 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_PRECHARGE_FAILED);
}

static void test_pos_weld_failure_detection(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    // Force state to CLOSED
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_POS_OPEN;
    model.contactor_sm.last_transition_time = stored_millis;

    // Set up model to simulate POS contactor weld (stuck closed)
    model.pos_contactor_voltage_mV = 100; // Below threshold (closed)
    model.current_millis = stored_millis;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    
    tick_sm(&model, 2100);

    assert_int_equal(get_event_count(ERR_CONTACTOR_POS_STUCK_CLOSED), 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_FAILED);
}

static void test_open_request_from_closed(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    model.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model.contactor_req = CONTACTORS_REQUEST_OPEN;

    model.current_mA = 500; 

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 10);

    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
}

static void test_delayed_open_request_from_closed(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    model.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model.contactor_sm.last_transition_time = stored_millis;
    model.contactor_req = CONTACTORS_REQUEST_OPEN;

    model.current_mA = 3000; 
    model.current_millis = stored_millis;

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 500);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CLOSED);

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 2000);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
}

static void test_force_open_request(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    model.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model.contactor_sm.last_transition_time = stored_millis;
    model.contactor_req = CONTACTORS_REQUEST_FORCE_OPEN;

    model.current_mA = 10000; 
    model.current_millis = stored_millis;

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 10);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CLOSED);

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 2000);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
}

static void test_calibrate_request(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    model.contactor_sm.state = CONTACTORS_STATE_OPEN;
    model.contactor_req = CONTACTORS_REQUEST_CALIBRATE;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 10);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CALIBRATING);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_NULL);

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 501);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CALIBRATING_CLOSE_NEG);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_initial_state),
        cmocka_unit_test(test_close_request_to_testing_sequence),
        cmocka_unit_test(test_precharge_success),
        cmocka_unit_test(test_precharge_failure_timeout),
        cmocka_unit_test(test_pos_weld_failure_detection),
        // cmocka_unit_test(test_open_request_from_closed),
        // cmocka_unit_test(test_delayed_open_request_from_closed),
        // cmocka_unit_test(test_force_open_request),
        // cmocka_unit_test(test_calibrate_request),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
