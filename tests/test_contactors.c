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

// #define CONTACTORS_TEST_WAIT_MS 1000

// Mock globals
// Time starts non-zero, as on the BMS (a zero timestamp means "no reading yet")
millis64_t stored_millis64 = 1;
millis_t stored_millis = 1;

uint32_t time_us_32() { return stored_millis * 1000; }

#include "sys/logging/logging.h"
void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

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

    model->high_voltages.battery_millis = stored_millis;
    model->high_voltages.output_millis = stored_millis;
    model->high_voltages.pos_contactor_millis = stored_millis;
    model->high_voltages.neg_contactor_millis = stored_millis;
#if BMS_BOARD == BMS_BOARD_REV1
    model->high_voltages.link_millis = stored_millis;
    model->high_voltages.fuse_drop_millis = stored_millis;
#endif
    model->current_millis = stored_millis;

    // The supervisor is enabling the contactor drivers
    model->supervisor.version = SUP_PROTOCOL_VERSION;
    model->supervisor.state = SUP_STATE_ON;
    model->supervisor_millis = stored_millis;

    contactor_sm_tick(model);
}

static void expect_all_open(void) {
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
}

static void test_close_held_until_supervisor_enables(void **state) {
    (void) state;
    bms_model_t model = {0};
    clear_bms_event(ERR_SUPERVISOR_NOT_ENABLED);

    model.contactor_req = CONTACTORS_REQUEST_CLOSE;

    // No supervisor reply yet: stay open, keep the request
    stored_millis += 100;
    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_CLOSE);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_NOT_ENABLED), LEVEL_WARNING);

    // Supervisor replying but not enabled (still settling)
    model.supervisor.version = SUP_PROTOCOL_VERSION;
    model.supervisor.state = SUP_STATE_OFF;
    model.supervisor.reasons = SUP_REASON_SETTLING;
    model.supervisor_millis = stored_millis;
    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_CLOSE);

    // Enabled: the self-test starts and the warning clears
    expect_all_open();
    tick_sm(&model, 100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_PRE_CLOSED);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_NOT_ENABLED), LEVEL_NONE);
}

static void test_stale_supervisor_reply_holds_close(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    model.supervisor.version = SUP_PROTOCOL_VERSION;
    model.supervisor.state = SUP_STATE_ON;
    stored_millis += 100;
    model.supervisor_millis = stored_millis;
    stored_millis += SUPERVISOR_REPLY_STALE_MS + 100;

    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_CLOSE);
}

static void test_supervisor_drop_while_closed_is_fatal(void **state) {
    (void) state;
    bms_model_t model = {0};
    clear_bms_event(ERR_SUPERVISOR_DISABLED_CONTACTORS);

    model.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model.high_voltages.pos_contactor = 0.1f;
    model.high_voltages.neg_contactor = 0.1f;
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 100);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_DISABLED_CONTACTORS), LEVEL_NONE);

    // The supervisor latches off
    model.supervisor.state = SUP_STATE_LATCHED;
    model.supervisor.latch_reasons = SUP_REASON_CELL_HIGH;
    stored_millis += 100;
    model.supervisor_millis = stored_millis;
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    contactor_sm_tick(&model);
    events_tick();
    assert_int_equal(get_event_level(ERR_SUPERVISOR_DISABLED_CONTACTORS), LEVEL_FATAL);
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
    model.high_voltages.pos_contactor = 15.0f; // Above threshold (open)
    model.high_voltages.neg_contactor = 5.0f;  // Above threshold (open)
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_NEG_CLOSED);

    // 4. TESTING_NEG_CLOSED
    model.high_voltages.neg_contactor = 0.1f; // Below threshold (closed)
    expect_value(contactors_set_pos_pre_neg, pos, false);
#ifdef PRECHARGE_ON_NEGATIVE
    // pre is logically true here (leaves the bypass contactor open)
    expect_value(contactors_set_pos_pre_neg, pre, true);
#else
    expect_value(contactors_set_pos_pre_neg, pre, false);
#endif
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_POS_OPEN);

    // 5. TESTING_POS_OPEN
    model.high_voltages.neg_contactor = 5.0f;  // Above threshold (open)
    model.high_voltages.pos_contactor = 15.0f; // Above threshold (open)
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_POS_CLOSED);

    // 6. TESTING_POS_CLOSED
    model.high_voltages.pos_contactor = 0.5f; // Below threshold (closed)
    expect_value(contactors_set_pos_pre_neg, pos, true);
#ifdef PRECHARGE_ON_NEGATIVE
    expect_value(contactors_set_pos_pre_neg, pre, false);
#else
    // pre is logically true here (leaves the bypass contactor open)
    expect_value(contactors_set_pos_pre_neg, pre, true);
#endif
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_PRECHARGING_INIT);
}

static void test_precharge_success(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    // Force state to PRECHARGING
    model.contactor_sm.state = CONTACTORS_STATE_PRECHARGING;
    model.contactor_sm.last_transition_time = stored_millis64;

    // Set up model for successful precharge
    model.high_voltages.battery = 400.0f;
    model.high_voltages.battery_millis = stored_millis;
    model.high_voltages.output = 399.0f; // 1V diff
    model.high_voltages.output_millis = stored_millis;
    model.high_voltages.neg_contactor = 0.1f; // < threshold
    model.high_voltages.neg_contactor_millis = stored_millis;
#if BMS_BOARD == BMS_BOARD_REV1
    model.high_voltages.link = 399.5f; // link-output diff within limit
    model.high_voltages.link_millis = stored_millis;
#endif
    model.current_mA = 300.0f; // < PRECHARGE_SUCCESS_MAX_MA
    model.current_filtered_mA = 300.0f;
    model.contactor_sm.pre_close_current_mA = 300.0f;
    model.current_millis = stored_millis;

#ifdef PRECHARGE_ON_NEGATIVE
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, false);
#else
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#endif
    
    tick_sm(&model, 1100);

    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CLOSED);
}

static void test_precharge_failure_timeout(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    model.contactor_sm.state = CONTACTORS_STATE_PRECHARGING;
    model.contactor_sm.last_transition_time = stored_millis64;

    // Output voltage never rises
    model.high_voltages.battery = 400.0f;
    model.high_voltages.battery_millis = stored_millis;
    model.high_voltages.output = 0.0f;
    model.high_voltages.output_millis = stored_millis;
    model.high_voltages.neg_contactor = 0.0f;
    model.high_voltages.neg_contactor_millis = stored_millis;
#if BMS_BOARD == BMS_BOARD_REV1
    model.high_voltages.link = 400.0f; // link is live, output never rises
    model.high_voltages.link_millis = stored_millis;
#endif
    model.current_millis = stored_millis;

#ifdef PRECHARGE_ON_NEGATIVE
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, false);
#else
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#endif
    
    tick_sm(&model, 15000);

    assert_int_equal(get_event_count(ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH), 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_PRECHARGE_FAILED);
}

#if BMS_BOARD == BMS_BOARD_REV1
static void test_precharge_failure_link_voltage(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_PRECHARGING;
    model.contactor_sm.last_transition_time = stored_millis64;

    // Battery vs output looks acceptable, but the directly-sensed link rail
    // shows a large drop across the PTC + FC negative contactor path
    model.high_voltages.battery = 400.0f;
    model.high_voltages.battery_millis = stored_millis;
    model.high_voltages.output = 390.0f; // 10V diff, within PRECHARGE_SUCCESS_MAX_MV
    model.high_voltages.output_millis = stored_millis;
    model.high_voltages.neg_contactor = 0.1f;
    model.high_voltages.neg_contactor_millis = stored_millis;
    model.high_voltages.link = 410.0f; // 20V from output, above PRECHARGE_SUCCESS_LINK_MAX_MV
    model.high_voltages.link_millis = stored_millis;
    model.current_mA = 300.0f;
    model.current_filtered_mA = 300.0f;
    model.contactor_sm.pre_close_current_mA = 300.0f;
    model.current_millis = stored_millis;

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, false);

    // Event counts accumulate across tests, so compare against the count
    // before the failing tick
    int16_t count_before = get_event_count(ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH);

    tick_sm(&model, 15000);

    assert_int_equal(get_event_count(ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH), count_before + 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_PRECHARGE_FAILED);
}
#endif

static void test_pos_weld_failure_detection(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    // Force state to TESTING_POS_OPEN
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_POS_OPEN;
    model.contactor_sm.last_transition_time = stored_millis64;

    // Set up model to simulate POS contactor weld (stuck closed)
    model.high_voltages.pos_contactor = 0.1f; // Below threshold (closed)
    model.high_voltages.pos_contactor_millis = stored_millis;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    
    tick_sm(&model, 2100);

    assert_int_equal(get_event_count(ERR_CONTACTOR_POS_STUCK_CLOSED), 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_FAILED);
}

// The inverter holds the output at about pack voltage while both contactors are
// open: the open check would read "stuck closed". Instead of the FATAL weld
// event, the BMS skips the weld test and precharges onto the port.
static void test_energized_port_skips_weld_test(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_TESTING_POS_OPEN;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.high_voltages.battery = 388.0f;
    model.high_voltages.output = 385.0f;
    model.high_voltages.pos_contactor = 0.5f; // looks closed
    model.high_voltages.neg_contactor = 2.9f; // looks closed

    int16_t welds_before = get_event_count(ERR_CONTACTOR_POS_STUCK_CLOSED);
    int16_t energized_before = get_event_count(ERR_CONTACTOR_PORT_ENERGIZED);

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 2100);

    assert_int_equal(get_event_count(ERR_CONTACTOR_POS_STUCK_CLOSED), welds_before);
    assert_int_equal(get_event_count(ERR_CONTACTOR_PORT_ENERGIZED), energized_before + 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_PRECHARGING_INIT);
    // The later "still closed" check must tolerate the pos channel's small
    // offset once closed, as it does after a normal (dead-port) test
    assert_float_equal(model.contactor_sm.pre_close_pos_contactor_voltage, 388.0f, 0.01f);
}

// After a hot-join the contactors must stay CLOSED even though the pos channel
// reads a few volts (seen on ck0: -3.2 V with the contactor closed)
static void test_hot_join_stays_closed(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.contactor_sm.pre_close_pos_contactor_voltage = 388.0f; // as set by the hot-join
    model.high_voltages.battery = 388.8f;
    model.high_voltages.output = 384.0f;
    model.high_voltages.pos_contactor = -3.2f;
    model.high_voltages.neg_contactor = 0.2f;
#if BMS_BOARD == BMS_BOARD_REV1
    model.high_voltages.link = 382.5f;
    model.high_voltages.fuse_drop = 1.0f;
#endif

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 2500);

    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CLOSED);
}

// A dead output (no inverter energizing it) still detects a weld immediately
static void test_dead_port_weld_still_detected(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_TESTING_POS_OPEN;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.high_voltages.battery = 388.0f;
    model.high_voltages.output = 0.5f;
    model.high_voltages.pos_contactor = 0.1f;
    model.high_voltages.neg_contactor = 200.0f;

    int16_t energized_before = get_event_count(ERR_CONTACTOR_PORT_ENERGIZED);

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 2100);

    assert_int_equal(get_event_count(ERR_CONTACTOR_PORT_ENERGIZED), energized_before);
    assert_int_not_equal(get_event_level(ERR_CONTACTOR_POS_STUCK_CLOSED), LEVEL_NONE);
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

    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);

    // The current is already low, but a requested open holds the zero limits
    // long enough for the inverter to see them
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, CONTACTORS_REQUESTED_OPEN_HOLD_MS - 100);

    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);
    assert_false(model.contactor_sm.enable_current);

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 100);

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
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);

    // Drop current to allow opening once the hold for the inverter is over
    model.current_mA = 0;

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 2000);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, CONTACTORS_REQUESTED_OPEN_HOLD_MS - 2000);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
}

static void test_force_open_low_current_is_immediate(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model.contactor_req = CONTACTORS_REQUEST_FORCE_OPEN;

    model.current_mA = 500;

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 10);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);

    // No hold for the inverter when forced
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 10);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
}

static void test_force_open_ends_requested_open_hold(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model.contactor_req = CONTACTORS_REQUEST_OPEN;

    model.current_mA = 500;

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 10);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 1000);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);

    // A fault during the hold opens at once
    model.contactor_req = CONTACTORS_REQUEST_FORCE_OPEN;

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 10);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_NULL);
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
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);

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
    tick_sm(&model, 1001);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CALIBRATING_PRECHARGE_INIT);
}

static void test_precharging_aborts_immediately_on_open_request(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_PRECHARGING;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.contactor_req = CONTACTORS_REQUEST_OPEN;

    // Force/open request is handled before state switch body, so this tick
    // executes OPEN outputs immediately.
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 10);

    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_NULL);
}

static void test_precharge_failed_recovers_to_open_after_cooldown(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_PRECHARGE_FAILED;
    model.contactor_sm.last_transition_time = stored_millis64;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 10000);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_PRECHARGE_FAILED);

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 10001);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
}

static void test_testing_failed_recovers_to_open_after_timeout(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_TESTING_FAILED;
    model.contactor_sm.last_transition_time = stored_millis64;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 15000);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_FAILED);

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 5001);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
}

static void test_force_open_from_testing_state_is_immediate(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_TESTING_NEG_CLOSED;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.contactor_req = CONTACTORS_REQUEST_FORCE_OPEN;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1);

    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_NULL);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_initial_state),
        cmocka_unit_test(test_close_request_to_testing_sequence),
        cmocka_unit_test(test_close_held_until_supervisor_enables),
        cmocka_unit_test(test_stale_supervisor_reply_holds_close),
        cmocka_unit_test(test_supervisor_drop_while_closed_is_fatal),
        cmocka_unit_test(test_precharge_success),
        cmocka_unit_test(test_precharge_failure_timeout),
#if BMS_BOARD == BMS_BOARD_REV1
        cmocka_unit_test(test_precharge_failure_link_voltage),
#endif
        cmocka_unit_test(test_pos_weld_failure_detection),
        cmocka_unit_test(test_energized_port_skips_weld_test),
        cmocka_unit_test(test_hot_join_stays_closed),
        cmocka_unit_test(test_dead_port_weld_still_detected),
        cmocka_unit_test(test_open_request_from_closed),
        cmocka_unit_test(test_delayed_open_request_from_closed),
        cmocka_unit_test(test_force_open_request),
        cmocka_unit_test(test_force_open_low_current_is_immediate),
        cmocka_unit_test(test_force_open_ends_requested_open_hold),
        cmocka_unit_test(test_calibrate_request),
        cmocka_unit_test(test_precharging_aborts_immediately_on_open_request),
        cmocka_unit_test(test_precharge_failed_recovers_to_open_after_cooldown),
        cmocka_unit_test(test_testing_failed_recovers_to_open_after_timeout),
        cmocka_unit_test(test_force_open_from_testing_state_is_immediate),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
