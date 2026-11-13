#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

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

void contactors_test_pre_bypass(bool pre) {
    check_expected(pre);
}

// A fresh reply from a supervisor enabling the contactor drivers for our map
static void supervisor_enabled(bms_model_t *model) {
    model->supervisor.version = SUP_PROTOCOL_VERSION;
    model->supervisor.state = SUP_STATE_ON;
    model->supervisor.chemistry = CHEMISTRY;
    model->supervisor.map_hash = cell_map_hash(bms_cell_map());
    model->supervisor_millis = stored_millis;
}

// Test helper to tick the SM multiple times if needed and advance time
void tick_sm(bms_model_t *model, uint32_t ms) {
    stored_millis += ms;
    stored_millis64 += ms;

    model->high_voltages.battery_millis = stored_millis;
    model->high_voltages.output_millis = stored_millis;
    model->high_voltages.pos_contactor_millis = stored_millis;
    model->high_voltages.neg_contactor_millis = stored_millis;
#if IS_BMS_BOARD(REV1)
    model->high_voltages.link_millis = stored_millis;
    model->high_voltages.fuse_drop_millis = stored_millis;
#endif
    model->current_millis = stored_millis;
    supervisor_enabled(model);

    contactor_sm_tick(model);
}

static void expect_all_open(void) {
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
}

static void expect_closed(void) {
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
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
    supervisor_enabled(&model);
    model.supervisor.state = SUP_STATE_OFF;
    model.supervisor.reasons = SUP_REASON_SETTLING;
    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_CLOSE);

    // Enabled: the self-test starts and the warning clears
    expect_all_open();
    tick_sm(&model, 100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_BYPASS_CLOSED);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_NOT_ENABLED), LEVEL_NONE);
}

static void test_calibration_held_until_supervisor_enables(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_req = CONTACTORS_REQUEST_CALIBRATE;
    stored_millis += 100;
    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_CALIBRATE);

    model.contactor_req = CONTACTORS_REQUEST_CALIBRATE_ONLY_NEG;
    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_CALIBRATE_ONLY_NEG);
}

static void test_open_request_not_held(void **state) {
    (void) state;
    bms_model_t model = {0};
    clear_bms_event(ERR_SUPERVISOR_NOT_ENABLED);

    // Opening never waits for the supervisor, and isn't a reason to warn
    model.contactor_req = CONTACTORS_REQUEST_OPEN;
    stored_millis += 100;
    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_NULL);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_NOT_ENABLED), LEVEL_NONE);
}

static void test_supervisor_enforcing_another_map_holds_close(void **state) {
    (void) state;
    bms_model_t model = {0};
    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    stored_millis += 100;

    // ON, but checking the cells against a map that isn't ours
    supervisor_enabled(&model);
    model.supervisor.map_hash ^= 1;
    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);

    // Or against another chemistry's limits
    supervisor_enabled(&model);
    model.supervisor.chemistry = (CHEMISTRY == LFP) ? NMC : LFP;
    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);

    // Or in another protocol version
    supervisor_enabled(&model);
    model.supervisor.version = SUP_PROTOCOL_VERSION + 1;
    expect_all_open();
    contactor_sm_tick(&model);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_CLOSE);
}

static void test_stale_supervisor_reply_holds_close(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    stored_millis += 100;
    supervisor_enabled(&model);
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
    expect_closed();
    tick_sm(&model, 100);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_DISABLED_CONTACTORS), LEVEL_NONE);

    // The supervisor latches off
    stored_millis += 100;
    supervisor_enabled(&model);
    model.supervisor.state = SUP_STATE_LATCHED;
    model.supervisor.latch_reasons = SUP_REASON_CELL_HIGH;
    expect_closed();
    contactor_sm_tick(&model);
    events_tick();
    assert_int_equal(get_event_level(ERR_SUPERVISOR_DISABLED_CONTACTORS), LEVEL_FATAL);
    // Its state and latch reasons are in the event data
    assert_int_equal(bms_event_slots[ERR_SUPERVISOR_DISABLED_CONTACTORS].data64,
        ((uint64_t)SUP_STATE_LATCHED << 32) | ((uint64_t)SUP_REASON_CELL_HIGH << 16));
}

// A fatal event can't be cleared, so reset its slot directly
static void reset_supervisor_disabled_event(void) {
    memset(&bms_event_slots[ERR_SUPERVISOR_DISABLED_CONTACTORS], 0, sizeof(bms_event_slots[0]));
}

// The failed states hold everything open while they wait, but they're outside
// OPEN, where the supervisor has no normal reason to drop (no slow mode there)
static void test_supervisor_drop_in_failed_states_is_fatal(void **state) {
    (void) state;
    const int failed_states[] = {CONTACTORS_STATE_TESTING_FAILED, CONTACTORS_STATE_PRECHARGE_FAILED};
    for (size_t i = 0; i < sizeof(failed_states) / sizeof(failed_states[0]); i++) {
        bms_model_t model = {0};
        reset_supervisor_disabled_event();
        model.contactor_sm.state = failed_states[i];
        model.contactor_sm.last_transition_time = stored_millis64;

        expect_all_open();
        tick_sm(&model, 100);
        events_tick();
        assert_int_equal(get_event_level(ERR_SUPERVISOR_DISABLED_CONTACTORS), LEVEL_NONE);

        // Partway through the cooldown, the supervisor drops (a cell below its limit)
        stored_millis += 100;
        supervisor_enabled(&model);
        model.supervisor.state = SUP_STATE_OFF;
        model.supervisor.reasons = SUP_REASON_CELL_LOW;
        expect_all_open();
        contactor_sm_tick(&model);
        events_tick();
        assert_int_equal(model.contactor_sm.state, failed_states[i]);
        assert_int_equal(get_event_level(ERR_SUPERVISOR_DISABLED_CONTACTORS), LEVEL_FATAL);
        assert_int_equal(bms_event_slots[ERR_SUPERVISOR_DISABLED_CONTACTORS].data64,
            ((uint64_t)SUP_STATE_OFF << 32) | SUP_REASON_CELL_LOW);
    }
    reset_supervisor_disabled_event();
}

// In OPEN the supervisor drops routinely (in slow mode the scans are too far
// apart for it), so that's never an event; a close just waits for it
static void test_supervisor_drop_while_open_is_not_fatal(void **state) {
    (void) state;
    bms_model_t model = {0};
    reset_supervisor_disabled_event();
    clear_bms_event(ERR_SUPERVISOR_NOT_ENABLED);

    expect_all_open();
    tick_sm(&model, 100);

    stored_millis += 100;
    supervisor_enabled(&model);
    model.supervisor.state = SUP_STATE_OFF;
    model.supervisor.reasons = SUP_REASON_NO_SCAN;
    expect_all_open();
    contactor_sm_tick(&model);
    events_tick();
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_DISABLED_CONTACTORS), LEVEL_NONE);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_NOT_ENABLED), LEVEL_NONE);

    // A close request while it's off waits for it, and only warns
    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    expect_all_open();
    contactor_sm_tick(&model);
    events_tick();
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_CLOSE);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_NOT_ENABLED), LEVEL_WARNING);
    assert_int_equal(get_event_level(ERR_SUPERVISOR_DISABLED_CONTACTORS), LEVEL_NONE);
    clear_bms_event(ERR_SUPERVISOR_NOT_ENABLED);
}

static void expect_close_pre_side_via_precharge(void) {
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, false);
}

static void expect_close_other_side_direct(void) {
#if IS_PRECHARGE_ON_NEGATIVE()
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
#else
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#endif
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
    
    // 1. Transition to TESTING_BYPASS_CLOSED
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 0);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_BYPASS_CLOSED);

    // 2. TESTING_BYPASS_CLOSED
    model.precharge_closed = true;
    expect_value(contactors_test_pre_bypass, pre, true);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_INITIALLY_OPEN);

    // 3. TESTING_INITIALLY_OPEN
    model.high_voltages.pos_contactor = 15.0f; // Above threshold (open)
    model.high_voltages.neg_contactor = 5.0f;  // Above threshold (open)
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_PRE_SIDE_CLOSED);

    // 4. Precharge side closed via the precharge element
    model.high_voltages.neg_contactor = 0.1f; // Below threshold (closed)
    model.high_voltages.pos_contactor = 0.5f; // Below threshold (closed)
    expect_close_pre_side_via_precharge();
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_REOPENED);

    // 5. TESTING_REOPENED
    model.high_voltages.neg_contactor = 5.0f;  // Above threshold (open)
    model.high_voltages.pos_contactor = 15.0f; // Above threshold (open)
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_OTHER_SIDE_CLOSED);

    // 6. Other side closed directly
    model.high_voltages.neg_contactor = 0.1f; // Below threshold (closed)
    model.high_voltages.pos_contactor = 0.5f; // Below threshold (closed)
    expect_close_other_side_direct();
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
#if IS_BMS_BOARD(REV1)
    model.high_voltages.link = 399.5f; // link-output diff within limit
    model.high_voltages.link_millis = stored_millis;
#endif
    model.current_mA = 300.0f; // < PRECHARGE_SUCCESS_MAX_MA
    model.current_filtered_mA = 300.0f;
    model.contactor_sm.pre_close_current_mA = 300.0f;
    model.current_millis = stored_millis;

#if IS_PRECHARGE_ON_NEGATIVE()
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
#if IS_BMS_BOARD(REV1)
    model.high_voltages.link = 400.0f; // link is live, output never rises
    model.high_voltages.link_millis = stored_millis;
#endif
    model.current_millis = stored_millis;

#if IS_PRECHARGE_ON_NEGATIVE()
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

#if IS_BMS_BOARD(REV1)
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

#if IS_PRECHARGE_ON_NEGATIVE()
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, false);
#else
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#endif

    // Event counts accumulate across tests, so compare against the count
    // before the failing tick
    int16_t count_before = get_event_count(ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH);

    tick_sm(&model, 15000);

    assert_int_equal(get_event_count(ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH), count_before + 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_PRECHARGE_FAILED);
}
#endif

#if IS_BMS_BOARD(REV1)
// Checks made while closed. Each test starts with the contactors closed and
// settled, with healthy readings.

static void closed_test_setup(bms_model_t *model) {
    // Fatal events can't be cleared, so reset these between tests
    memset(&bms_event_slots[ERR_FUSE_VOLTAGE_DROP_HIGH], 0, sizeof(bms_event_slot_t));
    memset(&bms_event_slots[ERR_LINK_VOLTAGE_MISMATCH], 0, sizeof(bms_event_slot_t));

    model->contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model->contactor_sm.last_transition_time = stored_millis64;
    model->high_voltages.battery = 400.0f;
    model->high_voltages.output = 400.0f;
    model->high_voltages.link = 400.0f;
    model->high_voltages.pos_contactor = 0.0f;
    model->high_voltages.neg_contactor = 0.0f;
    model->high_voltages.fuse_drop = 0.0f;
    model->current_mA = 50000;

    // Bring the event escalation timer up to date
    events_tick();
}

// Tick in the closed state for ms, in 100ms steps
static void tick_closed(bms_model_t *model, uint32_t ms) {
    for(uint32_t t = 0; t < ms; t += 100) {
        expect_value(contactors_set_pos_pre_neg, pos, true);
        expect_value(contactors_set_pos_pre_neg, pre, false);
        expect_value(contactors_set_pos_pre_neg, neg, true);
        tick_sm(model, 100);
        events_tick();
    }
}

static void test_link_mismatch_opens_contactors(void **state) {
    (void) state;
    bms_model_t model = {0};
    closed_test_setup(&model);
    tick_closed(&model, 2500);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CLOSED);

    // The link rail drops away from the battery (a link contactor has opened)
    model.high_voltages.link = 380.0f;
    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 100);

    assert_int_equal(get_event_level(ERR_LINK_VOLTAGE_MISMATCH), LEVEL_WARNING);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);
}

static void test_link_within_tolerance_stays_closed(void **state) {
    (void) state;
    bms_model_t model = {0};
    closed_test_setup(&model);

    // Battery and link are read by different ADCs, which may be calibrated
    // differently, so a smaller discrepancy is tolerated
    model.high_voltages.link = 390.0f;
    tick_closed(&model, 5000);

    assert_int_equal(get_event_level(ERR_LINK_VOLTAGE_MISMATCH), LEVEL_NONE);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CLOSED);
}

static void test_fuse_drop_sustained_is_fatal(void **state) {
    (void) state;
    bms_model_t model = {0};
    closed_test_setup(&model);
    tick_closed(&model, 2500);
    assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_NONE);

    model.high_voltages.fuse_drop = 50.0f;
    tick_closed(&model, FUSE_DROP_FAULT_DELAY_MS - 200);
    assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_CRITICAL);

    tick_closed(&model, 300);
    assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_FATAL);

    // Stays fatal even once the drop goes away (until reset)
    model.high_voltages.fuse_drop = 0.0f;
    tick_closed(&model, 500);
    assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_FATAL);
}

static void test_fuse_drop_spikes_are_not_fatal(void **state) {
    (void) state;
    bms_model_t model = {0};
    closed_test_setup(&model);
    tick_closed(&model, 2500);

    // Repeated brief spikes, with as long again between them
    for(int i = 0; i < 10; i++) {
        model.high_voltages.fuse_drop = 50.0f;
        tick_closed(&model, 500);
        assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_CRITICAL);
        model.high_voltages.fuse_drop = 0.0f;
        tick_closed(&model, 500);
        assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_NONE);
    }
}

static void test_fuse_drop_without_current_is_fatal(void **state) {
    (void) state;
    bms_model_t model = {0};
    closed_test_setup(&model);
    tick_closed(&model, 2500);

    // A blown fuse can itself stop the current, so it must still be detected
    model.current_mA = 0;
    model.high_voltages.fuse_drop = 50.0f;
    tick_closed(&model, FUSE_DROP_FAULT_DELAY_MS + 100);
    assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_FATAL);
}

static void test_fuse_drop_frozen_reading_is_not_fatal(void **state) {
    (void) state;
    bms_model_t model = {0};
    closed_test_setup(&model);
    tick_closed(&model, 2500);

    // A single high reading (e.g. a noise spike), after which ADC B stops
    // updating, so the same reading stays in place
    model.high_voltages.fuse_drop = 50.0f;
    tick_closed(&model, 100);
    assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_CRITICAL);
    millis_t frozen_reading = model.high_voltages.fuse_drop_millis;

    // The reading counts until it goes stale, which is sooner than the fault
    // delay, and then the event clears rather than escalating
    for(int i = 0; i < 30; i++) {
        expect_value(contactors_set_pos_pre_neg, pos, true);
        expect_value(contactors_set_pos_pre_neg, pre, false);
        expect_value(contactors_set_pos_pre_neg, neg, true);
        stored_millis += 100;
        stored_millis64 += 100;
        model.high_voltages.battery_millis = stored_millis;
        model.high_voltages.output_millis = stored_millis;
        model.high_voltages.pos_contactor_millis = stored_millis;
        model.high_voltages.neg_contactor_millis = stored_millis;
        model.high_voltages.link_millis = stored_millis;
        model.high_voltages.fuse_drop_millis = frozen_reading;
        model.current_millis = stored_millis;
        supervisor_enabled(&model);
        contactor_sm_tick(&model);
        events_tick();
        assert_int_not_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_FATAL);
    }
    assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_NONE);
}

static void test_fuse_drop_cleared_when_contactors_open(void **state) {
    (void) state;
    bms_model_t model = {0};
    closed_test_setup(&model);
    tick_closed(&model, 2500);

    model.high_voltages.fuse_drop = 50.0f;
    tick_closed(&model, 500);
    assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_CRITICAL);

    model.contactor_sm.state = CONTACTORS_STATE_OPEN;
    for(int i = 0; i < 20; i++) {
        expect_value(contactors_set_pos_pre_neg, pos, false);
        expect_value(contactors_set_pos_pre_neg, pre, false);
        expect_value(contactors_set_pos_pre_neg, neg, false);
        tick_sm(&model, 100);
        events_tick();
    }
    assert_int_equal(get_event_level(ERR_FUSE_VOLTAGE_DROP_HIGH), LEVEL_NONE);
}
#endif

static void test_pos_weld_failure_detection(void **state) {
    (void) state;
    bms_model_t model = {0};
    
    // Force state to TESTING_REOPENED
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_REOPENED;
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
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);

    // Drop current to allow opening
    model.current_mA = 0;

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

static void test_bypass_test_aborts_immediately_on_open_request(void **state) {
    (void) state;
    bms_model_t model = {0};

    // The precharge contactor alone is driven in this test step: an open
    // request must drop it at once, not after the step's wait
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_BYPASS_CLOSED;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.contactor_req = CONTACTORS_REQUEST_FORCE_OPEN;

    expect_all_open();
    tick_sm(&model, 10);

    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_NULL);
}

static void test_close_held_while_isolation_low(void **state) {
    (void) state;
    bms_model_t model = {0};
    memset(&bms_event_slots[ERR_ISOLATION_LOW], 0, sizeof(bms_event_slot_t));

    model.contactor_sm.state = CONTACTORS_STATE_OPEN;
    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    raise_bms_event(ERR_ISOLATION_LOW, 0);

    // Low isolation: stay open, keep the request
    for(int i = 0; i < 3; i++) {
        expect_all_open();
        tick_sm(&model, 100);
        assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
        assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_CLOSE);
    }

    // Isolation good again: the self-test starts
    clear_bms_event(ERR_ISOLATION_LOW);
    expect_all_open();
    tick_sm(&model, 100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_BYPASS_CLOSED);
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

// Mode 2 (assisted) helpers: voltages are role-based, since the precharge
// side is neg on rev1 (PRECHARGE_ON_NEGATIVE) and pos on rev0.
static void set_assisted_pre_pass_voltages(bms_model_t *model) {
#if IS_PRECHARGE_ON_NEGATIVE()
    model->high_voltages.neg_contactor = 0.1f; // pre closed (<=2V)
    model->high_voltages.pos_contactor = 15.0f; // other open (>=3V)
#else
    model->high_voltages.pos_contactor = 0.5f; // pre closed (<=5V)
    model->high_voltages.neg_contactor = 5.0f; // other open (>=3V)
#endif
}

static void set_assisted_other_pass_voltages(bms_model_t *model) {
#if IS_PRECHARGE_ON_NEGATIVE()
    model->high_voltages.pos_contactor = 0.5f; // other closed (<=2V)
    model->high_voltages.neg_contactor = 5.0f; // pre open (>=3V)
#else
    model->high_voltages.neg_contactor = 0.1f; // other closed (<=2V)
    model->high_voltages.pos_contactor = 15.0f; // pre open (>=10V)
#endif
}

static void set_assisted_other_weld_voltages(bms_model_t *model) {
    // Other side welded: stays small after pre-side PTC pull.
#if IS_PRECHARGE_ON_NEGATIVE()
    model->high_voltages.neg_contactor = 0.1f;
    model->high_voltages.pos_contactor = 0.1f;
#else
    model->high_voltages.pos_contactor = 0.5f;
    model->high_voltages.neg_contactor = 0.1f;
#endif
}

static void set_assisted_pre_weld_voltages(bms_model_t *model) {
    // Pre side welded: stays small after other-side direct close.
#if IS_PRECHARGE_ON_NEGATIVE()
    model->high_voltages.pos_contactor = 0.5f;
    model->high_voltages.neg_contactor = 0.1f;
#else
    model->high_voltages.neg_contactor = 0.1f;
    model->high_voltages.pos_contactor = 0.5f;
#endif
}

static void test_assisted_happy_path(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Dead bus, link floating near the precharge rail: pre side reads closed,
    // other side reads open. Passive both-open fails, but assisted gate passes.
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_INITIALLY_OPEN;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.high_voltages.battery = 400.0f;
    model.high_voltages.output = 0.0f;
    set_assisted_pre_pass_voltages(&model);
    model.precharge_closed = false;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_ASSISTED_PRE);

    // Tentative PTC pull: pre reads closed, shift proves other open.
    set_assisted_pre_pass_voltages(&model);
    model.precharge_closed = false;
    expect_close_pre_side_via_precharge();
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_ASSISTED_OPEN);

    // Deadband: open-all break-before-make, no voltage verdict.
    model.precharge_closed = false;
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_ASSISTED_OTHER);

    // Opposite pull: other reads closed, pre reads open.
    set_assisted_other_pass_voltages(&model);
    model.precharge_closed = false;
    expect_close_other_side_direct();
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_PRECHARGING_INIT);
    // Stored reference must be the pos *open* reading (15V on both board
    // variants: other-side shift on PRECHARGE_ON_NEGATIVE, pre-side shift
    // otherwise) - never the closed reading.
    assert_int_equal((int)(model.contactor_sm.pre_close_pos_contactor_voltage * 1000), 15000);
}

static void test_assisted_other_weld_detected(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_TESTING_INITIALLY_OPEN;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.high_voltages.battery = 400.0f;
    model.high_voltages.output = 0.0f;
    set_assisted_pre_pass_voltages(&model);
    model.precharge_closed = false;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_ASSISTED_PRE);

    // Other side welded: never looks open after the PTC pull.
    int16_t count_before = get_event_count(ERR_CONTACTOR_CLOSING_FAILED);
    set_assisted_other_weld_voltages(&model);
    model.precharge_closed = false;
    expect_close_pre_side_via_precharge();
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_FAILED);
    assert_int_equal(get_event_count(ERR_CONTACTOR_CLOSING_FAILED), count_before + 1);
}

static void test_assisted_pre_weld_detected(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Reach ASSISTED_OTHER via a passing first phase plus deadband.
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_ASSISTED_PRE;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.high_voltages.battery = 400.0f;
    model.high_voltages.output = 0.0f;
    set_assisted_pre_pass_voltages(&model);
    model.precharge_closed = false;

    expect_close_pre_side_via_precharge();
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_ASSISTED_OPEN);

    model.precharge_closed = false;
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_ASSISTED_OTHER);

    // Pre side welded: never looks open after the direct close.
    int16_t count_before = get_event_count(ERR_CONTACTOR_CLOSING_FAILED);
    set_assisted_pre_weld_voltages(&model);
    model.precharge_closed = false;
    expect_close_other_side_direct();
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_FAILED);
    assert_int_equal(get_event_count(ERR_CONTACTOR_CLOSING_FAILED), count_before + 1);
}

static void test_assisted_hot_bus_refused(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Live bus (hot-join): small battery-output difference. Assisted gate
    // denies, original 0x2000 failure path preserved.
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_INITIALLY_OPEN;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.high_voltages.battery = 400.0f;
    model.high_voltages.output = 399.0f;
    set_assisted_pre_pass_voltages(&model);
    model.precharge_closed = false;

    int16_t count_before = get_event_count(ERR_CONTACTOR_CLOSING_FAILED);
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_FAILED);
    assert_int_equal(get_event_count(ERR_CONTACTOR_CLOSING_FAILED), count_before + 1);
}

static void test_force_open_from_assisted_is_immediate(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_TESTING_ASSISTED_PRE;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.contactor_req = CONTACTORS_REQUEST_FORCE_OPEN;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_NULL);

    model.contactor_sm.state = CONTACTORS_STATE_TESTING_ASSISTED_OTHER;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.contactor_req = CONTACTORS_REQUEST_OPEN;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_NULL);

    model.contactor_sm.state = CONTACTORS_STATE_TESTING_ASSISTED_OPEN;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.contactor_req = CONTACTORS_REQUEST_FORCE_OPEN;

    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
    assert_int_equal(model.contactor_req, CONTACTORS_REQUEST_NULL);
}

static void test_pos_dropout_detected_despite_large_open_reference(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Forced full-pack open reference as stored on the assisted path, then
    // pos drops out onto a half-bled bus: 50V across it. Uncapped half-open
    // (200V) would still read closed; capped fallback (5V rev1 / 8V rev0)
    // must trip to AWAITING_OPEN.
    model.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.contactor_sm.pre_close_pos_contactor_voltage = 400.0f;
    model.high_voltages.battery = 400.0f;
    model.high_voltages.output = 350.0f;
    model.high_voltages.pos_contactor = 50.0f;
    model.high_voltages.neg_contactor = 0.1f;
#if IS_BMS_BOARD(REV1)
    model.high_voltages.link = 400.0f;
    model.high_voltages.fuse_drop = 0.0f;
#endif
    model.current_mA = 0;

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 2500);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_AWAITING_OPEN);
}

static void test_pos_closed_tolerates_small_offset(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Same large open reference, but a small closed residual (miscalibration
    // slop) must still read closed via the fallback band between the fixed
    // closed threshold and the open threshold: 2.5V on rev1 (2V/3V), 6V on
    // rev0 (5V/10V). Neither passes the fixed threshold, so this exercises
    // the capped fallback specifically.
    model.contactor_sm.state = CONTACTORS_STATE_CLOSED;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.contactor_sm.pre_close_pos_contactor_voltage = 400.0f;
    model.high_voltages.battery = 400.0f;
    model.high_voltages.output = 400.0f;
#if IS_BMS_BOARD(REV1)
    model.high_voltages.pos_contactor = 2.5f;
#else
    model.high_voltages.pos_contactor = 6.0f;
#endif
    model.high_voltages.neg_contactor = 0.1f;
#if IS_BMS_BOARD(REV1)
    model.high_voltages.link = 400.0f;
    model.high_voltages.fuse_drop = 0.0f;
#endif
    model.current_mA = 0;

    expect_value(contactors_set_pos_pre_neg, pos, true);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
    tick_sm(&model, 2500);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CLOSED);
}

static void test_calib_neg_init_proceeds_when_current_low(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_CALIBRATING_ONLY_NEG_INIT;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.current_mA = 0;

#if IS_PRECHARGE_ON_NEGATIVE()
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#else
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#endif
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_CALIBRATING_ONLY_NEG);
}

static void test_calib_neg_init_aborts_when_bus_live(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Quiet current but the bus is live (welded pos feeding it, or caps
    // holding charge): must bail without ever driving the direct close.
    model.contactor_sm.state = CONTACTORS_STATE_CALIBRATING_ONLY_NEG_INIT;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.current_mA = 0;
    model.high_voltages.output = 400.0f;

#if IS_PRECHARGE_ON_NEGATIVE()
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#else
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#endif
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
}

static void test_calib_neg_init_aborts_when_current_high(void **state) {
    (void) state;
    bms_model_t model = {0};

    // PTC probe already carrying fault current (e.g. welded pos + load):
    // must bail to OPEN without ever driving the direct close.
    model.contactor_sm.state = CONTACTORS_STATE_CALIBRATING_ONLY_NEG_INIT;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.current_mA = 3000;

#if IS_PRECHARGE_ON_NEGATIVE()
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, true);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#else
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, true);
#endif
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_OPEN);
}

static void test_assisted_deadband_bypass_weld_aborts(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Bypass welds shut mid-sequence and is still shut at the deadband:
    // must fail without ever driving the direct other-side close.
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_ASSISTED_OPEN;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.high_voltages.battery = 400.0f;
    model.high_voltages.output = 0.0f;
    set_assisted_other_pass_voltages(&model);
    model.precharge_closed = true;

    int16_t count_before = get_event_count(ERR_CONTACTOR_CLOSING_FAILED);
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 1100);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_FAILED);
    assert_int_equal(get_event_count(ERR_CONTACTOR_CLOSING_FAILED), count_before + 1);
}

static void test_assisted_bypass_weld_aborts_before_driving(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Bypass welded (aux closed) on entry to ASSISTED_PRE: must fail
    // immediately with open_all, never driving the PTC pull.
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_ASSISTED_PRE;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.high_voltages.battery = 400.0f;
    model.high_voltages.output = 0.0f;
    set_assisted_pre_pass_voltages(&model);
    model.precharge_closed = true;

    int16_t count_before = get_event_count(ERR_CONTACTOR_CLOSING_FAILED);
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 10);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_FAILED);
    assert_int_equal(get_event_count(ERR_CONTACTOR_CLOSING_FAILED), count_before + 1);

    // Bypass welded on entry to ASSISTED_OTHER: must fail with open_all,
    // never driving the direct other-side close (which would be a dead
    // short onto a discharged bus with no PTC in circuit).
    memset(&model, 0, sizeof(model));
    model.contactor_sm.state = CONTACTORS_STATE_TESTING_ASSISTED_OTHER;
    model.contactor_sm.last_transition_time = stored_millis64;
    model.high_voltages.battery = 400.0f;
    model.high_voltages.output = 0.0f;
    set_assisted_other_pass_voltages(&model);
    model.precharge_closed = true;

    count_before = get_event_count(ERR_CONTACTOR_CLOSING_FAILED);
    expect_value(contactors_set_pos_pre_neg, pos, false);
    expect_value(contactors_set_pos_pre_neg, pre, false);
    expect_value(contactors_set_pos_pre_neg, neg, false);
    tick_sm(&model, 10);
    assert_int_equal(model.contactor_sm.state, CONTACTORS_STATE_TESTING_FAILED);
    assert_int_equal(get_event_count(ERR_CONTACTOR_CLOSING_FAILED), count_before + 1);
}

static void test_force_open_from_testing_state_is_immediate(void **state) {
    (void) state;
    bms_model_t model = {0};

    model.contactor_sm.state = CONTACTORS_STATE_TESTING_PRE_SIDE_CLOSED;
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
        cmocka_unit_test(test_close_held_while_isolation_low),
        cmocka_unit_test(test_bypass_test_aborts_immediately_on_open_request),
        cmocka_unit_test(test_calibration_held_until_supervisor_enables),
        cmocka_unit_test(test_open_request_not_held),
        cmocka_unit_test(test_supervisor_enforcing_another_map_holds_close),
        cmocka_unit_test(test_stale_supervisor_reply_holds_close),
        cmocka_unit_test(test_supervisor_drop_while_closed_is_fatal),
        cmocka_unit_test(test_supervisor_drop_in_failed_states_is_fatal),
        cmocka_unit_test(test_supervisor_drop_while_open_is_not_fatal),
        cmocka_unit_test(test_precharge_success),
        cmocka_unit_test(test_precharge_failure_timeout),
#if IS_BMS_BOARD(REV1)
        cmocka_unit_test(test_precharge_failure_link_voltage),
        cmocka_unit_test(test_link_mismatch_opens_contactors),
        cmocka_unit_test(test_link_within_tolerance_stays_closed),
        cmocka_unit_test(test_fuse_drop_sustained_is_fatal),
        cmocka_unit_test(test_fuse_drop_spikes_are_not_fatal),
        cmocka_unit_test(test_fuse_drop_without_current_is_fatal),
        cmocka_unit_test(test_fuse_drop_frozen_reading_is_not_fatal),
        cmocka_unit_test(test_fuse_drop_cleared_when_contactors_open),
#endif
        cmocka_unit_test(test_pos_weld_failure_detection),
        cmocka_unit_test(test_open_request_from_closed),
        cmocka_unit_test(test_delayed_open_request_from_closed),
        cmocka_unit_test(test_force_open_request),
        cmocka_unit_test(test_calibrate_request),
        cmocka_unit_test(test_precharging_aborts_immediately_on_open_request),
        cmocka_unit_test(test_precharge_failed_recovers_to_open_after_cooldown),
        cmocka_unit_test(test_testing_failed_recovers_to_open_after_timeout),
        cmocka_unit_test(test_force_open_from_testing_state_is_immediate),
        cmocka_unit_test(test_assisted_happy_path),
        cmocka_unit_test(test_assisted_other_weld_detected),
        cmocka_unit_test(test_assisted_pre_weld_detected),
        cmocka_unit_test(test_assisted_hot_bus_refused),
        cmocka_unit_test(test_assisted_bypass_weld_aborts_before_driving),
        cmocka_unit_test(test_assisted_deadband_bypass_weld_aborts),
        cmocka_unit_test(test_pos_dropout_detected_despite_large_open_reference),
        cmocka_unit_test(test_pos_closed_tolerates_small_offset),
        cmocka_unit_test(test_calib_neg_init_proceeds_when_current_low),
        cmocka_unit_test(test_calib_neg_init_aborts_when_current_high),
        cmocka_unit_test(test_calib_neg_init_aborts_when_bus_live),
        cmocka_unit_test(test_force_open_from_assisted_is_immediate),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
