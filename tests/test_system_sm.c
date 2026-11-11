#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sys/logging/logging.h"

void logging_printf(log_level_t level, const char *fmt, ...) {
    (void)level;
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

#include "app/model.h"
#include "app/state_machines/system.h"
#include "app/state_machines/base.h"
#include "sys/events/events.h"
#include "config/limits.h"

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;

// Mock NVM functions
void nvm_schedule_save_persistent_fast(bms_model_t *model) { (void)model; }

// Stub for INA228 (not used in these tests)
void ina228_calibrate_offset(void) {}

// External model
extern bms_model_t model;

// ============================================================================
// Test Helpers
// ============================================================================

static void advance_time(uint32_t ms) {
    stored_millis += ms;
    stored_millis64 += ms;
}

static void setup_fresh_timestamps(bms_model_t *m) {
    m->high_voltages.battery_millis = stored_millis;
    m->high_voltages.output_millis = stored_millis;
    m->high_voltages.neg_contactor_millis = stored_millis;
    m->high_voltages.pos_contactor_millis = stored_millis;
#if IS_BMS_BOARD(REV1)
    m->high_voltages.link_millis = stored_millis;
#endif
    m->current_millis = stored_millis;
    m->cell_voltage_millis = stored_millis;
    m->temperature_millis = stored_millis;
    m->thermistor_temperatures_millis = stored_millis;
}

static void reset_model_for_test(bms_model_t *m) {
    memset(m, 0, sizeof(bms_model_t));
    setup_fresh_timestamps(m);
    m->temperature_max = 25.0f;
    m->temperature_min = 25.0f;
}

static void reset_events(void) {
    extern bms_event_slot_t bms_event_slots[ERR_HIGHEST];
    for(int i = 0; i < ERR_HIGHEST; i++) {
        bms_event_slots[i] = (bms_event_slot_t){0};
        bms_event_slots[i].level = LEVEL_WARNING;
        clear_bms_event(i);
    }
}

static int setup(void **state) {
    (void) state;
    stored_millis = 1000;
    stored_millis64 = 1000;
    reset_events();
    return 0;
}

// ============================================================================
// Initial State Tests
// ============================================================================

static void test_system_starts_uninitialized(void **state) {
    (void) state;
    bms_model_t m = {0};
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_UNINITIALIZED);
}

static void test_system_transitions_to_initializing(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_INITIALIZING);
}

// ============================================================================
// Initialization Tests
// ============================================================================

static void test_system_stays_initializing_without_data(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_INITIALIZING;
    
    // Make data stale
    m.high_voltages.battery_millis = 0;
    
    advance_time(5000);
    system_sm_tick(&m);
    
    // Should still be initializing (waiting for valid data)
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_INITIALIZING);
}

static void test_system_becomes_inactive_after_init(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_INITIALIZING;
    m.system_sm.last_transition_time = stored_millis64;
    
    // Wait for init timeout (10s)
    advance_time(11000);
    setup_fresh_timestamps(&m);
    
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_INACTIVE);
}

#if IS_BMS_BOARD(REV1)
static void test_rev1_init_needs_link_reading(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_INITIALIZING;
    m.system_sm.last_transition_time = stored_millis64;

    // Every other reading is fresh, but there has never been a link reading
    // (e.g. ADC B isn't responding)
    advance_time(5000);
    setup_fresh_timestamps(&m);
    m.high_voltages.link_millis = 0;
    system_sm_tick(&m);
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_INITIALIZING);

    // Past the init timeout, it gives up rather than becoming inactive
    advance_time(6000);
    setup_fresh_timestamps(&m);
    m.high_voltages.link_millis = 0;
    system_sm_tick(&m);
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_FAULT);
}
#endif

// ============================================================================
// Request Handling Tests
// ============================================================================

static void test_run_request_from_inactive(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_INACTIVE;
    m.system_sm.last_transition_time = stored_millis64;
    
    m.system_req = SYSTEM_REQUEST_RUN;
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_OPERATING);
    assert_true(m.operating);
    assert_int_equal(m.system_req, SYSTEM_REQUEST_NULL);
}

static void test_stop_request_from_operating(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_OPERATING;
    m.system_sm.last_transition_time = stored_millis64;
    m.operating = true;
    
    m.system_req = SYSTEM_REQUEST_STOP;
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_INACTIVE);
    assert_false(m.operating);
    assert_int_equal(m.contactor_req, CONTACTORS_REQUEST_OPEN);
}

static void test_calibrate_request_from_inactive(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_INACTIVE;
    m.system_sm.last_transition_time = stored_millis64;
    
    m.system_req = SYSTEM_REQUEST_CALIBRATE_OFFLINE_LONG;
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_CALIBRATING);
    assert_int_equal(m.contactor_req, CONTACTORS_REQUEST_CALIBRATE);
    assert_int_equal(m.system_req, SYSTEM_REQUEST_NULL);
}

// ============================================================================
// Fault Handling Tests
// ============================================================================

static void test_fatal_event_causes_fault(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_OPERATING;
    m.system_sm.last_transition_time = stored_millis64;
    m.operating = true;
    
    // Raise a fatal event
    raise_bms_event(ERR_RESTARTING, 0);
    
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_FAULT);
    assert_false(m.operating);
}

static void test_fault_state_forces_contactors_open(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_FAULT;
    m.system_sm.last_transition_time = stored_millis64;
    
    system_sm_tick(&m);
    
    assert_int_equal(m.contactor_req, CONTACTORS_REQUEST_FORCE_OPEN);
}

static void test_fatal_event_from_initializing(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_INITIALIZING;
    m.system_sm.last_transition_time = stored_millis64;
    
    raise_bms_event(ERR_RESTARTING, 0);
    
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_FAULT);
}

static void test_fatal_event_from_inactive(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_INACTIVE;
    m.system_sm.last_transition_time = stored_millis64;
    
    raise_bms_event(ERR_RESTARTING, 0);
    
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_FAULT);
}

// ============================================================================
// Operating State Tests
// ============================================================================

static void test_operating_continuously_requests_close(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_OPERATING;
    m.system_sm.last_transition_time = stored_millis64;
    m.operating = true;
    
    // Clear contactor request
    m.contactor_req = CONTACTORS_REQUEST_OPEN;
    
    system_sm_tick(&m);
    
    // Operating state keeps requesting close
    assert_int_equal(m.contactor_req, CONTACTORS_REQUEST_CLOSE);
}

// ============================================================================
// Calibration Tests
// ============================================================================

static void test_calibration_completes_returns_to_inactive(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_CALIBRATING;
    m.system_sm.last_transition_time = stored_millis64;
    
    // Calibration completes (goes to idle state)
    m.calibration_sm.state = CALIBRATION_STATE_IDLE;
    
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_INACTIVE);
    assert_int_equal(m.contactor_req, CONTACTORS_REQUEST_OPEN);
}

static void test_calibration_timeout_causes_fault(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model_for_test(&m);
    m.system_sm.state = SYSTEM_STATE_CALIBRATING;
    m.system_sm.last_transition_time = stored_millis64;
    
    // Calibration never completes
    m.calibration_sm.state = CALIBRATION_STATE_LONG_MEASURING;
    
    // Wait past timeout (120s)
    advance_time(121000);
    
    system_sm_tick(&m);
    
    assert_int_equal(m.system_sm.state, SYSTEM_STATE_FAULT);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    const struct CMUnitTest tests[] = {
        // Initial state
        cmocka_unit_test_setup(test_system_starts_uninitialized, setup),
        cmocka_unit_test_setup(test_system_transitions_to_initializing, setup),
        
        // Initialization
        cmocka_unit_test_setup(test_system_stays_initializing_without_data, setup),
        cmocka_unit_test_setup(test_system_becomes_inactive_after_init, setup),
#if IS_BMS_BOARD(REV1)
        cmocka_unit_test_setup(test_rev1_init_needs_link_reading, setup),
#endif
        
        // Request handling
        cmocka_unit_test_setup(test_run_request_from_inactive, setup),
        cmocka_unit_test_setup(test_stop_request_from_operating, setup),
        cmocka_unit_test_setup(test_calibrate_request_from_inactive, setup),
        
        // Fault handling
        cmocka_unit_test_setup(test_fatal_event_causes_fault, setup),
        cmocka_unit_test_setup(test_fault_state_forces_contactors_open, setup),
        cmocka_unit_test_setup(test_fatal_event_from_initializing, setup),
        cmocka_unit_test_setup(test_fatal_event_from_inactive, setup),
        
        // Operating state
        cmocka_unit_test_setup(test_operating_continuously_requests_close, setup),
        
        // Calibration
        cmocka_unit_test_setup(test_calibration_completes_returns_to_inactive, setup),
        cmocka_unit_test_setup(test_calibration_timeout_causes_fault, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
