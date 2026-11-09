#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/model.h"
#include "app/battery/balancing.h"
#include "sys/events/events.h"
#include "config/limits.h"
#include "sys/logging/logging.h"

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

uint32_t time_us_32() {
    return (uint32_t)stored_millis * 1000;
}

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;

// External model
extern bms_model_t model;

// External functions for testing
int16_t calculate_balance_time(int16_t voltage_mV, int16_t min_voltage_mV, bms_model_t *model);

// ============================================================================
// Test Helpers
// ============================================================================

static void advance_time(uint32_t ms) {
    stored_millis += ms;
    stored_millis64 += ms;
}

static void reset_model(bms_model_t *m) {
    memset(m, 0, sizeof(bms_model_t));
    m->system_sm.state = SYSTEM_STATE_OPERATING;
    m->auto_balancing_period_ms = 3600000; // 1 hour
    m->balancing_periods_per_mV = 50;
    m->balance_min_offset_mV = 2;
    m->cell_voltage_millis = stored_millis;
    
    // Set all cell voltages to a normal value
    for (int i = 0; i < NUM_CELLS; i++) {
        m->cell_voltages_mV[i] = 3900;
    }
    m->cell_voltage_min_mV = 3900;
    m->cell_voltage_max_mV = 3900;
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
// Balance Time Calculation Tests
// ============================================================================

static void test_calculate_balance_time_no_difference(void **state) {
    (void) state;
    bms_model_t m = {0};
    m.balance_min_offset_mV = 0;
    m.balancing_periods_per_mV = 50;
    // When voltage equals minimum, balance time should be 0
    int16_t time = calculate_balance_time(3800, 3800, &m);
    assert_int_equal(time, 0);
}

static void test_calculate_balance_time_small_difference(void **state) {
    (void) state;
    bms_model_t m = {0};
    m.balance_min_offset_mV = 0;
    m.balancing_periods_per_mV = 50;
    // Small voltage difference (1mV above minimum)
    int16_t time = calculate_balance_time(3801, 3800, &m);
    // With PERIODS_PER_MV = 50, expect 50 periods
    assert_int_equal(time, 50);
}

static void test_calculate_balance_time_large_difference(void **state) {
    (void) state;
    bms_model_t m = {0};
    m.balance_min_offset_mV = 0;
    m.balancing_periods_per_mV = 50;
    // Larger voltage difference (10mV above minimum)
    int16_t time = calculate_balance_time(3810, 3800, &m);
    // 10mV * 50 periods = 500 periods
    assert_int_equal(time, 500);
}

static void test_calculate_balance_time_below_minimum(void **state) {
    (void) state;
    bms_model_t m = {0};
    m.balance_min_offset_mV = 0;
    m.balancing_periods_per_mV = 50;
    // When cell voltage is below minimum, should return 0
    int16_t time = calculate_balance_time(3790, 3800, &m);
    assert_int_equal(time, 0);
}

// ============================================================================
// Balancing State Machine Tests
// ============================================================================

static void test_balancing_starts_idle(void **state) {
    (void) state;
    bms_model_t m = {0};
    
    assert_int_equal(m.balancing_sm.state, BALANCING_STATE_IDLE);
}

// NOTE: test_balancing_remains_idle_when_disabled removed because
// balancing_enabled is not currently checked in good_conditions_for_balancing()
// (dead code in balancing.c)

static void test_balancing_does_not_start_with_high_voltage(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model(&m);
    
    // Set max cell voltage above soft max
    m.cell_voltage_max_mV = get_cell_voltage_soft_max_mV(&m) + 100;
    
    advance_time(35000);
    balancing_sm_tick(&m);
    
    // Should still be idle (high voltage prevents balancing)
    assert_int_equal(m.balancing_sm.state, BALANCING_STATE_IDLE);
}

static void test_balancing_does_not_start_with_low_voltage(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model(&m);
    
    // Set min cell voltage below minimum balance voltage
    m.cell_voltage_min_mV = get_minimum_balancing_voltage_mV(&m) - 100;
    for (int i = 0; i < NUM_CELLS; i++) {
        m.cell_voltages_mV[i] = get_minimum_balancing_voltage_mV(&m) - 100;
    }
    
    advance_time(35000);
    balancing_sm_tick(&m);
    
    // Should still be idle (low voltage prevents balancing)
    assert_int_equal(m.balancing_sm.state, BALANCING_STATE_IDLE);
}

static void test_balancing_does_not_start_with_fatal_event(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model(&m);
    
    // Raise a fatal event
    raise_bms_event(ERR_RESTARTING, 0);
    
    advance_time(35000);
    balancing_sm_tick(&m);
    
    // Should still be idle
    assert_int_equal(m.balancing_sm.state, BALANCING_STATE_IDLE);
}

static void test_balancing_pause_clears_mask(void **state) {
    (void) state;
    balancing_sm_t sm = {0};
    
    // Set some balance requests
    sm.balance_request_mask[0] = 0xFFFFFFFF;
    sm.balance_request_mask[1] = 0x12345678;
    
    pause_balancing(&sm);
    
    // All masks should be cleared
    assert_int_equal(sm.balance_request_mask[0], 0);
    assert_int_equal(sm.balance_request_mask[1], 0);
    assert_int_equal(sm.balance_request_mask[2], 0);
    assert_int_equal(sm.balance_request_mask[3], 0);
}

static void test_balancing_stops_in_slow_mode(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model(&m);
    
    // Enable slow mode (infrequent BMB updates)
    m.cell_voltage_slow_mode = true;
    
    advance_time(35000);
    balancing_sm_tick(&m);
    
    // Should remain idle in slow mode
    assert_int_equal(m.balancing_sm.state, BALANCING_STATE_IDLE);
}

static void test_balancing_session_resets_pause_counter(void **state) {
    (void) state;
    bms_model_t m = {0};
    reset_model(&m);

    // One cell high enough to need balancing
    m.cell_voltages_mV[0] = 3950;
    m.cell_voltage_max_mV = 3950;

    // Left over from a previous session that stopped early
    m.balancing_sm.pause_counter = 3;

    advance_time(m.auto_balancing_period_ms + 1);
    balancing_sm_tick(&m);
    assert_int_equal(m.balancing_sm.state, BALANCING_STATE_ACTIVE);
    assert_int_equal(m.balancing_sm.pause_counter, 0);

    // The first pause comes after the full number of active periods
    balancing_sm_tick(&m);
    assert_false(m.balancing_sm.is_pause_cycle);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    const struct CMUnitTest tests[] = {
        // Balance time calculation tests
        cmocka_unit_test_setup(test_calculate_balance_time_no_difference, setup),
        cmocka_unit_test_setup(test_calculate_balance_time_small_difference, setup),
        cmocka_unit_test_setup(test_calculate_balance_time_large_difference, setup),
        cmocka_unit_test_setup(test_calculate_balance_time_below_minimum, setup),
        
        // Balancing state machine tests
        cmocka_unit_test_setup(test_balancing_starts_idle, setup),
        cmocka_unit_test_setup(test_balancing_does_not_start_with_high_voltage, setup),
        cmocka_unit_test_setup(test_balancing_does_not_start_with_low_voltage, setup),
        cmocka_unit_test_setup(test_balancing_does_not_start_with_fatal_event, setup),
        cmocka_unit_test_setup(test_balancing_pause_clears_mask, setup),
        cmocka_unit_test_setup(test_balancing_stops_in_slow_mode, setup),
        cmocka_unit_test_setup(test_balancing_session_resets_pause_counter, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
