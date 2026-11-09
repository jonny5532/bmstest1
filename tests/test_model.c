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
#include "app/battery/safety_checks.h"
#include "sys/events/events.h"
#include "config/limits.h"

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;
uint32_t stored_timestep = 0;

// External model
extern bms_model_t model;

// ============================================================================
// Test Helpers
// ============================================================================

static void advance_time(uint32_t ms) {
    stored_millis += ms;
    stored_millis64 += ms;
}

static void reset_events(void) {
    extern bms_event_slot_t bms_event_slots[ERR_HIGHEST];
    for (int i = 0; i < ERR_HIGHEST; i++) {
        bms_event_slots[i] = (bms_event_slot_t){0};
        bms_event_slots[i].level = LEVEL_WARNING;
        clear_bms_event(i);
    }
}

static void setup_model_operating(bms_model_t *m) {
    memset(m, 0, sizeof(bms_model_t));
    m->system_sm.state = SYSTEM_STATE_OPERATING;
    m->contactor_sm.enable_current = true;
    
    // Set fresh timestamps
    m->high_voltages.battery_millis = stored_millis;
    m->cell_voltage_millis = stored_millis;
    m->cell_voltages_millis = stored_millis;
    m->temperature_millis = stored_millis;
    m->module_temperatures_millis = stored_millis;
    m->current_millis = stored_millis;
    
    // Normal operating values
    m->high_voltages.battery = (float)(3.7f * NUM_CELLS);
    m->temperature_min = 25.0f;
    m->temperature_max = 25.0f;
    
    for (int i = 0; i < NUM_CELLS; i++) {
        m->cell_voltages_mV[i] = 3700;
    }
    for (int i = 0; i < NUM_MODULE_TEMPS; i++) {
        m->module_temperatures_raw_dC[i] = 250;
        m->module_temperatures[i] = 25.0f;
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
// Cell Voltage Processing Tests
// ============================================================================

static void test_cell_voltage_min_max_extraction(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set varied cell voltages
    m.cell_voltages_mV[0] = 3500;  // min
    m.cell_voltages_mV[1] = 3600;
    m.cell_voltages_mV[2] = 4000;  // max
    for (int i = 3; i < NUM_CELLS; i++) {
        m.cell_voltages_mV[i] = 3700;
    }
    
    model_tick(&m);
    
    assert_int_equal(m.cell_voltage_min_mV, 3500);
    assert_int_equal(m.cell_voltage_max_mV, 4000);
}

static void test_cell_voltage_total_calculation(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set all cells to same voltage for easy calculation
    for (int i = 0; i < NUM_CELLS; i++) {
        m.cell_voltages_mV[i] = 3700;
    }
    
    model_tick(&m);
    
    assert_int_equal(m.cell_voltage_total_mV, 3700 * NUM_CELLS);
}

static void test_temperature_min_max_extraction(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    m.module_temperatures_raw_dC[0] = 200;  // min (20.0C)
    m.module_temperatures[0] = 20.0f;
    m.module_temperatures_raw_dC[1] = 350;  // max (35.0C)
    m.module_temperatures[1] = 35.0f;
    for (int i = 2; i < NUM_MODULE_TEMPS; i++) {
        m.module_temperatures_raw_dC[i] = 250;
        m.module_temperatures[i] = 25.0f;
    }
    
    model_tick(&m);
    
    assert_true(m.temperature_min >= 19.9f && m.temperature_min <= 20.1f);
    assert_true(m.temperature_max >= 34.9f && m.temperature_max <= 35.1f);
}

static void test_temperature_min_max_not_truncated(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);

    // Module 0 seeds min/max; later modules must not be truncated to int
    m.module_temperatures_raw_dC[1] = -9;
    m.module_temperatures[1] = -0.9f;
    m.module_temperatures_raw_dC[2] = 459;
    m.module_temperatures[2] = 45.9f;

    model_tick(&m);

    assert_true(m.temperature_min <= -0.85f && m.temperature_min >= -0.95f);
    assert_true(m.temperature_max >= 45.85f && m.temperature_max <= 45.95f);
}

// ============================================================================
// Current Limit Application Tests
// ============================================================================

static void test_current_disabled_when_contactors_not_enabled(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    m.contactor_sm.enable_current = false;
    
    model_tick(&m);
    
    assert_int_equal(m.charge_current_limit_dA, 0);
    assert_int_equal(m.discharge_current_limit_dA, 0);
}

static void test_temperature_limits_applied(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set hot temperature that should derate
    m.module_temperatures_raw_dC[0] = 480;  // 48.0C, near max of 50C
    m.module_temperatures[0] = 48.0f;
    m.module_temperatures_millis = stored_millis;
    
    model_tick(&m);
    
    // Charge limit should be derated
    // (500 - 480) * 2 = 40 dA = 4A
    assert_true(m.charge_current_limit_dA < CHARGE_MAX_CURRENT_dA);
    assert_int_equal(m.temp_charge_current_limit_dA, 40);
}

static void test_cell_voltage_limits_applied(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set cell voltage above soft max - should stop charging
    for (int i = 0; i < NUM_CELLS; i++) {
        m.cell_voltages_mV[i] = get_cell_voltage_soft_max_mV(&m) + 10;
    }
    
    model_tick(&m);
    
    assert_int_equal(m.cell_voltage_charge_current_limit_dA, 0);
    assert_int_equal(m.charge_current_limit_dA, 0);
}

// ============================================================================
// Overcurrent Accumulation Tests
// ============================================================================

static void test_overcurrent_accumulation_charge(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set hot temperature to limit charging to a low value
    // At 490 dC, limit = (500 - 490) * 2 = 20 dA = 2A
    for (int i = 0; i < NUM_MODULE_TEMPS; i++) {
        m.module_temperatures_raw_dC[i] = 490;
        m.module_temperatures[i] = 49.0f;
    }
    
    // Set charge current well above what temperature will allow
    // 50A = 50000 mA, limit will be ~2A
    m.current_mA = 50000;
    
    model_tick(&m);
    
    // Should accumulate excess
    assert_true(m.excess_charge_buffer_dC > 0);
}

static void test_overcurrent_accumulation_discharge(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set hot temperature to limit discharging
    // At 540 dC (near max 550), limit = (550 - 540) * 2 = 20 dA = 2A
    for (int i = 0; i < NUM_MODULE_TEMPS; i++) {
        m.module_temperatures_raw_dC[i] = 540;
        m.module_temperatures[i] = 54.0f;
    }
    
    // Discharge at 50A when limit is ~2A
    m.current_mA = -50000;
    
    model_tick(&m);
    
    assert_true(m.excess_discharge_buffer_dC > 0);
}

static void test_overcurrent_buffer_decay(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set initial buffer value
    m.excess_charge_buffer_dC = 100;
    m.current_mA = 0;  // No current
    
    model_tick(&m);
    
    // Buffer should decay by 1
    assert_int_equal(m.excess_charge_buffer_dC, 99);
}

// ============================================================================
// Soft Limit Buffer Tests
// ============================================================================

static void test_soft_limit_buffer_resets_in_normal_range(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set some buffer value
    m.soft_limit_charge_buffer_dC = 50;
    
    // Cell voltages in normal range
    m.cell_voltage_min_mV = 3500;
    m.cell_voltage_max_mV = 3800;
    
    model_tick(&m);
    
    // Buffer should be reset
    assert_int_equal(m.soft_limit_charge_buffer_dC, 0);
}

static void test_soft_limit_buffer_accumulates_when_overcharged(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Cell voltage above soft max
    for (int i = 0; i < NUM_CELLS; i++) {
        m.cell_voltages_mV[i] = get_cell_voltage_soft_max_mV(&m) + 50;
    }
    m.charge_current_limit_dA = 0;  // Limit should be 0
    m.current_mA = 5000;  // But still charging at 5A
    
    model_tick(&m);
    
    // Buffer should accumulate
    assert_true(m.soft_limit_charge_buffer_dC > 0);
}

// ============================================================================
// Voltage Mismatch Detection Tests
// ============================================================================

static void test_voltage_mismatch_not_triggered_when_matched(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set cell voltages that sum to match battery voltage
    int32_t cell_voltage = 3700;
    for (int i = 0; i < NUM_CELLS; i++) {
        m.cell_voltages_mV[i] = cell_voltage;
    }
    m.high_voltages.battery_millis = stored_millis;
    m.cell_voltage_millis = stored_millis;
    m.high_voltages.battery = (float)cell_voltage * NUM_CELLS * 0.001f;

    model_tick(&m);
    confirm_battery_safety(&m);
    
    assert_int_equal(get_event_level(ERR_VOLTAGE_MISMATCH), LEVEL_NONE);
}

static void test_voltage_mismatch_triggered_on_large_difference(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set cell voltages
    for (int i = 0; i < NUM_CELLS; i++) {
        m.cell_voltages_mV[i] = 3700;
    }
    // Battery voltage way off
    m.high_voltages.battery = (3700 * NUM_CELLS + VOLTAGE_MISMATCH_THRESHOLD_mV + 1000) * 0.001f;
    m.high_voltages.battery_millis = stored_millis;
    
    model_tick(&m);
    confirm_battery_safety(&m);
    
    assert_int_equal(get_event_level(ERR_VOLTAGE_MISMATCH), LEVEL_CRITICAL);
}

// ============================================================================
// E-Stop Tests
// ============================================================================

static void test_estop_raises_event(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    m.estop_pressed = true;
    
    confirm_battery_safety(&m);
    
    assert_int_equal(get_event_level(ERR_ESTOP_PRESSED), LEVEL_CRITICAL);
}

static void test_estop_clears_when_released(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // First press
    m.estop_pressed = true;
    confirm_battery_safety(&m);
    assert_int_equal(get_event_level(ERR_ESTOP_PRESSED), LEVEL_CRITICAL);
    
    // Then release
    m.estop_pressed = false;
    confirm_battery_safety(&m);
    assert_int_equal(get_event_level(ERR_ESTOP_PRESSED), LEVEL_NONE);
}

// ============================================================================
// Initialization State Safety Tests
// ============================================================================

static void test_stale_data_not_faulted_during_init(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set to initializing state
    m.system_sm.state = SYSTEM_STATE_INITIALIZING;
    
    // Make data stale
    m.high_voltages.battery_millis = stored_millis - BATTERY_VOLTAGE_STALE_THRESHOLD_MS - 100;
    
    confirm_battery_safety(&m);
    
    // Should not trigger stale event during init
    assert_int_equal(get_event_level(ERR_BATTERY_VOLTAGE_STALE), LEVEL_NONE);
}

static void test_stale_data_faulted_when_operating(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Make data stale
    m.high_voltages.battery_millis = stored_millis - BATTERY_VOLTAGE_STALE_THRESHOLD_MS - 100;
    
    confirm_battery_safety(&m);
    
    assert_int_equal(get_event_level(ERR_BATTERY_VOLTAGE_STALE), LEVEL_CRITICAL);
}

// ============================================================================
// Combined Safety Scenario Tests
// ============================================================================

static void test_multiple_faults_highest_level_tracked(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);
    
    // Set up multiple faults
    m.cell_voltage_max_mV = get_cell_voltage_soft_max_mV(&m) + 10;  // Warning
    m.cell_voltage_min_mV = CELL_VOLTAGE_HARD_MIN_mV - 10;  // Critical
    
    confirm_battery_safety(&m);
    
    // Should have both events
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_HIGH), LEVEL_WARNING);
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_VERY_LOW), LEVEL_CRITICAL);
    
    // Highest should be critical
    assert_int_equal(get_highest_event_level(), LEVEL_CRITICAL);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    const struct CMUnitTest tests[] = {
        // Cell voltage processing
        cmocka_unit_test_setup(test_cell_voltage_min_max_extraction, setup),
        cmocka_unit_test_setup(test_cell_voltage_total_calculation, setup),
        cmocka_unit_test_setup(test_temperature_min_max_extraction, setup),
        cmocka_unit_test_setup(test_temperature_min_max_not_truncated, setup),
        
        // Current limit application
        cmocka_unit_test_setup(test_current_disabled_when_contactors_not_enabled, setup),
        cmocka_unit_test_setup(test_temperature_limits_applied, setup),
        cmocka_unit_test_setup(test_cell_voltage_limits_applied, setup),
        
        // Overcurrent accumulation
        cmocka_unit_test_setup(test_overcurrent_accumulation_charge, setup),
        cmocka_unit_test_setup(test_overcurrent_accumulation_discharge, setup),
        cmocka_unit_test_setup(test_overcurrent_buffer_decay, setup),
        
        // Soft limit buffer
        cmocka_unit_test_setup(test_soft_limit_buffer_resets_in_normal_range, setup),
        cmocka_unit_test_setup(test_soft_limit_buffer_accumulates_when_overcharged, setup),
        
        // Voltage mismatch
        cmocka_unit_test_setup(test_voltage_mismatch_not_triggered_when_matched, setup),
        cmocka_unit_test_setup(test_voltage_mismatch_triggered_on_large_difference, setup),
        
        // E-Stop
        cmocka_unit_test_setup(test_estop_raises_event, setup),
        cmocka_unit_test_setup(test_estop_clears_when_released, setup),
        
        // Initialization safety
        cmocka_unit_test_setup(test_stale_data_not_faulted_during_init, setup),
        cmocka_unit_test_setup(test_stale_data_faulted_when_operating, setup),
        
        // Combined scenarios
        cmocka_unit_test_setup(test_multiple_faults_highest_level_tracked, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
