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
    m->thermistor_temperatures_millis = stored_millis;
    m->current_millis = stored_millis;
    
    // Normal operating values
    m->high_voltages.battery = (float)(3.7f * NUM_CELLS);
    m->temperature_min = 25.0f;
    m->temperature_max = 25.0f;
    
    for (int i = 0; i < NUM_CELLS; i++) {
        m->cell_voltages_mV[i] = 3700;
    }
    for (int i = 0; i < NUM_THERMISTORS; i++) {
        m->thermistor_temperatures_raw_dC[i] = 250;
        m->thermistor_temperatures[i] = 25.0f;
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
    
    m.thermistor_temperatures_raw_dC[0] = 200;  // min (20.0C)
    m.thermistor_temperatures[0] = 20.0f;
    m.thermistor_temperatures_raw_dC[1] = 350;  // max (35.0C)
    m.thermistor_temperatures[1] = 35.0f;
    for (int i = 2; i < NUM_THERMISTORS; i++) {
        m.thermistor_temperatures_raw_dC[i] = 250;
        m.thermistor_temperatures[i] = 25.0f;
    }
    
    model_tick(&m);
    
    assert_true(m.temperature_min >= 19.9f && m.temperature_min <= 20.1f);
    assert_true(m.temperature_max >= 34.9f && m.temperature_max <= 35.1f);
}

static void test_missing_cell_leaves_derived_values_stale(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);

    // First tick with all cells present: derived values update
    model_tick(&m);
    assert_int_equal(m.cell_voltage_min_mV, 3700);
    millis_t good_millis = m.cell_voltage_millis;

    // A missing cell 0 (and separately, a missing cell in the middle) must
    // not update min/max/total or the timestamp, so staleness trips.
    for (int missing = 0; missing < 2; missing++) {
        int idx = missing == 0 ? 0 : NUM_CELLS / 2;
        setup_model_operating(&m);
        model_tick(&m);
        good_millis = m.cell_voltage_millis;

        advance_time(1000);
        m.cell_voltages_millis = stored_millis;
        m.cell_voltages_mV[idx] = -1;
        m.cell_voltages_mV[(idx + 1) % NUM_CELLS] = 3500;

        model_tick(&m);

        assert_int_equal(m.cell_voltage_millis, good_millis);
        assert_int_equal(m.cell_voltage_min_mV, 3700);
        assert_int_equal(m.cell_voltage_max_mV, 3700);
        assert_int_equal(m.cell_voltage_total_mV, 3700 * NUM_CELLS);
    }
}

static void test_missing_cell_recovers_when_present_again(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);

    m.cell_voltages_mV[0] = -1;
    model_tick(&m);
    assert_int_equal(m.cell_voltage_millis, stored_millis); // unchanged from setup

    advance_time(1000);
    m.cell_voltages_millis = stored_millis;
    m.cell_voltages_mV[0] = 3500;
    model_tick(&m);

    assert_int_equal(m.cell_voltage_millis, stored_millis);
    assert_int_equal(m.cell_voltage_min_mV, 3500);
    assert_int_equal(m.cell_voltage_max_mV, 3700);
    assert_int_equal(m.cell_voltage_total_mV, 3500 + 3700 * (NUM_CELLS - 1));
}

static void test_missing_cell_event_lists_indices(void **state) {
    (void) state;
    extern bms_event_slot_t bms_event_slots[];
    bms_model_t m = {0};
    setup_model_operating(&m);

    // One missing cell: index in the low byte, the rest 0xFF
    m.cell_voltages_mV[3] = -1;
    model_tick(&m);
    assert_int_equal(get_event_level(ERR_CELL_MISSING), LEVEL_WARNING);
    assert_int_equal(get_event_count(ERR_CELL_MISSING), 1);
    assert_true(bms_event_slots[ERR_CELL_MISSING].data64 == 0xFFFFFFFFFFFFFF03ULL);

    // Still missing on later ticks: count doesn't climb
    model_tick(&m);
    assert_int_equal(get_event_count(ERR_CELL_MISSING), 1);

    // More than 8 missing: the first 8, lowest index first from the LSB
    assert_true(NUM_CELLS >= 10);
    for (int i = 0; i < 10; i++) m.cell_voltages_mV[i] = -1;
    model_tick(&m);
    assert_true(bms_event_slots[ERR_CELL_MISSING].data64 == 0x0706050403020100ULL);

    // All present again: event clears, keeping the count and last data
    for (int i = 0; i < NUM_CELLS; i++) m.cell_voltages_mV[i] = 3700;
    model_tick(&m);
    assert_int_equal(get_event_level(ERR_CELL_MISSING), LEVEL_NONE);
    assert_int_equal(get_event_count(ERR_CELL_MISSING), 1);
    assert_true(bms_event_slots[ERR_CELL_MISSING].data64 == 0x0706050403020100ULL);
}

static void test_last_cell_missing_reported(void **state) {
    (void) state;
    extern bms_event_slot_t bms_event_slots[];
    bms_model_t m = {0};
    setup_model_operating(&m);

    m.cell_voltages_mV[NUM_CELLS - 1] = -1;
    model_tick(&m);
    assert_true(bms_event_slots[ERR_CELL_MISSING].data64 ==
                (0xFFFFFFFFFFFFFF00ULL | (uint64_t)(NUM_CELLS - 1)));
}

static void test_temperature_min_max_not_truncated(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);

    // Module 0 seeds min/max; later modules must not be truncated to int
    m.thermistor_temperatures_raw_dC[1] = -9;
    m.thermistor_temperatures[1] = -0.9f;
    m.thermistor_temperatures_raw_dC[2] = 459;
    m.thermistor_temperatures[2] = 45.9f;

    model_tick(&m);

    assert_true(m.temperature_min <= -0.85f && m.temperature_min >= -0.95f);
    assert_true(m.temperature_max >= 45.85f && m.temperature_max <= 45.95f);
}

static void setup_thermistors_read(void) {
    setup_model_operating(&model);
    for (int i = 0; i < NUM_THERMISTORS; i++) {
        model.thermistor_temperatures[i] = NAN;
        store_thermistor_temperature(&model, i, 250, true);
    }
    for (int i = 0; i < NUM_MODULES; i++) {
        model.bmb_die_temperatures[i] = NAN;
    }
    model.thermistor_temperatures_millis = stored_millis;
}

static void test_faulty_thermistor_left_out_and_raised(void **state) {
    (void) state;
    setup_thermistors_read();
    store_thermistor_temperature(&model, 1, 3000, false); // open: reads 300C
    model.thermistor_temperatures[2] = NAN;       // so it takes 20C straight away
    store_thermistor_temperature(&model, 2, 200, true);

    assert_int_equal(thermistor_fault_mask(&model), 1u << 1);
    assert_true(isnan(model.thermistor_temperatures[1]));
    assert_int_equal(model.thermistor_temperatures_raw_dC[1], THERMISTOR_FAULT_dC);

    model_tick(&model);
    confirm_battery_safety(&model);

    // The open one makes the pack read hot, so the over-temperature limits
    // trip; the others still give the minimum
    assert_true(model.temperature_min >= 19.9f && model.temperature_min <= 20.1f);
    assert_true(model.temperature_max == THERMISTOR_FAULT_dC * 0.1f);
    assert_int_equal(model.temperature_millis, stored_millis);
    assert_int_equal(get_event_level(ERR_THERMISTOR_FAULT), LEVEL_CRITICAL);
    assert_true(bms_event_slots[ERR_THERMISTOR_FAULT].data64 == (1u << 1));
    assert_int_not_equal(get_event_level(ERR_BATTERY_TEMPERATURE_VERY_HIGH), LEVEL_NONE);

    // Back in range: counted again, and the fault clears
    store_thermistor_temperature(&model, 1, 260, true);
    assert_int_equal(thermistor_fault_mask(&model), 0);
    assert_true(model.thermistor_temperatures[1] >= 25.9f && model.thermistor_temperatures[1] <= 26.1f);
    model_tick(&model);
    confirm_battery_safety(&model);
    assert_true(model.temperature_max >= 25.9f && model.temperature_max <= 26.1f);
    assert_int_equal(get_event_level(ERR_THERMISTOR_FAULT), LEVEL_NONE);
}

static void test_all_thermistors_faulty_reads_hot(void **state) {
    (void) state;
    setup_thermistors_read();
    for (int i = 0; i < NUM_THERMISTORS; i++) {
        store_thermistor_temperature(&model, i, -600, false); // shorted / disconnected low
    }

    model_tick(&model);
    confirm_battery_safety(&model);

    // A short reads cold, but a fault reads hot, not stale
    assert_true(model.temperature_min == THERMISTOR_FAULT_dC * 0.1f);
    assert_true(model.temperature_max == THERMISTOR_FAULT_dC * 0.1f);
    assert_int_equal(model.temperature_millis, stored_millis);
    assert_int_equal(get_event_level(ERR_THERMISTOR_FAULT), LEVEL_CRITICAL);
    assert_int_not_equal(get_event_level(ERR_BATTERY_TEMPERATURE_VERY_HIGH), LEVEL_NONE);
    assert_int_equal(get_event_level(ERR_BATTERY_TEMPERATURE_STALE), LEVEL_NONE);
}

// One thermistor read's worth of time, outside slow mode
#define THERMISTOR_READ_PERIOD_MS 1280

static void read_thermistor_and_tick(int16_t temp_dC, bool valid) {
    store_thermistor_temperature(&model, 1, temp_dC, valid);
    model.thermistor_temperatures_millis = stored_millis;
    model_tick(&model);
    confirm_battery_safety(&model);
    advance_time(THERMISTOR_READ_PERIOD_MS);
    events_tick();
}

static void test_thermistor_fault_escalation(void **state) {
    (void) state;
    extern millis_t last_tick_at;
    setup_thermistors_read();
    last_tick_at = stored_millis; // start the escalation clock now

    // Two bad reads in a row, then good again: not fatal
    read_thermistor_and_tick(3000, false);
    read_thermistor_and_tick(3000, false);
    assert_int_equal(get_event_level(ERR_THERMISTOR_FAULT), LEVEL_CRITICAL);
    assert_int_equal(get_event_level(ERR_BATTERY_TEMPERATURE_VERY_HIGH), LEVEL_CRITICAL);
    read_thermistor_and_tick(250, true);
    assert_true(get_event_level(ERR_THERMISTOR_FAULT) < LEVEL_FATAL);
    assert_true(get_event_level(ERR_BATTERY_TEMPERATURE_VERY_HIGH) < LEVEL_FATAL);

    // Long enough after, a fault that persists goes fatal
    for (int i = 0; i < 10; i++) {
        read_thermistor_and_tick(250, true);
    }
    for (int i = 0; i < 4; i++) {
        read_thermistor_and_tick(3000, false);
    }
    assert_int_equal(get_event_level(ERR_THERMISTOR_FAULT), LEVEL_FATAL);
    assert_int_equal(get_event_level(ERR_BATTERY_TEMPERATURE_VERY_HIGH), LEVEL_FATAL);
}

static void test_cold_misreads_not_fatal(void **state) {
    (void) state;
    extern millis_t last_tick_at;
    setup_thermistors_read();
    last_tick_at = stored_millis;

    // Two misreads at -40C (still on the curve, so through the filter): the
    // pack reads very cold, but recovers before it's fatal
    read_thermistor_and_tick(-400, true);
    read_thermistor_and_tick(-400, true);
    assert_int_equal(get_event_level(ERR_BATTERY_TEMPERATURE_VERY_LOW), LEVEL_CRITICAL);
    read_thermistor_and_tick(250, true);
    read_thermistor_and_tick(250, true);
    assert_true(get_event_level(ERR_BATTERY_TEMPERATURE_VERY_LOW) < LEVEL_FATAL);
}

static void test_temperature_glitch_events(void **state) {
    (void) state;
    setup_thermistors_read();
    assert_int_equal(get_event_count(ERR_BMB_TEMPERATURE_GLITCH), 0);

    store_thermistor_temperature(&model, 3, 250 + TEMPERATURE_GLITCH_THRESHOLD_dC, true);
    assert_int_equal(get_event_count(ERR_BMB_TEMPERATURE_GLITCH), 1);
    assert_true(bms_event_slots[ERR_BMB_TEMPERATURE_GLITCH].data64 ==
                ((3ULL << 32) | (250ULL << 16) | (uint64_t)(250 + TEMPERATURE_GLITCH_THRESHOLD_dC)));

    // A BMB die: no glitch on its first reading, then one flagged as a die
    store_bmb_die_temperature(&model, 1, 300);
    assert_int_equal(get_event_count(ERR_BMB_TEMPERATURE_GLITCH), 1);
    store_bmb_die_temperature(&model, 1, 300 - TEMPERATURE_GLITCH_THRESHOLD_dC);
    assert_int_equal(get_event_count(ERR_BMB_TEMPERATURE_GLITCH), 2);
    assert_true(bms_event_slots[ERR_BMB_TEMPERATURE_GLITCH].data64 ==
                ((1ULL << 40) | (1ULL << 32) | (300ULL << 16) | (uint64_t)(300 - TEMPERATURE_GLITCH_THRESHOLD_dC)));

    // After a fault, the next good reading starts afresh rather than glitching
    store_thermistor_temperature(&model, 4, 3000, false);
    store_thermistor_temperature(&model, 4, 250, true);
    assert_int_equal(get_event_count(ERR_BMB_TEMPERATURE_GLITCH), 2);
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
    m.thermistor_temperatures_raw_dC[0] = 480;  // 48.0C, near max of 50C
    m.thermistor_temperatures[0] = 48.0f;
    m.thermistor_temperatures_millis = stored_millis;
    
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
    for (int i = 0; i < NUM_THERMISTORS; i++) {
        m.thermistor_temperatures_raw_dC[i] = 490;
        m.thermistor_temperatures[i] = 49.0f;
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
    for (int i = 0; i < NUM_THERMISTORS; i++) {
        m.thermistor_temperatures_raw_dC[i] = 540;
        m.thermistor_temperatures[i] = 54.0f;
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

static void test_voltage_mismatch_checked_when_cells_newer(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);

    // Battery reading 500 ms older than the cell reading: still close enough
    // in time to compare
    m.high_voltages.battery = (3700 * NUM_CELLS + VOLTAGE_MISMATCH_THRESHOLD_mV + 1000) * 0.001f;
    m.high_voltages.battery_millis = stored_millis;
    advance_time(500);
    m.cell_voltages_millis = stored_millis;

    model_tick(&m);
    confirm_battery_safety(&m);

    assert_int_equal(get_event_level(ERR_VOLTAGE_MISMATCH), LEVEL_CRITICAL);
}

static void test_voltage_mismatch_skipped_when_readings_far_apart(void **state) {
    (void) state;
    bms_model_t m = {0};
    setup_model_operating(&m);

    m.high_voltages.battery = (3700 * NUM_CELLS + VOLTAGE_MISMATCH_THRESHOLD_mV + 1000) * 0.001f;
    m.high_voltages.battery_millis = stored_millis;
    advance_time(1500);
    m.cell_voltages_millis = stored_millis;

    model_tick(&m);
    confirm_battery_safety(&m);

    assert_int_equal(get_event_level(ERR_VOLTAGE_MISMATCH), LEVEL_NONE);
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
        cmocka_unit_test_setup(test_missing_cell_leaves_derived_values_stale, setup),
        cmocka_unit_test_setup(test_missing_cell_recovers_when_present_again, setup),
        cmocka_unit_test_setup(test_temperature_min_max_not_truncated, setup),
        cmocka_unit_test_setup(test_faulty_thermistor_left_out_and_raised, setup),
        cmocka_unit_test_setup(test_all_thermistors_faulty_reads_hot, setup),
        cmocka_unit_test_setup(test_thermistor_fault_escalation, setup),
        cmocka_unit_test_setup(test_cold_misreads_not_fatal, setup),
        cmocka_unit_test_setup(test_temperature_glitch_events, setup),
        cmocka_unit_test_setup(test_missing_cell_event_lists_indices, setup),
        cmocka_unit_test_setup(test_last_cell_missing_reported, setup),
        
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
        cmocka_unit_test_setup(test_voltage_mismatch_checked_when_cells_newer, setup),
        cmocka_unit_test_setup(test_voltage_mismatch_skipped_when_readings_far_apart, setup),
        
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
