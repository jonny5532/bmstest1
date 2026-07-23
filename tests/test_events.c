#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app/model.h"
#include "sys/events/events.h"
#include "app/battery/safety_checks.h"
#include "app/hardware_checks.h"
#include "sys/time/time.h"
#include "sys/logging/logging.h"
#include "config/limits.h"

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;

static int setup(void **state) {
    (void) state;
    stored_millis = 1000;
    stored_millis64 = 1000;
    extern bms_event_slot_t bms_event_slots[ERR_HIGHEST];
    for(int i = 0; i < ERR_HIGHEST; i++) {
        bms_event_slots[i] = (bms_event_slot_t){0};
        // Strange way to reset events
        bms_event_slots[i].level = LEVEL_WARNING;
        clear_bms_event(i);
    }
    return 0;
}

static void setup_model(bms_model_t *model) {
    // Basic operating state

    model->system_sm.state = SYSTEM_STATE_OPERATING;
    model->high_voltages.battery_millis = stored_millis;
    model->cell_voltage_millis = stored_millis;
    model->temperature_millis = stored_millis;

    model->cell_voltage_min_mV = 3500;
    model->cell_voltage_max_mV = 3500;
    for(int i = 0; i < 8; i++) {
        model->module_temperatures_raw_dC[i] = 250; // 25.0C
    }
    model->contactor_sm.enable_current = true;
}

static void test_manual_trigger(void **state) {
    (void) state;
    // Test that every event can at least be manually raised
    for (int i = 0; i < ERR_HIGHEST; i++) {
        raise_bms_event(i, 0);
        assert_int_not_equal(get_event_level(i), LEVEL_NONE);
        
        if (get_event_level(i) != LEVEL_FATAL) {
            clear_bms_event(i);
            assert_int_equal(get_event_level(i), LEVEL_NONE);
        }
    }
}

static void test_safety_voltage_events(void **state) {
    (void) state;
    bms_model_t model = {0};
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    model.high_voltages.battery_millis = stored_millis;
    model.cell_voltage_millis = stored_millis;
    model.temperature_millis = stored_millis;

    // Normal voltages
    model.cell_voltage_min_mV = 3300;
    model.cell_voltage_max_mV = 3300;
    model.cell_voltage_total_mV = model.cell_voltage_max_mV * NUM_CELLS;
    model.high_voltages.battery = (float)model.cell_voltage_max_mV * NUM_CELLS * 0.001f;
    model.cell_voltages_millis = stored_millis;
    model.high_voltages.battery_millis = stored_millis;
    model.module_temperatures_millis = stored_millis;
    model.temperature_millis = stored_millis;
    model.current_millis = stored_millis;
    model.supply_voltages.voltage_3V3_millis = stored_millis;
    model.supply_voltages.voltage_5V_millis = stored_millis;
    model.supply_voltages.voltage_12V_millis = stored_millis;
    model.supply_voltages.voltage_contactor_millis = stored_millis;

    // Set all cell voltages to same value so total matches battery voltage
    for(int i=0; i<NUM_CELLS; i++) model.cell_voltages_mV[i] = 3300;

    model_tick(&model);
    
    // Clear initial staleness and other events
    for(int i=0; i<ERR_HIGHEST; i++) clear_bms_event(i);

    confirm_battery_safety(&model);
    print_bms_events();
    assert_int_equal(get_highest_event_level(), LEVEL_NONE);

    // Battery Voltage High
    // We don't have a BATTERY_VOLTAGE_SOFT_MAX_mV define, but safety_checks.c uses (CELL_VOLTAGE_SOFT_MAX_mV * NUM_CELLS)
    model.cell_voltage_max_mV = get_cell_voltage_soft_max_mV(&model) + 1;
    model.cell_voltage_millis = stored_millis;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_HIGH), LEVEL_WARNING);

    // Battery Voltage Very High
    model.cell_voltage_max_mV = CELL_VOLTAGE_HARD_MAX_mV + 1;
    model.cell_voltage_millis = stored_millis;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_VERY_HIGH), LEVEL_CRITICAL);

    // Cell Voltage Low
    model.cell_voltage_min_mV = get_cell_voltage_soft_min_mV(&model) - 1;
    model.cell_voltage_millis = stored_millis;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_LOW), LEVEL_WARNING);

    // Cell Voltage Very Low
    model.cell_voltage_min_mV = CELL_VOLTAGE_HARD_MIN_mV - 1;
    model.cell_voltage_millis = stored_millis;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_VERY_LOW), LEVEL_CRITICAL);
}

static void test_safety_temp_events(void **state) {
    (void) state;
    bms_model_t model = {0};
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    model.temperature_millis = stored_millis;

    // High temp
    model.temperature_max = TEMPERATURE_SOFT_MAX + 0.1f;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_BATTERY_TEMPERATURE_HIGH), LEVEL_WARNING);

    // Very high temp
    model.temperature_max = TEMPERATURE_HARD_MAX + 0.1f;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_BATTERY_TEMPERATURE_VERY_HIGH), LEVEL_CRITICAL);

    // Low temp
    model.temperature_max = 25.0f;
    model.temperature_min = TEMPERATURE_SOFT_MIN - 0.1f;
    model.temperature_millis = stored_millis;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_BATTERY_TEMPERATURE_LOW), LEVEL_WARNING);

    // Very low temp
    model.temperature_min = TEMPERATURE_HARD_MIN - 0.1f;
    model.temperature_millis = stored_millis;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_BATTERY_TEMPERATURE_VERY_LOW), LEVEL_CRITICAL);
}

static void test_battery_pack_voltage_hard_limits(void **state) {
    (void) state;
    bms_model_t model = {0};
    setup_model(&model);

    model.high_voltages.battery_millis = stored_millis;

    // Very high pack voltage
    model.high_voltages.battery = (BATTERY_VOLTAGE_HARD_MAX_mV + 1) * 0.001f;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_BATTERY_VOLTAGE_VERY_HIGH), LEVEL_CRITICAL);

    // Very low pack voltage
    model.high_voltages.battery = (BATTERY_VOLTAGE_HARD_MIN_mV - 1) * 0.001f;
    model.high_voltages.battery_millis = stored_millis;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_BATTERY_VOLTAGE_VERY_LOW), LEVEL_CRITICAL);
}

static void test_soft_charge_buffer_exceeded_event(void **state) {
    (void) state;
    bms_model_t model = {0};
    setup_model(&model);

    model.soft_limit_charge_buffer_dC = OVERCHARGE_BUFFER_LIMIT_dC + 1;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_SOFT_CHARGE_BUFFER_EXCEEDED), LEVEL_CRITICAL);

    clear_bms_event(ERR_SOFT_CHARGE_BUFFER_EXCEEDED);

    model.soft_limit_charge_buffer_dC = -(OVERDISCHARGE_BUFFER_LIMIT_dC + 1);
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_SOFT_CHARGE_BUFFER_EXCEEDED), LEVEL_CRITICAL);
}

static void test_estop_event(void **state) {
    (void) state;
    bms_model_t model = {0};
    setup_model(&model);

    model.estop_pressed = true;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_ESTOP_PRESSED), LEVEL_CRITICAL);

    model.estop_pressed = false;
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_ESTOP_PRESSED), LEVEL_NONE);
}

static void test_hardware_voltage_events(void **state) {
    (void) state;
    bms_model_t model = {0};
    model.supply_voltages.voltage_3V3_millis = stored_millis;
    model.supply_voltages.voltage_5V_millis = stored_millis;
    model.supply_voltages.voltage_12V_millis = stored_millis;
    model.supply_voltages.voltage_contactor_millis = stored_millis;

    // Start from nominal values for all rails
    model.supply_voltages.voltage_3V3_mV = (SUPPLY_VOLTAGE_3V3_MIN_MV + SUPPLY_VOLTAGE_3V3_MAX_MV) / 2;
    model.supply_voltages.voltage_5V_mV = (SUPPLY_VOLTAGE_5V_MIN_MV + SUPPLY_VOLTAGE_5V_MAX_MV) / 2;
    model.supply_voltages.voltage_12V_mV = (SUPPLY_VOLTAGE_12V_MIN_MV + SUPPLY_VOLTAGE_12V_MAX_MV) / 2;
    model.supply_voltages.voltage_contactor_mV = (SUPPLY_VOLTAGE_CONTACTOR_SOFT_MIN_MV + SUPPLY_VOLTAGE_CONTACTOR_MAX_MV) / 2;

    // 3.3V Low
    model.supply_voltages.voltage_3V3_mV = SUPPLY_VOLTAGE_3V3_MIN_MV - 1;
    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_3V3_LOW), LEVEL_WARNING);
    model.supply_voltages.voltage_3V3_mV = (SUPPLY_VOLTAGE_3V3_MIN_MV + SUPPLY_VOLTAGE_3V3_MAX_MV) / 2;
    clear_bms_event(ERR_SUPPLY_VOLTAGE_3V3_LOW);

    // 3.3V High
    model.supply_voltages.voltage_3V3_mV = SUPPLY_VOLTAGE_3V3_MAX_MV + 1;
    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_3V3_HIGH), LEVEL_WARNING);
    model.supply_voltages.voltage_3V3_mV = (SUPPLY_VOLTAGE_3V3_MIN_MV + SUPPLY_VOLTAGE_3V3_MAX_MV) / 2;
    clear_bms_event(ERR_SUPPLY_VOLTAGE_3V3_HIGH);

    // 5V Low
    model.supply_voltages.voltage_5V_mV = SUPPLY_VOLTAGE_5V_MIN_MV - 1;
    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_5V_LOW), LEVEL_WARNING);
    model.supply_voltages.voltage_5V_mV = (SUPPLY_VOLTAGE_5V_MIN_MV + SUPPLY_VOLTAGE_5V_MAX_MV) / 2;
    clear_bms_event(ERR_SUPPLY_VOLTAGE_5V_LOW);

    // 5V High
    model.supply_voltages.voltage_5V_mV = SUPPLY_VOLTAGE_5V_MAX_MV + 1;
    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_5V_HIGH), LEVEL_WARNING);
    model.supply_voltages.voltage_5V_mV = (SUPPLY_VOLTAGE_5V_MIN_MV + SUPPLY_VOLTAGE_5V_MAX_MV) / 2;
    clear_bms_event(ERR_SUPPLY_VOLTAGE_5V_HIGH);

    // 12V Low
    model.supply_voltages.voltage_12V_mV = SUPPLY_VOLTAGE_12V_MIN_MV - 1;
    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_12V_LOW), LEVEL_WARNING);
    model.supply_voltages.voltage_12V_mV = (SUPPLY_VOLTAGE_12V_MIN_MV + SUPPLY_VOLTAGE_12V_MAX_MV) / 2;
    clear_bms_event(ERR_SUPPLY_VOLTAGE_12V_LOW);

    // 12V High
    model.supply_voltages.voltage_12V_mV = SUPPLY_VOLTAGE_12V_MAX_MV + 1;
    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_12V_HIGH), LEVEL_WARNING);
    model.supply_voltages.voltage_12V_mV = (SUPPLY_VOLTAGE_12V_MIN_MV + SUPPLY_VOLTAGE_12V_MAX_MV) / 2;
    clear_bms_event(ERR_SUPPLY_VOLTAGE_12V_HIGH);

    // Contactor Voltage Low (soft threshold)
    model.supply_voltages.voltage_contactor_mV = SUPPLY_VOLTAGE_CONTACTOR_SOFT_MIN_MV - 1;
    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_CONTACTOR_LOW), LEVEL_WARNING);
    model.supply_voltages.voltage_contactor_mV = (SUPPLY_VOLTAGE_CONTACTOR_SOFT_MIN_MV + SUPPLY_VOLTAGE_CONTACTOR_MAX_MV) / 2;
    clear_bms_event(ERR_SUPPLY_VOLTAGE_CONTACTOR_LOW);

    // Contactor Voltage Very Low
    model.supply_voltages.voltage_contactor_mV = SUPPLY_VOLTAGE_CONTACTOR_HARD_MIN_MV - 1;
    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_CONTACTOR_VERY_LOW), LEVEL_CRITICAL);
    model.supply_voltages.voltage_contactor_mV = (SUPPLY_VOLTAGE_CONTACTOR_SOFT_MIN_MV + SUPPLY_VOLTAGE_CONTACTOR_MAX_MV) / 2;
    clear_bms_event(ERR_SUPPLY_VOLTAGE_CONTACTOR_VERY_LOW);

    // Contactor Voltage High
    model.supply_voltages.voltage_contactor_mV = SUPPLY_VOLTAGE_CONTACTOR_MAX_MV + 1;
    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_CONTACTOR_HIGH), LEVEL_WARNING);
}

static void test_stale_data_events(void **state) {
    (void) state;
    bms_model_t model = {0};
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    
    // Set all timestamps to old
    model.high_voltages.battery_millis = stored_millis - BATTERY_VOLTAGE_STALE_THRESHOLD_MS - 1;
    model.cell_voltage_millis = stored_millis - CELL_VOLTAGE_STALE_THRESHOLD_MS - 1;
    model.temperature_millis = stored_millis - TEMPERATURE_STALE_THRESHOLD_MS(&model) - 1;
    
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_BATTERY_VOLTAGE_STALE), LEVEL_CRITICAL);
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGES_STALE), LEVEL_CRITICAL);
    assert_int_equal(get_event_level(ERR_BATTERY_TEMPERATURE_STALE), LEVEL_CRITICAL);
}

static void test_current_stale_event(void **state) {
    (void) state;
    bms_model_t model = {0};
    setup_model(&model);

    model.current_millis = stored_millis - CURRENT_STALE_FAULT_THRESHOLD_MS - 1;

    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_CURRENT_STALE), LEVEL_CRITICAL);
}

static void test_voltage_mismatch_ignored_when_timestamps_far_apart(void **state) {
    (void) state;
    bms_model_t model = {0};
    setup_model(&model);

    // Make voltages badly mismatched, but timestamps too far apart for check
    model.cell_voltage_total_mV = 3300 * NUM_CELLS;
    model.high_voltages.battery = 4200.0f * NUM_CELLS * 0.001f;
    model.cell_voltage_millis = stored_millis;
    model.high_voltages.battery_millis = stored_millis + 1500;

    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_VOLTAGE_MISMATCH), LEVEL_NONE);
}

static void test_init_state_still_flags_hard_cell_voltage_faults(void **state) {
    (void) state;
    bms_model_t model = {0};
    setup_model(&model);

    model.system_sm.state = SYSTEM_STATE_INITIALIZING;
    model.cell_voltage_millis = stored_millis;
    model.cell_voltage_max_mV = CELL_VOLTAGE_HARD_MAX_mV + 50;

    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_VERY_HIGH), LEVEL_CRITICAL);
}

static void test_hardware_checks_ignored_before_first_supply_read(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Explicitly in uninitialized hardware state
    model.supply_voltages.voltage_contactor_millis = 0;
    model.supply_voltages.voltage_3V3_millis = 0;
    model.supply_voltages.voltage_5V_millis = 0;
    model.supply_voltages.voltage_12V_millis = 0;

    confirm_hardware_integrity(&model);
    assert_int_equal(get_event_level(ERR_SUPPLY_VOLTAGE_STALE), LEVEL_NONE);
}

static void test_hardware_stale_event_when_all_rails_stale(void **state) {
    (void) state;
    bms_model_t model = {0};

    // Simulate that hardware sampling has started, then all rails become stale
    millis_t old_sample = stored_millis;
    stored_millis += SUPPLY_VOLTAGE_STALE_THRESHOLD_MS + 10;
    stored_millis64 += SUPPLY_VOLTAGE_STALE_THRESHOLD_MS + 10;

    model.supply_voltages.voltage_contactor_millis = old_sample;
    model.supply_voltages.voltage_3V3_millis = old_sample;
    model.supply_voltages.voltage_5V_millis = old_sample;
    model.supply_voltages.voltage_12V_millis = old_sample;

    model.supply_voltages.voltage_3V3_mV = 3300;
    model.supply_voltages.voltage_5V_mV = 5000;
    model.supply_voltages.voltage_12V_mV = 12000;
    model.supply_voltages.voltage_contactor_mV = SUPPLY_VOLTAGE_CONTACTOR_SOFT_MIN_MV;

    confirm_hardware_integrity(&model);
    assert_int_not_equal(get_event_level(ERR_SUPPLY_VOLTAGE_STALE), LEVEL_NONE);
}

static void test_event_escalation(void **state) {
    (void) state;
    // Test escalation of WARNING to FATAL after enough counts
    // ERR_CONTACTOR_POS_STUCK_OPEN escalates after 20 counts
    for (int i = 0; i < 19; i++) {
        count_bms_event(ERR_CONTACTOR_POS_STUCK_OPEN, 0);
        assert_int_equal(get_event_level(ERR_CONTACTOR_POS_STUCK_OPEN), LEVEL_WARNING);
    }
    count_bms_event(ERR_CONTACTOR_POS_STUCK_OPEN, 0);
    assert_int_equal(get_event_level(ERR_CONTACTOR_POS_STUCK_OPEN), LEVEL_FATAL);

    // Test escalation of CRITICAL to FATAL after time
    // ERR_BATTERY_VOLTAGE_VERY_HIGH escalates after 1000ms

    // Get timestamps up to date
    events_tick(); 
    // Check event isn't set yet
    assert_int_equal(get_event_level(ERR_BATTERY_VOLTAGE_VERY_HIGH), LEVEL_NONE);
    raise_bms_event(ERR_BATTERY_VOLTAGE_VERY_HIGH, 0);
    assert_int_equal(get_event_level(ERR_BATTERY_VOLTAGE_VERY_HIGH), LEVEL_CRITICAL);
    
    // tick events
    events_tick(); // first tick sets last_tick_at
    
    stored_millis += 500;
    events_tick();
    assert_int_equal(get_event_level(ERR_BATTERY_VOLTAGE_VERY_HIGH), LEVEL_CRITICAL);
    
    stored_millis += 600;
    events_tick();
    assert_int_equal(get_event_level(ERR_BATTERY_VOLTAGE_VERY_HIGH), LEVEL_FATAL);
}

static void test_overcurrent_accumulation(void **state) {
    (void) state;
    bms_model_t model = {0};
    setup_model(&model);
    model.cell_voltages_millis = stored_millis;
    model.high_voltages.battery_millis = stored_millis;

    // Normal current (within limits + margin)
    // 50A limit + 0.5A margin = 50.5A = 50500mA
    model.current_mA = 50000; 
    model_tick(&model);
    // Decay might leave it above 0 if it was higher from setup_model (which sets 3500mV)
    model.excess_charge_buffer_dC = 0; 
    
    confirm_battery_safety(&model);
    assert_int_equal(get_event_level(ERR_OVERCURRENT_CHARGING), LEVEL_NONE);

    // Excess charge current: 60A = 60000mA
    model.current_mA = 60000;
    model_tick(&model);
    // excess_dA = (60000/100) - model->charge_current_limit_dA - 10
    // charge_limit = calculate_cell_voltage_charge_current_limit(3500) = 500 dA (for LFP < 3350 but we are at 3500)
    // Actually at 3500, charge_limit is quadratic.
    // Let's just check it's > 0.
    assert_true(model.excess_charge_buffer_dC > 0);
    
    // Run it enough times to exceed OVERCURRENT_BUFFER_LIMIT_dC (1000)
    // 11 more ticks. Total 12 ticks.
    // 12 * 90 dA = 1080 dC > 1000 dC limit
    for(int i = 0; i < 112; i++) {
        model.current_millis = stored_millis;
        model_tick(&model);
    }
    assert_true(model.excess_charge_buffer_dC > OVERCURRENT_BUFFER_LIMIT_dC);
    confirm_battery_safety(&model);
    assert_int_not_equal(get_event_level(ERR_OVERCURRENT_CHARGING), LEVEL_NONE);

    // Now test discharge
    model.excess_charge_buffer_dC = 0; // reset
    clear_bms_event(ERR_OVERCURRENT_CHARGING);
    
    // 60A discharge = -60000mA
    model.current_mA = -60000;
    model_tick(&model);
    assert_true(model.excess_discharge_buffer_dC > 0);

    for(int i = 0; i < 112; i++) {
        model.current_millis = stored_millis;
        model_tick(&model);
    }
    assert_true(model.excess_discharge_buffer_dC > OVERCURRENT_BUFFER_LIMIT_dC);
    confirm_battery_safety(&model);
    assert_int_not_equal(get_event_level(ERR_OVERCURRENT_DISCHARGING), LEVEL_NONE);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_manual_trigger, setup),
        cmocka_unit_test_setup(test_safety_voltage_events, setup),
        cmocka_unit_test_setup(test_safety_temp_events, setup),
        cmocka_unit_test_setup(test_battery_pack_voltage_hard_limits, setup),
        cmocka_unit_test_setup(test_soft_charge_buffer_exceeded_event, setup),
        cmocka_unit_test_setup(test_estop_event, setup),
        cmocka_unit_test_setup(test_hardware_voltage_events, setup),
        cmocka_unit_test_setup(test_stale_data_events, setup),
        cmocka_unit_test_setup(test_current_stale_event, setup),
        cmocka_unit_test_setup(test_voltage_mismatch_ignored_when_timestamps_far_apart, setup),
        cmocka_unit_test_setup(test_init_state_still_flags_hard_cell_voltage_faults, setup),
        cmocka_unit_test_setup(test_hardware_checks_ignored_before_first_supply_read, setup),
        cmocka_unit_test_setup(test_hardware_stale_event_when_all_rails_stale, setup),
        cmocka_unit_test_setup(test_event_escalation, setup),
        cmocka_unit_test_setup(test_overcurrent_accumulation, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
