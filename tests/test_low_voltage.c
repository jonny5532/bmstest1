#include "physical_model.h"

#include "app/model.h"
#include "app/battery/safety_checks.h"
#include "app/battery/current_limits.h"
#include "config/limits.h"
#include "sys/events/events.h"
#include "sys/time/time.h"

#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;
uint32_t stored_timestep = 0;



// Update the physical model and the BMS state
// void battery_step(battery_model_t *bat, bms_model_t *model, float current_A, uint32_t dt_ms) {
//     stored_millis += dt_ms;
//     stored_millis64 += dt_ms;
//     stored_timestep++;

//     float dt_s = dt_ms / 1000.0f;
//     // current_A: positive is charging
//     bat->soc += (current_A * dt_s) / (bat->capacity_Ah * 3600.0f);
//     if (bat->soc < 0) bat->soc = 0;
//     if (bat->soc > 1.0) bat->soc = 1.0;

//     float ocv = soc_to_ocv(bat->soc);
//     // V = OCV + I*R
//     float v_cell = ocv + current_A * bat->internal_resistance_Ohm;

//     model->current_mA = (int32_t)(current_A * 1000.0f);
//     model->current_millis = stored_millis;

//     model->cell_voltage_min_mV = (int16_t)(v_cell * 1000.0f);
//     model->cell_voltage_max_mV = (int16_t)(v_cell * 1000.0f);
//     model->cell_voltage_millis = stored_millis;

//     model->battery_voltage_mV = model->cell_voltage_min_mV * NUM_CELLS;
//     model->cell_voltage_total_mV = model->battery_voltage_mV;
//     model->battery_voltage_millis = stored_millis;

//     model->module_temperatures_millis = stored_millis;

//     // Run BMS logic
//     model_tick(model);
//     confirm_battery_safety(model);
//     model->cell_voltage_discharge_current_limit_dA = calculate_cell_voltage_discharge_current_limit(model->cell_voltage_min_mV, model->cell_voltage_max_mV);
//     events_tick();
// }

static void tick(battery_model_t *bat, bms_model_t *model, float current_A, uint32_t dt_ms) {
    battery_model_tick(bat, model, current_A, dt_ms);

    model_tick(model);
    confirm_battery_safety(model);
    //model->cell_voltage_discharge_current_limit_dA = calculate_cell_voltage_discharge_current_limit(model->cell_voltage_min_mV, model->cell_voltage_max_mV);
    events_tick();
}

static void test_low_voltage_protection(void **state) {
    (void) state;
    bms_model_t model = {0};
    battery_model_t bat = {
        .capacity_Ah = 2.0f,
        .soc = 0.1f, // Start at 10% SoC
        .internal_resistance_Ohm = 0.005f
    };

    // System must be in RUNNING state to trigger safety checks properly
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    
    // Set some initial timestamps to avoid "stale" errors immediately
    model.battery_voltage_millis = stored_millis;
    model.cell_voltage_millis = stored_millis;
    model.module_temperatures_millis = stored_millis;
    model.current_millis = stored_millis;
    model.temperature_max_dC = 250; // 25.0C
    model.temperature_min_dC = 250;

    // 1. Initially safe
    tick(&bat, &model, 0, 100);

    print_bms_events();
    assert_int_equal(get_highest_event_level(), LEVEL_NONE);
    // Discharge limit should be healthy
    assert_true(model.cell_voltage_discharge_current_limit_dA > 100);

    printf("Starting discharge from %d mV...\n", model.cell_voltage_min_mV);

    // 2. Discharge until we hit SOFT_MIN (3200mV)
    // We expect derating to start before that if we are below 3500mV (per current_limits.c)
    while (model.cell_voltage_min_mV > CELL_VOLTAGE_SOFT_MIN_mV) {
        tick(&bat, &model, -10.0f, 10000); // 20A discharge
        printf("Cell voltage: %d mV, Discharge limit: %d dA\n", 
               model.cell_voltage_min_mV, model.cell_voltage_discharge_current_limit_dA);
    }

    printf("Reached SOFT_MIN: %d mV. Discharge limit: %d dA\n", 
           model.cell_voltage_min_mV, model.cell_voltage_discharge_current_limit_dA);
    
    // Per calculate_cell_voltage_discharge_current_limit: 
    // if(cell_voltage_min_mV < CELL_VOLTAGE_SOFT_MIN_mV) discharge_limit = 0;
    assert_int_equal(model.cell_voltage_discharge_current_limit_dA, 0);
    
    // Highest level should still be NONE or WARNING (if LOW is warning)
    // ERR_CELL_VOLTAGE_LOW is LEVEL_WARNING
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_LOW), LEVEL_WARNING);

    // 3. Continue discharge until we hit HARD_MIN (2800mV)
    while(true) {
        battery_model_t new_bat = bat;
        bms_model_t new_model = model;
        tick(&new_bat, &new_model, -5.0f, 100); // 5A discharge
        if(new_model.cell_voltage_min_mV <= CELL_VOLTAGE_HARD_MIN_mV) {
            // Stop before crossing threshold
            break;
        }
        bat = new_bat;
        model = new_model;
    }
    // while (model.cell_voltage_min_mV > 2840) { // Stay slightly above to avoid jumping past it in one step
    //      tick(&bat, &model, -5.0f, 100);
    // }
    tick(&bat, &model, -5.0f, 100); // One more step to cross 2800
    tick(&bat, &model, -5.0f, 100); // One more step to cross 2800

    printf("Reached HARD_MIN: %d mV. Highest level: %d\n", 
           model.cell_voltage_min_mV, get_highest_event_level());
    
    assert_true(model.cell_voltage_min_mV <= CELL_VOLTAGE_HARD_MIN_mV);
    // Should have recorded VERY_LOW event (LEVEL_CRITICAL)
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_VERY_LOW), LEVEL_CRITICAL);
    assert_int_equal(get_highest_event_level(), LEVEL_CRITICAL);

    // 4. Wait for escalation (leeway is 1000ms = 1s)
    printf("Waiting for escalation...\n");
    for (int i = 0; i < 11; i++) {
        tick(&bat, &model, -0.1f, 100);
    }

    printf("Final state: %d mV. Highest level: %d (%s)\n", 
           model.cell_voltage_min_mV, get_highest_event_level(),
           get_highest_event_level() == LEVEL_FATAL ? "FATAL" : "NOT FATAL");

    // Should have escalated to FATAL
    assert_int_equal(get_highest_event_level(), LEVEL_FATAL);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_low_voltage_protection),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
