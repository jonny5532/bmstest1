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

#include "sys/logging/logging.h"
void logging_printf(log_level_t level, const char *fmt, ...) {
    (void)level;
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

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

static void discharge(battery_model_t *bat, bms_model_t *model, uint32_t target_cell_voltage_mV) {
    float current = model->discharge_current_limit_dA / 10.0f; // in A

    while (model->cell_voltage_min_mV > target_cell_voltage_mV) {
        tick(bat, model, -current, 10); // 10ms timestep

        // Respect current limit
        float new_limit = model->discharge_current_limit_dA / 10.0f;
        if (new_limit < current) {
            current = new_limit;
        }

        // Converge towards target voltage
        float convergence_limit = (model->cell_voltage_min_mV - target_cell_voltage_mV) / 10.0f; // in A
        if (convergence_limit < current) {
            current = convergence_limit;
        }

        if(current<0.1f) {
            current = 0.1f; // minimum current to avoid stalling
        }

        // printf("Current cell voltage: %d mV, Next current: %.2f A\n",
        //       model->cell_voltage_min_mV,
        //       current);
    }
}

static void test_low_voltage_protection(void **state) {
    (void) state;
    bms_model_t model = {0};
    battery_model_t bat = {
        .capacity_Ah = 2.0f,
        .soc = 0.1f, // Start at 10% SoC
        .internal_resistance_Ohm = 0.00005f
    };

    // System must be in RUNNING state to trigger safety checks properly
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    
    // Set some initial timestamps to avoid "stale" errors immediately
    model.high_voltages.battery_millis = stored_millis;
    model.cell_voltage_millis = stored_millis;
    model.module_temperatures_millis = stored_millis;
    model.current_millis = stored_millis;
    model.temperature_max = 25.0f; // 25.0C
    model.temperature_min = 25.0f;
    model.contactor_sm.enable_current = true;

    // 1. Initially safe
    tick(&bat, &model, 0, 100);

    print_bms_events();
    assert_int_equal(get_highest_event_level(), LEVEL_NONE);
    // Discharge limit should be healthy
    assert_true(model.cell_voltage_discharge_current_limit_dA > 100);

    // 2. Discharge to just below SOFT_MIN
    discharge(&bat, &model, get_cell_voltage_soft_min_mV(&model) - 1);
    // Check that discharge limit has been zeroed           
    assert_int_equal(model.cell_voltage_discharge_current_limit_dA, 0);
    // Below the soft minimum: critical, escalating only after an hour
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_LOW), LEVEL_CRITICAL);

    // 3. Sudden discharge to below HARD_MIN
    tick(&bat, &model, -10000.0f, 100);
    assert_true(model.cell_voltage_min_mV <= CELL_VOLTAGE_HARD_MIN_mV);
    // Should have recorded VERY_LOW event (LEVEL_CRITICAL)
    assert_int_equal(get_event_level(ERR_CELL_VOLTAGE_VERY_LOW), LEVEL_CRITICAL);
    // And charge buffer exceeded
    assert_int_equal(get_event_level(ERR_SOFT_CHARGE_BUFFER_EXCEEDED), LEVEL_CRITICAL);
    // But nothing higher yet
    assert_int_equal(get_highest_event_level(), LEVEL_CRITICAL);

    // 4. Wait for escalation (leeway is 1000ms = 1s)
    for (int i = 0; i < 11; i++) {
        tick(&bat, &model, -0.1f, 100);
    }

    // Should have escalated to FATAL
    assert_int_equal(get_highest_event_level(), LEVEL_FATAL);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_low_voltage_protection),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
