#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drivers/bmb3y/bmb3y_power.h"
#include "app/model.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "config/limits.h"

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    (void)format;
}

millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;

static int setup(void **state) {
    (void) state;
    stored_millis = 1000;
    stored_millis64 = 1000;
    for(int i = 0; i < ERR_HIGHEST; i++) {
        bms_event_slots[i] = (bms_event_slot_t){0};
    }
    extern uint16_t highest_level;
    highest_level = LEVEL_NONE;
    return 0;
}

// Pack sitting with the contactors open, the system in `system_state` since
// `state_age_ms` ago, and the lowest cell at cell_mV.
static void setup_model(bms_model_t *model, enum system_states system_state, uint32_t state_age_ms, uint16_t cell_mV) {
    *model = (bms_model_t){0};
    model->system_sm.state = system_state;
    model->contactor_sm.state = CONTACTORS_STATE_OPEN;
    stored_millis64 += state_age_ms;
    stored_millis += state_age_ms;
    model->system_sm.last_transition_time = stored_millis64 - state_age_ms;
    model->cell_voltages_millis = stored_millis;
    model->cell_voltage_min_mV = cell_mV;
}

static void test_inactive_too_low_to_balance(void **state) {
    (void) state;
    bms_model_t model = {0};
    uint16_t below_balancing = get_minimum_balancing_voltage_mV(&model) - 1;

    setup_model(&model, SYSTEM_STATE_INACTIVE, 61000, below_balancing);
    assert_true(bmb3y_should_use_slow_mode(&model));

    // Not until it has been inactive for a minute
    setup_model(&model, SYSTEM_STATE_INACTIVE, 59000, below_balancing);
    assert_false(bmb3y_should_use_slow_mode(&model));

    // Not while high enough to balance
    setup_model(&model, SYSTEM_STATE_INACTIVE, 61000, get_minimum_balancing_voltage_mV(&model));
    assert_false(bmb3y_should_use_slow_mode(&model));
}

static void test_cell_voltage_low_with_contactors_open(void **state) {
    (void) state;
    bms_model_t model;

    // Fresh into INACTIVE (so the idle rule alone wouldn't apply), but below
    // the soft minimum
    setup_model(&model, SYSTEM_STATE_INACTIVE, 0, 3000);
    assert_false(bmb3y_should_use_slow_mode(&model));
    raise_bms_event(ERR_CELL_VOLTAGE_LOW, 0);
    assert_true(bmb3y_should_use_slow_mode(&model));
}

static void test_cell_voltage_low_fault_uses_slow_mode(void **state) {
    (void) state;
    bms_model_t model;

    // The forgotten flat pack: CELL_VOLTAGE_LOW has escalated, the BMS has
    // faulted and opened the contactors. Keep monitoring, slowly.
    setup_model(&model, SYSTEM_STATE_FAULT, 0, 3000);
    raise_bms_event(ERR_CELL_VOLTAGE_LOW, 0);
    bms_event_slots[ERR_CELL_VOLTAGE_LOW].level = LEVEL_FATAL;

    assert_true(bmb3y_should_use_slow_mode(&model));
    assert_false(bmb3y_should_stop(&model));
}

static void test_never_with_contactors_closed(void **state) {
    (void) state;
    bms_model_t model = {0};
    uint16_t below_balancing = get_minimum_balancing_voltage_mV(&model) - 1;

    static const enum contactors_states not_open[] = {
        CONTACTORS_STATE_PRECHARGING,
        CONTACTORS_STATE_CLOSED,
        CONTACTORS_STATE_AWAITING_OPEN,
        CONTACTORS_STATE_CALIBRATING_CLOSED,
    };

    raise_bms_event(ERR_CELL_VOLTAGE_LOW, 0);
    for(size_t i = 0; i < sizeof(not_open) / sizeof(not_open[0]); i++) {
        setup_model(&model, SYSTEM_STATE_INACTIVE, 61000, below_balancing);
        model.contactor_sm.state = not_open[i];
        assert_false(bmb3y_should_use_slow_mode(&model));

        setup_model(&model, SYSTEM_STATE_FAULT, 0, below_balancing);
        model.contactor_sm.state = not_open[i];
        assert_false(bmb3y_should_use_slow_mode(&model));
    }
}

static void test_never_while_operating(void **state) {
    (void) state;
    bms_model_t model;

    // Operating with the contactors (still) open: they may close at any moment
    setup_model(&model, SYSTEM_STATE_OPERATING, 61000, 3000);
    raise_bms_event(ERR_CELL_VOLTAGE_LOW, 0);
    assert_false(bmb3y_should_use_slow_mode(&model));
}

static void test_stop_below_hard_minimum(void **state) {
    (void) state;
    bms_model_t model;
    setup_model(&model, SYSTEM_STATE_FAULT, 0, CELL_VOLTAGE_HARD_MIN_mV - 1);

    raise_bms_event(ERR_CELL_VOLTAGE_VERY_LOW, 0);
    assert_false(bmb3y_should_stop(&model));
    bms_event_slots[ERR_CELL_VOLTAGE_VERY_LOW].level = LEVEL_FATAL;
    assert_true(bmb3y_should_stop(&model));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_inactive_too_low_to_balance, setup),
        cmocka_unit_test_setup(test_cell_voltage_low_with_contactors_open, setup),
        cmocka_unit_test_setup(test_cell_voltage_low_fault_uses_slow_mode, setup),
        cmocka_unit_test_setup(test_never_with_contactors_closed, setup),
        cmocka_unit_test_setup(test_never_while_operating, setup),
        cmocka_unit_test_setup(test_stop_below_hard_minimum, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
