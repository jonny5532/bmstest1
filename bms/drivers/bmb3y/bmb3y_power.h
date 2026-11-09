#pragma once

#include "app/model.h"
#include "app/state_machines/base.h"
#include "sys/events/events.h"

#include <stdbool.h>

// Determine when we should slow/stop BMB activity to save power.

// Stop talking to the BMBs if the cell voltage is very low
static inline bool bmb3y_should_stop(const bms_model_t *model) {
    (void)model;
    // (after stopping we will soon gain a cell-voltages-stale error also)
    return get_event_level(ERR_CELL_VOLTAGE_VERY_LOW) == LEVEL_FATAL;
}

// Sample less often, to reduce battery self-discharge.
static inline bool bmb3y_should_use_slow_mode(bms_model_t *model) {
    if(model->system_sm.state == SYSTEM_STATE_OPERATING ||
       model->contactor_sm.state != CONTACTORS_STATE_OPEN) {
        // Don't use slow mode unless the contactors are explicitly open, since
        // the supervisor will intervene and prevent the contactors from staying
        // closed.
        return false;
    }

    if(model->system_sm.state == SYSTEM_STATE_INACTIVE &&
       state_time((sm_t*)&model->system_sm) > 60000 &&
       (model->cell_voltages_millis > 0 && model->cell_voltage_min_mV < get_minimum_balancing_voltage_mV(model))) {
        // Inactive for a minute, and too low to balance, save power.
        return true;
    }

    if(get_event_level(ERR_CELL_VOLTAGE_LOW) != LEVEL_NONE) {
        // In a cell-voltage-low state, save power with slow mode.
        return true;
    }

    return false;
}
