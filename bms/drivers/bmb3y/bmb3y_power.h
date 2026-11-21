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

// Current flowing with the contactors open (eg. a welded contactor). A stale
// reading doesn't count, so a faulted current sensor can't keep the BMBs fast.
static inline bool bmb3y_current_flowing(const bms_model_t *model) {
    return millis_recent_enough(model->current_millis, CURRENT_STALE_FAULT_THRESHOLD_MS) &&
        (model->current_mA > CONTACTORS_INSTANT_OPEN_MA || model->current_mA < -CONTACTORS_INSTANT_OPEN_MA);
}

// Pack at or above the BATTERY_TEMPERATURE_HIGH limit. A stale reading or a
// faulty thermistor (which makes the pack read THERMISTOR_FAULT_dC) doesn't
// count, so a broken sensor can't keep the BMBs fast.
static inline bool bmb3y_pack_hot(bms_model_t *model) {
    return millis_recent_enough(model->temperature_millis, TEMPERATURE_STALE_THRESHOLD_MS(model)) &&
        model->temperature_max >= TEMPERATURE_SOFT_MAX &&
        model->temperature_max < THERMISTOR_FAULT_dC * 0.1f;
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

    if(model->system_sm.state == SYSTEM_STATE_FAULT &&
       state_time((sm_t*)&model->system_sm) > 60000 &&
       model->balancing_sm.state != BALANCING_STATE_ACTIVE &&
       !bmb3y_current_flowing(model) &&
       !bmb3y_pack_hot(model)) {
        // Faulted (contactors forced open until a reboot) for a minute, not
        // finishing a balancing session, and nothing worth watching closely:
        // save power.
        return true;
    }

    if(get_event_level(ERR_CELL_VOLTAGE_LOW) != LEVEL_NONE) {
        // In a cell-voltage-low state, save power with slow mode.
        return true;
    }

    return false;
}
