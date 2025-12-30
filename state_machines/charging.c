#include "charging.h"
#include "../model.h"
#include "../hw/chip/time.h"

// voltage thresholds:
// max - dangerously high, stop. eg, 4300mV for NMC
// high - over max normal voltage, eg, 4200mV for NMC. allow discharge but not charge.
// normal - between low and high
// low - below min normal voltage, eg, 3000mV for NMC. allow charge but not discharge.
// min - dangerously low, stop. eg, 2500mV


bool is_charging(bms_model_t *model) {
    // what threshold?
    return model->current_mA > 0;
}

bool is_discharging(bms_model_t *model) {
    // what threshold?
    return model->current_mA < 0;
}

bool voltage_is_under_low(bms_model_t *model) {
    // check min cellvoltage
    return true;
}

bool voltage_is_under_min(bms_model_t *model) {
    // check min cellvoltage
    return true;
}

bool voltage_is_over_high(bms_model_t *model) {
    // check max cellvoltage
    return true;
}

bool voltage_is_over_max(bms_model_t *model) {
    // check max cellvoltage
    return true;
}




void charging_sm_tick(bms_model_t *model) {
    charging_sm_t *charging_sm = &model->charging_sm;
    switch(charging_sm->state) {
        case CHARGING_STATE_IDLE:
            if(is_charging(model)) {
                state_transition((sm_t*)charging_sm, CHARGING_STATE_CHARGING);
            } else if(is_discharging(model)) {
                state_transition((sm_t*)charging_sm, CHARGING_STATE_DISCHARGING);
            }
            break;
        case CHARGING_STATE_CHARGING:
            if(voltage_is_high(model)) {
                // badness, stop after a grace period
            }
            if(!is_charging(model)) {
                state_transition((sm_t*)charging_sm, CHARGING_STATE_IDLE);
            }
            break;
        case CHARGING_STATE_DISCHARGING:
            if(voltage_is_low(model)) {
                // badness, stop

                //state_transition((sm_t*)charging_sm, CHARGING_STATE_LOW_CHARGING);
            } 
            if(!is_discharging(model)) {
                state_transition((sm_t*)charging_sm, CHARGING_STATE_IDLE);
            }
            break;
    }
}