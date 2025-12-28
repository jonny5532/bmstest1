#include "contactors.h"
#include "../model.h"
#include "../hw/time.h"

#include <stdio.h>

#define PRECHARGE_SUCCESS_MA 2000
#define CONTACTORS_INSTANT_OPEN_MA 1000
#define CONTACTORS_DELAYED_OPEN_MA 5000
#define CONTACTORS_OPEN_DELAY_MS 2000
#define CONTACTORS_TEST_WAIT_MS 500


inline int32_t abs_int32(int32_t v) {
    return (v < 0) ? -v : v;
}

bool current_is_below(bms_model_t *model, int32_t threshold_ma) {
    if(!millis_recent_enough(millis(), 100)) {
        // stale reading
        return false;
    }
    
    if(abs_int32(model->current_mA)<=threshold_ma) {
        return true;
    }
    return false;
}

bool voltage_diff_is_below(bms_model_t *model, int32_t threshold_mv) {
    if(!millis_recent_enough(model->battery_voltage_millis, 200) ||
       !millis_recent_enough(model->output_voltage_millis, 200)) {
        // stale reading
        return false;
    }

    int32_t diff = abs_int32(model->battery_voltage_mv - model->output_voltage_mv);

    if(diff <= threshold_mv) {
        return true;
    }
    return false;
}

bool battery_is_healthy(bms_model_t *model) {
    // check battery voltage and temperature

    // should the contactor SM check all these safety conditions, or are they
    // handled upstream by whatever is requesting contactor state changes?

    // main concern is whether to allow external requests to close contactors
    // even if the battery is unhealthy


    return true;
}

bool contactor_neg_seems_closed() {
    // TODO - check that reading is fresh, and that voltage across neg contactor is low
    return true;
}

bool contactor_neg_seems_open() {
    // TODO - check that reading is fresh, and that voltage across neg contactor is high
    return true;
}

bool contactor_pos_seems_closed() {
    // TODO - check that reading is fresh, and that voltage across pos contactor is low
    return true;
}

bool contactor_pos_seems_open() {
    // TODO - check that reading is fresh, and that voltage across pos contactor is high
    return true;
}

void contactor_sm_tick(bms_model_t *model) {
    contactors_sm_t *contactor_sm = &model->contactor_sm;
    switch(contactor_sm->state) {
        case CONTACTORS_STATE_OPEN:
            contactors_set_pos_pre_neg(false, false, false);
            if(model->contactor_req == CONTACTORS_REQUEST_CLOSE) {
                model->contactor_req = CONTACTORS_REQUEST_NULL;

                // Do a self-test of the contactors before precharging
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_NEG_CLOSED);
                //state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING);
            }
            break;
        case CONTACTORS_STATE_PRECHARGING:
            contactors_set_pos_pre_neg(false, true, true);

            // TODO: check voltage diff too
            if(state_timeout((sm_t*)contactor_sm, 1000) && voltage_diff_is_below(model, 10000)) {
                // successful precharge
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CLOSED);
            } else if(state_timeout((sm_t*)contactor_sm, 10000)) {
                // failed to precharge
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGE_FAILED);
            }
            break;
        case CONTACTORS_STATE_CLOSED:
             contactors_set_pos_pre_neg(true, false, true);

             if((
                model->contactor_req == CONTACTORS_REQUEST_OPEN && (
                    current_is_below(model, CONTACTORS_INSTANT_OPEN_MA)
                    || (current_is_below(model, CONTACTORS_DELAYED_OPEN_MA) && state_timeout((sm_t*)contactor_sm, CONTACTORS_OPEN_DELAY_MS)))
             ) || model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN) {
                 model->contactor_req = CONTACTORS_REQUEST_NULL;
                 state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
             }
             
             break;
        case CONTACTORS_STATE_PRECHARGE_FAILED:
            contactors_set_pos_pre_neg(false, false, false);
            // wait for the precharge to cool down
            if(state_timeout((sm_t*)contactor_sm, 20000)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }            
            break;
        case CONTACTORS_STATE_TESTING_NEG_OPEN:
            contactors_set_pos_pre_neg(false, false, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(contactor_neg_seems_open()) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_NEG_CLOSED);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_NEG_CLOSED:
            contactors_set_pos_pre_neg(false, false, true);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(contactor_neg_seems_closed()) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_POS_OPEN);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_POS_OPEN:
            contactors_set_pos_pre_neg(false, false, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(contactor_pos_seems_open()) {
                    // all tests passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_POS_CLOSED);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_POS_CLOSED:
            contactors_set_pos_pre_neg(true, false, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(contactor_pos_seems_closed()) {
                    // all tests passed, go to precharging
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_FAILED:
            contactors_set_pos_pre_neg(false, false, false);
            // wait for some arbitrary time
            if(state_timeout((sm_t*)contactor_sm, 20000)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }
            break;
        default:
            // panic instead?
            printf("erk!");
            state_reset((sm_t*)contactor_sm);
            break;
    }
}
