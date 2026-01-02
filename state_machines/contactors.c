#include "contactors.h"
#include "../model.h"
#include "../hw/actuators/contactors.h"
#include "../hw/chip/time.h"

#include <stdio.h>

// TODO - as well as staleness, check for noisy readings (or use max value sometimes?)

// How recent a reading must be to be considered valid. The slowest sensors are 10Hz so this should be fine.
#define STALENESS_THRESHOLD_MS 200

// Contactor testing constants

// How long to wait for voltage to settle during contactor tests
#define CONTACTORS_TEST_WAIT_MS 500
// How long to wait after a failed precharge (for the PTCs to cool down)
#define CONTACTORS_FAILED_PRECHARGE_TIMEOUT_MS 20000
// How long to wait after a failed contactor test (to avoid rapid cycling)
#define CONTACTORS_FAILED_TEST_TIMEOUT_MS 20000
// Max voltage across a contactor to consider it closed
#define CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV 1000
// Min voltage across a contactor to consider it open
#define CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV 5000

// Precharge constants

// Largest allowable pack current to consider precharge successful
#define PRECHARGE_SUCCESS_MAX_MA 2000
// Minimum voltage difference to consider precharge successful
#define PRECHARGE_SUCCESS_MIN_MV 10000

// Contactor opening constants

// Current below which we can instantly open contactors
#define CONTACTORS_INSTANT_OPEN_MA 1000
// Current below which we can open contactors (after a delay)
#define CONTACTORS_DELAYED_OPEN_MA 5000
// How long we wait for the current to fall before potentially failing to open contactors
#define CONTACTORS_OPEN_TIMEOUT_MS 2000



static inline int32_t abs_int32(int32_t v) {
    return (v < 0) ? -v : v;
}

bool current_is_below(bms_model_t *model, int32_t threshold_ma) {
    if(!millis_recent_enough(model->current_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        return false;
    }
    
    if(abs_int32(model->current_mA)<=threshold_ma) {
        return true;
    }
    return false;
}

bool precharge_successful(bms_model_t *model) {
    if(!millis_recent_enough(model->battery_voltage_millis, STALENESS_THRESHOLD_MS) ||
       !millis_recent_enough(model->output_voltage_millis, STALENESS_THRESHOLD_MS) ||
       !millis_recent_enough(model->neg_contactor_voltage_millis, STALENESS_THRESHOLD_MS) ||
       !millis_recent_enough(model->current_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        return false;
    }

    // Check negative contactor is still closed
    if(abs_int32(model->neg_contactor_voltage_mV) > CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV) {
        return false;
    }

    // Is current still too high?
    if(abs_int32(model->current_mA) > PRECHARGE_SUCCESS_MAX_MA) {
        return false;
    }

    // Is voltage difference low enough?
    if(abs_int32(model->battery_voltage_mV - model->output_voltage_mV) > PRECHARGE_SUCCESS_MIN_MV) {
        return false;
    }

    return true;
}

bool battery_is_healthy(bms_model_t *model) {
    // check battery voltage and temperature

    // should the contactor SM check all these safety conditions, or are they
    // handled upstream by whatever is requesting contactor state changes?

    // main concern is whether to allow external requests to close contactors
    // even if the battery is unhealthy


    return true;
}

bool contactor_neg_seems_closed(bms_model_t *model) {
    if(!millis_recent_enough(model->neg_contactor_voltage_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        return false;
    }

    return abs_int32(model->neg_contactor_voltage_mV) <= CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV; // less than 100mV drop
}

bool contactor_neg_seems_open(bms_model_t *model) {
    if(!millis_recent_enough(model->neg_contactor_voltage_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        return false;
    }

    // FIXME testing
    return true;

    return abs_int32(model->neg_contactor_voltage_mV) > CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV;
}

bool contactor_pos_seems_closed(bms_model_t *model) {
    if(!millis_recent_enough(model->pos_contactor_voltage_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        return false;
    }

    return abs_int32(model->pos_contactor_voltage_mV) <= CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV; // less than 100mV drop
}

bool contactor_pos_seems_open(bms_model_t *model) {
    if(!millis_recent_enough(model->pos_contactor_voltage_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        return false;
    }

    return abs_int32(model->pos_contactor_voltage_mV) > CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV;
}

void contactor_sm_tick(bms_model_t *model) {
    contactors_sm_t *contactor_sm = &model->contactor_sm;
    switch(contactor_sm->state) {
        case CONTACTORS_STATE_OPEN:
            contactors_set_pos_pre_neg(false, false, false);
            if(model->contactor_req == CONTACTORS_REQUEST_CLOSE) {
                model->contactor_req = CONTACTORS_REQUEST_NULL;

                // Start a self-test of the contactors before precharging
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_NEG_OPEN);
            }
            break;
        case CONTACTORS_STATE_PRECHARGING_NEG:
            // Close negative contactor first
            contactors_set_pos_pre_neg(false, false, true);
            if(state_timeout((sm_t*)contactor_sm, 500)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING);
            }
            break;
        case CONTACTORS_STATE_PRECHARGING:
            // Now close precharge contactor (actually just the Bat+ one)
            contactors_set_pos_pre_neg(false, true, true);

            if(state_timeout((sm_t*)contactor_sm, 1000) && precharge_successful(model)) {
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
                    || (current_is_below(model, CONTACTORS_DELAYED_OPEN_MA) && state_timeout((sm_t*)contactor_sm, CONTACTORS_OPEN_TIMEOUT_MS)))
             ) || model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN) {
                 model->contactor_req = CONTACTORS_REQUEST_NULL;
                 state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
             }
             
             break;
        case CONTACTORS_STATE_PRECHARGE_FAILED:
            contactors_set_pos_pre_neg(false, false, false);
            // wait for the precharge to cool down
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_FAILED_PRECHARGE_TIMEOUT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }            
            break;
        case CONTACTORS_STATE_TESTING_NEG_OPEN:
            contactors_set_pos_pre_neg(false, false, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(contactor_neg_seems_open(model)) {
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
                if(contactor_neg_seems_closed(model)) {
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
                if(contactor_pos_seems_open(model)) {
                    // all tests passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_POS_CLOSED);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_POS_CLOSED:
            // Both positive and precharge (since this actually leaves the precharge open due to the inverted logic)
            contactors_set_pos_pre_neg(true, true, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(contactor_pos_seems_closed(model)) {
                    // all tests passed, go to precharging
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING_NEG);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_FAILED:
            contactors_set_pos_pre_neg(false, false, false);
            // Wait for some arbitrary time to avoid cycling too fast
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_FAILED_TEST_TIMEOUT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }
            break;
        default:
            // panic instead?
            printf("Invalid contactor state!");
            state_reset((sm_t*)contactor_sm);
            break;
    }
}
