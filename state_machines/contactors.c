#include "contactors.h"
#include "../model.h"
#include "../hw/time.h"

#include <stdio.h>

#define PRECHARGE_SUCCESS_MA 2000
#define CONTACTORS_INSTANT_OPEN_MA 1000
#define CONTACTORS_DELAYED_OPEN_MA 5000
#define CONTACTORS_OPEN_DELAY_MS 2000



inline int32_t abs_int32(int32_t v) {
    return (v < 0) ? -v : v;
}

bool current_is_below(bms_model_t *model, int32_t threshold_ma) {
    if(!millis_recent_enough(model->current_millis, 100)) {
        // stale reading
        return false;
    }
    
    if(abs_int32(model->current_mA)<=threshold_ma) {
        return true;
    }
    return false;
}

#define PWM_INITIAL_LEVEL 0x1400
#define PWM_DECREMENT 0x4
#define PWM_MIN_LEVEL 0x300

// should these go in the model? or the state machine?
uint32_t pos_level = 0;
uint32_t pre_level = 0;
uint32_t neg_level = 0;

void contactors_set_pos_pre_neg(bool pos, bool pre, bool neg) {
    // Positive and precharge contactors are in series
    bool actual_pos = pos || pre;
    // Precharge contactor actually bypasses the precharge resistor
    bool actual_pre = pos && !pre;
    bool actual_neg = neg;

    if(!actual_pos) {
        pos_level = 0;
    } else if(actual_pos && pos_level==0) {
        pos_level = PWM_INITIAL_LEVEL;
    } else if(actual_pos) {
        pos_level -= PWM_DECREMENT;
        if(pos_level<PWM_MIN_LEVEL) pos_level = PWM_MIN_LEVEL;
    }
    pwm_set(PIN_CONTACTOR_POS, pos_level);

    
}

void contactor_sm_tick(bms_model_t *model) {
    contactors_sm_t *contactor_sm = &model->contactor_sm;
    switch(contactor_sm->state) {
        case CONTACTORS_STATE_OPEN:
            contactors_set_pos_pre_neg(false, false, false);
            if(model->contactor_req == CONTACTORS_REQUEST_CLOSE) {
                model->contactor_req = CONTACTORS_REQUEST_NULL;
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING);
            }
            break;
        case CONTACTORS_STATE_PRECHARGING:
            contactors_set_pos_pre_neg(false, true, true);

            // TODO: check voltage diff too
            if(state_timeout((sm_t*)contactor_sm, 1000) && current_is_below(model, PRECHARGE_SUCCESS_MA)) {
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
        default:
            // panic instead?
            printf("erk!");
            state_reset((sm_t*)contactor_sm);
            break;
    }
}
