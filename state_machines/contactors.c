#include "contactors.h"

#include <stdio.h>

#define PRECHARGE_SUCCESS_MA 2000
#define CONTACTORS_INSTANT_OPEN_MA 1000
#define CONTACTORS_DELAYED_OPEN_MA 5000
#define CONTACTOR_OPEN_DELAY_MS 2000

bool current_is_below(int32_t threshold_ma) {
    // TODO - check age of current reading
    if(millis_older_than(model.current_millis, 100)) {
        return false;
    }
    
    if(abs_int32(model.current_dA)<=threshold_ma) {
        return true;
    }
    return false;
}

void contactor_sm_tick(contactors_sm_t *contactor_sm) {
    switch(contactor_sm->state) {
        case CONTACTORS_OPEN:
            contactors_set_pos_pre_neg(false, false, false);
            if(model.contactors_req == CONTACTORS_REQUEST_CLOSE) {
                model.contactors_req = CONTACTORS_REQUEST_NULL;
                state_transition((sm_t*)contactor_sm, CONTACTORS_PRECHARGING);
            }
            break;
        case CONTACTORS_PRECHARGING:
            contactors_set_pos_pre_neg(false, true, true);

            if(state_timeout((sm_t*)contactor_sm, 1000) && current_is_below(PRECHARGE_SUCCESS_MA)) {
                // successful precharge
                state_transition((sm_t*)contactor_sm, CONTACTORS_CLOSED);
            } elseif(state_timeout((sm_t*)contactor_sm, 10000)) {
                // failed to precharge
                state_transition((sm_t*)contactor_sm, CONTACTORS_PRECHARGE_FAILED);
            }
            break;
        case CONTACTORS_CLOSED:
             contactors_set_pos_pre_neg(true, false, true);

             if((
                model.contactors_req == CONTACTORS_REQUEST_OPEN && (
                    current_is_below(CONTACTOR_INSTANT_OPEN_MA)
                    || (current_is_below(CONTACTOR_DELAYED_OPEN_MA) && state_timeout((sm_t*)contactor_sm, CONTACTOR_OPEN_DELAY_MS))) {
             ) || model.contactors_req == CONTACTORS_REQUEST_FORCE_OPEN) {
                 model.contactors_req = CONTACTORS_REQUEST_NULL;
                 state_transition((sm_t*)contactor_sm, CONTACTORS_OPEN);
             }
             
             break;
        case CONTACTORS_PRECHARGE_FAILED:
            contactors_set_pos_pre_neg(false, false, false);
            // wait for the precharge to cool down
            if(state_timeout((sm_t*)contactor_sm, 20000)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_OPEN);
            }            
            break;
        default:
            // panic instead?
            printf("erk!");
            state_reset((sm_t*)contactor_sm);
            break;
    }
}
