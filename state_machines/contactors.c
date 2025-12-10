#include "contactors.h"

#include <stdio.h>

void contactor_sm_tick(contactors_sm_t *contactor_sm) {
    switch(contactor_sm->state) {
        case CONTACTORS_OPEN:
            if(state_timeout((sm_t*)contactor_sm, 1000)) {
                printf("changing state1\n");
                state_transition((sm_t*)contactor_sm, CONTACTORS_PRECHARGING);
            }
            break;
        case CONTACTORS_PRECHARGING:
            if(state_timeout((sm_t*)&contactor_sm, 5000)) {
                printf("changing state2\n");
                state_transition((sm_t*)contactor_sm, CONTACTORS_CLOSED);
            }
            break;
        case CONTACTORS_CLOSED:
            break;
        default:
            printf("erk!");
            state_reset((sm_t*)contactor_sm);
            break;
    }
}
