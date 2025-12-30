#include "base.h"

#include "../hw/chip/time.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void sm_init(sm_t* sm, const char* name) {
    sm->name = name;
    state_reset(sm);
}

void state_transition(sm_t* sm, uint16_t new_state) {
    printf("%llums [%s] state %d->%d\n", millis64(), sm->name, sm->state, new_state);
    sm->state = new_state;
    sm->last_transition_time = millis();

}

void state_reset(sm_t* sm) {
    return state_transition(sm, 0);
}

bool state_timeout(sm_t* sm, uint32_t timeout) {
    return (millis()-sm->last_transition_time) >= timeout;
}
