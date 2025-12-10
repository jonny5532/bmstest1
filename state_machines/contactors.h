#include "base.h"

typedef struct {
    // Anonymous base struct
    sm_t;   
} contactors_sm_t;

enum contactors_states {
    CONTACTORS_OPEN = 0,
    CONTACTORS_PRECHARGING = 1,
    CONTACTORS_CLOSED = 2,
};

void contactor_sm_tick();
