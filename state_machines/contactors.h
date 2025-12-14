#include "base.h"

typedef struct {
    // Anonymous base struct
    sm_t; 
} contactors_sm_t;

enum contactors_states {
    CONTACTORS_STATE_OPEN = 0,
    CONTACTORS_STATE_PRECHARGING = 1,
    CONTACTORS_STATE_CLOSED = 2,
};

enum contactors_requests {
    CONTACTORS_REQUEST_NULL = 0,
    CONTACTORS_REQUEST_CLOSE = 1,
    CONTACTORS_REQUEST_OPEN = 2,
};

void contactor_sm_tick();
