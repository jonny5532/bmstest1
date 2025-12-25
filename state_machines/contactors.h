#pragma once

#include "base.h"

typedef struct {
    // Anonymous base struct
    sm_t; 
} contactors_sm_t;

enum contactors_states {
    CONTACTORS_STATE_OPEN = 0,
    CONTACTORS_STATE_PRECHARGING = 1,
    CONTACTORS_STATE_CLOSED = 2,
    CONTACTORS_STATE_PRECHARGE_FAILED = 3,
    CONTACTORS_STATE_TESTING_POS_OPEN = 4,
    CONTACTORS_STATE_TESTING_POS_CLOSED = 5,
    CONTACTORS_STATE_TESTING_NEG_OPEN = 6,
    CONTACTORS_STATE_TESTING_NEG_CLOSED = 7,
    CONTACTORS_STATE_TESTING_FAILED = 8,
};

typedef enum contactors_requests {
    CONTACTORS_REQUEST_NULL = 0,
    CONTACTORS_REQUEST_CLOSE = 1,
    CONTACTORS_REQUEST_OPEN = 2,
    CONTACTORS_REQUEST_FORCE_OPEN = 3,
} contactors_requests_t;

void contactor_sm_tick();
