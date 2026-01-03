#pragma once

#include "base.h"

typedef struct bms_model bms_model_t;

typedef struct {
    // Anonymous base struct
    sm_t; 
} contactors_sm_t;

enum contactors_states {
    CONTACTORS_STATE_OPEN = 0,
    CONTACTORS_STATE_PRECHARGING_NEG = 1,
    CONTACTORS_STATE_PRECHARGING = 2,
    CONTACTORS_STATE_CLOSED = 3,
    CONTACTORS_STATE_PRECHARGE_FAILED = 4,
    CONTACTORS_STATE_TESTING_POS_OPEN = 5,
    CONTACTORS_STATE_TESTING_POS_CLOSED = 6,
    CONTACTORS_STATE_TESTING_NEG_OPEN = 7,
    CONTACTORS_STATE_TESTING_NEG_CLOSED = 8,
    CONTACTORS_STATE_TESTING_FAILED = 9,
};

typedef enum contactors_requests {
    CONTACTORS_REQUEST_NULL = 0,
    CONTACTORS_REQUEST_CLOSE = 1,
    CONTACTORS_REQUEST_OPEN = 2,
    CONTACTORS_REQUEST_FORCE_OPEN = 3,
} contactors_requests_t;

void contactor_sm_tick(bms_model_t *model);
