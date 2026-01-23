#pragma once

#include "base.h"

typedef struct bms_model bms_model_t;

typedef struct {
    // Anonymous base struct
    sm_t; 
} system_sm_t;

enum system_states {
    SYSTEM_STATE_UNINITIALIZED = 0,
    SYSTEM_STATE_INITIALIZING = 1,
    SYSTEM_STATE_CALIBRATING = 2,
    SYSTEM_STATE_INACTIVE = 3,
    SYSTEM_STATE_OPERATING = 4,
    SYSTEM_STATE_FAULT = 5,
};

typedef enum system_requests {
    SYSTEM_REQUEST_NULL = 0,
    SYSTEM_REQUEST_RUN = 1,
    SYSTEM_REQUEST_STOP = 2,
    SYSTEM_REQUEST_CALIBRATE = 3,
} system_requests_t;

void system_sm_tick(bms_model_t *model);

