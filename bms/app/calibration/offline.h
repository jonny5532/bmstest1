#pragma once

#include "../state_machines/base.h"

typedef struct bms_model bms_model_t;

typedef struct {
    // Anonymous base struct
    sm_t; 
} offline_calibration_sm_t;

enum offline_calibration_states {
    OFFLINE_CALIBRATION_STATE_IDLE = 0,
    OFFLINE_CALIBRATION_STATE_WAITING_FOR_CONTACTORS = 1,
    OFFLINE_CALIBRATION_STATE_STABILIZING = 2,
    OFFLINE_CALIBRATION_STATE_MEASURING = 3,
};

typedef enum offline_calibration_requests {
    OFFLINE_CALIBRATION_REQUEST_NULL = 0,
    OFFLINE_CALIBRATION_REQUEST_START = 1,
} offline_calibration_requests_t;

void offline_calibration_sm_tick(bms_model_t *model);
