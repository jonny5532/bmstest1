#pragma once

#include "app/state_machines/base.h"
#include "config/limits.h"
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define VOLTAGE_CALIBRATION_SAMPLES 1024
#define SHORT_VOLTAGE_CALIBRATION_SAMPLES 64


typedef struct bms_model bms_model_t;

// The voltage multiplier to actually use for a stored calibration value, which
// is zero until that channel has been calibrated
static inline float effective_voltage_mul(float stored_mul) {
    return fabsf(stored_mul) > 0.001f ? stored_mul : ADS1115_DEFAULT_MUL;
}

typedef enum {
    CALIBRATION_TYPE_OFFLINE_LONG,
    CALIBRATION_TYPE_OFFLINE_SHORT,
    CALIBRATION_TYPE_ONLINE,
} calibration_type_t;

typedef enum {
    CALIBRATION_STATE_IDLE = 0,
    
    // OFFLINE SHORT sequence
    CALIBRATION_STATE_SHORT_WAIT_OPEN,
    CALIBRATION_STATE_SHORT_STABILIZING_OPEN,
    CALIBRATION_STATE_SHORT_MEASURING_OPEN,
    
    CALIBRATION_STATE_SHORT_WAIT_NEG,
    CALIBRATION_STATE_SHORT_STABILIZING_NEG,
    CALIBRATION_STATE_SHORT_MEASURING_NEG,

    // OFFLINE LONG sequence
    CALIBRATION_STATE_LONG_WAIT_BOTH,
    CALIBRATION_STATE_LONG_STABILIZING,
    CALIBRATION_STATE_LONG_MEASURING,

    // ONLINE sequence
    CALIBRATION_STATE_ONLINE_STABILIZING,
    CALIBRATION_STATE_ONLINE_MEASURING,

    // ISOLATION zero (contactors open, pack floating)
    CALIBRATION_STATE_ISOLATION_STABILIZING,
    CALIBRATION_STATE_ISOLATION_MEASURING,
} calibration_state_t;

typedef enum {
    CALIBRATION_REQUEST_NULL = 0,
    CALIBRATION_REQUEST_START_OFFLINE_LONG = 1,
    CALIBRATION_REQUEST_START_OFFLINE_SHORT = 2,
    CALIBRATION_REQUEST_START_ONLINE = 3,
    CALIBRATION_REQUEST_START_ISOLATION = 4,
} calibration_requests_t;

typedef struct {
    // Anonymous base struct
    sm_t; 
    uint16_t initial_contactor_state;
} calibration_sm_t;

void calibration_sm_tick(bms_model_t *model);
