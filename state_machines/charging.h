#pragma once

#include "base.h"

typedef struct {
    // Anonymous base struct
    sm_t; 
} charging_sm_t;

enum charging_states {
    CHARGING_STATE_IDLE = 0,
    CHARGING_STATE_CHARGING = 1,
    CHARGING_STATE_DISCHARGING = 2,
    CHARGING_STATE_LOW_CHARGING = 3,
    CHARGING_STATE_HIGH_DISCHARGING = 4,
};

void charging_sm_tick();
