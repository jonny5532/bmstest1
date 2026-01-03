// balance resistors are 90 ohm, so at 4V that's ~44mA.
// so 44mC per second, or about 1C every 23 seconds.

// a uint32_t at mC resolution is 1193 Ah, plenty
// a uint16_t at mC resolution is 18 Ah, not enough.

#pragma once

#include "../hw/chip/time.h"
#include "../state_machines/base.h"

#include <stdint.h>

typedef struct bms_model bms_model_t;

typedef struct {
    sm_t;

    uint32_t balance_request_mask[4];
    int16_t balance_time_remaining[120];
    bool even_cells;
//    millis_t last_update_millis;
} balancing_sm_t;

enum balancing_states {
    BALANCING_STATE_IDLE = 0,
    BALANCING_STATE_ACTIVE = 1,
    
};

void balancing_sm_tick(bms_model_t *model);
