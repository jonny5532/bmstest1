// balance resistors are 90 ohm, so at 4V that's ~44mA.
// so 44mC per second, or about 1C every 23 seconds.

// a uint32_t at mC resolution is 1193 Ah, plenty
// a uint16_t at mC resolution is 18 Ah, not enough.

#pragma once

#include "../../sys/time/time.h"
#include "../state_machines/base.h"

#include <stdint.h>

typedef struct bms_model bms_model_t;

typedef struct {
    sm_t;

    // Bitmap of which cells to balance this cycle. Each bit corresponds to a
    // cell (1=balance, 0=no balance).
    // Bit 0 of the first uint32_t corresponds to cell 0.
    uint32_t balance_request_mask[4];
    // Remaining balance time for each cell, in balancing periods, for the
    // current balancing session.
    int16_t balance_time_remaining[120];
    // Whether we're balancing even or odd cells this cycle.
    bool even_cells;
    uint16_t pause_counter;
    bool is_pause_cycle;
} balancing_sm_t;

enum balancing_states {
    BALANCING_STATE_IDLE = 0,
    BALANCING_STATE_ACTIVE = 1,
};

void balancing_sm_tick(bms_model_t *model);
void pause_balancing(balancing_sm_t *balancing_sm);
