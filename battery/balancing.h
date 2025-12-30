// balance resistors are 90 ohm, so at 4V that's ~44mA.
// so 44mC per second, or about 1C every 23 seconds.

// a uint32_t at mC resolution is 1193 Ah, plenty
// a uint16_t at mC resolution is 18 Ah, not enough.

#pragma once

#include "../hw/chip/time.h"

#include <stdint.h>

typedef struct {
    uint32_t balance_request_mask[4];
    uint32_t balance_active_mask[4];
    int32_t balance_charge_mC[120];
    millis_t last_update_millis;
} balancing_state_t;
