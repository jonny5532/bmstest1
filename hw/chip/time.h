#pragma once

#include <stdbool.h>
#include <stdint.h>

#define TIMESTEP_PERIOD_MS 20

typedef uint64_t millis64_t;
typedef uint32_t millis_t;
// 64-bit version avoids rollovers
extern millis64_t stored_millis64;
// 32-bit version for efficiency
extern millis_t stored_millis;
// timestep counter which increments on each main loop tick
extern uint32_t stored_timestep;

void update_millis();
static inline millis_t millis() {
    return stored_millis;
}
static inline millis64_t millis64() {
    return stored_millis64;
}

static inline bool millis_recent_enough(millis_t t, uint32_t delta_ms) {
    if(t==0) {
        // no reading yet
        return false;
    }
    return (millis() - t) <= delta_ms;    
}

void update_timestep();

// Return the current timestep. The timestep is incremented every main loop tick, never skipping values.
static inline uint32_t timestep() {
    return stored_timestep;
}

// static inline bool timestep_period_ms(uint32_t period_ms, uint32_t offset_ticks) {
//     uint32_t ticks_per_period = period_ms / TIMESTEP_PERIOD_MS;
//     if(ticks_per_period == 0) {
//         ticks_per_period = 1;
//     }
//     return ((timestep() + offset_ticks) % ticks_per_period) == 0;
// }

// Check if the given period in ms has elapsed. Minimum resolution is
// TIMESTEP_PERIOD_MS. Needs passing a pointer to a uint32_t which is used to
// store the last timestep. The initial value of the last timestep can be offset
// slightly, to stagger regular routines.
static inline bool timestep_every_ms(uint32_t period_ms, uint32_t *last_timestep) {
    uint32_t ticks_per_period = period_ms / TIMESTEP_PERIOD_MS;
    if(timestep() >= *last_timestep + ticks_per_period) {
        *last_timestep = timestep();
        return true;
    }
    return false;
}
