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

// Return the current time in milliseconds since boot, as of the start of the
// current tick. This should change by approximately TIMESTEP_PERIOD_MS each
// tick, although might jump if a slow operation (such as a flash write) causes
// a larger delay.
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

// Return the current timestep. The timestep is incremented every main loop
// tick, never skipping values. Unless a strict schedule is required, it is
// better to distribute operations based on timesteps, as this makes it possible
// to explicitly separate operations into different ticks, avoiding the problem
// of the tick after a slow operation itself taking longer than expected due to
// every timer having expired at once.
static inline uint32_t timestep() {
    return stored_timestep;
}

// Check if the given period in ms has elapsed, by counting the equivalent
// number of timesteps. Minimum resolution is TIMESTEP_PERIOD_MS. Needs passing
// a pointer to a uint32_t which is used to store the last timestep. The initial
// value of the last timestep can be offset slightly, to stagger regular
// routines.
static inline bool timestep_every_ms(uint32_t period_ms, uint32_t *last_timestep) {
    uint32_t ticks_per_period = period_ms / TIMESTEP_PERIOD_MS;
    if(timestep() >= *last_timestep + ticks_per_period) {
        *last_timestep = timestep();
        return true;
    }
    return false;
}
