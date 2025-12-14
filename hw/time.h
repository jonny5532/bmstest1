#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef uint64_t millis_t;
extern millis_t stored_millis;

void update_millis();
static inline millis_t millis() {
    return stored_millis;
}

static inline bool millis_recent_enough(millis_t t, uint32_t delta_ms) {
    if(t==0) {
        // no reading yet
        return false;
    }
    return (millis() - t) <= delta_ms;    
}
