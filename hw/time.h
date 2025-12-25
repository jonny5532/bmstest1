#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef uint64_t millis64_t;
typedef uint32_t millis_t;
// 64-bit version avoids rollovers
extern millis64_t stored_millis64;
// 32-bit version for efficiency
extern millis_t stored_millis;

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
