#include "time.h"

#include "pico/stdlib.h"

#include <stdint.h>

millis64_t stored_millis64;
millis_t stored_millis;

void update_millis() {
    // Update the global millis variable with the current time in milliseconds
    stored_millis64 = time_us_64() / 1000;
    if(stored_millis64 == 0) {
        // Avoid zero value to distinguish uninitialized timestamps
        stored_millis64 = 1;
    }
    stored_millis = (millis_t)(stored_millis64 & 0xFFFFFFFF);
}
