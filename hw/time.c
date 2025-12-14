#include "time.h"

#include "pico/stdlib.h"

#include <stdint.h>


millis_t stored_millis;

void update_millis() {
    // Update the global millis variable with the current time in milliseconds
    stored_millis = time_us_64() / 1000;
}
