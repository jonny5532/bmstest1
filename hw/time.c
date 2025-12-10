#include "pico/stdlib.h"

#include <stdint.h>


uint64_t millis;

void update_millis() {
    // Update the global millis variable with the current time in milliseconds
    millis = time_us_64() / 1000;
}
