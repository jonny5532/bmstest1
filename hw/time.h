#include <stdint.h>

extern uint64_t millis;

void update_millis();
inline uint64_t current_millis() {
    // Placeholder function to return current time in milliseconds
    return millis;
}
