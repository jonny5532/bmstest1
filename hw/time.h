#include <stdint.h>

typedef uint64_t millis_t;
extern millis_t millis;

void update_millis();
inline millis_t current_millis() {
    // Placeholder function to return current time in milliseconds
    return millis;
}
