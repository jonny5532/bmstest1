#include <stdint.h>

typedef uint64_t millis_t;
extern millis_t stored_millis;

void update_millis();
inline millis_t millis() {
    return stored_millis;
}
