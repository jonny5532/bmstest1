#include <stdbool.h>
#include <stdint.h>

typedef struct sm_struct {
    uint16_t state;
    uint64_t last_transition_time;
    const char* name;
} sm_t;

void sm_init(sm_t* sm, const char* name);
void state_reset(sm_t* sm);
void state_transition(sm_t* sm, uint16_t new_state);
bool state_timeout(sm_t* sm, uint32_t timeout);
