#include "../model.h"

// API for inverters

// Initialise the inverter (setting up CAN, etc). Shouldn't send any messages.
void init_inverter();
// Called every main loop iteration (TIMESTEP_PERIOD_MS ms). Should perform any
// handshakes and then send regular messages. Consider staggering messages to
// avoid exceeding the CAN transmit buffer size.
void inverter_tick(bms_model_t *model);
