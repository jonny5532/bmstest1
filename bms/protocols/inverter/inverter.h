#pragma once

typedef struct inverter_outputs inverter_outputs_t;

// API for inverters

// Initialise the inverter (setting up CAN, etc). Shouldn't send any messages.
void init_inverter();
// Called every main loop iteration (TIMESTEP_PERIOD_MS ms). Should perform any
// handshakes and then send regular messages. Consider staggering messages to
// avoid exceeding the CAN transmit buffer size.
void inverter_tick(inverter_outputs_t *outputs);
// Print a one-line status summary (presence, handshake state, link health) to
// the log. Intended for the periodic debug dump.
void inverter_debug_print(void);
