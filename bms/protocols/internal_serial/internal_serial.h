#pragma once

// Internal serial link to the supervisor MCU (shared/supervisor_protocol.h).
// Sends a status every 100ms and keeps the latest reply in the model; it never
// waits for the supervisor.

typedef struct bms_model bms_model_t;

void init_internal_serial();
void internal_serial_tick(bms_model_t *model);
