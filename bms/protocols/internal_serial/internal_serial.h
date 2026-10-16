#pragma once

typedef struct bms_model bms_model_t;

void init_internal_serial();
void internal_serial_tick(bms_model_t *model);
