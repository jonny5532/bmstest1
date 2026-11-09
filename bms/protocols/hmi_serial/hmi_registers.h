#pragma once

// Encoding of single HMI registers (HMI_REG_* in hmi_serial.h), kept apart
// from the message handling so it can be tested on the host.

#include <stdbool.h>
#include <stdint.h>

#include "app/model.h"

// Whether the register has had its first reading since power-on
bool hmi_register_is_available(uint16_t reg_id, bms_model_t *model);

// Append <reg_id (2 bytes)> <type (1 byte)> <value> to buf, returning the
// number of bytes written, or 0 for an unknown register.
uint8_t hmi_append_register_value(uint8_t *buf, uint16_t reg_id, bms_model_t *model);
