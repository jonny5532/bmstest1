#pragma once

// Encoding of single HMI registers (HMI_REG_* in hmi_serial.h), kept apart
// from the message handling so it can be tested on the host.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "app/model.h"

// Whether the register has had its first reading since power-on
bool hmi_register_is_available(uint16_t reg_id, bms_model_t *model);

// Append <reg_id (2 bytes)> <type (1 byte)> <value> to buf, returning the
// number of bytes written, or 0 for an unknown register.
uint8_t hmi_append_register_value(uint8_t *buf, uint16_t reg_id, bms_model_t *model);

// A register as LIST_REGISTERS gives it
typedef struct {
    uint16_t id;
    uint8_t type;  // HMI_TYPE_*
    uint8_t flags; // HMI_REG_FLAG_*
} hmi_register_info_t;

// The registers this build has, in increasing ID order
size_t hmi_register_count(void);
const hmi_register_info_t *hmi_register_info(size_t index); // NULL past the end
const hmi_register_info_t *hmi_find_register(uint16_t reg_id); // NULL if unknown

// Bytes before the entries in a LIST_REGISTERS_RESPONSE, and per entry
#define HMI_LIST_REGISTERS_HEADER_LEN 7
#define HMI_LIST_REGISTERS_ENTRY_LEN  4

// Build a LIST_REGISTERS_RESPONSE in buf holding as many entries from the
// start index as fit in capacity bytes. Returns the response length.
size_t hmi_build_list_registers_response(uint8_t *buf, size_t capacity, uint8_t device_address,
                                         uint16_t start);
