#pragma once

// The supervisor's side of the HMI bus (protocols/hmi_serial/hmi_serial.h,
// 2.11): it shares the lines with its BMS, answers HMI_MSG_READ_SUPERVISOR at
// the BMS's address and nothing else, and never sends anything unasked.

#include "supervisor_logic.h"

#include <stddef.h>
#include <stdint.h>

#define SUP_HMI_RESPONSE_LEN 27

// Handle one HMI frame's payload. Returns the length of the response payload
// written to out, or 0 if there is nothing to send (not ours to answer).
size_t sup_hmi_handle(const sup_logic_t *logic, const uint8_t *payload, size_t len,
    uint32_t now, uint8_t *out, size_t out_size);
