#pragma once

#include <stdint.h>

// Only what logging.c may use: it must not drive tinyusb itself (no
// tud_task/tud_cdc_write/tud_cdc_write_flush here, so doing so won't link)
uint32_t tud_cdc_write_available(void);
