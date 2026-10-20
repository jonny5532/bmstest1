#pragma once

#include <stddef.h>
#include <stdint.h>

typedef enum {
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR
} log_level_t;

#define debug_printf(format, ...)   logging_printf(LOG_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define info_printf(format, ...)    logging_printf(LOG_LEVEL_INFO, format, ##__VA_ARGS__)
#define warning_printf(format, ...) logging_printf(LOG_LEVEL_WARNING, format, ##__VA_ARGS__)
#define error_printf(format, ...)   logging_printf(LOG_LEVEL_ERROR, format, ##__VA_ARGS__)

void logging_printf(log_level_t level, const char *format, ...);
// Copy buffered log output to USB stdio, as much as fits without waiting.
// Main loop only: not from an IRQ, where it could interrupt a printf().
void logging_flush_to_stdout();
size_t logging_read(uint8_t *dest, uint32_t len);
