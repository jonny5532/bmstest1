#pragma once

// Stand-in for the DUART driver (implemented by tests that need it)

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pico/stdlib.h"

typedef struct {
    int unused;
} duart;

extern duart duart0;
extern duart duart1;

bool init_duart(duart *u, uint baud_rate, uint tx_pin, uint rx_pin, bool deassert_tx_when_idle);
size_t duart_read_packet(duart *u, uint8_t *buf, size_t buf_size);
bool duart_send_packet(duart *u, const uint8_t *payload, size_t payload_len);
size_t duart_get_tx_free_space(duart *u);
