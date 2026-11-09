#pragma once

#include <stdint.h>

#define PICO_UNIQUE_BOARD_ID_SIZE_BYTES 8

typedef struct {
    uint8_t id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES];
} pico_unique_board_id_t;

// Implemented by tests that need it
void pico_get_unique_board_id(pico_unique_board_id_t *id_out);
