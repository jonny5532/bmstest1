#pragma once

// Host-side stand-in for the Pico SDK header. Tests provide the function.

#include <stdint.h>

#define PICO_UNIQUE_BOARD_ID_SIZE_BYTES 8

typedef struct {
    uint8_t id[PICO_UNIQUE_BOARD_ID_SIZE_BYTES];
} pico_unique_board_id_t;

void pico_get_unique_board_id(pico_unique_board_id_t *id_out);
