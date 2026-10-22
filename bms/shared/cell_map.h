#pragma once

// Cell map shared by the BMS and the supervisor. Both decode the same BMB
// readings (the BMS by polling, the supervisor by snooping the isoSPI bus), so
// they must agree on which channels are populated. The map comes from the
// CELL_PRESENCE_MASK of the selected BMS_PROFILE in config/limits.h.

#include <stdbool.h>
#include <stdint.h>

// Channel slots on the BMB chain
#define CELL_MAP_MODULES 8
#define CELL_MAP_CELLS_PER_MODULE 15
#define CELL_MAP_BANKS 5
#define CELL_MAP_CELLS_PER_BANK 3
#define CELL_MAP_CHANNELS (CELL_MAP_MODULES * CELL_MAP_CELLS_PER_MODULE)

// Physical channel number of a cell reading on the BMBs
static inline int cell_map_physical_index(int module, int bank, int cell_in_bank) {
    return (module * CELL_MAP_CELLS_PER_MODULE) + (bank * CELL_MAP_CELLS_PER_BANK) + cell_in_bank;
}

// Whether a physical channel has a cell connected
bool cell_map_is_present(int physical_index);

// The logical cell number (counting only present channels) of a physical
// channel, or -1 if the channel isn't present
int cell_map_logical_index(int physical_index);

// Hash of the pack configuration the cell checks depend on (profile, presence
// mask, cell count, hard cell limits). The BMS and supervisor exchange it so a
// pair built for different packs never enables the contactors.
uint32_t cell_map_hash(void);

// A reading that was never decoded (module not read, or failed its CRC)
#define CELL_MV_NOT_READ INT16_MIN

// Convert a raw BMB cell reading to mV. A non-existent cell reads 0xFFFF and
// becomes -1; a zero reading becomes 1mV to tell it apart from unmeasured.
static inline int16_t cell_raw_to_mV(uint16_t raw) {
    if(raw == 0xFFFF) {
        return -1;
    } else if(raw == 0) {
        return 1;
    }
    return (int16_t)((raw * 2) / 25);
}
