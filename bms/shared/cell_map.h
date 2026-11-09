#pragma once

// The BMB channel layout, and the cell map: which channels have a cell.
//
// The BMS and the supervisor decode the same BMB readings (the BMS by polling,
// the supervisor by snooping the isoSPI bus). The map comes from the BMS's
// pack profile, and the BMS sends it to the supervisor in every status message
// (shared/supervisor_protocol.h). The supervisor only trusts it as far as the
// checks below and its own readings allow: every channel the map leaves out
// must read as empty, so a wrong map can't hide a connected cell.

#include "chemistry/chemistry.h"

#include <stdbool.h>
#include <stdint.h>

// Channel slots the map can describe: room for 16 BMBs (eg. two 8-BMB packs
// in parallel on one chain), whatever the BMS reads today. A pack uses the
// first num_modules of them; the rest are zero in its mask.
#define CELL_MAP_MODULES 16
#define CELL_MAP_CELLS_PER_MODULE 15
#define CELL_MAP_BANKS 5
#define CELL_MAP_CELLS_PER_BANK 3
#define CELL_MAP_CHANNELS (CELL_MAP_MODULES * CELL_MAP_CELLS_PER_MODULE)
// The presence mask as 32-bit words (CELL_PRESENCE_MASK fills the first ones
// and leaves the rest zero), and as sent on the wire (only the channels)
#define CELL_MAP_MASK_WORDS ((CELL_MAP_CHANNELS + 31) / 32)
#define CELL_MAP_MASK_BYTES ((CELL_MAP_CHANNELS + 7) / 8)

// Widen these, and bump SUP_PROTOCOL_VERSION, before raising CELL_MAP_MODULES
_Static_assert(CELL_MAP_CHANNELS <= 255, "num_cells and cells_ok are uint8_t");
_Static_assert(CELL_MAP_MODULES <= 16, "module bitmasks are uint16_t");

typedef struct {
    uint8_t chemistry;   // CHEMISTRY: LFP or NMC (chemistry/chemistry.h)
    uint8_t num_cells;   // NUM_CELLS
    uint8_t num_modules; // NUM_MODULE_VOLTAGES: BMBs the BMS reads
    // Bit n (word n / 32, bit n % 32) set if physical channel n has a cell
    uint32_t presence[CELL_MAP_MASK_WORDS];
} cell_map_t;

// Physical channel number of a cell reading on the BMBs
static inline int cell_map_physical_index(int module, int bank, int cell_in_bank) {
    return (module * CELL_MAP_CELLS_PER_MODULE) + (bank * CELL_MAP_CELLS_PER_BANK) + cell_in_bank;
}

// Whether a physical channel has a cell connected (false out of range)
bool cell_map_is_present(const cell_map_t *map, int physical_index);

// Problems that make a cell map unusable (cell_map_check)
#define CELL_MAP_ERR_CHEMISTRY    0x01 // not LFP or NMC
#define CELL_MAP_ERR_MODULES      0x02 // num_modules not 1 to CELL_MAP_MODULES
#define CELL_MAP_ERR_COUNT        0x04 // no cells, or the mask doesn't have num_cells
#define CELL_MAP_ERR_BEYOND       0x08 // a cell on a module past num_modules
#define CELL_MAP_ERR_EMPTY_MODULE 0x10 // a module up to num_modules with no cells

// Checks a map is self-consistent. Returns CELL_MAP_ERR_* bits, 0 if usable.
uint8_t cell_map_check(const cell_map_t *map);

bool cell_map_equal(const cell_map_t *a, const cell_map_t *b);

// Hash every field with FNV-1a to produce a value the BMS can check to ensure
// the supervisor is using the right cell map.
uint32_t cell_map_hash(const cell_map_t *map);

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
