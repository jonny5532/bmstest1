#include "cell_map.h"

#include "config/limits.h"

static const uint32_t cell_presence_mask[] = CELL_PRESENCE_MASK;

#define CELL_PRESENCE_WORDS ((int)(sizeof(cell_presence_mask) / sizeof(cell_presence_mask[0])))

bool cell_map_is_present(int physical_index) {
    if(physical_index < 0 || physical_index >= CELL_MAP_CHANNELS) {
        return false;
    }
    int word = physical_index / 32;
    if(word >= CELL_PRESENCE_WORDS) {
        return false;
    }
    return (cell_presence_mask[word] & (1U << (physical_index % 32))) != 0;
}

int cell_map_logical_index(int physical_index) {
    if(!cell_map_is_present(physical_index)) {
        return -1;
    }

    // Count the present channels before this one
    int word_offset = physical_index / 32;
    int logical_index = 0;
    for(int word=0; word<word_offset; word++) {
        logical_index += __builtin_popcount(cell_presence_mask[word]);
    }
    logical_index += __builtin_popcount(cell_presence_mask[word_offset] & ((1U << (physical_index % 32)) - 1));
    return logical_index;
}

// FNV-1a
static uint32_t hash_u32(uint32_t hash, uint32_t value) {
    for(int i=0; i<4; i++) {
        hash ^= (value >> (i * 8)) & 0xFF;
        hash *= 16777619u;
    }
    return hash;
}

uint32_t cell_map_hash(void) {
    uint32_t hash = 2166136261u;
    hash = hash_u32(hash, BMS_PROFILE);
    hash = hash_u32(hash, NUM_CELLS);
    hash = hash_u32(hash, NUM_MODULE_VOLTAGES);
    for(int word=0; word<CELL_PRESENCE_WORDS; word++) {
        hash = hash_u32(hash, cell_presence_mask[word]);
    }
    hash = hash_u32(hash, CELL_VOLTAGE_HARD_MIN_mV);
    hash = hash_u32(hash, CELL_VOLTAGE_HARD_MAX_mV);
    return hash;
}
