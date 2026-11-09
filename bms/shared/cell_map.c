#include "cell_map.h"

bool cell_map_is_present(const cell_map_t *map, int physical_index) {
    if(physical_index < 0 || physical_index >= CELL_MAP_CHANNELS) {
        return false;
    }
    return (map->presence[physical_index / 32] & (1U << (physical_index % 32))) != 0;
}

uint8_t cell_map_check(const cell_map_t *map) {
    uint8_t errors = 0;

    if(map->chemistry != LFP && map->chemistry != NMC) {
        errors |= CELL_MAP_ERR_CHEMISTRY;
    }
    if(map->num_modules < 1 || map->num_modules > CELL_MAP_MODULES) {
        errors |= CELL_MAP_ERR_MODULES;
    }

    // Walk every bit of the words, including the ones past the last channel
    int count = 0;
    for(int i=0; i<CELL_MAP_MASK_WORDS * 32; i++) {
        if((map->presence[i / 32] & (1U << (i % 32))) == 0) {
            continue;
        }
        count++;
        if(i / CELL_MAP_CELLS_PER_MODULE >= map->num_modules) {
            errors |= CELL_MAP_ERR_BEYOND;
        }
    }
    if(count == 0 || count != map->num_cells) {
        errors |= CELL_MAP_ERR_COUNT;
    }

    int modules = map->num_modules <= CELL_MAP_MODULES ? map->num_modules : CELL_MAP_MODULES;
    for(int module=0; module<modules; module++) {
        bool any = false;
        for(int cell=0; cell<CELL_MAP_CELLS_PER_MODULE; cell++) {
            if(cell_map_is_present(map, module * CELL_MAP_CELLS_PER_MODULE + cell)) {
                any = true;
                break;
            }
        }
        if(!any) {
            errors |= CELL_MAP_ERR_EMPTY_MODULE;
        }
    }

    return errors;
}

bool cell_map_equal(const cell_map_t *a, const cell_map_t *b) {
    if(a->chemistry != b->chemistry || a->num_cells != b->num_cells || a->num_modules != b->num_modules) {
        return false;
    }
    for(int word=0; word<CELL_MAP_MASK_WORDS; word++) {
        if(a->presence[word] != b->presence[word]) {
            return false;
        }
    }
    return true;
}

// FNV-1a
static uint32_t hash_u32(uint32_t hash, uint32_t value) {
    for(int i=0; i<4; i++) {
        hash ^= (value >> (i * 8)) & 0xFF;
        hash *= 16777619u;
    }
    return hash;
}

uint32_t cell_map_hash(const cell_map_t *map) {
    uint32_t hash = 2166136261u;
    hash = hash_u32(hash, map->chemistry);
    hash = hash_u32(hash, map->num_cells);
    hash = hash_u32(hash, map->num_modules);
    for(int word=0; word<CELL_MAP_MASK_WORDS; word++) {
        hash = hash_u32(hash, map->presence[word]);
    }
    return hash;
}
