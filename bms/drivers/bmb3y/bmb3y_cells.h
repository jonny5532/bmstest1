#pragma once

#include "bmb3y.h"

#include <stdint.h>

// Cell channel indexing for the BMB3Y.
//
// Physical index: the channel number across the BMB chain, 15 per module
// (module 0 is channels 0-14). Each voltage bank carries 3 channels per
// module (bank A = channels 0-2, ... bank E = 12-14).
//
// Logical index: the cell number with unpopulated channels removed, i.e. 0 for
// the first present cell, 1 for the second, etc.
//
// Presence masks are four 32-bit words, LSB of word 0 = physical channel 0.

#define BMB3Y_CHANNELS_PER_MODULE 15
#define BMB3Y_CHANNELS_PER_BANK 3
#define BMB3Y_PRESENCE_MASK_WORDS 4

static inline int bmb3y_physical_index(int module, int bank, int cell) {
    return (module * BMB3Y_CHANNELS_PER_MODULE) + (bank * BMB3Y_CHANNELS_PER_BANK) + cell;
}

static inline int bmb3y_presence_word(int physical_index) {
    return physical_index / 32;
}

static inline uint32_t bmb3y_presence_bit(int physical_index) {
    return 1U << (physical_index % 32);
}

// Logical index of a physical channel, or -1 if the channel is not present.
static inline int bmb3y_logical_index(const uint32_t presence_mask[BMB3Y_PRESENCE_MASK_WORDS], int physical_index) {
    int word_offset = bmb3y_presence_word(physical_index);
    uint32_t bit = bmb3y_presence_bit(physical_index);

    if((presence_mask[word_offset] & bit) == 0) {
        return -1;
    }

    int logical_index = 0;
    // Count all the bits in previous presence mask words
    for(int word=0; word<word_offset; word++) {
        logical_index += __builtin_popcount(presence_mask[word]);
    }
    // Count the bits in this word below the physical index
    logical_index += __builtin_popcount(presence_mask[word_offset] & (bit - 1));
    return logical_index;
}

// Update a single module's balancing request payload, enabling balancing for
// the cell at the requested physical index. The records run in reverse chain
// order, with the first being module 7.
#define BMB3Y_BALANCE_RECORD_LEN 6
#define BMB3Y_BALANCE_RECORD_FLAGS_OFFSET 2
#define BMB3Y_BALANCE_RECORDS_LEN (BMB3Y_MAX_MODULES * BMB3Y_BALANCE_RECORD_LEN)

static inline void bmb3y_set_balance_flag(uint8_t records[BMB3Y_BALANCE_RECORDS_LEN], int physical_index) {
    int module = physical_index / BMB3Y_CHANNELS_PER_MODULE;
    int channel = physical_index % BMB3Y_CHANNELS_PER_MODULE;
    int record = (BMB3Y_MAX_MODULES - 1) - module;
    records[record * BMB3Y_BALANCE_RECORD_LEN + BMB3Y_BALANCE_RECORD_FLAGS_OFFSET + (channel / 8)] |=
        (uint8_t)(1U << (channel % 8));
}

// Sets the balance flag of each requested cell. The request mask is by logical
// index, which this maps to physical index and splits across the per-module records.
static inline void bmb3y_pack_balance_requests(const uint32_t presence_mask[BMB3Y_PRESENCE_MASK_WORDS],
        const uint32_t request_mask[BMB3Y_PRESENCE_MASK_WORDS], uint8_t records[BMB3Y_BALANCE_RECORDS_LEN]) {
    int logical_index = 0;
    for(int physical_index = 0; physical_index < BMB3Y_MAX_MODULES * BMB3Y_CHANNELS_PER_MODULE; physical_index++) {
        if((presence_mask[bmb3y_presence_word(physical_index)] & bmb3y_presence_bit(physical_index)) == 0) {
            continue;
        }
        if(request_mask[bmb3y_presence_word(logical_index)] & bmb3y_presence_bit(logical_index)) {
            bmb3y_set_balance_flag(records, physical_index);
        }
        logical_index++;
    }
}
