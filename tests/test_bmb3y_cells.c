#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drivers/bmb3y/bmb3y_cells.h"

// Synthetic tests of the BMB3Y channel indexing. Deliberately independent of
// any pack profile: every mask here is made up.

#define TEST_MODULES 8
#define NUM_BANKS 5
#define NUM_CHANNELS (TEST_MODULES * BMB3Y_CHANNELS_PER_MODULE)
#define MAX_PHYSICAL (BMB3Y_PRESENCE_MASK_WORDS * 32)
#define NUM_RANDOM_MASKS 1000

static const uint32_t edge_masks[][BMB3Y_PRESENCE_MASK_WORDS] = {
    { 0, 0, 0, 0 },
    { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF },
    { 0x80000000, 0x80000000, 0x80000000, 0x80000000 }, // bit 31 of each word
    { 0x00000001, 0x00000001, 0x00000001, 0x00000001 },
    { 0x80000001, 0, 0x00000001, 0x80000000 },          // empty word in between
    { 0, 0, 0, 0x80000000 },                            // last channel only
    { 0x55555555, 0xAAAAAAAA, 0x55555555, 0xAAAAAAAA },
    { 0x00007FFF, 0, 0, 0 },                            // one full module
};

// xorshift32: fixed seed so failures are reproducible
static uint32_t rng_state;

static uint32_t rng_next(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

// Random mask over the NUM_CHANNELS real channels, with a varying density
// so both sparse and nearly-full packs get exercised.
static void random_mask(uint32_t mask[BMB3Y_PRESENCE_MASK_WORDS]) {
    memset(mask, 0, BMB3Y_PRESENCE_MASK_WORDS * sizeof(uint32_t));
    uint32_t density = rng_next() % 101;
    for(int p = 0; p < NUM_CHANNELS; p++) {
        if(rng_next() % 100 < density) {
            mask[p / 32] |= 1U << (p % 32);
        }
    }
}

static int popcount_mask(const uint32_t mask[BMB3Y_PRESENCE_MASK_WORDS]) {
    int n = 0;
    for(int w = 0; w < BMB3Y_PRESENCE_MASK_WORDS; w++) {
        n += __builtin_popcount(mask[w]);
    }
    return n;
}

// Deliberately naive reference: walk every channel in order, numbering the
// present ones. Shares no arithmetic with bmb3y_logical_index().
static void reference_logical_indices(const uint32_t mask[BMB3Y_PRESENCE_MASK_WORDS], int out[MAX_PHYSICAL]) {
    int next = 0;
    for(int p = 0; p < MAX_PHYSICAL; p++) {
        bool present = (mask[p / 32] >> (p % 32)) & 1U;
        out[p] = present ? next++ : -1;
    }
}

static void check_mask_against_reference(const uint32_t mask[BMB3Y_PRESENCE_MASK_WORDS]) {
    int expected[MAX_PHYSICAL];
    reference_logical_indices(mask, expected);
    for(int p = 0; p < MAX_PHYSICAL; p++) {
        int got = bmb3y_logical_index(mask, p);
        if(got != expected[p]) {
            fail_msg("mask %08X %08X %08X %08X physical %d: got %d, expected %d",
                (unsigned)mask[0], (unsigned)mask[1], (unsigned)mask[2], (unsigned)mask[3],
                p, got, expected[p]);
        }
    }
}

// Read every bank of every module, as the driver does: each present cell on
// a real channel must come out exactly once, covering 0..n-1 with no gaps.
// Bits above the last real channel are never read, so aren't counted.
static void check_bank_reads_cover_every_cell_once(const uint32_t mask[BMB3Y_PRESENCE_MASK_WORDS]) {
    int num_cells = 0;
    for(int p = 0; p < NUM_CHANNELS; p++) {
        num_cells += (mask[p / 32] >> (p % 32)) & 1U;
    }
    int hits[NUM_CHANNELS];
    memset(hits, 0, sizeof(hits));

    for(int module = 0; module < TEST_MODULES; module++) {
        for(int bank = 0; bank < NUM_BANKS; bank++) {
            for(int cell = 0; cell < BMB3Y_CHANNELS_PER_BANK; cell++) {
                int logical = bmb3y_logical_index(mask, bmb3y_physical_index(module, bank, cell));
                if(logical < 0) {
                    continue;
                }
                assert_int_in_range(logical, 0, num_cells - 1);
                hits[logical]++;
            }
        }
    }
    for(int i = 0; i < num_cells; i++) {
        if(hits[i] != 1) {
            fail_msg("mask %08X %08X %08X %08X: logical cell %d read %d times",
                (unsigned)mask[0], (unsigned)mask[1], (unsigned)mask[2], (unsigned)mask[3],
                i, hits[i]);
        }
    }
}

// ============================================================================
// Physical index layout
// ============================================================================

// Walking modules, then banks, then cells must visit every channel exactly
// once, in order: 15 channels per module, 3 per bank.
static void test_physical_index_layout(void **state) {
    (void) state;
    int expected = 0;
    for(int module = 0; module < TEST_MODULES; module++) {
        for(int bank = 0; bank < NUM_BANKS; bank++) {
            for(int cell = 0; cell < BMB3Y_CHANNELS_PER_BANK; cell++) {
                int p = bmb3y_physical_index(module, bank, cell);
                assert_int_equal(p, expected);
                assert_int_equal(p / 15, module);
                assert_int_equal((p % 15) / 3, bank);
                expected++;
            }
        }
    }
    assert_int_equal(expected, NUM_CHANNELS);
    assert_true(NUM_CHANNELS <= MAX_PHYSICAL);
}

// The word/bit split must agree with plain bit numbering, including bit 31
// of each word (where a signed shift would be undefined).
static void test_presence_word_and_bit(void **state) {
    (void) state;
    for(int p = 0; p < MAX_PHYSICAL; p++) {
        uint32_t mask[BMB3Y_PRESENCE_MASK_WORDS] = {0};
        mask[bmb3y_presence_word(p)] |= bmb3y_presence_bit(p);
        assert_int_equal(popcount_mask(mask), 1);
        assert_true((mask[p >> 5] >> (p & 31)) & 1U);
    }
    assert_int_equal(bmb3y_presence_bit(31), 0x80000000u);
    assert_int_equal(bmb3y_presence_word(31), 0);
    assert_int_equal(bmb3y_presence_word(32), 1);
}

// ============================================================================
// Physical -> logical mapping
// ============================================================================

static void test_logical_index_spot_checks(void **state) {
    (void) state;
    // Hand-worked values, independent of the reference
    const uint32_t all[BMB3Y_PRESENCE_MASK_WORDS] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
    assert_int_equal(bmb3y_logical_index(all, 0), 0);
    assert_int_equal(bmb3y_logical_index(all, 31), 31);
    assert_int_equal(bmb3y_logical_index(all, 32), 32);
    assert_int_equal(bmb3y_logical_index(all, 127), 127);

    const uint32_t gappy[BMB3Y_PRESENCE_MASK_WORDS] = { 0x80000001, 0, 0x00000001, 0 };
    assert_int_equal(bmb3y_logical_index(gappy, 0), 0);
    assert_int_equal(bmb3y_logical_index(gappy, 1), -1);
    assert_int_equal(bmb3y_logical_index(gappy, 31), 1);
    assert_int_equal(bmb3y_logical_index(gappy, 32), -1);
    assert_int_equal(bmb3y_logical_index(gappy, 64), 2);

    // Module 1 missing its first channel: its second channel follows
    // straight on from module 0's last
    const uint32_t skip[BMB3Y_PRESENCE_MASK_WORDS] = { 0x3FFF7FFF, 0, 0, 0 };
    assert_int_equal(bmb3y_logical_index(skip, bmb3y_physical_index(0, 4, 2)), 14);
    assert_int_equal(bmb3y_logical_index(skip, bmb3y_physical_index(1, 0, 0)), -1);
    assert_int_equal(bmb3y_logical_index(skip, bmb3y_physical_index(1, 0, 1)), 15);
}

static void test_logical_index_edge_masks(void **state) {
    (void) state;
    for(size_t i = 0; i < sizeof(edge_masks) / sizeof(edge_masks[0]); i++) {
        check_mask_against_reference(edge_masks[i]);
        check_bank_reads_cover_every_cell_once(edge_masks[i]);
    }
}

static void test_logical_index_random_masks(void **state) {
    (void) state;
    rng_state = 0x2545F491;
    for(int i = 0; i < NUM_RANDOM_MASKS; i++) {
        uint32_t mask[BMB3Y_PRESENCE_MASK_WORDS];
        random_mask(mask);
        check_mask_against_reference(mask);
        check_bank_reads_cover_every_cell_once(mask);
    }
}

// ============================================================================
// Balancing payload
// ============================================================================

#define GUARD 0xA5

// Records buffer with guard bytes either side, to catch out-of-bounds writes
typedef struct {
    uint8_t before[8];
    uint8_t records[BMB3Y_BALANCE_RECORDS_LEN];
    uint8_t after[8];
} records_buf_t;

static void records_init(records_buf_t *buf) {
    memset(buf->before, GUARD, sizeof(buf->before));
    memset(buf->records, 0, sizeof(buf->records));
    memset(buf->after, GUARD, sizeof(buf->after));
}

static void assert_guards_intact(const records_buf_t *buf) {
    for(size_t i = 0; i < sizeof(buf->before); i++) {
        assert_int_equal(buf->before[i], GUARD);
        assert_int_equal(buf->after[i], GUARD);
    }
}

// Decode the flags back out of the payload, written directly from the
// documented layout: record r holds module (7 - r), flags little-endian in
// bytes 2-3. Returns a presence-style mask of the flagged physical channels.
static void decode_balance_flags(const uint8_t records[BMB3Y_BALANCE_RECORDS_LEN], uint32_t out[BMB3Y_PRESENCE_MASK_WORDS]) {
    memset(out, 0, BMB3Y_PRESENCE_MASK_WORDS * sizeof(uint32_t));
    for(int r = 0; r < TEST_MODULES; r++) {
        const uint8_t *rec = &records[r * 6];
        // Only the flag bytes may ever be written
        assert_int_equal(rec[0], 0);
        assert_int_equal(rec[1], 0);
        assert_int_equal(rec[4], 0);
        assert_int_equal(rec[5], 0);
        uint16_t flags = (uint16_t)rec[2] | ((uint16_t)rec[3] << 8);
        assert_int_equal(flags & 0x8000, 0); // channel 15 doesn't exist
        int module = 7 - r;
        for(int ch = 0; ch < 15; ch++) {
            if(flags & (1U << ch)) {
                int p = module * 15 + ch;
                out[p / 32] |= 1U << (p % 32);
            }
        }
    }
}

// Pack, decode, and check that the flagged channels are exactly the present
// channels whose logical index (per the read side) was requested.
static void check_balance_round_trip(const uint32_t presence[BMB3Y_PRESENCE_MASK_WORDS], const uint32_t request[BMB3Y_PRESENCE_MASK_WORDS]) {
    records_buf_t buf;
    records_init(&buf);
    bmb3y_pack_balance_requests(presence, request, buf.records);
    assert_guards_intact(&buf);

    uint32_t flagged[BMB3Y_PRESENCE_MASK_WORDS];
    decode_balance_flags(buf.records, flagged);

    for(int p = 0; p < MAX_PHYSICAL; p++) {
        bool is_flagged = (flagged[p / 32] >> (p % 32)) & 1U;
        int logical = bmb3y_logical_index(presence, p);
        bool want = p < NUM_CHANNELS && logical >= 0 &&
            ((request[logical / 32] >> (logical % 32)) & 1U);
        if(is_flagged != want) {
            fail_msg("presence %08X %08X %08X %08X request %08X %08X %08X %08X: physical %d (logical %d) flagged %d, expected %d",
                (unsigned)presence[0], (unsigned)presence[1], (unsigned)presence[2], (unsigned)presence[3],
                (unsigned)request[0], (unsigned)request[1], (unsigned)request[2], (unsigned)request[3],
                p, logical, is_flagged, want);
        }
    }
}

// Each channel's flag lands in exactly one, hand-derivable, place
static void test_balance_flag_layout(void **state) {
    (void) state;
    for(int p = 0; p < NUM_CHANNELS; p++) {
        records_buf_t buf;
        records_init(&buf);
        bmb3y_set_balance_flag(buf.records, p);
        assert_guards_intact(&buf);

        int set_bits = 0;
        for(int i = 0; i < BMB3Y_BALANCE_RECORDS_LEN; i++) {
            set_bits += __builtin_popcount(buf.records[i]);
        }
        assert_int_equal(set_bits, 1);

        int record = 7 - (p / 15);
        uint16_t flags = (uint16_t)buf.records[record * 6 + 2] | ((uint16_t)buf.records[record * 6 + 3] << 8);
        assert_int_equal(flags, 1U << (p % 15));
    }

    // Hand-worked: first and last channel of the chain, and the byte boundary
    records_buf_t buf;
    records_init(&buf);
    bmb3y_set_balance_flag(buf.records, 0);   // module 0 ch 0 -> record 7 byte 2 bit 0
    bmb3y_set_balance_flag(buf.records, 119); // module 7 ch 14 -> record 0 byte 3 bit 6
    bmb3y_set_balance_flag(buf.records, 23);  // module 1 ch 8 -> record 6 byte 3 bit 0
    assert_int_equal(buf.records[7 * 6 + 2], 0x01);
    assert_int_equal(buf.records[0 * 6 + 3], 0x40);
    assert_int_equal(buf.records[6 * 6 + 3], 0x01);
}

static void test_balance_requests_hand_worked(void **state) {
    (void) state;
    // Module 0 missing channel 1: logical 1 is physical 2
    const uint32_t presence[BMB3Y_PRESENCE_MASK_WORDS] = { 0x7FFD, 0, 0, 0 };
    const uint32_t request[BMB3Y_PRESENCE_MASK_WORDS] = { 0x2, 0, 0, 0 };
    records_buf_t buf;
    records_init(&buf);
    bmb3y_pack_balance_requests(presence, request, buf.records);
    assert_guards_intact(&buf);
    assert_int_equal(buf.records[7 * 6 + 2], 0x04);
    for(int i = 0; i < BMB3Y_BALANCE_RECORDS_LEN; i++) {
        if(i != 7 * 6 + 2) {
            assert_int_equal(buf.records[i], 0);
        }
    }
}

static void test_balance_requests_edge_masks(void **state) {
    (void) state;
    const uint32_t none[BMB3Y_PRESENCE_MASK_WORDS] = { 0, 0, 0, 0 };
    const uint32_t all[BMB3Y_PRESENCE_MASK_WORDS] = { 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF };
    for(size_t i = 0; i < sizeof(edge_masks) / sizeof(edge_masks[0]); i++) {
        check_balance_round_trip(edge_masks[i], none);
        check_balance_round_trip(edge_masks[i], all);
        // Every single cell, including requests beyond the last cell
        for(int l = 0; l < MAX_PHYSICAL; l++) {
            uint32_t request[BMB3Y_PRESENCE_MASK_WORDS] = {0};
            request[l / 32] |= 1U << (l % 32);
            check_balance_round_trip(edge_masks[i], request);
        }
    }
}

static void test_balance_requests_random(void **state) {
    (void) state;
    rng_state = 0x1B873593;
    for(int i = 0; i < NUM_RANDOM_MASKS; i++) {
        uint32_t presence[BMB3Y_PRESENCE_MASK_WORDS];
        random_mask(presence);
        for(int j = 0; j < 20; j++) {
            uint32_t request[BMB3Y_PRESENCE_MASK_WORDS];
            for(int w = 0; w < BMB3Y_PRESENCE_MASK_WORDS; w++) {
                request[w] = rng_next();
            }
            check_balance_round_trip(presence, request);
        }
    }
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_physical_index_layout),
        cmocka_unit_test(test_presence_word_and_bit),
        cmocka_unit_test(test_logical_index_spot_checks),
        cmocka_unit_test(test_logical_index_edge_masks),
        cmocka_unit_test(test_logical_index_random_masks),
        cmocka_unit_test(test_balance_flag_layout),
        cmocka_unit_test(test_balance_requests_hand_worked),
        cmocka_unit_test(test_balance_requests_edge_masks),
        cmocka_unit_test(test_balance_requests_random),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
