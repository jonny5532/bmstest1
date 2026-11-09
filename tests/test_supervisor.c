// Supervisor firmware logic (supervisor/), the BMS <-> supervisor protocol and
// the cell map (bms/shared/). Built once per chemistry (test_supervisor_lfp,
// test_supervisor_nmc): the supervisor gets the cell map from the BMS, so
// nothing here depends on the pack profile.

#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "supervisor_config.h"
#include "drivers/bmb3y/crc.h"
#include "shared/cell_map.h"
#include "shared/supervisor_protocol.h"
#include "cell_snoop.h"
#include "supervisor_logic.h"
#include "supervisor_hmi.h"
#include "protocols/hmi_serial/hmi_serial.h"

#define STATUS_PERIOD_MS 100
#define SCAN_PERIOD_MS 1280
// A cell voltage in range for the chemistry
#define NOMINAL_mV ((CELL_VOLTAGE_HARD_MIN_mV + CELL_VOLTAGE_HARD_MAX_mV) / 2)
#define OTHER_CHEMISTRY (SUPERVISOR_CHEMISTRY == LFP ? NMC : LFP)

/* Test maps */

// 96 cells on 8 BMBs (the bluetesla profile's)
static cell_map_t pack_map(void) {
    cell_map_t map = {
        .chemistry = SUPERVISOR_CHEMISTRY,
        .num_cells = 96,
        .num_modules = 8,
        .presence = { 0xc7ff8f7f, 0xf3ffe3ff, 0xfcfff8ff, 0x1ffe3d },
    };
    return map;
}

// 15 cells on one BMB (the desk profile's)
static cell_map_t desk_map(void) {
    cell_map_t map = {
        .chemistry = SUPERVISOR_CHEMISTRY,
        .num_cells = 15,
        .num_modules = 1,
        .presence = { 0x7FFF, 0, 0, 0 },
    };
    return map;
}

// Two 108-cell packs of 8 BMBs each (the lfp1 profile's) in parallel, with
// the isoSPI daisy-chained: 216 cells on all 16 BMBs
static cell_map_t double_pack_map(void) {
    const uint32_t one[] = { 0x87ffbfff, 0xffffefff, 0xf9fffbff, 0x3ffeff };
    cell_map_t map = {
        .chemistry = SUPERVISOR_CHEMISTRY,
        .num_cells = 216,
        .num_modules = 16,
    };
    for(int pack=0; pack<2; pack++) {
        for(int i=0; i<8 * CELL_MAP_CELLS_PER_MODULE; i++) {
            if(one[i / 32] & (1U << (i % 32))) {
                int channel = pack * 8 * CELL_MAP_CELLS_PER_MODULE + i;
                map.presence[channel / 32] |= 1U << (channel % 32);
            }
        }
    }
    return map;
}

/* Helpers */

static int first_absent_channel(const cell_map_t *map) {
    for(int i=0; i<map->num_modules * CELL_MAP_CELLS_PER_MODULE; i++) {
        if(!cell_map_is_present(map, i)) {
            return i;
        }
    }
    return -1;
}

static int first_present_channel(const cell_map_t *map) {
    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        if(cell_map_is_present(map, i)) {
            return i;
        }
    }
    return -1;
}

static uint16_t modules_mask(int modules) {
    return (uint16_t)((1U << modules) - 1);
}

// Present cells at mV, unpopulated channels reading zero (1mV), modules off
// the chain not read
static void make_scan(const cell_map_t *map, int16_t *scan, int16_t mV, int chain_modules) {
    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        if(i / CELL_MAP_CELLS_PER_MODULE >= chain_modules) {
            scan[i] = CELL_MV_NOT_READ;
        } else {
            scan[i] = cell_map_is_present(map, i) ? mV : 1;
        }
    }
}

static sup_bms_status_t good_status(const cell_map_t *map, uint64_t uptime_ms, int16_t mV) {
    sup_bms_status_t status = {
        .version = SUP_PROTOCOL_VERSION,
        .uptime_ms = uptime_ms,
        .flags = SUP_BMS_FLAG_CELLS_VALID,
        .cell_min_mV = mV,
        .cell_max_mV = mV,
        .map = *map,
        .hmi_address = 1,
    };
    return status;
}

// Simulates a BMS sending status every 100ms and the BMB cycle every 1.28s
typedef struct {
    sup_logic_t logic;
    uint32_t now;
    cell_map_t map;          // what the BMS sends
    uint64_t bms_uptime;
    bool bms_running;
    bool scanning;
    int16_t scan[CELL_MAP_CHANNELS];
    uint16_t module_ok;      // modules that read cleanly in the scan
    int16_t bms_mV;
    uint8_t extra_flags;     // added to the BMS status flags
    uint8_t version;
    bool enabled;
} sim_t;

static void sim_init_map(sim_t *sim, cell_map_t map) {
    memset(sim, 0, sizeof(*sim));
    sup_logic_init(&sim->logic);
    sim->now = 1000;
    sim->map = map;
    sim->bms_uptime = 5000;
    sim->bms_running = true;
    sim->scanning = true;
    sim->bms_mV = NOMINAL_mV;
    sim->version = SUP_PROTOCOL_VERSION;
    make_scan(&map, sim->scan, NOMINAL_mV, map.num_modules);
    sim->module_ok = modules_mask(map.num_modules);
}

static void sim_init(sim_t *sim) {
    sim_init_map(sim, pack_map());
}

static void sim_run(sim_t *sim, uint32_t ms) {
    for(uint32_t t=0; t<ms; t+=STATUS_PERIOD_MS) {
        sim->now += STATUS_PERIOD_MS;
        if(sim->bms_running) {
            sim->bms_uptime += STATUS_PERIOD_MS;
            sup_bms_status_t status = good_status(&sim->map, sim->bms_uptime, sim->bms_mV);
            status.flags |= sim->extra_flags;
            status.version = sim->version;
            sup_logic_on_status(&sim->logic, &status, sim->now);
        }
        if(sim->scanning && (sim->now % SCAN_PERIOD_MS) < STATUS_PERIOD_MS) {
            sup_logic_on_scan(&sim->logic, sim->scan, sim->module_ok, sim->now);
        }
        sim->enabled = sup_logic_tick(&sim->logic, sim->now);
    }
}

static void sim_enable(sim_t *sim) {
    sim_run(sim, (SUPERVISOR_GOOD_SCANS_TO_ENABLE + 1) * SCAN_PERIOD_MS);
    assert_true(sim->enabled);
    assert_int_equal(sim->logic.state, SUP_STATE_ON);
}

// Until the next scan shows the problem just set up (failing, not hanging, if
// it never does)
static void run_until_scan_fault(sim_t *sim) {
    for(int i=0; i<2 * SCAN_PERIOD_MS / STATUS_PERIOD_MS && sim->logic.scan_reasons == 0; i++) {
        sim_run(sim, STATUS_PERIOD_MS);
    }
    assert_int_not_equal(sim->logic.scan_reasons, 0);
}

static void sim_restart_bms(sim_t *sim) {
    sim->bms_uptime = 100;
}

/* Protocol */

static void test_frame_matches_duart_keepalive(void **state) {
    (void) state;
    // The old BMS keepalive, as framed by duart_send_packet
    const uint8_t payload[] = { 'K', 'e', 'e', 'p', 'A', 'l', 'i', 'v' };
    const uint8_t expected[] = { 0xff, 0x07, 'K', 'e', 'e', 'p', 'A', 'l', 'i', 'v', 0x20, 0xe8 };
    uint8_t frame[16];
    assert_int_equal(sup_frame_encode(payload, sizeof(payload), frame, sizeof(frame)), sizeof(expected));
    assert_memory_equal(frame, expected, sizeof(expected));
}

static void test_status_round_trip(void **state) {
    (void) state;
    sup_bms_status_t status = good_status(&(cell_map_t){0}, 0x123456789AULL, 0);
    status.map = pack_map();
    status.flags = SUP_BMS_FLAG_CELLS_VALID | SUP_BMS_FLAG_BALANCING;
    status.cell_min_mV = -1;
    status.cell_max_mV = 4123;
    status.hmi_address = 0x42;

    uint8_t buf[SUP_BMS_STATUS_LEN];
    assert_int_equal(sup_pack_bms_status(&status, buf, sizeof(buf)), SUP_BMS_STATUS_LEN);
    assert_int_equal(SUP_BMS_STATUS_LEN, 49);
    // The mask, little-endian from channel 0
    assert_int_equal(buf[18], 0x7f);
    assert_int_equal(buf[21], 0xc7);
    assert_int_equal(buf[32], 0x1f);

    sup_bms_status_t out;
    assert_true(sup_unpack_bms_status(buf, sizeof(buf), &out));
    assert_int_equal(out.version, SUP_PROTOCOL_VERSION);
    assert_true(out.uptime_ms == status.uptime_ms);
    assert_int_equal(out.flags, status.flags);
    assert_int_equal(out.cell_min_mV, -1);
    assert_int_equal(out.cell_max_mV, 4123);
    assert_true(cell_map_equal(&out.map, &status.map));
    // Mask bytes past the pack's channels are zero, up to the 30th
    for(int i=33; i<48; i++) {
        assert_int_equal(buf[i], 0);
    }
    assert_int_equal(buf[48], 0x42);
    assert_int_equal(out.hmi_address, 0x42);

    // A reply isn't a status
    buf[0] = SUP_MSG_SUPERVISOR_REPLY;
    assert_false(sup_unpack_bms_status(buf, sizeof(buf), &out));
    // Nor is a status of the wrong length
    buf[0] = SUP_MSG_BMS_STATUS;
    assert_false(sup_unpack_bms_status(buf, sizeof(buf) - 1, &out));
}

static void test_status_drops_bits_past_last_channel(void **state) {
    (void) state;
    sup_bms_status_t status = good_status(&(cell_map_t){0}, 1000, NOMINAL_mV);
    status.map = desk_map();
    status.map.presence[7] = 0x00010000; // channel 240: doesn't exist
    assert_int_not_equal(cell_map_check(&status.map), 0);

    uint8_t buf[SUP_BMS_STATUS_LEN];
    sup_pack_bms_status(&status, buf, sizeof(buf));
    sup_bms_status_t out;
    assert_true(sup_unpack_bms_status(buf, sizeof(buf), &out));
    assert_int_equal(out.map.presence[7], 0);
}

static void test_reply_round_trip(void **state) {
    (void) state;
    sup_reply_t reply = {
        .version = SUP_PROTOCOL_VERSION,
        .state = SUP_STATE_LATCHED,
        .reasons = SUP_REASON_SETTLING | SUP_REASON_NO_SCAN,
        .latch_reasons = SUP_REASON_MAP_CHANGED,
        .map_hash = 0x01020304,
        .cell_min_mV = 2999,
        .cell_max_mV = 4301,
        .cells_ok = 95,
        .scans = 123456,
        .chemistry = NMC,
    };
    uint8_t buf[SUP_REPLY_LEN];
    assert_int_equal(sup_pack_reply(&reply, buf, sizeof(buf)), SUP_REPLY_LEN);

    sup_reply_t out;
    assert_true(sup_unpack_reply(buf, sizeof(buf), &out));
    assert_int_equal(out.version, reply.version);
    assert_int_equal(out.state, reply.state);
    assert_int_equal(out.reasons, reply.reasons);
    assert_int_equal(out.latch_reasons, reply.latch_reasons);
    assert_int_equal(out.map_hash, reply.map_hash);
    assert_int_equal(out.cell_min_mV, reply.cell_min_mV);
    assert_int_equal(out.cell_max_mV, reply.cell_max_mV);
    assert_int_equal(out.cells_ok, reply.cells_ok);
    assert_int_equal(out.scans, reply.scans);
    assert_int_equal(out.chemistry, reply.chemistry);
}

static void test_other_version_is_reported(void **state) {
    (void) state;
    // eg. a v1 status (19 bytes)
    uint8_t payload[19] = { SUP_MSG_BMS_STATUS, 1 };
    sup_bms_status_t out;
    assert_true(sup_unpack_bms_status(payload, sizeof(payload), &out));
    assert_int_equal(out.version, 1);

    sup_reply_t reply;
    payload[0] = SUP_MSG_SUPERVISOR_REPLY;
    assert_true(sup_unpack_reply(payload, sizeof(payload), &reply));
    assert_int_equal(reply.version, 1);
}

static size_t feed_all(sup_frame_parser_t *parser, const uint8_t *data, size_t len, int *frames) {
    size_t last_len = 0;
    for(size_t i=0; i<len; i++) {
        const uint8_t *payload;
        size_t n = sup_frame_feed(parser, data[i], &payload);
        if(n) {
            (*frames)++;
            last_len = n;
        }
    }
    return last_len;
}

static void test_parser_resyncs(void **state) {
    (void) state;
    sup_frame_parser_t parser;
    sup_frame_parser_init(&parser, SUP_BMS_STATUS_LEN);

    cell_map_t map = pack_map();
    sup_bms_status_t status = good_status(&map, 1234, NOMINAL_mV);
    uint8_t payload[SUP_BMS_STATUS_LEN];
    sup_pack_bms_status(&status, payload, sizeof(payload));
    uint8_t frame[SUP_MAX_FRAME_LEN];
    size_t frame_len = sup_frame_encode(payload, sizeof(payload), frame, sizeof(frame));
    assert_int_equal(frame_len, SUP_MAX_FRAME_LEN);

    int frames = 0;

    // Line noise, including false syncs with impossible lengths
    const uint8_t noise[] = { 0x12, 0xFF, 0xFF, 0xFF, 0x80, 0x00, 0xFF };
    feed_all(&parser, noise, sizeof(noise), &frames);
    assert_int_equal(frames, 0);
    assert_int_equal(feed_all(&parser, frame, frame_len, &frames), SUP_BMS_STATUS_LEN);
    assert_int_equal(frames, 1);

    // A corrupted frame is dropped and the next one still arrives
    uint8_t bad[SUP_MAX_FRAME_LEN];
    memcpy(bad, frame, frame_len);
    bad[20] ^= 0x01; // in the mask
    feed_all(&parser, bad, frame_len, &frames);
    assert_int_equal(frames, 1);
    assert_int_equal(parser.crc_errors, 1);
    feed_all(&parser, frame, frame_len, &frames);
    assert_int_equal(frames, 2);
}

/* Cell map */

static void test_cell_map_check_accepts_good_maps(void **state) {
    (void) state;
    cell_map_t map = pack_map();
    assert_int_equal(cell_map_check(&map), 0);
    map = desk_map();
    assert_int_equal(cell_map_check(&map), 0);
    map.chemistry = OTHER_CHEMISTRY;
    assert_int_equal(cell_map_check(&map), 0);
}

static void test_cell_map_check_rejects_bad_maps(void **state) {
    (void) state;
    cell_map_t map;

    map = pack_map(); map.chemistry = 0;
    assert_int_equal(cell_map_check(&map), CELL_MAP_ERR_CHEMISTRY);
    map = pack_map(); map.chemistry = 3;
    assert_int_equal(cell_map_check(&map), CELL_MAP_ERR_CHEMISTRY);

    map = desk_map(); map.num_modules = 0;
    assert_true(cell_map_check(&map) & CELL_MAP_ERR_MODULES);
    map = pack_map(); map.num_modules = CELL_MAP_MODULES + 1;
    assert_true(cell_map_check(&map) & CELL_MAP_ERR_MODULES);

    // The count must match the mask, and can't be zero
    map = pack_map(); map.num_cells = 95;
    assert_int_equal(cell_map_check(&map), CELL_MAP_ERR_COUNT);
    map = desk_map(); map.presence[0] &= ~1U;
    assert_int_equal(cell_map_check(&map), CELL_MAP_ERR_COUNT);
    map = desk_map(); map.presence[0] = 0; map.num_cells = 0;
    assert_true(cell_map_check(&map) & CELL_MAP_ERR_COUNT);

    // A cell on a module the BMS doesn't read
    map = desk_map(); map.presence[0] |= 1U << 15; map.num_cells = 16;
    assert_int_equal(cell_map_check(&map), CELL_MAP_ERR_BEYOND);
    // Or past the last channel
    map = pack_map(); map.presence[7] |= 1U << (CELL_MAP_CHANNELS % 32); map.num_cells = 97;
    assert_int_equal(cell_map_check(&map), CELL_MAP_ERR_BEYOND);

    // A module with no cells: the last BMB's cells dropped from the mask
    // would otherwise only need num_modules lowered too
    map = pack_map();
    for(int i=7*15; i<8*15; i++) {
        if(cell_map_is_present(&map, i)) {
            map.presence[i / 32] &= ~(1U << (i % 32));
            map.num_cells--;
        }
    }
    assert_int_equal(cell_map_check(&map), CELL_MAP_ERR_EMPTY_MODULE);
}

static void test_cell_map_hash_covers_every_field(void **state) {
    (void) state;
    cell_map_t a = pack_map();
    cell_map_t b = a;
    assert_int_equal(cell_map_hash(&a), cell_map_hash(&b));
    assert_true(cell_map_equal(&a, &b));

    b.chemistry = OTHER_CHEMISTRY;
    assert_int_not_equal(cell_map_hash(&a), cell_map_hash(&b));
    assert_false(cell_map_equal(&a, &b));
    b = a; b.num_cells++;
    assert_int_not_equal(cell_map_hash(&a), cell_map_hash(&b));
    assert_false(cell_map_equal(&a, &b));
    b = a; b.num_modules--;
    assert_int_not_equal(cell_map_hash(&a), cell_map_hash(&b));
    assert_false(cell_map_equal(&a, &b));
    for(int word=0; word<CELL_MAP_MASK_WORDS; word++) {
        b = a; b.presence[word] ^= 0x100;
        assert_int_not_equal(cell_map_hash(&a), cell_map_hash(&b));
        assert_false(cell_map_equal(&a, &b));
    }
}

static void test_cell_raw_conversion(void **state) {
    (void) state;
    assert_int_equal(cell_raw_to_mV(0xFFFF), -1);
    assert_int_equal(cell_raw_to_mV(0), 1);
    assert_int_equal(cell_raw_to_mV(46250), 3700);
}

/* Snooping */

typedef struct {
    cell_snoop_t *snoop;
    uint8_t byte;
    bool have_high;
} nibble_writer_t;

static void put_symbol(nibble_writer_t *w, uint8_t symbol) {
    if(!w->have_high) {
        w->byte = (uint8_t)(symbol << 4);
        w->have_high = true;
    } else {
        cell_snoop_feed(w->snoop, w->byte | symbol);
        w->have_high = false;
    }
}

static void flush_symbols(nibble_writer_t *w) {
    if(w->have_high) {
        // pad with idle
        put_symbol(w, 0x0);
    }
}

// One isoSPI transfer: CS, then request and reply bits interleaved, then CS
static void put_transfer(nibble_writer_t *w, uint32_t command, const uint8_t *reply, int reply_len) {
    put_symbol(w, 0xA);
    for(int i=0; i<reply_len*8; i++) {
        uint8_t request_bit = (i < 32) ? ((command >> (31 - i)) & 1) : 0;
        uint8_t reply_bit = (reply[i / 8] >> (7 - (i % 8))) & 1;
        put_symbol(w, request_bit ? 0x9 : 0x6);
        put_symbol(w, reply_bit ? 0x9 : 0x6);
    }
    put_symbol(w, 0xA);
    put_symbol(w, 0x0);
    flush_symbols(w);
}

static const uint32_t BANK_COMMANDS[] = { 0x47007000, 0x48003400, 0x4900DD00, 0x4A00C900, 0x4B002000 };

// Distinct within each 8-BMB pack, and in range for both chemistries
static int16_t test_cell_mV(int physical_index) {
    return (int16_t)(3000 + (physical_index % 120) * 4);
}

// Reply to a bank read: padding byte, then per module three cells + CRC14 +
// spare. Modules from chain_modules on aren't there (all ones, bad CRC).
static int make_bank_reply(const cell_map_t *map, uint8_t *reply, int bank, int corrupt_module, int chain_modules) {
    int len = 0;
    reply[len++] = 0x00;
    for(int module=0; module<CELL_MAP_MODULES; module++) {
        uint8_t *data = &reply[len];
        if(module >= chain_modules) {
            memset(data, 0xFF, 9);
            len += 9;
            continue;
        }
        for(int cell=0; cell<3; cell++) {
            int physical_index = cell_map_physical_index(module, bank, cell);
            uint16_t raw = cell_map_is_present(map, physical_index)
                ? (uint16_t)((test_cell_mV(physical_index) * 25) / 2)
                : 0;
            data[cell * 2] = raw & 0xFF;
            data[cell * 2 + 1] = raw >> 8;
        }
        uint16_t crc = crc14(data, 6, 0x1000, 0);
        data[6] = crc >> 8;
        data[7] = crc & 0xFF;
        data[8] = 0;
        if(module == corrupt_module) {
            data[0] ^= 0x04;
        }
        len += 9;
    }
    return len;
}

static void put_scan(nibble_writer_t *w, const cell_map_t *map, int corrupt_bank, int corrupt_module, int skip_bank, int chain_modules) {
    uint8_t reply[CELL_SNOOP_MAX_REPLY];
    // Other BMB traffic before the reads
    const uint8_t hello_reply[4] = { 0 };
    put_transfer(w, 0x27100000, hello_reply, sizeof(hello_reply));
    for(int bank=0; bank<CELL_MAP_BANKS; bank++) {
        if(bank == skip_bank) {
            continue;
        }
        int len = make_bank_reply(map, reply, bank, bank == corrupt_bank ? corrupt_module : -1, chain_modules);
        put_transfer(w, BANK_COMMANDS[bank], reply, len);
    }
    // Temperature read after the banks
    put_transfer(w, 0x0E1B0000, hello_reply, sizeof(hello_reply));
}

static void test_snoop_decodes_scan(void **state) {
    (void) state;
    cell_map_t map = pack_map();
    cell_snoop_t snoop;
    cell_snoop_init(&snoop);
    nibble_writer_t w = { .snoop = &snoop };

    put_scan(&w, &map, -1, -1, -1, CELL_MAP_MODULES);
    assert_true(cell_snoop_take_scan(&snoop));
    assert_false(cell_snoop_take_scan(&snoop));
    assert_int_equal(snoop.scan_module_ok, 0xFFFF);

    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        int16_t expected = cell_map_is_present(&map, i) ? cell_raw_to_mV((uint16_t)((test_cell_mV(i) * 25) / 2)) : 1;
        assert_int_equal(snoop.scan_mV[i], expected);
    }

    // And the logic is happy with it
    int16_t min, max;
    uint8_t ok;
    assert_int_equal(sup_check_cells(&map, snoop.scan_mV, &min, &max, &ok), 0);
    assert_int_equal(ok, 96);
}

static void test_snoop_short_chain(void **state) {
    (void) state;
    // One BMB: the other slots fail their CRCs every time, which is no error
    cell_map_t map = desk_map();
    cell_snoop_t snoop;
    cell_snoop_init(&snoop);
    nibble_writer_t w = { .snoop = &snoop };

    put_scan(&w, &map, -1, -1, -1, 1);
    assert_true(cell_snoop_take_scan(&snoop));
    assert_int_equal(snoop.scan_module_ok, 0x01);
    assert_int_equal(snoop.scan_mV[15], CELL_MV_NOT_READ);

    int16_t min, max;
    uint8_t ok;
    assert_int_equal(sup_check_cells(&map, snoop.scan_mV, &min, &max, &ok), 0);
    assert_int_equal(ok, 15);
}

static void test_snoop_crc_error_marks_module(void **state) {
    (void) state;
    cell_map_t map = pack_map();
    cell_snoop_t snoop;
    cell_snoop_init(&snoop);
    nibble_writer_t w = { .snoop = &snoop };

    // Module 2 with a bad CRC on bank C
    put_scan(&w, &map, 2, 2, -1, CELL_MAP_MODULES);
    assert_true(cell_snoop_take_scan(&snoop));
    assert_int_equal(snoop.scan_module_ok, 0xFFFF & ~(1U << 2));
    assert_int_equal(snoop.module_crc_errors[2], 1);
    assert_int_equal(snoop.scan_mV[2 * 15], CELL_MV_NOT_READ);

    // A scan with a mapped module unread isn't checked at all
    sup_logic_t logic;
    sup_logic_init(&logic);
    sup_bms_status_t status = good_status(&map, 1000, NOMINAL_mV);
    sup_logic_on_status(&logic, &status, 1000);
    sup_logic_on_scan(&logic, snoop.scan_mV, snoop.scan_module_ok, 1000);
    assert_false(logic.have_scan);
    assert_int_equal(logic.incomplete_scans, 1);

    // The next clean scan is
    put_scan(&w, &map, -1, -1, -1, CELL_MAP_MODULES);
    assert_true(cell_snoop_take_scan(&snoop));
    sup_logic_on_scan(&logic, snoop.scan_mV, snoop.scan_module_ok, 2000);
    assert_true(logic.have_scan);
    assert_int_equal(logic.scan_reasons, 0);
}

static void test_snoop_missing_bank_drops_scan(void **state) {
    (void) state;
    cell_map_t map = pack_map();
    cell_snoop_t snoop;
    cell_snoop_init(&snoop);
    nibble_writer_t w = { .snoop = &snoop };

    put_scan(&w, &map, -1, -1, 3, CELL_MAP_MODULES);
    assert_false(cell_snoop_take_scan(&snoop));
    assert_int_equal(snoop.scans, 0);
}

/* Enable logic */

static void test_enables_after_good_scans(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);

    sim_run(&sim, SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_SETTLING);

    sim_enable(&sim);
    assert_int_equal(sim.logic.reasons, 0);

    sup_reply_t reply;
    sup_logic_reply(&sim.logic, &reply);
    assert_int_equal(reply.state, SUP_STATE_ON);
    assert_int_equal(reply.map_hash, cell_map_hash(&sim.map));
    assert_int_equal(reply.chemistry, SUPERVISOR_CHEMISTRY);
    assert_int_equal(reply.cells_ok, 96);
}

static void test_double_pack_map(void **state) {
    (void) state;
    cell_map_t map = double_pack_map();
    assert_int_equal(cell_map_check(&map), 0);
    // The second pack is the first one again, 120 channels on
    for(int i=0; i<120; i++) {
        assert_int_equal(cell_map_is_present(&map, i), cell_map_is_present(&map, 120 + i));
    }

    // The whole map survives the status message
    sup_bms_status_t status = good_status(&map, 1000, NOMINAL_mV);
    uint8_t buf[SUP_BMS_STATUS_LEN];
    assert_int_equal(sup_pack_bms_status(&status, buf, sizeof(buf)), SUP_BMS_STATUS_LEN);
    sup_bms_status_t out;
    assert_true(sup_unpack_bms_status(buf, sizeof(buf), &out));
    assert_true(cell_map_equal(&out.map, &map));

    // The snooper decodes all 16 BMBs, and the logic checks every cell
    cell_snoop_t snoop;
    cell_snoop_init(&snoop);
    nibble_writer_t w = { .snoop = &snoop };
    put_scan(&w, &map, -1, -1, -1, 16);
    assert_true(cell_snoop_take_scan(&snoop));
    assert_int_equal(snoop.scan_module_ok, 0xFFFF);
    int16_t min, max;
    uint8_t ok;
    assert_int_equal(sup_check_cells(&map, snoop.scan_mV, &min, &max, &ok), 0);
    assert_int_equal(ok, 216);

    // A bad CRC on the last BMB holds the scan back
    put_scan(&w, &map, 4, 15, -1, 16);
    assert_true(cell_snoop_take_scan(&snoop));
    assert_int_equal(snoop.scan_module_ok, 0x7FFF);
}

static void test_double_pack_enables_and_checks_last_bmb(void **state) {
    (void) state;
    sim_t sim;
    sim_init_map(&sim, double_pack_map());
    sim_enable(&sim);

    // A cell on the 16th BMB going under the hard limit latches like any other
    int last = 0;
    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        if(cell_map_is_present(&sim.map, i)) {
            last = i;
        }
    }
    assert_true(last / CELL_MAP_CELLS_PER_MODULE == 15);
    sim.scan[last] = CELL_VOLTAGE_HARD_MIN_mV - 1;
    run_until_scan_fault(&sim);
    sim_run(&sim, SUPERVISOR_CELL_FAULT_DROP_MS + SCAN_PERIOD_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);
    assert_true(sim.logic.latch_reasons & SUP_REASON_CELL_LOW);
}

static void test_desk_map_enables(void **state) {
    (void) state;
    sim_t sim;
    sim_init_map(&sim, desk_map());
    sim_enable(&sim);
    assert_int_equal(sim.logic.cells_ok, 15);
}

static void test_no_heartbeat_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.bms_running = false;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_NO_HEARTBEAT);
    // With no map, the scans aren't even checked
    assert_false(sim.logic.have_map);
    assert_int_equal(sim.logic.scans, 0);

    sup_reply_t reply;
    sup_logic_reply(&sim.logic, &reply);
    assert_int_equal(reply.map_hash, 0);
}

static void test_invalid_map_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.map.num_cells--;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_MAP_INVALID);
    assert_false(sim.logic.have_map);
}

static void test_empty_map_blocks(void **state) {
    (void) state;
    // Nothing to check would otherwise be nothing wrong
    sim_t sim;
    sim_init(&sim);
    memset(sim.map.presence, 0, sizeof(sim.map.presence));
    sim.map.num_cells = 0;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_MAP_INVALID);
}

static void test_other_chemistry_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.map.chemistry = OTHER_CHEMISTRY;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_CHEMISTRY);
    assert_false(sim.logic.have_map);
}

static void test_map_change_blocks_until_bms_restart(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_run(&sim, SCAN_PERIOD_MS);
    assert_true(sim.logic.have_map);

    // A different (but valid) map from the same BMS boot
    sim.map = desk_map();
    make_scan(&sim.map, sim.scan, NOMINAL_mV, 1);
    sim.module_ok = 0x01;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_MAP_CHANGED);
    // Still checking against the first one
    assert_int_equal(sim.logic.map.num_cells, 96);

    // Changing back doesn't help
    sim.map = pack_map();
    sim_run(&sim, SCAN_PERIOD_MS);
    assert_true(sim.logic.reasons & SUP_REASON_MAP_CHANGED);

    // A restarted BMS gets its new map
    sim.map = desk_map();
    sim_restart_bms(&sim);
    sim_enable(&sim);
    assert_int_equal(sim.logic.map.num_cells, 15);
}

static void test_map_change_while_enabled_latches_at_once(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // One cell dropped from the map (and the count), so a bad cell there
    // would go unchecked. Nothing waits for the BMS to sort it out.
    cell_map_t map = sim.map;
    int cell = first_present_channel(&map);
    map.presence[cell / 32] &= ~(1U << (cell % 32));
    map.num_cells--;
    sim.map = map;
    sim_run(&sim, STATUS_PERIOD_MS);
    assert_false(sim.enabled);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);
    assert_int_equal(sim.logic.latch_reasons, SUP_REASON_MAP_CHANGED);

    // Until the BMS restarts
    sim.map = pack_map();
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);
    sim_restart_bms(&sim);
    sim_enable(&sim);
    assert_int_equal(sim.logic.latch_reasons, 0);
}

static void test_invalid_map_while_enabled_latches(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    sim.map.chemistry = OTHER_CHEMISTRY;
    sim_run(&sim, STATUS_PERIOD_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);
    assert_true(sim.logic.reasons & SUP_REASON_CHEMISTRY);
}

static void test_version_mismatch_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.version = SUP_PROTOCOL_VERSION + 1;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_VERSION);
    // The BMS's version is kept for display, apart from the last readable status
    assert_int_equal(sim.logic.status_version, SUP_PROTOCOL_VERSION + 1);
    assert_int_equal(sim.logic.status.version, 0);

    sim.version = SUP_PROTOCOL_VERSION;
    sim_run(&sim, STATUS_PERIOD_MS);
    assert_int_equal(sim.logic.status_version, SUP_PROTOCOL_VERSION);
}

static void test_version_change_while_enabled_drops(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    sim.version = SUP_PROTOCOL_VERSION - 1;
    sim_run(&sim, STATUS_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_VERSION);

    sim.version = SUP_PROTOCOL_VERSION;
    sim_run(&sim, SCAN_PERIOD_MS);
    assert_true(sim.enabled);
}

static void test_missing_cell_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.scan[first_present_channel(&sim.map)] = -1;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_CELL_MISSING);
}

static void test_out_of_range_cell_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.scan[first_present_channel(&sim.map)] = CELL_VOLTAGE_HARD_MIN_mV - 1;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_CELL_LOW);

    sim.scan[first_present_channel(&sim.map)] = CELL_VOLTAGE_HARD_MAX_mV + 1;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_CELL_HIGH);

    // At the limit is still in range
    sim.bms_mV = CELL_VOLTAGE_HARD_MIN_mV;
    make_scan(&sim.map, sim.scan, CELL_VOLTAGE_HARD_MIN_mV, 8);
    sim_enable(&sim);
}

static void test_bms_disagreement_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.bms_mV = NOMINAL_mV - SUPERVISOR_BMS_AGREE_TOLERANCE_mV - 1;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_BMS_DISAGREE);

    // Within tolerance is fine
    sim.bms_mV = NOMINAL_mV - SUPERVISOR_BMS_AGREE_TOLERANCE_mV;
    sim_run(&sim, SCAN_PERIOD_MS);
    assert_true(sim.enabled);
}

static void test_absent_channel_reading_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    int absent = first_absent_channel(&sim.map);
    assert_true(absent >= 0);

    // A cell the map leaves out would go unchecked
    sim.scan[absent] = SUPERVISOR_ABSENT_CHANNEL_MAX_mV + 1;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_ABSENT_READS);

    // Up to the limit (eg. a little leakage) reads as empty
    sim.scan[absent] = SUPERVISOR_ABSENT_CHANNEL_MAX_mV;
    sim_enable(&sim);
    // As does a channel reporting no cell
    sim.scan[absent] = -1;
    sim_run(&sim, 2 * SCAN_PERIOD_MS);
    assert_true(sim.enabled);
}

static void test_cell_on_module_past_map_blocks(void **state) {
    (void) state;
    // The map covers one BMB, but a second answers with a cell on it. Without
    // a check it would never be looked at.
    sim_t sim;
    sim_init_map(&sim, desk_map());
    make_scan(&sim.map, sim.scan, NOMINAL_mV, 2);
    sim.scan[CELL_MAP_CELLS_PER_MODULE + 4] = NOMINAL_mV;
    sim.module_ok = 0x03;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_ABSENT_READS);

    // A BMB there with only empty channels is fine
    make_scan(&sim.map, sim.scan, NOMINAL_mV, 2);
    sim_enable(&sim);
}

static void test_absent_channel_while_enabled_latches(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    sim.scan[first_absent_channel(&sim.map)] = NOMINAL_mV;
    run_until_scan_fault(&sim);
    sim_run(&sim, SUPERVISOR_CELL_FAULT_DROP_MS - 2 * STATUS_PERIOD_MS);
    assert_true(sim.enabled);
    sim_run(&sim, 2 * STATUS_PERIOD_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);
    assert_int_equal(sim.logic.latch_reasons, SUP_REASON_ABSENT_READS);
}

static void test_crc_errors_are_missing_scans_not_cells(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // A mapped module keeps failing its CRC: the BMS sees the same failed
    // reads (and goes stale), so this is treated like scans stopping
    sim.module_ok &= ~(1U << 3);
    sim_run(&sim, SUPERVISOR_CELL_FAULT_DROP_MS * 2);
    assert_true(sim.enabled);
    sim_run(&sim, SUPERVISOR_SCAN_STALE_DROP_MS);
    assert_false(sim.enabled);
    assert_int_equal(sim.logic.state, SUP_STATE_OFF);
    assert_true(sim.logic.incomplete_scans > 0);

    sim.module_ok = 0xFF;
    sim_enable(&sim);
}

// Deterministic pseudo-random numbers (an LCG), so failures repeat run to run
static uint32_t next_random(uint32_t *seed) {
    *seed = *seed * 1664525u + 1013904223u;
    return *seed >> 8;
}

// Runs for ms, with each of a scan's five bank reads failing (a CRC error on
// one of the mapped modules) one time in fail_one_in. Fails the test if the
// drivers are ever cut. Returns the longest run of incomplete scans.
static int run_with_flaky_reads(sim_t *sim, uint32_t ms, uint32_t *seed, uint32_t fail_one_in) {
    int run = 0;
    int longest = 0;
    for(uint32_t t=0; t<ms; t+=STATUS_PERIOD_MS) {
        uint32_t next = sim->now + STATUS_PERIOD_MS;
        if(sim->scanning && (next % SCAN_PERIOD_MS) < STATUS_PERIOD_MS) {
            // sim_run() scans on this step: choose how it reads
            sim->module_ok = modules_mask(sim->map.num_modules);
            for(int bank=0; bank<CELL_MAP_BANKS; bank++) {
                if(next_random(seed) % fail_one_in == 0) {
                    sim->module_ok &= ~(1U << (next_random(seed) % sim->map.num_modules));
                }
            }
            if(sim->module_ok == modules_mask(sim->map.num_modules)) {
                run = 0;
            } else if(++run > longest) {
                longest = run;
            }
        }
        sim_run(sim, STATUS_PERIOD_MS);
        assert_true(sim->enabled);
    }
    sim->module_ok = modules_mask(sim->map.num_modules);
    return longest;
}

static void test_intermittent_crc_errors_stay_enabled(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // Dodgy wiring: one bank read in ten fails, so about 41% of scans are
    // incomplete, for an hour
    uint32_t seed = 12345;
    uint32_t scans_before = sim.logic.scans;
    int longest = run_with_flaky_reads(&sim, 3600000, &seed, 10);
    uint32_t complete = sim.logic.scans - scans_before;
    uint32_t incomplete = sim.logic.incomplete_scans;
    uint32_t total = complete + incomplete;

    // The failures happened at about the intended rate...
    assert_true(total > 2700);
    assert_true(incomplete * 100 > total * 35);
    assert_true(incomplete * 100 < total * 47);
    // ...including runs of several in a row, but nowhere near the
    // SUPERVISOR_SCAN_STALE_DROP_MS needed to cut the drivers
    assert_true(longest >= 4);
    assert_true(longest * SCAN_PERIOD_MS < SUPERVISOR_SCAN_STALE_DROP_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_ON);
    assert_int_equal(sim.logic.latch_reasons, 0);
}

static void test_crc_burst_outlasting_bms_stale_stays_enabled(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // Every scan fails for as long as the BMS takes to go stale and open the
    // contactors itself (and a bit more). The supervisor leaves that to it.
    uint32_t bms_opens_ms = CELL_VOLTAGE_STALE_THRESHOLD_MS + CELL_VOLTAGES_STALE_ESCALATION_MS +
        CONTACTORS_FORCE_OPEN_TIMEOUT_MS;
    sim.module_ok &= ~(1U << 5);
    for(uint32_t t=0; t<bms_opens_ms + 2 * SCAN_PERIOD_MS; t+=STATUS_PERIOD_MS) {
        sim_run(&sim, STATUS_PERIOD_MS);
        assert_true(sim.enabled);
    }
    assert_true(sim.logic.incomplete_scans * SCAN_PERIOD_MS >= bms_opens_ms);

    // Good scans again. The gap reset the good-scan count, which would block
    // enabling, but the drivers were never cut, so they stay enabled while the
    // count builds back up.
    sim.module_ok = modules_mask(sim.map.num_modules);
    sim_run(&sim, SCAN_PERIOD_MS);
    assert_true(sim.enabled);
    assert_int_equal(sim.logic.state, SUP_STATE_ON);
    assert_int_equal(sim.logic.reasons, SUP_REASON_SETTLING);
    sim_run(&sim, SUPERVISOR_GOOD_SCANS_TO_ENABLE * SCAN_PERIOD_MS);
    assert_true(sim.enabled);
    assert_int_equal(sim.logic.reasons, 0);
}

static void test_cell_fault_latches_after_twice_bms_delay(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // A cell goes over the hard limit; the BMS should open the contactors
    // within CELL_VOLTAGE_FAULT_ESCALATION_MS + CONTACTORS_FORCE_OPEN_TIMEOUT_MS
    sim.scan[first_present_channel(&sim.map)] = CELL_VOLTAGE_HARD_MAX_mV + 1;
    run_until_scan_fault(&sim);
    uint32_t fault_seen = sim.now;
    assert_true(sim.enabled);

    sim_run(&sim, SUPERVISOR_CELL_FAULT_DROP_MS - 2 * STATUS_PERIOD_MS);
    assert_true(sim.enabled);
    assert_true(sim.now - fault_seen < SUPERVISOR_CELL_FAULT_DROP_MS);

    sim_run(&sim, 2 * STATUS_PERIOD_MS);
    assert_false(sim.enabled);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);
    assert_true(sim.logic.latch_reasons & SUP_REASON_CELL_HIGH);

    // The cell recovering doesn't unlatch
    make_scan(&sim.map, sim.scan, NOMINAL_mV, 8);
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);

    // A BMS restart does, once the scans check out again
    sim_restart_bms(&sim);
    sim_run(&sim, STATUS_PERIOD_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_OFF);
    assert_int_equal(sim.logic.latch_reasons, 0);
    sim_enable(&sim);
}

static void test_brief_cell_fault_does_not_latch(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // One bad scan, then good ones
    sim.scan[first_present_channel(&sim.map)] = CELL_VOLTAGE_HARD_MAX_mV + 1;
    run_until_scan_fault(&sim);
    make_scan(&sim.map, sim.scan, NOMINAL_mV, 8);
    sim_run(&sim, SUPERVISOR_CELL_FAULT_DROP_MS * 2);
    assert_true(sim.enabled);
}

static void test_heartbeat_loss_drops_then_recovers(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // BMS stops (hung or being flashed); the BMB scans stop with it
    sim.bms_running = false;
    sim.scanning = false;
    sim_run(&sim, SUPERVISOR_HEARTBEAT_DROP_MS);
    assert_true(sim.enabled);
    sim_run(&sim, 2 * STATUS_PERIOD_MS);
    assert_false(sim.enabled);
    assert_int_equal(sim.logic.state, SUP_STATE_OFF);

    // BMS back after a restart
    sim.bms_running = true;
    sim.scanning = true;
    sim.bms_uptime = 0;
    sim_run(&sim, STATUS_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_SETTLING);
    sim_enable(&sim);
}

static void test_restart_seen_after_a_long_gap(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.bms_uptime = 1000;
    sim_enable(&sim);
    uint64_t uptime_before = sim.bms_uptime;

    // The link drops for a minute, during which the BMS reboots and runs long
    // enough to pass its old uptime
    sim.bms_running = false;
    sim_run(&sim, 60000);
    sim.bms_running = true;
    sim.bms_uptime = uptime_before + 20000;
    sim.map = desk_map();
    make_scan(&sim.map, sim.scan, NOMINAL_mV, 1);
    sim.module_ok = 0x01;
    sim_run(&sim, STATUS_PERIOD_MS);
    // Its new map is taken as that of a new boot, not a change
    assert_false(sim.logic.map_changed);
    assert_int_equal(sim.logic.map.num_cells, 15);
    sim_enable(&sim);
}

static void test_link_gap_does_not_clear_latch(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    sim.scan[first_present_channel(&sim.map)] = CELL_VOLTAGE_HARD_MAX_mV + 1;
    sim_run(&sim, SUPERVISOR_CELL_FAULT_DROP_MS + 2 * SCAN_PERIOD_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);
    make_scan(&sim.map, sim.scan, NOMINAL_mV, 8);

    // The serial link drops for longer than the heartbeat timeout while the
    // BMS keeps running: it hasn't restarted, so it stays latched
    uint64_t uptime = sim.bms_uptime;
    sim.bms_running = false;
    sim_run(&sim, 2 * SUPERVISOR_HEARTBEAT_DROP_MS);
    sim.bms_uptime = uptime + 2 * SUPERVISOR_HEARTBEAT_DROP_MS;
    sim.bms_running = true;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);
}

static void test_bms_restart_while_enabled_rechecks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    sim_restart_bms(&sim);
    sim_run(&sim, STATUS_PERIOD_MS);
    assert_false(sim.enabled);
    sim_enable(&sim);
}

static void stop_scans_until_dropped(sim_t *sim) {
    uint32_t last_scan = sim->logic.scan_millis;

    // eg. the BMB chain unplugged: the snooper sees no traffic at all
    sim->scanning = false;
    sim_run(sim, SUPERVISOR_SCAN_STALE_DROP_MS - (sim->now - last_scan) - STATUS_PERIOD_MS);
    assert_true(sim->enabled);
    sim_run(sim, 2 * STATUS_PERIOD_MS);
    assert_false(sim->enabled);
    assert_int_equal(sim->logic.state, SUP_STATE_OFF);
    assert_int_equal(sim->logic.latch_reasons, 0);
    assert_true(sim->logic.reasons & SUP_REASON_NO_SCAN);
}

static void test_scans_stopping_drops_until_fresh_scans(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);
    stop_scans_until_dropped(&sim);

    // Plugged back in: needs a fresh run of good scans, no BMS restart
    sim.scanning = true;
    sim_run(&sim, SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_SETTLING);
    sim_enable(&sim);
}

static void test_slow_mode_does_not_extend_scan_timeout(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // The BMS claiming slow mode (41s between scans) doesn't buy more time
    sim.extra_flags = SUP_BMS_FLAG_SLOW_MODE;
    stop_scans_until_dropped(&sim);
}

static void test_slow_scans_never_enable(void **state) {
    (void) state;
    // The BMS idle in slow mode: good scans, but 41s apart
    sim_t sim;
    sim_init(&sim);
    sim.scanning = false;
    for(int i=0; i<10; i++) {
        sim_run(&sim, 40900);
        sup_logic_on_scan(&sim.logic, sim.scan, sim.module_ok, sim.now);
        assert_false(sup_logic_tick(&sim.logic, sim.now));
        assert_true(sim.logic.reasons & SUP_REASON_SETTLING);
    }
    // Back to normal speed when it starts operating
    sim.scanning = true;
    sim_enable(&sim);
}

/* HMI */

static size_t hmi_ask(const sim_t *sim, uint8_t msg_type, uint8_t address, uint8_t *out) {
    const uint8_t request[] = { msg_type, address, 0x00, 0x00 };
    return sup_hmi_handle(&sim->logic, request, sizeof(request), sim->now, out, SUP_HMI_RESPONSE_LEN);
}

static uint32_t get_u32(const uint8_t *p) {
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void test_hmi_silent_until_bms_address_known(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.bms_running = false;
    sim_run(&sim, 2 * SCAN_PERIOD_MS);
    uint8_t out[SUP_HMI_RESPONSE_LEN];
    for(int address=0; address<256; address++) {
        assert_int_equal(hmi_ask(&sim, HMI_MSG_READ_SUPERVISOR, (uint8_t)address, out), 0);
    }
}

static void test_hmi_answers_only_its_command_at_bms_address(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);
    uint8_t out[SUP_HMI_RESPONSE_LEN];

    // Everything else at the BMS's address is the BMS's to answer
    assert_int_equal(hmi_ask(&sim, HMI_MSG_READ_REGISTERS, 1, out), 0);
    assert_int_equal(hmi_ask(&sim, HMI_MSG_READ_EVENTS, 1, out), 0);
    assert_int_equal(hmi_ask(&sim, HMI_MSG_SET_DEVICE_ADDRESS, 1, out), 0);
    assert_int_equal(hmi_ask(&sim, HMI_MSG_ANNOUNCE_DEVICE, 1, out), 0);
    // And other BMSes' supervisors answer at theirs
    assert_int_equal(hmi_ask(&sim, HMI_MSG_READ_SUPERVISOR, 2, out), 0);
    // Too short to say which
    const uint8_t bare[] = { HMI_MSG_READ_SUPERVISOR };
    assert_int_equal(sup_hmi_handle(&sim.logic, bare, 1, sim.now, out, sizeof(out)), 0);

    assert_int_equal(hmi_ask(&sim, HMI_MSG_READ_SUPERVISOR, 1, out), SUP_HMI_RESPONSE_LEN);
}

static void test_hmi_response_fields(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);
    uint8_t out[SUP_HMI_RESPONSE_LEN];
    assert_int_equal(hmi_ask(&sim, HMI_MSG_READ_SUPERVISOR, 1, out), SUP_HMI_RESPONSE_LEN);

    assert_int_equal(out[0], HMI_MSG_READ_SUPERVISOR_RESPONSE);
    assert_int_equal(out[1], 1);
    assert_int_equal(out[2], SUP_PROTOCOL_VERSION);
    assert_int_equal(out[3], SUP_STATE_ON);
    assert_int_equal(out[4] | (out[5] << 8), 0);
    assert_int_equal(out[6] | (out[7] << 8), 0);
    assert_int_equal((int16_t)(out[8] | (out[9] << 8)), NOMINAL_mV);
    assert_int_equal((int16_t)(out[10] | (out[11] << 8)), NOMINAL_mV);
    assert_int_equal(out[12], 96);
    assert_int_equal(out[13], 96);
    assert_int_equal(out[14], SUPERVISOR_CHEMISTRY);
    assert_int_equal(get_u32(&out[15]), sim.logic.scans);
    assert_true(get_u32(&out[19]) < SCAN_PERIOD_MS);
    assert_true(get_u32(&out[23]) <= STATUS_PERIOD_MS);
}

static void test_hmi_keeps_answering_when_bms_stops(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // The BMS hangs: the supervisor drops the drivers, and can still say so
    sim.bms_running = false;
    sim.scanning = false;
    sim_run(&sim, SUPERVISOR_SCAN_STALE_DROP_MS + STATUS_PERIOD_MS);
    uint8_t out[SUP_HMI_RESPONSE_LEN];
    assert_int_equal(hmi_ask(&sim, HMI_MSG_READ_SUPERVISOR, 1, out), SUP_HMI_RESPONSE_LEN);
    assert_int_equal(out[3], SUP_STATE_OFF);
    assert_true((out[4] | (out[5] << 8)) & SUP_REASON_NO_HEARTBEAT);
    assert_true(get_u32(&out[23]) >= SUPERVISOR_SCAN_STALE_DROP_MS);
}

static void test_hmi_follows_bms_address(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_run(&sim, STATUS_PERIOD_MS);

    // The HMI gives the BMS another address
    cell_map_t map = sim.map;
    sup_bms_status_t status = good_status(&map, sim.bms_uptime + 50, NOMINAL_mV);
    status.hmi_address = 7;
    sup_logic_on_status(&sim.logic, &status, sim.now + 50);
    uint8_t out[SUP_HMI_RESPONSE_LEN];
    assert_int_equal(hmi_ask(&sim, HMI_MSG_READ_SUPERVISOR, 1, out), 0);
    assert_int_equal(hmi_ask(&sim, HMI_MSG_READ_SUPERVISOR, 7, out), SUP_HMI_RESPONSE_LEN);
    assert_int_equal(out[1], 7);
}

static void test_delays_are_twice_the_bms(void **state) {
    (void) state;
    assert_int_equal(SUPERVISOR_CELL_FAULT_DROP_MS,
        2 * (CELL_VOLTAGE_FAULT_ESCALATION_MS + CONTACTORS_FORCE_OPEN_TIMEOUT_MS));
    assert_int_equal(SUPERVISOR_HEARTBEAT_DROP_MS, 2 * BMS_WATCHDOG_TIMEOUT_MS);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_frame_matches_duart_keepalive),
        cmocka_unit_test(test_status_round_trip),
        cmocka_unit_test(test_status_drops_bits_past_last_channel),
        cmocka_unit_test(test_reply_round_trip),
        cmocka_unit_test(test_other_version_is_reported),
        cmocka_unit_test(test_parser_resyncs),
        cmocka_unit_test(test_cell_map_check_accepts_good_maps),
        cmocka_unit_test(test_cell_map_check_rejects_bad_maps),
        cmocka_unit_test(test_cell_map_hash_covers_every_field),
        cmocka_unit_test(test_cell_raw_conversion),
        cmocka_unit_test(test_snoop_decodes_scan),
        cmocka_unit_test(test_snoop_short_chain),
        cmocka_unit_test(test_snoop_crc_error_marks_module),
        cmocka_unit_test(test_snoop_missing_bank_drops_scan),
        cmocka_unit_test(test_enables_after_good_scans),
        cmocka_unit_test(test_desk_map_enables),
        cmocka_unit_test(test_double_pack_map),
        cmocka_unit_test(test_double_pack_enables_and_checks_last_bmb),
        cmocka_unit_test(test_no_heartbeat_blocks),
        cmocka_unit_test(test_invalid_map_blocks),
        cmocka_unit_test(test_empty_map_blocks),
        cmocka_unit_test(test_other_chemistry_blocks),
        cmocka_unit_test(test_map_change_blocks_until_bms_restart),
        cmocka_unit_test(test_map_change_while_enabled_latches_at_once),
        cmocka_unit_test(test_invalid_map_while_enabled_latches),
        cmocka_unit_test(test_version_mismatch_blocks),
        cmocka_unit_test(test_version_change_while_enabled_drops),
        cmocka_unit_test(test_missing_cell_blocks),
        cmocka_unit_test(test_out_of_range_cell_blocks),
        cmocka_unit_test(test_bms_disagreement_blocks),
        cmocka_unit_test(test_absent_channel_reading_blocks),
        cmocka_unit_test(test_cell_on_module_past_map_blocks),
        cmocka_unit_test(test_absent_channel_while_enabled_latches),
        cmocka_unit_test(test_crc_errors_are_missing_scans_not_cells),
        cmocka_unit_test(test_intermittent_crc_errors_stay_enabled),
        cmocka_unit_test(test_crc_burst_outlasting_bms_stale_stays_enabled),
        cmocka_unit_test(test_cell_fault_latches_after_twice_bms_delay),
        cmocka_unit_test(test_brief_cell_fault_does_not_latch),
        cmocka_unit_test(test_heartbeat_loss_drops_then_recovers),
        cmocka_unit_test(test_restart_seen_after_a_long_gap),
        cmocka_unit_test(test_link_gap_does_not_clear_latch),
        cmocka_unit_test(test_bms_restart_while_enabled_rechecks),
        cmocka_unit_test(test_scans_stopping_drops_until_fresh_scans),
        cmocka_unit_test(test_slow_mode_does_not_extend_scan_timeout),
        cmocka_unit_test(test_slow_scans_never_enable),
        cmocka_unit_test(test_hmi_silent_until_bms_address_known),
        cmocka_unit_test(test_hmi_answers_only_its_command_at_bms_address),
        cmocka_unit_test(test_hmi_response_fields),
        cmocka_unit_test(test_hmi_keeps_answering_when_bms_stops),
        cmocka_unit_test(test_hmi_follows_bms_address),
        cmocka_unit_test(test_delays_are_twice_the_bms),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
