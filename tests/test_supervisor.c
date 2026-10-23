#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "config/limits.h"
#include "drivers/bmb3y/crc.h"
#include "shared/cell_map.h"
#include "shared/supervisor_protocol.h"
#include "cell_snoop.h"
#include "supervisor_logic.h"

#define STATUS_PERIOD_MS 100
#define SCAN_PERIOD_MS 1280
// A cell voltage in range for the chemistry
#define NOMINAL_mV ((CELL_VOLTAGE_HARD_MIN_mV + CELL_VOLTAGE_HARD_MAX_mV) / 2)

/* Helpers */

static int first_absent_channel(void) {
    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        if(!cell_map_is_present(i)) {
            return i;
        }
    }
    return -1;
}

static int first_present_channel(void) {
    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        if(cell_map_is_present(i)) {
            return i;
        }
    }
    return -1;
}

// Present cells at mV, unpopulated channels reading zero (1mV)
static void make_scan(int16_t *scan, int16_t mV) {
    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        scan[i] = cell_map_is_present(i) ? mV : 1;
    }
}

static sup_bms_status_t good_status(uint64_t uptime_ms, int16_t mV) {
    sup_bms_status_t status = {
        .version = SUP_PROTOCOL_VERSION,
        .uptime_ms = uptime_ms,
        .map_hash = cell_map_hash(),
        .flags = SUP_BMS_FLAG_CELLS_VALID,
        .cell_min_mV = mV,
        .cell_max_mV = mV,
    };
    return status;
}

// Simulates a BMS sending status every 100ms and the BMB cycle every 1.28s
typedef struct {
    sup_logic_t logic;
    uint32_t now;
    uint64_t bms_uptime;
    bool bms_running;
    bool scanning;
    int16_t scan[CELL_MAP_CHANNELS];
    int16_t bms_mV;
    uint8_t extra_flags; // added to the BMS status flags
    bool enabled;
} sim_t;

static void sim_init(sim_t *sim) {
    memset(sim, 0, sizeof(*sim));
    sup_logic_init(&sim->logic);
    sim->now = 1000;
    sim->bms_uptime = 5000;
    sim->bms_running = true;
    sim->scanning = true;
    sim->bms_mV = NOMINAL_mV;
    make_scan(sim->scan, NOMINAL_mV);
}

static void sim_run(sim_t *sim, uint32_t ms) {
    for(uint32_t t=0; t<ms; t+=STATUS_PERIOD_MS) {
        sim->now += STATUS_PERIOD_MS;
        if(sim->bms_running) {
            sim->bms_uptime += STATUS_PERIOD_MS;
            sup_bms_status_t status = good_status(sim->bms_uptime, sim->bms_mV);
            status.flags |= sim->extra_flags;
            sup_logic_on_status(&sim->logic, &status, sim->now);
        }
        if(sim->scanning && (sim->now % SCAN_PERIOD_MS) < STATUS_PERIOD_MS) {
            sup_logic_on_scan(&sim->logic, sim->scan, sim->now);
        }
        sim->enabled = sup_logic_tick(&sim->logic, sim->now);
    }
}

static void sim_enable(sim_t *sim) {
    sim_run(sim, (SUPERVISOR_GOOD_SCANS_TO_ENABLE + 1) * SCAN_PERIOD_MS);
    assert_true(sim->enabled);
    assert_int_equal(sim->logic.state, SUP_STATE_ON);
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
    sup_bms_status_t status = {
        .version = SUP_PROTOCOL_VERSION,
        .uptime_ms = 0x123456789AULL,
        .map_hash = 0xDEADBEEF,
        .flags = SUP_BMS_FLAG_CELLS_VALID | SUP_BMS_FLAG_BALANCING,
        .cell_min_mV = -1,
        .cell_max_mV = 4123,
    };
    uint8_t buf[SUP_BMS_STATUS_LEN];
    assert_int_equal(sup_pack_bms_status(&status, buf, sizeof(buf)), SUP_BMS_STATUS_LEN);

    sup_bms_status_t out;
    assert_true(sup_unpack_bms_status(buf, sizeof(buf), &out));
    assert_int_equal(out.version, SUP_PROTOCOL_VERSION);
    assert_true(out.uptime_ms == status.uptime_ms);
    assert_int_equal(out.map_hash, status.map_hash);
    assert_int_equal(out.flags, status.flags);
    assert_int_equal(out.cell_min_mV, -1);
    assert_int_equal(out.cell_max_mV, 4123);

    // A reply isn't a status
    buf[0] = SUP_MSG_SUPERVISOR_REPLY;
    assert_false(sup_unpack_bms_status(buf, sizeof(buf), &out));
}

static void test_reply_round_trip(void **state) {
    (void) state;
    sup_reply_t reply = {
        .version = SUP_PROTOCOL_VERSION,
        .state = SUP_STATE_LATCHED,
        .reasons = SUP_REASON_SETTLING | SUP_REASON_NO_SCAN,
        .latch_reasons = SUP_REASON_CELL_HIGH,
        .map_hash = 0x01020304,
        .cell_min_mV = 2999,
        .cell_max_mV = 4301,
        .cells_ok = 95,
        .scans = 123456,
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
}

static void test_other_version_is_reported(void **state) {
    (void) state;
    const uint8_t payload[] = { SUP_MSG_BMS_STATUS, SUP_PROTOCOL_VERSION + 1, 1, 2, 3 };
    sup_bms_status_t out;
    assert_true(sup_unpack_bms_status(payload, sizeof(payload), &out));
    assert_int_equal(out.version, SUP_PROTOCOL_VERSION + 1);
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

    sup_bms_status_t status = good_status(1234, NOMINAL_mV);
    uint8_t payload[SUP_BMS_STATUS_LEN];
    sup_pack_bms_status(&status, payload, sizeof(payload));
    uint8_t frame[SUP_MAX_FRAME_LEN];
    size_t frame_len = sup_frame_encode(payload, sizeof(payload), frame, sizeof(frame));

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
    bad[5] ^= 0x01;
    feed_all(&parser, bad, frame_len, &frames);
    assert_int_equal(frames, 1);
    assert_int_equal(parser.crc_errors, 1);
    feed_all(&parser, frame, frame_len, &frames);
    assert_int_equal(frames, 2);
}

/* Cell map */

static void test_cell_map_matches_cell_count(void **state) {
    (void) state;
    int logical = 0;
    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        if(cell_map_is_present(i)) {
            assert_int_equal(cell_map_logical_index(i), logical);
            logical++;
        } else {
            assert_int_equal(cell_map_logical_index(i), -1);
        }
    }
    assert_int_equal(logical, NUM_CELLS);
    assert_false(cell_map_is_present(-1));
    assert_false(cell_map_is_present(CELL_MAP_CHANNELS));
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

static int16_t test_cell_mV(int physical_index) {
    return (int16_t)(3000 + physical_index * 4);
}

// Reply to a bank read: padding byte, then per module three cells + CRC14 + spare
static int make_bank_reply(uint8_t *reply, int bank, int corrupt_module) {
    int len = 0;
    reply[len++] = 0x00;
    for(int module=0; module<CELL_MAP_MODULES; module++) {
        uint8_t *data = &reply[len];
        for(int cell=0; cell<3; cell++) {
            int physical_index = cell_map_physical_index(module, bank, cell);
            uint16_t raw = cell_map_is_present(physical_index)
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

static void put_scan(nibble_writer_t *w, int corrupt_bank, int corrupt_module, int skip_bank) {
    uint8_t reply[CELL_SNOOP_MAX_REPLY];
    // Other BMB traffic before the reads
    const uint8_t hello_reply[4] = { 0 };
    put_transfer(w, 0x27100000, hello_reply, sizeof(hello_reply));
    for(int bank=0; bank<CELL_MAP_BANKS; bank++) {
        if(bank == skip_bank) {
            continue;
        }
        int len = make_bank_reply(reply, bank, bank == corrupt_bank ? corrupt_module : -1);
        put_transfer(w, BANK_COMMANDS[bank], reply, len);
    }
    // Temperature read after the banks
    put_transfer(w, 0x0E1B0000, hello_reply, sizeof(hello_reply));
}

static void test_snoop_decodes_scan(void **state) {
    (void) state;
    cell_snoop_t snoop;
    cell_snoop_init(&snoop);
    nibble_writer_t w = { .snoop = &snoop };

    put_scan(&w, -1, -1, -1);
    assert_true(cell_snoop_take_scan(&snoop));
    assert_false(cell_snoop_take_scan(&snoop));
    assert_int_equal(snoop.crc_errors, 0);

    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        int16_t expected = cell_map_is_present(i) ? cell_raw_to_mV((uint16_t)((test_cell_mV(i) * 25) / 2)) : 1;
        assert_int_equal(snoop.scan_mV[i], expected);
    }

    // And the logic is happy with it
    int16_t min, max;
    uint8_t ok;
    uint16_t reasons = sup_check_cells(snoop.scan_mV, &min, &max, &ok);
    assert_int_equal(reasons & ~SUP_REASON_ABSENT_READS, 0);
    assert_int_equal(ok, NUM_CELLS);
}

static void test_snoop_crc_error_drops_scan(void **state) {
    (void) state;
    cell_snoop_t snoop;
    cell_snoop_init(&snoop);
    nibble_writer_t w = { .snoop = &snoop };

    // A pack module with a bad CRC on bank C
    put_scan(&w, 2, 0, -1);
    assert_false(cell_snoop_take_scan(&snoop));
    assert_int_equal(snoop.crc_errors, 1);

    // The next clean scan is used
    put_scan(&w, -1, -1, -1);
    assert_true(cell_snoop_take_scan(&snoop));
}

static void test_snoop_missing_bank_drops_scan(void **state) {
    (void) state;
    cell_snoop_t snoop;
    cell_snoop_init(&snoop);
    nibble_writer_t w = { .snoop = &snoop };

    put_scan(&w, -1, -1, 3);
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
    assert_int_equal(reply.map_hash, cell_map_hash());
    assert_int_equal(reply.cells_ok, NUM_CELLS);
}

static void test_no_heartbeat_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.bms_running = false;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_NO_HEARTBEAT);
}

static void test_map_mismatch_blocks(void **state) {
    (void) state;
    sup_logic_t logic;
    sup_logic_init(&logic);
    int16_t scan[CELL_MAP_CHANNELS];
    make_scan(scan, NOMINAL_mV);

    uint32_t now = 1000;
    for(int i=0; i<10; i++) {
        now += SCAN_PERIOD_MS;
        sup_bms_status_t status = good_status(now, NOMINAL_mV);
        status.map_hash ^= 1;
        sup_logic_on_status(&logic, &status, now);
        sup_logic_on_scan(&logic, scan, now);
        assert_false(sup_logic_tick(&logic, now));
    }
    assert_true(logic.reasons & SUP_REASON_MAP_MISMATCH);
}

static void test_version_mismatch_blocks(void **state) {
    (void) state;
    sup_logic_t logic;
    sup_logic_init(&logic);
    int16_t scan[CELL_MAP_CHANNELS];
    make_scan(scan, NOMINAL_mV);

    uint32_t now = 1000;
    for(int i=0; i<10; i++) {
        now += SCAN_PERIOD_MS;
        sup_bms_status_t status = good_status(now, NOMINAL_mV);
        status.version = SUP_PROTOCOL_VERSION + 1;
        sup_logic_on_status(&logic, &status, now);
        sup_logic_on_scan(&logic, scan, now);
        assert_false(sup_logic_tick(&logic, now));
    }
    assert_true(logic.reasons & SUP_REASON_VERSION);
}

static void test_missing_cell_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.scan[first_present_channel()] = -1;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_CELL_MISSING);
}

static void test_out_of_range_cell_blocks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim.scan[first_present_channel()] = CELL_VOLTAGE_HARD_MIN_mV - 1;
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
    assert_true(sim.logic.reasons & SUP_REASON_CELL_LOW);

    // At the limit is still in range
    sim.scan[first_present_channel()] = CELL_VOLTAGE_HARD_MIN_mV;
    sim.bms_mV = CELL_VOLTAGE_HARD_MIN_mV;
    make_scan(sim.scan, CELL_VOLTAGE_HARD_MIN_mV);
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

static void test_absent_channel_reading_is_reported(void **state) {
    (void) state;
    int absent = first_absent_channel();
    if(absent < 0) {
        skip();
    }
    sim_t sim;
    sim_init(&sim);
    sim.scan[absent] = NOMINAL_mV;
#ifdef SUPERVISOR_ENFORCE_ABSENT_CHANNELS
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_false(sim.enabled);
#else
    sim_enable(&sim);
#endif
    assert_true(sim.logic.reasons & SUP_REASON_ABSENT_READS);
}

static void test_cell_fault_latches_after_twice_bms_delay(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    // A cell goes over the hard limit; the BMS should open the contactors
    // within CELL_VOLTAGE_FAULT_ESCALATION_MS + CONTACTORS_FORCE_OPEN_TIMEOUT_MS
    sim.scan[first_present_channel()] = CELL_VOLTAGE_HARD_MAX_mV + 1;
    while(sim.logic.scan_reasons == 0) {
        sim_run(&sim, STATUS_PERIOD_MS);
    }
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
    make_scan(sim.scan, NOMINAL_mV);
    sim_run(&sim, 10 * SCAN_PERIOD_MS);
    assert_int_equal(sim.logic.state, SUP_STATE_LATCHED);

    // A BMS restart does, once the scans check out again
    sim.bms_uptime = 1000;
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
    sim.scan[first_present_channel()] = CELL_VOLTAGE_HARD_MAX_mV + 1;
    while(sim.logic.scan_reasons == 0) {
        sim_run(&sim, STATUS_PERIOD_MS);
    }
    make_scan(sim.scan, NOMINAL_mV);
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

static void test_bms_restart_while_enabled_rechecks(void **state) {
    (void) state;
    sim_t sim;
    sim_init(&sim);
    sim_enable(&sim);

    sim.bms_uptime = 100;
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
        cmocka_unit_test(test_reply_round_trip),
        cmocka_unit_test(test_other_version_is_reported),
        cmocka_unit_test(test_parser_resyncs),
        cmocka_unit_test(test_cell_map_matches_cell_count),
        cmocka_unit_test(test_cell_raw_conversion),
        cmocka_unit_test(test_snoop_decodes_scan),
        cmocka_unit_test(test_snoop_crc_error_drops_scan),
        cmocka_unit_test(test_snoop_missing_bank_drops_scan),
        cmocka_unit_test(test_enables_after_good_scans),
        cmocka_unit_test(test_no_heartbeat_blocks),
        cmocka_unit_test(test_map_mismatch_blocks),
        cmocka_unit_test(test_version_mismatch_blocks),
        cmocka_unit_test(test_missing_cell_blocks),
        cmocka_unit_test(test_out_of_range_cell_blocks),
        cmocka_unit_test(test_bms_disagreement_blocks),
        cmocka_unit_test(test_absent_channel_reading_is_reported),
        cmocka_unit_test(test_cell_fault_latches_after_twice_bms_delay),
        cmocka_unit_test(test_brief_cell_fault_does_not_latch),
        cmocka_unit_test(test_heartbeat_loss_drops_then_recovers),
        cmocka_unit_test(test_bms_restart_while_enabled_rechecks),
        cmocka_unit_test(test_scans_stopping_drops_until_fresh_scans),
        cmocka_unit_test(test_slow_mode_does_not_extend_scan_timeout),
        cmocka_unit_test(test_delays_are_twice_the_bms),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
