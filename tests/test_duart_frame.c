// DUART frame encoding and parsing (shared/duart_frame.h): frames at every position
// in the receive ring, so the header, payload and CRC each wrap around its end,
// and recovery from lost sync (false syncs, corrupted frames, starting
// mid-frame) without losing the frames that follow.

#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdint.h>
#include <string.h>

#include "shared/duart_frame.h"

#define MAX_RING 512

typedef struct {
    uint8_t data[MAX_RING];
    size_t size;  // power of two
    size_t start; // where the receiver's read pointer is
    size_t len;   // bytes written from start
} ring_t;

static void ring_init(ring_t *r, size_t size, size_t start, uint8_t fill) {
    memset(r->data, fill, sizeof(r->data));
    r->size = size;
    r->start = start % size;
    r->len = 0;
}

static void ring_put(ring_t *r, const uint8_t *bytes, size_t n) {
    for(size_t i=0; i<n; i++) {
        assert_true(r->len < r->size);
        r->data[(r->start + r->len) & (r->size - 1)] = bytes[i];
        r->len++;
    }
}

static void ring_fill(ring_t *r, uint8_t byte, size_t n) {
    for(size_t i=0; i<n; i++) {
        ring_put(r, &byte, 1);
    }
}

static duart_frame_parse_t ring_parse(ring_t *r, uint8_t *payload, size_t max_payload_len) {
    return duart_frame_parse(r->data, r->size, r->start, r->len, payload, max_payload_len);
}

// As the receiver does: drop what was consumed
static void ring_consume(ring_t *r, size_t n) {
    assert_true(n <= r->len);
    r->start = (r->start + n) & (r->size - 1);
    r->len -= n;
}

// A payload with a recognisable pattern (and some 0xFF bytes in it)
static void make_payload(uint8_t *p, size_t len, uint8_t seed) {
    for(size_t i=0; i<len; i++) {
        p[i] = (i % 7 == 3) ? 0xFF : (uint8_t)(seed + i * 13);
    }
}

static size_t make_frame(uint8_t *frame, size_t len, uint8_t seed) {
    uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
    make_payload(payload, len, seed);
    size_t n = duart_frame_encode(payload, len, frame, DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD);
    assert_int_equal(n, len + DUART_FRAME_OVERHEAD);
    return n;
}

static void assert_payload(const uint8_t *got, size_t len, uint8_t seed) {
    uint8_t expected[DUART_FRAME_MAX_PAYLOAD_LEN];
    make_payload(expected, len, seed);
    assert_memory_equal(got, expected, len);
}

static void test_crc16_is_modbus(void **state) {
    (void) state;
    const uint8_t check[] = "123456789";
    assert_int_equal(duart_frame_crc16(check, 9, DUART_FRAME_CRC16_INIT), 0x4B37);
}

static void test_encode(void **state) {
    (void) state;
    const uint8_t payload[] = { 0x01, 0x02, 0x03 };
    uint8_t out[16];
    assert_int_equal(duart_frame_encode(payload, 3, out, sizeof(out)), 7);
    assert_int_equal(out[0], DUART_FRAME_SYNC);
    assert_int_equal(out[1], 2);
    assert_memory_equal(&out[2], payload, 3);
    uint16_t crc = duart_frame_crc16(out, 5, DUART_FRAME_CRC16_INIT);
    assert_int_equal(out[5], crc & 0xFF);
    assert_int_equal(out[6], crc >> 8);

    // Empty, too long, or not fitting
    uint8_t big[DUART_FRAME_MAX_PAYLOAD_LEN + 1] = {0};
    uint8_t big_out[sizeof(big) + DUART_FRAME_OVERHEAD];
    assert_int_equal(duart_frame_encode(payload, 0, out, sizeof(out)), 0);
    assert_int_equal(duart_frame_encode(big, sizeof(big), big_out, sizeof(big_out)), 0);
    assert_int_equal(duart_frame_encode(big, DUART_FRAME_MAX_PAYLOAD_LEN, big_out, sizeof(big_out)), DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD);
    assert_int_equal(duart_frame_encode(payload, 3, out, 6), 0);
}

// A frame at every start position, so that each of its bytes (sync, length,
// payload, both CRC bytes) is the one that wraps at some point
static void test_frame_at_every_ring_offset(void **state) {
    (void) state;
    const size_t lengths[] = { 1, 2, 3, 21, 49, 64 };
    for(size_t l=0; l<sizeof(lengths)/sizeof(lengths[0]); l++) {
        size_t len = lengths[l];
        uint8_t frame[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
        size_t frame_len = make_frame(frame, len, (uint8_t)l);
        for(size_t start=0; start<128; start++) {
            ring_t r;
            ring_init(&r, 128, start, 0xFF); // stale sync-looking bytes around it
            ring_put(&r, frame, frame_len);
            uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
            duart_frame_parse_t p = ring_parse(&r, payload, 64);
            assert_int_equal(p.payload_len, len);
            assert_int_equal(p.consumed, frame_len);
            assert_int_equal(p.crc_errors, 0);
            assert_payload(payload, len, (uint8_t)l);
        }
    }
}

// The biggest frame, wrapping anywhere in the DUART-sized ring
static void test_max_length_frame_at_every_ring_offset(void **state) {
    (void) state;
    uint8_t frame[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t frame_len = make_frame(frame, DUART_FRAME_MAX_PAYLOAD_LEN, 7);
    for(size_t start=0; start<512; start++) {
        ring_t r;
        ring_init(&r, 512, start, 0x00);
        ring_put(&r, frame, frame_len);
        uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
        duart_frame_parse_t p = ring_parse(&r, payload, DUART_FRAME_MAX_PAYLOAD_LEN);
        assert_int_equal(p.payload_len, DUART_FRAME_MAX_PAYLOAD_LEN);
        assert_int_equal(p.consumed, frame_len);
        assert_payload(payload, DUART_FRAME_MAX_PAYLOAD_LEN, 7);
    }
}

// Each partial frame (cut anywhere, wrapping anywhere) waits, keeping the
// frame's bytes, and the frame arrives once it's all there
static void test_partial_frame_waits(void **state) {
    (void) state;
    uint8_t frame[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t frame_len = make_frame(frame, 21, 3);
    for(size_t start=0; start<64; start++) {
        for(size_t have=0; have<frame_len; have++) {
            ring_t r;
            ring_init(&r, 64, start, 0xFF);
            ring_put(&r, frame, have);
            uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
            duart_frame_parse_t p = ring_parse(&r, payload, 64);
            assert_int_equal(p.payload_len, 0);
            assert_int_equal(p.consumed, 0);
            assert_int_equal(p.crc_errors, 0);

            ring_put(&r, &frame[have], frame_len - have);
            p = ring_parse(&r, payload, 64);
            assert_int_equal(p.payload_len, 21);
            assert_int_equal(p.consumed, frame_len);
        }
    }
}

// Bytes that can't start a frame are consumed, without waiting for more
static void test_garbage_before_frame(void **state) {
    (void) state;
    uint8_t frame[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t frame_len = make_frame(frame, 10, 9);
    for(size_t start=0; start<64; start++) {
        for(size_t garbage=0; garbage<20; garbage++) {
            ring_t r;
            ring_init(&r, 64, start, 0xFF);
            ring_fill(&r, (uint8_t)(0x10 + garbage), garbage);
            uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
            duart_frame_parse_t p = ring_parse(&r, payload, 64);
            assert_int_equal(p.payload_len, 0);
            assert_int_equal(p.consumed, garbage);

            ring_put(&r, frame, frame_len);
            p = ring_parse(&r, payload, 64);
            assert_int_equal(p.payload_len, 10);
            assert_int_equal(p.consumed, garbage + frame_len);
            assert_payload(payload, 10, 9);
        }
    }

    // A trailing 0xFF might be the start of a frame: kept
    ring_t r;
    ring_init(&r, 64, 60, 0x00);
    const uint8_t tail[] = { 0x12, 0x34, DUART_FRAME_SYNC };
    ring_put(&r, tail, sizeof(tail));
    uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
    duart_frame_parse_t p = ring_parse(&r, payload, 64);
    assert_int_equal(p.payload_len, 0);
    assert_int_equal(p.consumed, 2);
}

// A false sync claiming more than the receiver accepts is skipped at once,
// rather than waiting for (and then discarding) the bytes it claims
static void test_false_sync_too_long_is_skipped(void **state) {
    (void) state;
    uint8_t frame[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t frame_len = make_frame(frame, 21, 1);
    for(size_t start=0; start<64; start++) {
        ring_t r;
        ring_init(&r, 64, start, 0x00);
        const uint8_t false_sync[] = { DUART_FRAME_SYNC, 64 }; // a 65-byte payload
        ring_put(&r, false_sync, sizeof(false_sync));
        ring_put(&r, frame, frame_len);
        uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
        duart_frame_parse_t p = ring_parse(&r, payload, 64);
        assert_int_equal(p.payload_len, 21);
        assert_int_equal(p.consumed, 2 + frame_len);
        assert_int_equal(p.crc_errors, 0);
    }

    // The longest accepted payload is max_payload_len
    uint8_t long_frame[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t long_len = make_frame(long_frame, 33, 2);
    ring_t r;
    ring_init(&r, 64, 50, 0x00);
    ring_put(&r, long_frame, long_len);
    uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
    assert_int_equal(ring_parse(&r, payload, 33).payload_len, 33);
    duart_frame_parse_t p = ring_parse(&r, payload, 32);
    assert_int_equal(p.payload_len, 0);
    // Its sync byte is skipped (the 0xFF bytes inside it are each still a
    // possible start)
    assert_true(p.consumed >= 1);
}

// A false sync with a plausible length, with a real frame inside the bytes it
// claims: it waits for those bytes (at most max_payload_len + 4), fails its CRC,
// and the real frame is still found
static void test_frame_inside_false_sync(void **state) {
    (void) state;
    uint8_t frame[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t frame_len = make_frame(frame, 21, 5);
    const uint8_t false_sync[] = { DUART_FRAME_SYNC, 48 }; // claims 53 bytes in all
    for(size_t start=0; start<128; start++) {
        ring_t r;
        ring_init(&r, 128, start, 0xFF);
        ring_put(&r, false_sync, sizeof(false_sync));
        ring_put(&r, frame, frame_len);
        uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];

        // Waiting for the rest of the false frame
        duart_frame_parse_t p = ring_parse(&r, payload, 64);
        assert_int_equal(p.payload_len, 0);
        assert_int_equal(p.consumed, 0);

        ring_fill(&r, 0x00, 53 - sizeof(false_sync) - frame_len);
        p = ring_parse(&r, payload, 64);
        assert_int_equal(p.crc_errors, 1);
        assert_int_equal(p.payload_len, 21);
        assert_int_equal(p.consumed, sizeof(false_sync) + frame_len);
        assert_payload(payload, 21, 5);
    }
}

// Corrupting any byte of a frame (including its sync and length) loses only
// that frame: the next one is found, wherever they sit in the ring
static void test_corrupted_frame_then_good_frame(void **state) {
    (void) state;
    uint8_t first[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    uint8_t second[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t first_len = make_frame(first, 21, 11);
    size_t second_len = make_frame(second, 21, 12);
    const uint8_t corruptions[] = { 0x01, 0x80, 0xFF };
    for(size_t start=0; start<128; start++) {
        for(size_t pos=0; pos<first_len; pos++) {
            for(size_t c=0; c<sizeof(corruptions); c++) {
                uint8_t bad[sizeof(first)];
                memcpy(bad, first, first_len);
                bad[pos] = (corruptions[c] == 0xFF) ? 0xFF : (uint8_t)(bad[pos] ^ corruptions[c]);
                if(bad[pos] == first[pos]) {
                    continue;
                }

                ring_t r;
                ring_init(&r, 128, start, 0xFF);
                ring_put(&r, bad, first_len);
                ring_put(&r, second, second_len);
                // Enough after them for any false sync among them to complete
                ring_fill(&r, 0x00, 64 + DUART_FRAME_OVERHEAD);

                uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
                duart_frame_parse_t p = ring_parse(&r, payload, 64);
                assert_int_equal(p.payload_len, 21);
                assert_int_equal(p.consumed, first_len + second_len);
                assert_payload(payload, 21, 12);
            }
        }
    }
}

// The receiver starts listening partway through a frame whose payload is all
// 0xFF (the worst case for false syncs): the next frame still arrives intact
static void test_start_mid_frame(void **state) {
    (void) state;
    uint8_t ff[64];
    memset(ff, 0xFF, sizeof(ff));
    uint8_t first[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t first_len = duart_frame_encode(ff, sizeof(ff), first, sizeof(first));
    uint8_t second[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t second_len = make_frame(second, 21, 13);
    for(size_t start=0; start<256; start += 7) {
        for(size_t cut=1; cut<first_len; cut++) {
            ring_t r;
            ring_init(&r, 256, start, 0x00);
            ring_put(&r, &first[cut], first_len - cut);
            ring_put(&r, second, second_len);
            ring_fill(&r, 0x00, 64 + DUART_FRAME_OVERHEAD);

            // Drain as a receiver would, until a frame arrives
            uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
            duart_frame_parse_t p = ring_parse(&r, payload, 64);
            ring_consume(&r, p.consumed);
            assert_int_equal(p.payload_len, 21);
            assert_payload(payload, 21, 13);
            // Nothing but the filler is left
            assert_int_equal(r.len, 64 + DUART_FRAME_OVERHEAD);
        }
    }
}

// Back-to-back frames come out one per call, in order
static void test_back_to_back_frames(void **state) {
    (void) state;
    for(size_t start=0; start<64; start++) {
        ring_t r;
        ring_init(&r, 64, start, 0xFF);
        uint8_t frame[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
        for(uint8_t i=0; i<5; i++) {
            ring_put(&r, frame, make_frame(frame, 1 + i * 2, i));
        }
        uint8_t payload[DUART_FRAME_MAX_PAYLOAD_LEN];
        for(uint8_t i=0; i<5; i++) {
            duart_frame_parse_t p = ring_parse(&r, payload, 64);
            assert_int_equal(p.payload_len, 1 + i * 2);
            assert_payload(payload, 1 + i * 2, i);
            ring_consume(&r, p.consumed);
        }
        assert_int_equal(r.len, 0);
        assert_int_equal(ring_parse(&r, payload, 64).payload_len, 0);
    }
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_crc16_is_modbus),
        cmocka_unit_test(test_encode),
        cmocka_unit_test(test_frame_at_every_ring_offset),
        cmocka_unit_test(test_max_length_frame_at_every_ring_offset),
        cmocka_unit_test(test_partial_frame_waits),
        cmocka_unit_test(test_garbage_before_frame),
        cmocka_unit_test(test_false_sync_too_long_is_skipped),
        cmocka_unit_test(test_frame_inside_false_sync),
        cmocka_unit_test(test_corrupted_frame_then_good_frame),
        cmocka_unit_test(test_start_mid_frame),
        cmocka_unit_test(test_back_to_back_frames),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
