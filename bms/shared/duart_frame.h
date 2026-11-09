#pragma once

// The packet format of the DUART links (the HMI bus and the internal link to
// the supervisor), shared by the BMS (drivers/comms/duart.c) and the
// supervisor:
//
//   0xFF, <payload length - 1>, <payload (1-256 bytes)>, <CRC16 lo>, <CRC16 hi>
//
// with the CRC16 (Modbus) taken over the two header bytes and the payload.
//
// The receiver will try to recover from each potential sync byte, checking the
// CRC to see if it forms a valid frame.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DUART_FRAME_SYNC 0xFF
#define DUART_FRAME_OVERHEAD 4 // sync, length and CRC bytes
#define DUART_FRAME_MAX_PAYLOAD_LEN 256

// Encode a frame. Returns its length, or 0 if the payload is empty, longer
// than DUART_FRAME_MAX_PAYLOAD_LEN or doesn't fit in out.
size_t duart_frame_encode(const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_size);

uint16_t duart_frame_crc16(const uint8_t *data, size_t len, uint16_t crc);
#define DUART_FRAME_CRC16_INIT 0xFFFF

typedef struct {
    // Bytes to discard from the start (includes any pre-frame junk)
    size_t consumed;
    // The frame's payload length (copied to the payload buffer), or 0 if no
    // complete frame was found
    size_t payload_len;
    // How many frame parse attempts resulted in CRC errors
    uint32_t crc_errors;
} duart_frame_parse_t;

// Look for the next frame in the received bytes ring[(start + i) & (ring_size
// - 1)], for i < available (ring_size must be a power of two). Frames with a
// payload longer than max_payload_len are not accepted (they're taken as false
// syncs). The payload buffer must hold max_payload_len bytes.
duart_frame_parse_t duart_frame_parse(const uint8_t *ring, size_t ring_size, size_t start, size_t available,
    uint8_t *payload, size_t max_payload_len);
