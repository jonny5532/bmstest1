#pragma once

// Messages on the internal serial link between the BMS and the supervisor.
//
// The BMS sends a status message every 100ms; the supervisor answers each one
// with a reply saying whether it is enabling the contactor drivers (CTR_EN) and,
// if not, why. Frames use the DUART packet format (see duart_send_packet):
//
//   0xFF, <payload length - 1>, <payload>, <CRC16 lo>, <CRC16 hi>
//
// with the CRC16 (Modbus) taken over the two header bytes and the payload.
// Multi-byte fields are little-endian.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SUP_PROTOCOL_VERSION 1

#define SUP_MSG_BMS_STATUS 'S'
#define SUP_MSG_SUPERVISOR_REPLY 'R'

// BMS status flags
#define SUP_BMS_FLAG_CELLS_VALID 0x01 // the min/max come from a fresh reading
#define SUP_BMS_FLAG_SLOW_MODE   0x02 // BMBs are sampled every ~41s, not 1.28s
#define SUP_BMS_FLAG_BALANCING   0x04 // readings are skewed by balancing

typedef struct {
    uint8_t version;
    // Time since the BMS booted. It going backwards means the BMS restarted.
    uint64_t uptime_ms;
    uint32_t map_hash;
    uint8_t flags;
    int16_t cell_min_mV;
    int16_t cell_max_mV;
} sup_bms_status_t;

typedef enum {
    // Contactor drivers disabled, will enable once every check passes
    SUP_STATE_OFF = 0,
    // Contactor drivers enabled
    SUP_STATE_ON = 1,
    // Disabled after a fault; stays off until the BMS restarts
    SUP_STATE_LATCHED = 2,
} sup_state_t;

// Reasons the supervisor isn't enabling the contactor drivers
#define SUP_REASON_NO_HEARTBEAT  0x0001 // no recent BMS status
#define SUP_REASON_VERSION       0x0002 // BMS speaks a different protocol version
#define SUP_REASON_MAP_MISMATCH  0x0004 // BMS was built for a different pack
#define SUP_REASON_NO_SCAN       0x0008 // no recent complete cell scan snooped
#define SUP_REASON_CELL_HIGH     0x0010 // a present cell is above the hard limit
#define SUP_REASON_CELL_LOW      0x0020 // a present cell is below the hard limit
#define SUP_REASON_CELL_MISSING  0x0040 // a present channel has no cell reading
#define SUP_REASON_BMS_DISAGREE  0x0080 // BMS min/max missing or don't match ours
#define SUP_REASON_SETTLING      0x0100 // not enough consecutive good scans yet
#define SUP_REASON_ABSENT_READS  0x0200 // an unpopulated channel reads like a cell

typedef struct {
    uint8_t version;
    uint8_t state;            // sup_state_t
    uint16_t reasons;         // what currently blocks enabling
    uint16_t latch_reasons;   // what caused the latch (SUP_STATE_LATCHED)
    uint32_t map_hash;
    int16_t cell_min_mV;      // from the last snooped scan
    int16_t cell_max_mV;
    uint8_t cells_ok;         // present cells in range in the last scan
    uint32_t scans;           // complete scans decoded since boot
} sup_reply_t;

#define SUP_BMS_STATUS_LEN 19
#define SUP_REPLY_LEN 20
// Largest frame on the link
#define SUP_MAX_FRAME_LEN (SUP_REPLY_LEN + 4)

size_t sup_pack_bms_status(const sup_bms_status_t *status, uint8_t *buf, size_t buf_size);
bool sup_unpack_bms_status(const uint8_t *payload, size_t len, sup_bms_status_t *status);
size_t sup_pack_reply(const sup_reply_t *reply, uint8_t *buf, size_t buf_size);
bool sup_unpack_reply(const uint8_t *payload, size_t len, sup_reply_t *reply);

// CRC16 (Modbus: reflected 0x8005, init 0xFFFF), as used by DUART
uint16_t sup_crc16(const uint8_t *data, size_t len, uint16_t crc);

// Wrap a payload in a frame. Returns the frame length, or 0 if it doesn't fit.
size_t sup_frame_encode(const uint8_t *payload, size_t payload_len, uint8_t *out, size_t out_size);

// Byte-at-a-time frame receiver, for the supervisor (the BMS uses DUART)
typedef struct {
    uint8_t buf[256 + 4];
    size_t pos;
    size_t max_payload_len;
    uint32_t crc_errors;
} sup_frame_parser_t;

// Frames claiming a longer payload than max_payload_len (at most 256) are
// treated as a false sync, so the parser realigns quickly.
void sup_frame_parser_init(sup_frame_parser_t *parser, size_t max_payload_len);
// Feed one received byte. When it completes a valid frame, returns the payload
// length and points *payload at it (valid until the next call); otherwise 0.
size_t sup_frame_feed(sup_frame_parser_t *parser, uint8_t byte, const uint8_t **payload);
