#pragma once

// Decodes the BMS's cell voltage reads from the isoSPI snooper's symbol stream
// and assembles them into complete scans.
//
// The snooper reports one 4-bit symbol per isoSPI pulse (two per byte, high
// nibble first): 1010 = CS (start or end of a transfer), 0110 = data 0,
// 1001 = data 1, 0000 = idle. Data bits alternate between the request (from
// the BMS) and the reply (from the BMBs).
//
// A cell voltage read (banks A-E, three cells each) comes in two forms:
//  - long: a 4-byte request whose first byte is 0x47-0x4B. The reply is nine
//    bytes per module: a tag byte, three little-endian cell readings and a
//    CRC14 (initial value 0x1000, which covers the zero tag byte).
//  - short: a 2-byte request whose first byte is 0x07-0x0B. The reply is eight
//    bytes per module: three little-endian cell readings and a CRC14 (initial
//    value 0x0010), the same as the temperature reads.
// The BMS reads the five banks back to back once per BMB cycle; a scan is
// complete when all five arrive in order, in the same form, with good CRCs for
// every module in the pack.

#include "shared/cell_map.h"

#include <stdbool.h>
#include <stdint.h>

#define CELL_SNOOP_MAX_REPLY 100

typedef struct {
    // Current transfer
    uint32_t request_word;
    uint8_t request_bits;
    uint8_t reply_byte;
    uint8_t reply_bits;
    bool is_reply;
    uint8_t reply[CELL_SNOOP_MAX_REPLY];
    uint16_t reply_len;

    // Scan being assembled
    int8_t next_bank; // bank expected next, or -1 to wait for bank A
    bool pending_is_long; // form of the reads in this scan
    uint8_t pending_module_ok; // modules with good CRCs on every bank so far
    int16_t pending_mV[CELL_MAP_CHANNELS];

    // Last complete scan
    bool scan_ready;
    int16_t scan_mV[CELL_MAP_CHANNELS];
    uint8_t scan_module_ok;

    uint32_t scans;
    uint32_t bank_reads;
    uint32_t crc_errors;
    uint32_t incomplete_scans;
} cell_snoop_t;

void cell_snoop_init(cell_snoop_t *snoop);
// Feed one byte (two symbols) from the snooper
void cell_snoop_feed(cell_snoop_t *snoop, uint8_t byte);
// If a new complete scan is available, clear the flag and return true. The
// scan is then in scan_mV (CELL_MV_NOT_READ for modules outside the pack or
// with a bad CRC) and scan_module_ok.
bool cell_snoop_take_scan(cell_snoop_t *snoop);
