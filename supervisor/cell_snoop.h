#pragma once

// Decodes the BMS's cell voltage reads from the isoSPI snooper's symbol stream
// and assembles them into complete scans.
//
// The snooper reports one 4-bit symbol per isoSPI pulse (two per byte, high
// nibble first): 1010 = CS (start or end of a transfer), 0110 = data 0,
// 1001 = data 1, 0000 = idle. Data bits alternate between the request (from
// the BMS) and the reply (from the BMBs).
//
// A cell voltage read is a request whose first byte is 0x47-0x4B (banks A-E,
// three cells each). The reply is one padding byte and then nine bytes per
// module slot (eight, whatever the chain length): three little-endian cell
// readings, a CRC14 and a spare byte. The BMS reads the five banks back to back
// once per BMB cycle; a scan is complete when all five arrive in order. A
// module's readings are only kept if its CRC was good in all five, and the
// scan says which modules those were. The decoder knows nothing of the pack.

#include "shared/cell_map.h"

#include <stdbool.h>
#include <stdint.h>

// A bank read's reply: a padding byte, then 9 bytes per module
#define CELL_SNOOP_MAX_REPLY (1 + CELL_MAP_MODULES * 9)

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
    uint16_t pending_module_ok; // modules with good CRCs on every bank so far
    int16_t pending_mV[CELL_MAP_CHANNELS];

    // Last complete scan
    bool scan_ready;
    int16_t scan_mV[CELL_MAP_CHANNELS];
    uint16_t scan_module_ok;

    uint32_t scans;
    uint32_t bank_reads;
    uint32_t module_crc_errors[CELL_MAP_MODULES]; // in scans that were going well
    uint32_t incomplete_scans; // banks missing or out of order
} cell_snoop_t;

void cell_snoop_init(cell_snoop_t *snoop);
// Feed one byte (two symbols) from the snooper
void cell_snoop_feed(cell_snoop_t *snoop, uint8_t byte);
// If a new complete scan is available, clear the flag and return true. The
// scan is then in scan_mV (CELL_MV_NOT_READ on modules with a bad CRC) and
// scan_module_ok (bit n set if module n read cleanly).
bool cell_snoop_take_scan(cell_snoop_t *snoop);
