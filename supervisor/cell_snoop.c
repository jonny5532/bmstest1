#include "cell_snoop.h"

#include "config/limits.h"
#include "drivers/bmb3y/crc.h"

#include <string.h>

#define SYMBOL_IDLE 0x0
#define SYMBOL_DATA0 0x6
#define SYMBOL_DATA1 0x9
#define SYMBOL_CS 0xA

// First request byte of the bank A voltage read, long (BMB3Y_CMD_READ_A) and
// short (BMB3Y_CMD_READ_A_SHORT) forms
#define READ_BANK_A_COMMAND_LONG 0x47
#define READ_BANK_A_COMMAND_SHORT 0x07

// Modules the pack actually has, which must all read cleanly for a scan
#define PACK_MODULES_MASK ((uint8_t)((1U << NUM_MODULE_VOLTAGES) - 1))

static void clear_cells(int16_t *mV) {
    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        mV[i] = CELL_MV_NOT_READ;
    }
}

void cell_snoop_init(cell_snoop_t *snoop) {
    memset(snoop, 0, sizeof(*snoop));
    snoop->next_bank = -1;
    clear_cells(snoop->pending_mV);
    clear_cells(snoop->scan_mV);
}

static void process_bank(cell_snoop_t *snoop, int bank, bool is_long) {
    snoop->bank_reads++;

    if(bank == 0) {
        // Start a new scan
        if(snoop->next_bank > 0) {
            snoop->incomplete_scans++;
        }
        snoop->next_bank = 0;
        snoop->pending_is_long = is_long;
        snoop->pending_module_ok = 0xFF;
        clear_cells(snoop->pending_mV);
    }
    if(bank != snoop->next_bank || is_long != snoop->pending_is_long) {
        // Out of sequence (or missed bank A, or the read form changed), wait
        // for the next scan
        if(snoop->next_bank > 0) {
            snoop->incomplete_scans++;
        }
        snoop->next_bank = -1;
        return;
    }

    // The long form has a tag byte before each module's data, which is part of
    // the CRC; the short form has none
    int first_slot = is_long ? 1 : 0;
    int slot_bytes = is_long ? 9 : 8;
    uint16_t crc_init = is_long ? 0x1000 : 0x0010;

    for(int module=0; module<CELL_MAP_MODULES; module++) {
        int base = first_slot + module * slot_bytes;
        if(base + 8 > snoop->reply_len) {
            snoop->pending_module_ok &= ~(1U << module);
            continue;
        }

        uint8_t *data = &snoop->reply[base];
        uint16_t module_crc = (uint16_t)((data[6] << 8) | data[7]);
        uint16_t calc_crc = crc14(data, 6, crc_init, module_crc);
        if((module_crc & 0x3fff) != calc_crc) {
            if(snoop->pending_module_ok & (1U << module) && (PACK_MODULES_MASK & (1U << module))) {
                snoop->crc_errors++;
            }
            snoop->pending_module_ok &= ~(1U << module);
            continue;
        }

        for(int cell=0; cell<CELL_MAP_CELLS_PER_BANK; cell++) {
            uint16_t raw = (uint16_t)(data[cell * 2] | (data[cell * 2 + 1] << 8));
            snoop->pending_mV[cell_map_physical_index(module, bank, cell)] = cell_raw_to_mV(raw);
        }
    }

    snoop->next_bank++;
    if(snoop->next_bank < CELL_MAP_BANKS) {
        return;
    }

    // All five banks read
    snoop->next_bank = -1;
    if((snoop->pending_module_ok & PACK_MODULES_MASK) != PACK_MODULES_MASK) {
        snoop->incomplete_scans++;
        return;
    }
    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        int module = i / CELL_MAP_CELLS_PER_MODULE;
        snoop->scan_mV[i] = (snoop->pending_module_ok & (1U << module)) ? snoop->pending_mV[i] : CELL_MV_NOT_READ;
    }
    snoop->scan_module_ok = snoop->pending_module_ok;
    snoop->scans++;
    snoop->scan_ready = true;
}

static void end_transfer(cell_snoop_t *snoop) {
    if(snoop->reply_bits) {
        // Keep a trailing partial byte, as it may complete a module's spare byte
        if(snoop->reply_len < CELL_SNOOP_MAX_REPLY) {
            snoop->reply[snoop->reply_len++] = snoop->reply_byte;
        }
    }

    if(snoop->request_bits >= 8 && snoop->reply_len > 0) {
        // Left-align the request so its first byte is the command
        uint32_t request = snoop->request_bits < 32
            ? (snoop->request_word << (32 - snoop->request_bits))
            : snoop->request_word;
        int command = (int)((request >> 24) & 0xFF);
        int long_bank = command - READ_BANK_A_COMMAND_LONG;
        int short_bank = command - READ_BANK_A_COMMAND_SHORT;
        if(long_bank >= 0 && long_bank < CELL_MAP_BANKS) {
            process_bank(snoop, long_bank, true);
        } else if(short_bank >= 0 && short_bank < CELL_MAP_BANKS) {
            process_bank(snoop, short_bank, false);
        }
    }

    snoop->request_word = 0;
    snoop->request_bits = 0;
    snoop->reply_byte = 0;
    snoop->reply_bits = 0;
    snoop->reply_len = 0;
    snoop->is_reply = false;
}

static void data_bit(cell_snoop_t *snoop, uint8_t bit) {
    if(snoop->is_reply) {
        snoop->reply_byte = (uint8_t)((snoop->reply_byte << 1) | bit);
        snoop->reply_bits++;
        if(snoop->reply_bits == 8) {
            if(snoop->reply_len < CELL_SNOOP_MAX_REPLY) {
                snoop->reply[snoop->reply_len++] = snoop->reply_byte;
            }
            snoop->reply_byte = 0;
            snoop->reply_bits = 0;
        }
    } else if(snoop->request_bits < 32) {
        snoop->request_word = (snoop->request_word << 1) | bit;
        snoop->request_bits++;
    }
    // The message bits alternate between request and reply
    snoop->is_reply = !snoop->is_reply;
}

static void feed_symbol(cell_snoop_t *snoop, uint8_t symbol) {
    switch(symbol) {
        case SYMBOL_CS:
            end_transfer(snoop);
            break;
        case SYMBOL_IDLE:
            // Requests with no reply have idle in its place, so the next bit
            // is a request bit again
            snoop->is_reply = false;
            break;
        case SYMBOL_DATA0:
            data_bit(snoop, 0);
            break;
        case SYMBOL_DATA1:
            data_bit(snoop, 1);
            break;
        default:
            break;
    }
}

void cell_snoop_feed(cell_snoop_t *snoop, uint8_t byte) {
    feed_symbol(snoop, byte >> 4);
    feed_symbol(snoop, byte & 0x0F);
}

bool cell_snoop_take_scan(cell_snoop_t *snoop) {
    if(!snoop->scan_ready) {
        return false;
    }
    snoop->scan_ready = false;
    return true;
}
