#include "bmb3y.h"
#include "crc.h"

#include "../limits.h"
#include "../model.h"
#include "../isospi/isospi_master.h"

#include "pico/stdlib.h"

#include <stdint.h>

const uint8_t CELLS_PER_MODULE = 15;
const uint8_t CELLS_PER_BANK = 3;
const uint8_t MODULE_COUNT = 8;

// Low level functions

void bmb3y_send_command_blocking(uint16_t cmd_word) {
    // Send a 16-bit command with normal CS pattern
    uint8_t tx[2];
    uint8_t rx[2] = {0};
    
    // Split 16-bit command into 2 bytes (MSB first)
    tx[0] = (cmd_word >> 8) & 0xFF;
    tx[1] = cmd_word & 0xFF;
    
    isospi_write_read_blocking(tx, rx, 2, 2);
}

bool bmb3y_long_command_get_data_blocking(uint32_t cmd, uint8_t *response, int response_len) {
    // Send a four-byte command and read data from BMBs into the supplied buffer.
    // Returns true if the read was successful.
    
    // TODO: Check whether we need to wake up?

    uint8_t tx[104] = {0};
    if(response_len > 100) {
        // Transmit buffer isn't large enough
        return false;
    }

    // CRC is CRC8 with polynomial 2f (AUTOSAR) but we just bake it into the commands

    tx[0] = (cmd >> 24) & 0xFF;
    tx[1] = (cmd >> 16) & 0xFF;
    tx[2] = (cmd >> 8) & 0xFF;
    tx[3] = cmd & 0xFF;

    // tx[0] = reg_cmd; // command byte
    // tx[1] = 0;       // dummy byte
    // tx[2] = 0x70;    // CRC8 (poly=0x2F, init=0x10)
    // tx[3] = 0;       // 
    
    return isospi_write_read_blocking(tx, response, 4 + response_len, 4);
}

bool bmb3y_short_command_get_data_blocking(uint16_t cmd, uint8_t *response, int response_len) {
    // Send a two-byte command and read data from BMBs into the supplied buffer.
    // Returns true if the read was successful.
    
    // TODO: Check whether we need to wake up?

    uint8_t tx[104] = {0};
    if(response_len > 100) {
        // Transmit buffer isn't large enough
        return false;
    }

    tx[0] = (cmd >> 8) & 0xFF;
    tx[1] = cmd & 0xFF;
    
    return isospi_write_read_blocking(tx, response, 2 + response_len, 2);
}

void bmb3y_send_wakeup_cs_blocking() {
    isospi_send_wakeup_cs_blocking();
}

// Higher level functions

bool bmb3y_set_balancing(uint8_t bitmap[16], bool even) {
    uint8_t tx_buf[50] = {0};
    
    tx_buf[0] = (BMB3Y_CMD_WRITE_CONFIG >> 8) & 0xFF;
    tx_buf[1] = BMB3Y_CMD_WRITE_CONFIG & 0xFF;

    uint8_t balance_mask = even ? 0xAA : 0x55;

    for(int module=0; module<8; module++) {
        // was f3
        tx_buf[2 + module*6] = 0xf3;
        // was 00
        tx_buf[3 + module*6] = 0;

        tx_buf[4 + module*6] = bitmap[module*2 + 1] & balance_mask;
        tx_buf[5 + module*6] = bitmap[module*2] & balance_mask;

        uint16_t calc_crc = crc14(&tx_buf[2 + module*6], 4, 0x0010);

        tx_buf[6 + module*6] = (calc_crc >> 8) & 0xFF;
        tx_buf[7 + module*6] = calc_crc & 0xFF;
    }

    // Balance command:
    // for(int i=0; i<50; i++) {
    //     printf("%02X ", tx_buf[i]);
    // }
    // printf("\n");

    // We skip all of the response bytes
    isospi_write_read_blocking(tx_buf, NULL, 50, 50);

    return true;
}

void bmb3y_send_balancing(bms_model_t *model) {
    balancing_sm_t *balancing_sm = &model->balancing_sm;

    uint8_t tx_buf[50] = {0};
    
    tx_buf[0] = (BMB3Y_CMD_WRITE_CONFIG >> 8) & 0xFF;
    tx_buf[1] = BMB3Y_CMD_WRITE_CONFIG & 0xFF;

    /* 
    The balancing mask consists of four 32-bit ints, with a bit for each cell -
    the LSB of the last int is cell 0.

    However the BMB modules consume the mask 15-bits at a time, so it is
    necessary to re-pack the bits accordingly.
    */

    uint64_t window = 0;
    // We will throw away the top 8 bits of the first mask int.
    int bits_in_window = -8;
    int mask_idx = 4;

    //printf("Sending mask ");

    for(int module=0; module<8; module++) {
        tx_buf[2 + module*6] = 0xf3;
        tx_buf[3 + module*6] = 0;

        // Pull more bits into the window if needed
        while (bits_in_window < 15 && mask_idx > 0) {
            window |= ((uint64_t)balancing_sm->balance_request_mask[--mask_idx]) << (32 - bits_in_window);
            bits_in_window += 32;
        }

        // Extract 15 bits for this module
        uint16_t balance_bits = (uint16_t)(window >> 49);
        window <<= 15;
        bits_in_window -= 15;

        tx_buf[4 + module*6] = (uint8_t)(balance_bits & 0xFF);
        tx_buf[5 + module*6] = (uint8_t)((balance_bits >> 8) & 0xFF);
        
        /* old code (assumes 16 bits per module rather than 15) */
        // int mask_idx2 = 3 - (module / 2);
        // uint32_t mask = balancing_sm->balance_request_mask[mask_idx2];
        // if (module % 2 == 0) {
        //     tx_buf[4 + module*6] = (uint8_t)((mask >> 16) & 0xFF);
        //     tx_buf[5 + module*6] = (uint8_t)((mask >> 24) & 0xFF);
        // } else {
        //     tx_buf[4 + module*6] = (uint8_t)(mask & 0xFF);
        //     tx_buf[5 + module*6] = (uint8_t)((mask >> 8) & 0xFF);
        // }

        //printf("0x%02X%02X ", tx_buf[5 + module*6], tx_buf[4 + module*6]);

        uint16_t calc_crc = crc14(&tx_buf[2 + module*6], 4, 0x0010);

        tx_buf[6 + module*6] = (calc_crc >> 8) & 0xFF;
        tx_buf[7 + module*6] = calc_crc & 0xFF;
    }

    //printf("\n");

    // We skip all of the response bytes
    isospi_write_read_blocking(tx_buf, NULL, 50, 50);
}

static const uint32_t READ_COMMANDS[] = {
    BMB3Y_CMD_READ_A,
    BMB3Y_CMD_READ_B,
    BMB3Y_CMD_READ_C,
    BMB3Y_CMD_READ_D,
    BMB3Y_CMD_READ_E
};

bool bmb3y_read_cell_voltage_bank_blocking(bms_model_t *model, int bank_index) {
    uint8_t rx_buf[72];
    uint32_t cmd = READ_COMMANDS[bank_index];
    if (!bmb3y_long_command_get_data_blocking(cmd, rx_buf, 72)) {
        printf("BMB3Y read failed for cmd 0x%02lX\n", cmd);
        return false;
    }

    uint8_t cell_offset = bank_index * 3;
    bool ret = true;

    for(int module=0; module<NUM_MODULE_VOLTAGES; module++) {
        uint16_t module_crc = (uint16_t)(rx_buf[module * 9 + 6] << 8) | (uint16_t)(rx_buf[module * 9 + 7]);
        uint16_t calc_crc = crc14(&rx_buf[module * 9], 6, 0x1000);

        if(module==0 && module_crc != calc_crc) {
            printf("CRC: %s %d %02X %02X %02X %02X %02X %02X %02X %02X %02X calc %04X\n",
                module_crc == calc_crc ? "OK" : "FAIL",
                bank_index,
                rx_buf[module * 9 + 0],
                rx_buf[module * 9 + 1],
                rx_buf[module * 9 + 2],
                rx_buf[module * 9 + 3],
                rx_buf[module * 9 + 4],
                rx_buf[module * 9 + 5],
                rx_buf[module * 9 + 6],
                rx_buf[module * 9 + 7],
                rx_buf[module * 9 + 8],
                calc_crc
            );
        }

        if(module==0 && module_crc != calc_crc) {
            ret = false;
        }

        // If some modules have fewer than 3 cells per bank, we will need to:
        //   detect/skip them (we probably still need to read them even if unset?)
        //   calculate the stride (currently module*15) taking into account missing cells

        // Go through each cell (three per bank)
        for(int cell=0; cell<3; cell++) {
            int cell_index = (module * 15) + cell_offset + cell;

            uint16_t voltage = 
                (uint16_t)(rx_buf[module * 9 + cell * 2]) |
                (uint16_t)(rx_buf[module * 9 + cell * 2 + 1] << 8);

            if(voltage == 0xFFFF) {
                // A non-existent cell
                model->cell_voltage_mV[cell_index] = -1;
            } else if(voltage == 0) {
                // Record as 1mV to distinguish from unmeasured
                model->cell_voltage_mV[cell_index] = 1;
            } else {
                model->cell_voltage_mV[cell_index] = (voltage * 2) / 25;
            }
        }
    }
    return ret;
}

bool bmb3y_read_temperatures_blocking(bms_model_t *model) {
    uint8_t rx_buf[90];

    //isosnoop_flush();

    if(!bmb3y_short_command_get_data_blocking(BMB3Y_CMD_READ_TEMPS3, rx_buf, 8)) {
        printf("BMB3Y temperature read failed\n");
        // printf("temptest failed hex: ");
        // for(int i=0;i<8;i++) {
        //     printf("%02X ", rx_buf[i]);
        // }
        // printf("\n");
        // isosnoop_print_buffer();
        return false;
    }

    bool crc_ok = true;
    for(int module=0; module<NUM_MODULE_TEMPS; module++) {
        // Each module has 8 bytes: 6 bytes data, 2 bytes CRC
        // The temp is in bytes 2 and 3 (little-endian)
        
        uint16_t module_crc = (uint16_t)(rx_buf[module * 8 + 6] << 8) | (uint16_t)(rx_buf[module * 8 + 7]);
        uint16_t calc_crc = crc14(&rx_buf[module * 8], 6, 0x0010);

        // printf("temptest gooded hex: ");
        // for(int i=0;i<8;i++) {
        //     printf("%02X ", rx_buf[module * 8 + i]);
        // }
        // printf("\n");
        //isosnoop_print_buffer();

        // ridiculous hack to deal with dodgy first bit
        // if(module_crc != calc_crc) {
        //     rx_buf[module * 8 + 0] ^= 0x80;
        //     calc_crc = crc14(&rx_buf[module * 8], 6, 0x0010);
        //     printf("temptest fixing...\n");
        // }


        if(module_crc != calc_crc) {
            printf("Bad Temp CRC on module %d: msg 0x%04X calc 0x%04X\n", module, module_crc, calc_crc);
            crc_ok = false;
            continue;
        }

        // D/T's algorithm:
        // // Unlike everything else, temps are little-endian??
        // int16_t raw_temp = 
        //     (int16_t)(rx_buf[module * 8 + 2]) |
        //     (int16_t)(rx_buf[module * 8 + 3] << 8);

        // // is this in dC? is the offset right? what about negative temps?
        // model->module_temperatures_dC[module] = raw_temp - 1131;
        
        int16_t raw_temp = 
             (int16_t)(rx_buf[module * 8 + 0]) |
             (int16_t)(rx_buf[module * 8 + 1] << 8);

        // super-crude cal, 28c = 0x4300, 100c = 0x6e00
        // FIXME - do proper thermistor conversion
        model->module_temperatures_dC[module] = ((raw_temp - 0x4300) * (1000 - 280)) / (0x6e00 - 0x4300) + 280;


        //printf("Module %d temp raw %d converted %d dC\n", module, raw_temp, model->module_temperatures_dC[module]);

    }

    if(crc_ok) {
        model->module_temperatures_millis = millis();
    }

    return crc_ok;
}

bool last_read_crc_failed = false;

void bmb3y_tick(bms_model_t *model) {
    // TODO - maybe split this in half, into separate 'read' and 'write' phases?
    // Otherwise we're writing out balancing during the read phase, (even though
    // it'll be related to data discovered on previous cycles anyway)

    // We do a full communication cycle every 64 ticks (1.28s).
    // We offset the steps by 5 to interleave with other BMS tasks.

    int step = (timestep() & 0x3f) - 5; // was 3f

    //uint32_t start = time_us_32();

    // if(step==1) {
    //     bmb3y_send_wakeup_cs_blocking();

    //     bmb3y_read_temperatures_blocking(model);

    // }
    // return;


    if(step==0) {
        // Wake up and request snapshot

        bmb3y_send_wakeup_cs_blocking();

        // The data packets received include CRC14s, although sometimes the BMB
        // gets into a state where the CRCs are all invalid (they're not even
        // CRC14s as they have higher bits set), even though the data looks
        // fine. It is possible to 'fix' this by sending extra IDLE_WAKE
        // commands before the SNAPSHOT command, for unknown reasons.

        // However this also depends on whether the balancing registers are
        // being written - if they are, then two IDLE_WAKEs usually works. If
        // however it gets 'out of sync', sending one IDLE_WAKE seems to fix it.

        // So current best strategy:
        // - always write balancing config after reading
        // - if any last read failed, do one IDLE_WAKE next time
        // - otherwise do two IDLE_WAKEs

        // Also seems to depend on how long we wait between cycles - 1.28s works
        // with this strategy, 5.12s doesn't...
        
        // read config
        //uint8_t rx_buf[72];
        //bmb3y_get_data_blocking(BMB3Y_CMD_READ_CONFIG, rx_buf, 72);
        // for(int i=0; i<72; i++) {
        //     printf("%02X ", rx_buf[i]);
        // }
        // printf("\n");

        if(last_read_crc_failed) {
            // Last read failed - try a single IDLE_WAKE to re-sync
            bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
        } else {
            // Normal case - two IDLE_WAKEs keep the CRCs happy
            bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
            bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
        }


        last_read_crc_failed = false;
        bmb3y_send_command_blocking(BMB3Y_CMD_SNAPSHOT);

        // As we wait longer than 5ms before the next command, the BMB will go
        // to sleep and need a wakeup. A single CS wakeup each time seems to work
        // fine, although need to check how multiple BMBs affect things.

        // uint32_t end = time_us_32();
        // printf("BMB3Y snapshot took %ld us\n", end - start);
    } else if(step>=1 && step <= 5) {
        // Read one of the five banks depending on the step number

        bmb3y_send_wakeup_cs_blocking();

        if(!bmb3y_read_cell_voltage_bank_blocking(model, step - 1)) {
            // Read failed
            last_read_crc_failed = true;
        } else if(step==5 && !last_read_crc_failed) {
            // All banks read successfully
            model->cell_voltage_millis = millis();
        }
        
        // uint32_t end = time_us_32();
        // printf("BMB3Y bank %d read took %ld us\n", step - 1, end - start);
    } else if(step==6) {
        // Module temperatures

        bmb3y_send_wakeup_cs_blocking();

        bmb3y_read_temperatures_blocking(model);

        // See what is in F...
        if(false) {
            uint8_t rx_buf[100];
            bmb3y_long_command_get_data_blocking(BMB3Y_CMD_READ_F, rx_buf, 20);
            printf("F hex: ");
            // supposedly bytes 2-3 (LE) might contain voltage, reads 0xabff at 56468mV tho
            // could be /0.8? pr *1.2825? or 12328mV offset? who knows...
            for(int i=0;i<20;i++) {
                printf("%02X ", rx_buf[i]);
            }
            printf("\n");
        }

    } else if(step==7) {
        // Enable/disable per-cell balancing as needed.

        if(true) {
            bmb3y_send_wakeup_cs_blocking();
            balancing_sm_tick(model);
            bmb3y_send_balancing(model);
        }
    }
}