#include "bmb3y.h"

#include "../model.h"
#include "../isospi/isospi_master.h"

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
    
    isospi_write_read_blocking(tx, rx, 2, 0);
}

bool bmb3y_get_data_blocking(uint32_t cmd, uint8_t *response, int response_len) {
    // Read data from BMBs into the supplied buffer.
    // Returns true if the read was successful.
    
    // TODO: Check whether we need to wake up?

    uint8_t tx[54] = {0};
    if(response_len > 50) {
        // Response buffer isn't large enough
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

void bmb3y_send_wakeup_cs_blocking() {
    isospi_send_wakeup_cs_blocking();
}

// Higher level functions

void bmb3y_wakeup_blocking(void) {
    bmb3y_send_wakeup_cs_blocking();

    // isospi_write_read_blocking(tx, rx, 2, 0);
    bmb3y_send_command_blocking(BMB3Y_CMD_WAKEUP);
}

void bmb3y_request_snapshot_blocking() {
    // Tell the BMBs to take a snapshot of cell voltages.

    bmb3y_wakeup_blocking();

    bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
    sleep_us(70);

    bmb3y_send_command_blocking(BMB3Y_CMD_SNAPSHOT);
}


// weird, so Voltages is a 2d array, with 8 rows and 15 columns. Each row must be a battery module, each column a cell within that?
// when we read a single bank, we get a value for every module, but only a restricted range of columns? specifically, 3 cells. weird!

bool bmb3y_read_test_blocking(uint32_t cmd, int cells) {
    uint8_t rx_buf[16];

    // uint8_t tx[34] = {0};
    // // tx[0] = (BMB3Y_CMD_READ_TEMPS >> 8) & 0xFF;
    // // tx[1] = BMB3Y_CMD_READ_TEMPS & 0xFF;
    // isospi_write_read_blocking((char*)tx, (char*)rx_buf, 34, 2, false);
    // for(int i=0; i<34; i++) {
    //     printf("%02X ", rx_buf[i]);
    // }
    // printf("\n");
    // return false;

    if (!bmb3y_get_data_blocking(cmd, rx_buf, 16)) {
        return false;
    }

    for(int i=0;i<cells;i++) {
        uint16_t raw = (uint16_t)(rx_buf[i * 2 + 1] << 8) | (uint16_t)(rx_buf[i * 2]);
        if(raw == 0xFFFF) {
            break;
        }
        uint16_t voltage = (raw * 2) / 25;
        printf("%dmV | ", voltage);
    }

    for(int i=0; i<16; i++) {
        printf("%02X ", rx_buf[i]);
    }
    printf("\n");

    uint16_t msg_crc = (uint16_t)(rx_buf[cells*2] << 8) | (uint16_t)(rx_buf[cells*2 + 1]);
    uint16_t calc_crc = crc14(rx_buf, cells * 2, 0x1000);

    if(msg_crc != calc_crc) {
        printf("Bad CRC!\n");
        return false;
    }
    //printf("BMB Message CRC: msg 0x%04X calc 0x%04X\n", msg_crc, calc_crc);

    return true;
}

bool bmb3y_set_balancing(uint8_t bitmap[15], bool even) {
    uint8_t tx_buf[50] = {0};
    
    tx_buf[0] = (BMB3Y_CMD_WRITE_CONFIG >> 8) & 0xFF;
    tx_buf[1] = BMB3Y_CMD_WRITE_CONFIG & 0xFF;

    uint8_t balance_mask = even ? 0x55 : 0xAA;

    for(int module=0; module<8; module++) {
        tx_buf[2 + module*6] = 0xF3; // not sure which way
        tx_buf[3 + module*6] = 0x00; // round these two go

        tx_buf[4 + module*6] = bitmap[module*2] & balance_mask;
        tx_buf[5 + module*6] = bitmap[module*2 + 1] & balance_mask;

        uint16_t calc_crc = crc14(tx_buf[2 + module*6], 4, 0x0010);

        tx_buf[6 + module*6] = (calc_crc >> 8) & 0xFF;
        tx_buf[7 + module*6] = calc_crc & 0xFF;
    }

    // We skip all of the response bytes
    isospi_write_read_blocking(tx_buf, NULL, 50, 50);

    return true;
}

bool bmb3y_read_cell_voltages_blocking(bms_model_t *model) {
    uint8_t rx_buf[72];

    //bmb3y_request_snapshot_blocking();

    // wait for the snapshot to be ready?

    uint8_t cell_offset = 0;
    
    const uint32_t READ_COMMANDS[] = {
        BMB3Y_CMD_READ_A,
        BMB3Y_CMD_READ_B,
        BMB3Y_CMD_READ_C,
        BMB3Y_CMD_READ_D,
        BMB3Y_CMD_READ_E
    };

    bool success = true;

    for(uint8_t i=0; i<5; i++) {
        uint32_t cmd = READ_COMMANDS[i];
        if (!bmb3y_get_data_blocking(cmd, rx_buf, 72)) {
            printf("BMB3Y read failed for cmd 0x%02X\n", cmd);
            success = false;
            continue;
        }

        // Each bank has data for all 8 modules, with 3 cells, a 2-byte CRC and a padding byte per module

        // Go through each module (0-7, 8 total)
        for(int module=0; module<8; module++) {

            uint16_t module_crc = (uint16_t)(rx_buf[module * 9 + 6] << 8) | (uint16_t)(rx_buf[module * 9 + 7]);
            uint16_t calc_crc = crc14(&rx_buf[module * 9], 6, 0x1000);

            if(module_crc != calc_crc) {
                //printf("Bad CRC on module %d!\n", module);
                success = false;
                continue;
                //return false;
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
                    model->cell_voltages_mV[cell_index] = -1;
                } else if(voltage == 0) {
                    // Record as 1mV to distinguish from unmeasured
                    model->cell_voltages_mV[cell_index] = 1;
                } else {
                    model->cell_voltages_mV[cell_index] = (voltage * 2) / 25;
                }
            }
        }

        cell_offset += 3;
    }

    if(success) {
        model->cell_voltages_millis = millis();
    }

    return success;
}
