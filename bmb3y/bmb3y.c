#include "bmb3y.h"

#include "../model.h"
#include "../isospi/isospi_master.h"

#include <stdint.h>

const uint8_t CELLS_PER_MODULE = 15;
const uint8_t CELLS_PER_BANK = 3;
const uint8_t MODULE_COUNT = 8;

void bmb3y_wakeup_blocking(void) {
    isospi_send_wakeup_cs_blocking();

    // isospi_write_read_blocking(tx, rx, 2, 0);
    isospi_send_command_blocking(BMB3Y_CMD_WAKEUP);
}

void bmb3y_request_snapshot_blocking() {
    // Tell the BMBs to take a snapshot of cell voltages.

    bmb3y_wakeup_blocking();

    isospi_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
    sleep_us(70);

    isospi_send_command_blocking(BMB3Y_CMD_SNAPSHOT);
}

bool bmb3y_get_data_blocking(uint32_t cmd, uint8_t *buf, int len) {
    // Read data from BMBs into the supplied buffer.
    
    // Returns true if the read was successful.

    // TODO: Check whether we need to wake up?

    return isospi_get_data_blocking(cmd, buf, len);
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



bool bmb3y_read_cell_voltages_blocking() {
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

            // Go through each cell (three per bank)
            for(int cell=0; cell<3; cell++) {
                int cell_index = (module * 15) + cell_offset + cell;

                uint16_t voltage = 
                    (uint16_t)(rx_buf[module * 9 + cell * 2]) |
                    (uint16_t)(rx_buf[module * 9 + cell * 2 + 1] << 8);

                if(voltage == 0xFFFF) {
                    model.cell_voltages_mV[cell_index] = -1;
                } else {
                    model.cell_voltages_mV[cell_index] = (voltage * 2) / 25;
                }
            }
        }

        cell_offset += 3;
    }

    if(success) {
        model.cell_voltages_millis = millis();
    }

    return success;
}
