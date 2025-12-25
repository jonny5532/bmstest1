#include "bmb3y.h"

#include "../model.h"
#include "../isospi/isospi_master.h"

#include <stdint.h>

const uint8_t CELLS_PER_MODULE = 15;
const uint8_t CELLS_PER_BANK = 3;
const uint8_t MODULE_COUNT = 8;

void bmb3y_wakeup_blocking(void) {
    // TODO - check whether we actually need to invert the first CS pulse

    // Send wakeup CS pulses (CS1 CS0 pattern) with WAKEUP command
    // uint8_t tx[2] = {0x2A, 0xD4};  // CMD_WAKEUP
    // uint8_t rx[2] = {0};

    // isospi_write_read_blocking(tx, rx, 2, 0);
    isospi_send_command_blocking(BMB3Y_CMD_WAKEUP);
}

void bmb3y_request_snapshot_blocking() {
    // Tell the BMBs to take a snapshot of cell voltages.

    isospi_wakeup_blocking();

    isospi_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
    sleep_us(70);

    isospi_send_command_blocking(BMB3Y_CMD_SNAPSHOT);
}

bool bmb3y_get_data_blocking(uint8_t cmd, uint8_t *buf, int len) {
    // Read data from BMBs into the supplied buffer.
    
    // Returns true if the read was successful.

    // TODO: Check whether we need to wake up?

    isospi_send_command_blocking(cmd);
    return isospi_get_data_blocking(cmd, buf, len);
}

// weird, so Voltages is a 2d array, with 8 rows and 15 columns. Each row must be a battery module, each column a cell within that?
// when we read a single bank, we get a value for every module, but only a restricted range of columns? specifically, 3 cells. weird!


bool bmb3y_read_cell_voltages_blocking() {
    uint8_t rx_buf[74];

    bmb3y_request_snapshot_blocking();

    uint8_t cell_offset = 0;

    // Read cell groups A to E (3 cells each)
    for (uint8_t cmd = BMB3Y_CMD_READ_A; cmd <= BMB3Y_CMD_READ_E; cmd++) {
        printf("Reading BMB3Y cell voltages with cmd 0x%02X\n", cmd);
        if (!bmb3y_get_data_blocking(cmd, rx_buf, 74)) {
            // Handle read error
            printf("BMB3Y read failed for cmd 0x%02X\n", cmd);
            return false;
        }

        for(int i=0; i<74; i++) {
            printf("%02X ", rx_buf[i]);
        }
        printf("\n");




        // It is not immediately clear whether the data is big-endian or little-endian...

        // The data format is very strange!

        // Performing, say, CMD_READ_A instructs every BMB to sample cells 0-3.
        // The readback then consists of the cells from the first BMB, then the same group in the next BMB, etc.
        // However, a module read seems to be 9 bytes long - two bytes per cell (6 bytes), plus a 2-byte PEC, plus a strange extra byte?

        for(uint8_t module=0; module<9; module++) {
            for(uint8_t cell_in_bank=0; cell_in_bank<3; cell_in_bank++) {
                
                uint8_t cell_index = (module * CELLS_PER_MODULE) + cell_offset + cell_in_bank;

                uint16_t voltage = 
                    (uint16_t)(rx_buf[module * 9 + cell_in_bank * 2] << 8) |
                    (uint16_t)(rx_buf[module * 9 + cell_in_bank * 2 + 1]);

                // TODO, what to do about 0xffff voltages?

                model.cell_voltages_mV[cell_index] = voltage;
            }
            
        }
        cell_offset += 3;       
    }

}

// void batman_query_blocking() {

//     uint8_t buf[32] = {0};
//     isospi_get_data_blocking(BATMAN_CMD_READ_A, buf, 12);
// }

/*

WriteCfg

uint8_t msg = {
    0x112f, // check which way round this 16-bit value is sent
/   0xf300,
|   0x0000, // 16-bit balance bitmask (1=balance)
\   crc14
    ... then repeat that triplet for the other 7 BMBs
} (25 words in all)



*/