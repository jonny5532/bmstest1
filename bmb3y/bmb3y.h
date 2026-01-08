#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct bms_model bms_model_t;

// Read commands have a CRC8(poly 0x2F, initial 0x10) of the first two bytes in the third byte. 
// This is precalculated in the read commands.

// The other commands are single byte followed by the CRC8 byte.

/*
def crc8_2f(data: bytes) -> int:
    crc = 0x10  # Initial value
    poly = 0x2F # The polynomial
    
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                # If MSB is set, shift left and XOR with poly
                crc = (crc << 1) ^ poly
            else:
                # Otherwise, just shift left
                crc = crc << 1
            
            crc &= 0xFF
            
    return crc

print(list((hex(crc8_2f(bytes([n, 0]))) for n in range(0x47, 0x4C+10))))
*/

// BATMan Protocol Commands (from original BatMan.cpp)
#define BMB3Y_CMD_WAKEUP       0x2AD4
#define BMB3Y_CMD_MUTE         0x20DD
#define BMB3Y_CMD_IDLE_WAKE    0x21F2  // Also called UNMUTE - makes IC responsive
#define BMB3Y_CMD_SNAPSHOT     0x2BFB
#define BMB3Y_CMD_READ_A       0x47007000    // Cells 0-2
#define BMB3Y_CMD_READ_B       0x48003400    // Cells 3-5
#define BMB3Y_CMD_READ_C       0x4900DD00    // Cells 6-8
#define BMB3Y_CMD_READ_D       0x4A00C900    // Cells 9-11
#define BMB3Y_CMD_READ_E       0x4B002000    // Cells 12-14
#define BMB3Y_CMD_READ_F       0x4C00E100    // Chip total voltage
#define BMB3Y_CMD_READ_AUX_A   0x4D000800    // Aux register A
#define BMB3Y_CMD_READ_AUX_B   0x4E001C00    // Aux register B
#define BMB3Y_CMD_READ_STATUS  0x4F00F500    // Status register
#define BMB3Y_CMD_READ_CONFIG  0x50009400    // Config register
#define BMB3Y_CMD_WRITE_CONFIG 0x112F        // Write config
#define BMB3Y_CMD_READ_TEMPS   0x0E1B


void bmb3y_tick(bms_model_t *model);

void bmb3y_send_command_blocking(uint16_t cmd_word);
bool bmb3y_get_data_blocking(uint32_t cmd, uint8_t *buf, int len);

void bmb3y_wakeup_blocking(void);
void bmb3y_request_snapshot_blocking();
bool bmb3y_read_test_blocking(uint32_t cmd, int cells);
bool bmb3y_read_cell_voltages_blocking(bms_model_t *model);
bool bmb3y_read_cell_voltage_bank_blocking(bms_model_t *model, int bank_index);
void bmb3y_send_balancing(bms_model_t *model);