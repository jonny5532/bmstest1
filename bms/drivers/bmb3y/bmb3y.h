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
print(list(((hex(n), hex(crc8_2f(bytes([n])))) for n in range(0, 0xFF))))
*/

// BATMan Protocol Commands (from original BatMan.cpp)
#define BMB3Y_CMD_WAKEUP       0x2AD4
#define BMB3Y_CMD_MUTE         0x20DD        // Pauses balancing
#define BMB3Y_CMD_IDLE_WAKE    0x21F2        // Also called UNMUTE (resumes balancing)
#define BMB3Y_CMD_SNAPSHOT     0x2BFB
#define BMB3Y_CMD_READ_A       0x47007000    // Cells 0-2
#define BMB3Y_CMD_READ_B       0x48003400    // Cells 3-5
#define BMB3Y_CMD_READ_C       0x4900DD00    // Cells 6-8
#define BMB3Y_CMD_READ_D       0x4A00C900    // Cells 9-11
#define BMB3Y_CMD_READ_E       0x4B002000    // Cells 12-14
#define BMB3Y_CMD_READ_F       0x4C00E100    // Chip total voltage?
#define BMB3Y_CMD_READ_AUX_A   0x4D000800    // Aux register A
#define BMB3Y_CMD_READ_AUX_B   0x4E001C00    // Aux register B
#define BMB3Y_CMD_READ_STATUS  0x4F00F500    // Status register
#define BMB3Y_CMD_READ_CONFIG  0x50009400    // Config register
#define BMB3Y_CMD_WRITE_CONFIG 0x112F        // Write config

// Short cell voltage reads, as used by the real BMS. Each BMB replies with
// eight bytes: three little-endian cells and a CRC14 with initial value 0x0010
// (as for every short-form reply). The long reads above instead carry a tag
// byte (0x00) before each BMB's data, which the CRC covers: initial value
// 0x1000 over the data is the same as 0x0010 over the tag and the data.
#define BMB3Y_CMD_READ_A_SHORT 0x0763
#define BMB3Y_CMD_READ_B_SHORT 0x08F9
#define BMB3Y_CMD_READ_C_SHORT 0x09D6
#define BMB3Y_CMD_READ_D_SHORT 0x0AA7
#define BMB3Y_CMD_READ_E_SHORT 0x0B88
#define BMB3Y_CMD_READ_F_SHORT 0x0C45

// Mysterious command found in sniffing, that begins every read sequence, seems
// to be some kind of wakeup. Does ensure that a subsequent SNAPSHOT actually
// works, after a long sleep.
#define BMB3Y_CMD_HELLO        0x2710

// Module ID (register 0x33). Each BMB replies with a 2-byte little-endian word:
// an 11-bit module ID plus 5 Hamming SECDED check bits (bits 11-15, see
// module_id_check_bits() in crc.h). Registers 0x83/0x93 have the same format.
#define BMB3Y_CMD_READ_MODULE_ID_SHORT       0x3302      // primary: 4 bytes per BMB, CRC14 init 0x0010
#define BMB3Y_CMD_READ_MODULE_ID             0x7300CA00  // long form, tag 0x00: 5 bytes per BMB, CRC14 init 0x1000
#define BMB3Y_CMD_READ_MODULE_ID_BANK1_SHORT 0x833C      // register 0x83
#define BMB3Y_CMD_READ_MODULE_ID_BANK2_SHORT 0x9392      // register 0x93
#define BMB3Y_CMD_PATTERN_5555               0x254E      // test pattern; may be needed before reading 0x33
#define BMB3Y_CMD_PATTERN_AAAA               0x263F      // test pattern (pairs with 0x30-0x32 reads)

#define BMB3Y_MODULE_ID_MASK 0x07FF
#define BMB3Y_MODULE_ID_CHECK_SHIFT 11

#define BMB3Y_CMD_READ_TEMPS   0x0E1B
#define BMB3Y_CMD_READ_TEMPS2  0x0F34 // returns 4 + CRC?
  // Temp hex: FF 40 C0 93 15 AF FF FF 
  // Temp hex: FF 40 C0 0A 10 88 FF FF 
  // Temp hex: FF 40 C0 81 15 01 FF FF 
  // Temp hex: FF 40 C0 F8 28 4F FF FF 
  // Temp hex: FF 40 C0 6F 13 3B FF FF 
  // Temp hex: FF 40 C0 E6 04 6A FF FF 
  // Temp hex: FF 40 C0 5D 37 79 FF FF 
  // Temp hex: FF 40 C0 D5 29 44 FF FF 
  // Temp hex: FF 40 C0 4B 12 67 FF FF 
  // Temp hex: FF 40 C0 C3 0C 5A FF FF 
  // Temp hex: FF 40 C0 3A 26 12 FF FF 
  // Temp hex: FF 40 C0 B1 23 9B FF FF 
#define BMB3Y_CMD_READ_TEMPS3  0x0D6A // often fails to read, 6 + CRC  6a
  // but got hex: 15 42 2D FA 56 5D 20 46 
  // but got hex: 0B 42 23 FA AE 5D 37 A6 
  // but got hex: 0D 42 25 FA 5C 5D 3F AE 
  // but got hex: 1D 42 23 FA 58 5D 36 D4 
  // but got hex: 1B 42 1F FA 56 5D 3A A7 
  // but got hex: 23 42 15 FA 5C 5D 3A EA 
  // but got hex: 2D 42 21 F9 58 5D 22 E7 
  // but got hex: 2B 42 21 FA 5A 5D 34 8A 
  // but got hex: 31 42 2B FA 56 5D 1B 9B 
  // but got hex: 3B 42 23 FA 58 5D 27 CE 
  // but got hex: 45 42 27 FA 58 5D 00 CF 
  // but got hex: 43 42 2F FA 56 5D 19 1F
// seems temp is in 0,1, LE - 0x6F00 if 100cish (3.99V), 0x4300 at 30ish (2.44V)
// (divide by 7ish to get voltage, then solve for temp)

#define BMB3Y_CMD_READ_TEMPS4  0x0C45 // 4 data + 2 byte CRC?
  // but got hex: 01 00 06 AC 24 A6 FF FF 
  // but got hex: 02 00 07 AC 29 52 FF FF 
  // but got hex: 01 00 07 AC 0A AA FF FF 
  // but got hex: 02 00 07 AC 29 52 FF FF 
  // but got hex: 02 00 07 AC 29 52 FF FF 
  // but got hex: 02 00 06 AC 07 5E FF FF 
  // but got hex: 02 00 06 AC 07 5E FF FF 
  // but got hex: 01 00 06 AC 24 A6 FF FF 
  // but got hex: 02 00 06 AC 07 5E FF FF 
  // but got hex: 02 00 06 AC 07 5E FF FF


//#define BMB3Y_CMD_READ_TEMPS   0x0E1B0000

// extra RE'd ones:

// 10 00 seems to be a config read as well?
// 07 63 seems to read 6 bytes + CRC over 8 modules, values mostly 51130-51137 etc - oh, it is cells!
//    ba c7 bf c7 b8 c7 62 39 
//    c1 c7 c0 c7 bb c7 11 a6 
//    ca c7 c4 c7 bd c7 8e f2 
//    e4 c7 e7 c7 e8 c7 be 09 
//    b9 c7 b6 c7 bd c7 27 37 
//    b4 c7 b1 c7 bb c7 a8 74 
//    c6 c7 c1 c7 c6 c7 08 f7 
//    b6 c7 b5 c7 a6 c7 64 26 01
// 08 f9 is more cells
// 09 d6 likewise
// 0a a7 more cells
// 0b 88 more
// 0c 45 has extraneous stuff
// 0f 34

//various writes:
// 14 bc  (four data bytes (all zero) + CRC, for each module)
// 15 93  ditto
// 11 2f  looks like write config - yes it is

// 27 10  is interesting, gets sent before every IDLE_WAKE


void bmb3y_tick(bms_model_t *model);
// Print the last reading of every BMB channel, populated or not
void bmb3y_print_physical_cells(void);
// Read the module ID of every BMB into the model (module_ids_*). Wakes the
// BMBs itself. Returns true if at least one BMB gave a valid ID.
bool bmb3y_read_module_ids_blocking(bms_model_t *model);
// Print the last module ID read of every BMB slot, then the pack verdict
void bmb3y_print_module_ids(const bms_model_t *model);
// Number of BMB channels whose last reading is a plausible cell voltage, or -1
// before the first cell reading
int bmb3y_count_live_channels(void);
// Number of BMBs that passed the CRC on every bank of the last full cell read,
// or -1 before the first one
int bmb3y_count_cell_modules(void);
