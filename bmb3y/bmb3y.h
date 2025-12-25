#include <stdbool.h>
#include <stdint.h>

// BATMan Protocol Commands (from original BatMan.cpp)
#define BMB3Y_CMD_WAKEUP       0x2AD4
#define BMB3Y_CMD_MUTE         0x20DD
#define BMB3Y_CMD_IDLE_WAKE    0x21F2  // Also called UNMUTE - makes IC responsive
#define BMB3Y_CMD_SNAPSHOT     0x2BFB
#define BMB3Y_CMD_READ_A       0x47    // Cells 0-2
#define BMB3Y_CMD_READ_B       0x48    // Cells 3-5
#define BMB3Y_CMD_READ_C       0x49    // Cells 6-8
#define BMB3Y_CMD_READ_D       0x4A    // Cells 9-11
#define BMB3Y_CMD_READ_E       0x4B    // Cells 12-14
#define BMB3Y_CMD_READ_F       0x4C    // Chip total voltage
#define BMB3Y_CMD_READ_AUX_A   0x4D    // Aux register A
#define BMB3Y_CMD_READ_AUX_B   0x4E    // Aux register B
#define BMB3Y_CMD_READ_STATUS  0x4F    // Status register
#define BMB3Y_CMD_READ_CONFIG  0x50    // Config register
#define BMB3Y_CMD_WRITE_CONFIG 0x11    // Write config

void bmb3y_request_snapshot_blocking();
bool bmb3y_get_data_blocking(uint8_t cmd, uint8_t *buf, int len);
