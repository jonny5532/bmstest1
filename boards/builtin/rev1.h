#pragma once

// Rev1 PCB, unmodified

#define BMS_BOARD_REVISION BMS_BOARD_REV1

// A second ADS1115 (0x49) senses the Link and Drive Unit rails
#define HAS_ADS1115_SECONDARY() 1
// 501:1 HV dividers (12M/24k): 501 * 1.024/32768 V per count
#define ADS1115_DEFAULT_MUL 0.015656f
// Isolation monitor zero (ADC counts with the pack floating), measured on one
// board; used until the board is zeroed (calibrate isolation)
#define ISOLATION_ZERO_DEFAULT_COUNTS 13317.0f

// Board modifications (see boards/README.md)
#define USB_VBUS_SENSE 0
#define USB_EXTERNAL_PULLUP 0
