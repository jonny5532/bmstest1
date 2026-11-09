#pragma once

// Rev0 PCB, unmodified

#define BMS_BOARD_REVISION BMS_BOARD_REV0

// One ADS1115 (0x48)
#define HAS_ADS1115_SECONDARY() 0
// ~416:1 HV dividers
#define ADS1115_DEFAULT_MUL 0.013f
// Isolation monitor zero (ADC counts with the pack floating), used until the
// board is zeroed (calibrate isolation)
#define ISOLATION_ZERO_DEFAULT_COUNTS 22496.0f

// Board modifications (see boards/README.md)
#define USB_VBUS_SENSE 0
#define USB_EXTERNAL_PULLUP 0
