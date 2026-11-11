#pragma once

// Rev1.1 PCB, unmodified: Rev1 with USB VBUS wired to GPIO3 through a
// 3.3k/5.1k divider on the board (on both the BMS and the supervisor MCU)

#include "boards/builtin/rev1.h"

#undef  USB_VBUS_SENSE
#define USB_VBUS_SENSE 1
