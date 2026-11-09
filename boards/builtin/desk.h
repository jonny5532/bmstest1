#pragma once

// The desk bench board: Rev1 with USB VBUS wired to GPIO3 (J12 pin 2) through
// a 3.3k/5.1k divider

#include "boards/builtin/rev1.h"

#undef  USB_VBUS_SENSE
#define USB_VBUS_SENSE 1
