#pragma once

// ck0: Rev1 with an external D+ pull-up powered from the host's USB VBUS

#include "boards/builtin/rev1.h"

#undef  USB_EXTERNAL_PULLUP
#define USB_EXTERNAL_PULLUP 1
