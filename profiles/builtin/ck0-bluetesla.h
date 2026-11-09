#pragma once

// ck0: the BlueTesla pack on a board modified with an external VBUS-powered
// USB D+ pull-up (see config/limits.h). Only for that board: an unmodified one
// never connects over USB with this profile, so use bluetesla there.

#include "builtin/bluetesla.h"

#define USB_EXTERNAL_PULLUP 1
