#pragma once

// 108-cell LFP pack

#include "chemistry/lfp.h"

// Cell presence bitmask (1 = present, 0 = not present, 32-bit groups, LSB = cell 0)
#define CELL_PRESENCE_WORDS 0x87ffbfff, 0xffffefff, 0xf9fffbff, 0x3ffeff
#define NUM_CELLS 108
#define NUM_MODULES 8 // BMBs in the chain
#define NUM_MODULE_TEMPS 8 // temperatures read, one per BMB for now
#define NAMEPLATE_CAPACITY_AH 173
// Six years old: replaying 10 days of 1s data (kalman/replay, Sep-Oct 2026) put it at ~159Ah
#define ACTUAL_CAPACITY_AH 159
#define INA228_SHUNT_CAL 332
#define INVERTER_MODEL_STRING "CellKeeper Tesla LFP 1\x00\x00\x00\x00\x00"

// Absolute maximum current limits
#define CHARGE_MAX_CURRENT_dA 500 // 50A
#define DISCHARGE_MAX_CURRENT_dA 500 // 50A
