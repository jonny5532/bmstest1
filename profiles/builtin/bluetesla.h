#pragma once

// 96-cell NMC pack

#include "chemistry/nmc.h"

// Cell presence bitmask (1 = present, 0 = not present, 32-bit groups, LSB = cell 0)
#define CELL_PRESENCE_WORDS 0xc7ff8f7f, 0xf3ffe3ff, 0xfcfff8ff, 0x1ffe3d
#define NUM_CELLS 96
#define NUM_MODULES 8 // BMBs in the chain

// NTC thermistors: command 0x0D, word0 = A, word2 = B per BMB (bit0 = A, bit1 = B).
// Confirmed T16 from the isoSPI logs: every BMB reports 2 wired channels (all 16
// read ~18-20 C, none in the floating ~65-95 C band). The 0f7f cell map also ships
// as T8 (0,3,3,0,0,3,3,0) and T13 (2,3,3,3,2,3,3,2) variants for other packs.
#define NUM_THERMISTORS 16
#define THERMISTOR_PRESENCE 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3, 0x3
#define NAMEPLATE_CAPACITY_AH 210 // possibly; unclear what voltage range it's rated over
// Aged: three full-to-3.4V sweeps in Sep 2026 (kalman/replay) put it at ~173Ah on the G0 midpoint curve
#define ACTUAL_CAPACITY_AH 173
#define INA228_SHUNT_CAL 332
#define INVERTER_MODEL_STRING "CellKeeper Tesla NMC 1\x00\x00\x00\x00\x00"

// Absolute maximum current limits
#define CHARGE_MAX_CURRENT_dA 300 // 30A
#define DISCHARGE_MAX_CURRENT_dA 300 // 30A

// Trust the voltage more than the chemistry default does, now that the OCV curve
// fits this pack better (G0 midpoint). From replaying 10 days of 1s data and 40
// days of hourly data through ekf_tick: the worst error over the start and
// +/-150mA bias scenarios fell from 5.1% to 3.9% of capacity, and a correct start
// costs at most 0.4% more. Q_AH between 5e-7 and 1e-6 did equally well.
#undef  EKF_Q_AH
#define EKF_Q_AH 1e-6f
