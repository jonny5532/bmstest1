#pragma once

#include <stdbool.h>
#include <stdint.h>

// PCB revision strap on PIN_BOARD_REV (GPIO47 / ADC7):
//   Rev0     pin floating
//   Rev1.0   pin tied to GND (R76 = 0R)
//   Rev1.1+  10k to 3V3 (R75) and a per-revision R76 to GND:
//            1.1 1k, 1.2 2.2k, 1.3 5.1k, 1.4 10k, 1.5 24k, 1.6 47k, 1.7 100k
//
// Codes are major * 10 + minor, so Rev1.3 = 13.
#define BOARD_ID_REV0    0
#define BOARD_ID_UNKNOWN 0xFFFF

// A floating pin follows the internal pulls rail to rail; a strapped one moves
// by at most ~640 counts (the 100k case), because 10k outweighs the ~50k pulls.
#define BOARD_ID_FLOATING_SPAN_COUNTS 1500.0f

// Classify the strap from 12-bit ADC readings taken with the internal pull-up
// on, the pull-down on, and no pulls.
uint16_t board_id_classify(float pull_up_counts, float pull_down_counts, float counts);

// Whether a detected code fits the board this firmware was built for
// (BMS_BOARD). Every Rev1.x code fits a REV1 build.
bool board_id_matches_build(uint16_t code);
