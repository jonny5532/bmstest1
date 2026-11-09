#pragma once

#include <stdbool.h>

typedef struct bms_model bms_model_t;

// Isolation monitor. ADS1115 channels 4/5 read pack+ and pack- to chassis
// through the HV dividers (Rd, about 12M each), biased around a zero point: the
// count both channels read when the pack floats with no leakage.
//
// Each channel's offset from the zero, scaled by battery V / channel span,
// gives that terminal's voltage to chassis: pack+ positive, pack- negative, and
// a floating pack with no leakage sits at +/- half the battery voltage.
//
// A leak R from one terminal to chassis, in parallel with that side's divider,
// pulls that terminal towards chassis. With a = pos - zero and b = zero - neg,
// k = (a - b) / (a + b) gives |k| = Rd / (Rd + 2R), so R = Rd (1 - |k|) / (2 |k|).
// Equal leaks on both sides don't unbalance the pack, so they are invisible.
//
// With the contactors closed, the inverter's battery port sets the chassis
// voltage, so the voltages describe the inverter rather than the pack and the
// resistance is meaningless.

typedef struct {
    float pos_to_chassis_V; // NaN if the readings are unusable
    float neg_to_chassis_V; // NaN if the readings are unusable
    float resistance_ohms;  // capped at ISOLATION_RESISTANCE_CAP_OHMS; NaN if unusable
} isolation_reading_t;

// Rd is derived from the battery voltage and the channel span, so this works
// for any divider ratio.
isolation_reading_t isolation_compute(float pos_counts, float neg_counts, float zero_counts, float battery_V);

// Updates the model with the latest isolation readings.
void isolation_update(bms_model_t *model, float pos_counts, float neg_counts, bool contactors_open_settled);

// Whether an open-contactor resistance is below the warning threshold
bool isolation_is_low(float resistance_ohms, float battery_V);
