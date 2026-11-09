#pragma once

#include <stdint.h>

typedef struct bms_model bms_model_t;

// Lifetime charge and energy throughput, in and out of the pack.
//
// The main totals are stored in the 'fast' persistent data of the model.
// This accumulator holds the fractional part of the charge and energy that has
// not yet been transferred to the totals.
//
// What's below a unit is lost on reboot (under 1mAh/1Wh per total).
typedef struct {
    double acc_mAh; // + = charging
    double acc_Wh;
} lifetime_accum_t;

// Add a measured charge delta (Ah, + = charging, as from the INA228 count, so
// including the sensor bias) over dt_s seconds. bias_A is the current sensor
// bias to take out (measured = true + bias), pack_V the pack voltage.
void lifetime_accumulate(bms_model_t *model, double charge_Ah, double dt_s, double bias_A, double pack_V);
