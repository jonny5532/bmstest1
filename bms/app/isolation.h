#pragma once

#include <stdbool.h>
#include <stdint.h>

// Isolation monitor: ADS1115 channels 4/5 measure battery+ and battery- to
// chassis through the HV dividers, biased around a zero point (the ADC count
// both channels read when the pack floats with no leakage).

typedef struct {
    float pos_to_chassis_V;  // battery+ relative to chassis
    float neg_to_chassis_V;  // battery- relative to chassis (normally negative)
    float resistance_ohms;   // leakage estimate; NAN unless the contactors are open
} isolation_result_t;

// Zero point in ADC counts at board temperature temp_dC. zero_ref is the
// calibrated zero referred to ISOLATION_ZERO_CAL_TEMP_DC; 0 means "use the
// board default".
float isolation_zero_counts(float zero_ref, int32_t temp_dC);

// Converts a zero measured at temp_dC into the stored (temperature-referred) form.
float isolation_zero_ref_from_counts(float zero_counts, int32_t temp_dC);

// Converts the raw channel counts into pack-to-chassis voltages and, with the
// contactors open, a single-sided leakage estimate. The divider ratio is taken
// from the calibrated battery voltage and the channel span, so only the zero
// point needs calibrating. Returns all NAN if the inputs are unusable.
isolation_result_t isolation_compute(float pos_raw, float neg_raw, float zero,
                                     float battery_V, bool contactors_open);

// Automatic zero check, run once per boot while the pack floats (contactors
// open, before the system starts operating). After the readings have settled it
// averages the channel midpoint; a small offset from the current zero is
// treated as drift and blended in, a large one as a possible leak (rejected,
// the zero is kept). Calibrating unconditionally would hide a leak, because a
// one-sided leak moves the midpoint.
typedef enum {
    ISOLATION_AUTOZERO_PENDING,   // still waiting / settling / averaging
    ISOLATION_AUTOZERO_ACCEPTED,  // *new_zero holds the blended zero
    ISOLATION_AUTOZERO_REJECTED,  // midpoint too far from the zero: possible leak
    ISOLATION_AUTOZERO_ABORTED,   // the boot window ended before a stable reading
    ISOLATION_AUTOZERO_DONE,      // already finished this boot
} isolation_autozero_status_t;

typedef struct {
    uint32_t usable_since_ms; // 0 = not currently usable
    float mid_sum;
    float mid_min;
    float mid_max;
    uint32_t samples;
    bool done;
} isolation_autozero_t;

// usable: contactors open, inputs valid and the system still in its boot /
// inactive states. window_over: the system has moved on (operating, fault, ...),
// so give up for this boot. *measured_mid is set when a stable average exists.
isolation_autozero_status_t isolation_autozero_step(isolation_autozero_t *az, uint32_t now_ms,
                                                    bool usable, bool window_over,
                                                    float pos_raw, float neg_raw, float current_zero,
                                                    float *new_zero, float *measured_mid);
