#include "isolation.h"

#include "config/limits.h"

#include <math.h>

// ADS1115 at the 4.096 V PGA setting
#define ISOLATION_VOLTS_PER_COUNT (4.096f / 32768.0f)

float isolation_zero_counts(float zero_ref, int32_t temp_dC) {
    float ref = zero_ref > 0.0f ? zero_ref : ISOLATION_ZERO_DEFAULT_COUNTS;
    return ref + ((float)temp_dC - ISOLATION_ZERO_CAL_TEMP_DC) * ISOLATION_ZERO_TEMPCO_COUNTS_PER_DC;
}

float isolation_zero_ref_from_counts(float zero_counts, int32_t temp_dC) {
    return zero_counts - ((float)temp_dC - ISOLATION_ZERO_CAL_TEMP_DC) * ISOLATION_ZERO_TEMPCO_COUNTS_PER_DC;
}

isolation_result_t isolation_compute(float pos_raw, float neg_raw, float zero,
                                     float battery_V, bool contactors_open) {
    isolation_result_t res = { NAN, NAN, NAN };

    float span = pos_raw - neg_raw;
    if (!(span > ISOLATION_MIN_SPAN_COUNTS) || !(battery_V > ISOLATION_MIN_BATTERY_V)) {
        return res;
    }

    // Volts at the pack per ADC count, i.e. divider ratio * ADC LSB
    float volts_per_count = battery_V / span;
    res.pos_to_chassis_V = (pos_raw - zero) * volts_per_count;
    res.neg_to_chassis_V = (neg_raw - zero) * volts_per_count;

    if (!contactors_open) {
        // Closed, the inverter's non-isolated battery port sets the chassis
        // voltage, so the readings say nothing about the pack's insulation.
        return res;
    }

    // Each terminal sees its divider (rd) to chassis. A leak in parallel on one
    // side unbalances the channels: r = Vpos/|Vneg|; r < 1 -> pack+ leak
    // rd*r/(1-r), r > 1 -> pack- leak rd/(r-1). Equal leaks on both sides are
    // invisible to this method.
    float rd = (volts_per_count / ISOLATION_VOLTS_PER_COUNT) * ISOLATION_DIVIDER_BOTTOM_OHMS;
    float a = pos_raw - zero;
    float b = zero - neg_raw;
    float r_ohms;
    if (a <= 0.0f || b <= 0.0f) {
        r_ohms = 0.0f; // a terminal at or past chassis: hard short
    } else {
        float r = a / b;
        if (r < 1.0f) {
            r_ohms = rd * r / (1.0f - r);
        } else if (r > 1.0f) {
            r_ohms = rd / (r - 1.0f);
        } else {
            r_ohms = ISOLATION_RESISTANCE_CAP_OHMS;
        }
    }
    res.resistance_ohms = fminf(r_ohms, ISOLATION_RESISTANCE_CAP_OHMS);
    return res;
}

static void autozero_restart(isolation_autozero_t *az, uint32_t now_ms) {
    az->usable_since_ms = now_ms ? now_ms : 1;
    az->mid_sum = 0.0f;
    az->samples = 0;
}

isolation_autozero_status_t isolation_autozero_step(isolation_autozero_t *az, uint32_t now_ms,
                                                    bool usable, bool window_over,
                                                    float pos_raw, float neg_raw, float current_zero,
                                                    float *new_zero, float *measured_mid) {
    if (az->done) return ISOLATION_AUTOZERO_DONE;

    if (!usable) {
        az->usable_since_ms = 0;
        az->samples = 0;
        if (window_over) {
            az->done = true;
            return ISOLATION_AUTOZERO_ABORTED;
        }
        return ISOLATION_AUTOZERO_PENDING;
    }

    if (az->usable_since_ms == 0) autozero_restart(az, now_ms);
    uint32_t elapsed = now_ms - az->usable_since_ms;
    if (elapsed < ISOLATION_AUTOZERO_SETTLE_MS) return ISOLATION_AUTOZERO_PENDING;

    float mid = (pos_raw + neg_raw) / 2.0f;
    if (az->samples == 0) {
        az->mid_min = mid;
        az->mid_max = mid;
    }
    az->mid_sum += mid;
    az->mid_min = fminf(az->mid_min, mid);
    az->mid_max = fmaxf(az->mid_max, mid);
    az->samples++;

    if (elapsed < ISOLATION_AUTOZERO_SETTLE_MS + ISOLATION_AUTOZERO_WINDOW_MS) return ISOLATION_AUTOZERO_PENDING;

    if (az->mid_max - az->mid_min > ISOLATION_AUTOZERO_MAX_NOISE_COUNTS) {
        // Still moving: start another settle + average period
        autozero_restart(az, now_ms);
        return ISOLATION_AUTOZERO_PENDING;
    }

    float mean = az->mid_sum / (float)az->samples;
    *measured_mid = mean;
    az->done = true;
    float diff = mean - current_zero;
    if (fabsf(diff) > ISOLATION_AUTOZERO_MAX_STEP_COUNTS) {
        return ISOLATION_AUTOZERO_REJECTED;
    }
    *new_zero = current_zero + diff * ISOLATION_AUTOZERO_BLEND;
    return ISOLATION_AUTOZERO_ACCEPTED;
}
