#include "isolation.h"

#include "model.h"
#include "config/limits.h"
#include "sys/events/events.h"
#include "sys/time/time.h"

#include <math.h>
#include <stdint.h>

// The isolation channels use the ADS1115's 4.096 V range
#define ISOLATION_VOLTS_PER_COUNT (4.096f / 32768.0f)

isolation_reading_t isolation_compute(float pos_counts, float neg_counts, float zero_counts, float battery_V) {
    isolation_reading_t res = { NAN, NAN, NAN };

    float span = pos_counts - neg_counts;
    if (!(battery_V > ISOLATION_MIN_BATTERY_V) || !(span > 0.0f)) {
        return res;
    }

    // Volts at the pack per ADC count: the divider ratio times the ADC LSB
    float pack_volts_per_count = battery_V / span;
    res.pos_to_chassis_V = (pos_counts - zero_counts) * pack_volts_per_count;
    res.neg_to_chassis_V = (neg_counts - zero_counts) * pack_volts_per_count;

    // A terminal at or past chassis (a or b <= 0) is a hard short: |k| = 1
    float k = ((pos_counts - zero_counts) - (zero_counts - neg_counts)) / span;
    k = fmaxf(-1.0f, fminf(1.0f, k));

    // Each divider's resistance to chassis is its ratio times the bottom resistor
    float ratio = pack_volts_per_count / ISOLATION_VOLTS_PER_COUNT;
    float rd = ratio * ISOLATION_DIVIDER_BOTTOM_OHMS;
    float abs_k = fabsf(k);
    float r = ISOLATION_RESISTANCE_CAP_OHMS;
    if (rd * (1.0f - abs_k) < 2.0f * abs_k * ISOLATION_RESISTANCE_CAP_OHMS) {
        r = rd * (1.0f - abs_k) / (2.0f * abs_k);
    }
    res.resistance_ohms = r;
    return res;
}

bool isolation_is_low(float resistance_ohms, float battery_V) {
    return resistance_ohms < battery_V * ISOLATION_LOW_OHMS_PER_V;
}

void isolation_update(bms_model_t *model, float pos_counts, float neg_counts, bool contactors_open_settled) {
    float battery_V = model->high_voltages.battery;
    isolation_reading_t iso = isolation_compute(pos_counts, neg_counts, get_isolation_zero_counts(model), battery_V);
    model->isolation_pos_to_chassis_V = iso.pos_to_chassis_V;
    model->isolation_neg_to_chassis_V = iso.neg_to_chassis_V;

    if (!contactors_open_settled || isnan(iso.resistance_ohms)) {
        return;
    }

    model->isolation_open_resistance_ohms = iso.resistance_ohms;
    model->isolation_open_millis = millis() ? millis() : 1;

    // Uncalibrated, the default zero can be far enough off to look like a
    // leak, so only raise warnings if we're calibrated
    if (isolation_zeroed(model)) {
        confirm(!isolation_is_low(iso.resistance_ohms, battery_V), ERR_ISOLATION_LOW,
                (uint32_t)iso.resistance_ohms);
    }
}
