#include "lifetime.h"
#include "app/model.h"

// Once *acc passes 1.0 or -1.0, transfer the whole units to the in or out total.
// The totals wrap at 2^32 (unsigned arithmetic) rather than saturating.
static void transfer(double *acc, uint32_t *in, uint32_t *out) {
    double mag = *acc >= 0.0 ? *acc : -*acc;
    if (mag < 1.0) return;
    // Not a real step (and a NaN, which fails every comparison, would otherwise
    // stay in the accumulator for good): drop it
    if (!(mag < 4294967296.0)) {
        *acc = 0.0;
        return;
    }
    uint32_t whole = (uint32_t)mag;
    if (*acc > 0.0) {
        *in += whole;
        *acc -= whole;
    } else {
        *out += whole;
        *acc += whole;
    }
}

void lifetime_accumulate(bms_model_t *model, double charge_Ah, double dt_s, double bias_A, double pack_V) {
    lifetime_accum_t *acc = &model->lifetime;

    double dq_Ah = charge_Ah - bias_A * dt_s / 3600.0;
    acc->acc_mAh += dq_Ah * 1000.0;
    acc->acc_Wh += dq_Ah * pack_V;

    transfer(&acc->acc_mAh, &model->lifetime_charged_mAh, &model->lifetime_discharged_mAh);
    transfer(&acc->acc_Wh, &model->lifetime_charged_Wh, &model->lifetime_discharged_Wh);
}
