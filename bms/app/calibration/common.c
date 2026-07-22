#include "common.h"
#include "sys/logging/logging.h"
#include <math.h>

bool calibrate_battery_voltage(bms_model_t *model, int32_t raw_val, float sample_count) {
    float mul = (float)model->cell_voltage_total_mV * 0.001f * sample_count / (float)raw_val;

    if(mul > MIN_CALIBRATION_MUL && mul < MAX_CALIBRATION_MUL) {
        info_printf("Battery voltage calibration complete: raw=%ld mul=%f cvt=%ld\n", raw_val, mul, model->cell_voltage_total_mV);
        model->battery_voltage_mul = mul;
        return true;
    } else {
        error_printf("Battery voltage calibration out of range: %f\n", mul);
        return false;
    }
}

bool calibrate_output_voltage(bms_model_t *model, int32_t raw_val, float sample_count) {
    float mul = (float)model->cell_voltage_total_mV * 0.001f * sample_count / (float)raw_val;

    if(mul > MIN_CALIBRATION_MUL && mul < MAX_CALIBRATION_MUL) {
        info_printf("Output voltage calibration complete: raw=%ld mul=%f\n", raw_val, mul);
        model->output_voltage_mul = mul;
        return true;
    } else {
        error_printf("Output voltage calibration out of range: %f\n", mul);
        return false;
    }
}

bool calibrate_neg_contactor(bms_model_t *model, int32_t raw_val, float sample_count) {
    // lazily split the difference for now, not quite right but it'll do
    model->neg_contactor_mul = (model->battery_voltage_mul + model->output_voltage_mul) / 2.0f;

    // calculate the offset
    float neg_mV = (float)raw_val * model->neg_contactor_mul * 1000.0f / sample_count;

    debug_printf("Raw negcal=%ld neg_mV=%f, neg_contactor_mul=%f\n", raw_val, neg_mV, model->neg_contactor_mul);

    if(neg_mV > MIN_NEG_CONTACTOR_OFFSET_MV && neg_mV < MAX_NEG_CONTACTOR_OFFSET_MV) {
        info_printf("Neg contactor calibration complete: raw=%ld neg_mV=%f\n", raw_val, neg_mV);
        model->neg_contactor_offset_mV = -neg_mV;
        return true;
    } else {
        error_printf("Neg contactor calibration out of range: %f mV\n", neg_mV);
        return false;
    }
}

#if BMS_BOARD == BMS_BOARD_REV1

bool calibrate_link_voltage(bms_model_t *model, int32_t raw_val, float sample_count) {
    // Channel 7 measures LinkNeg - LinkPos, so negate to get the link voltage.
    // With the contactors closed the link rail sits at the battery voltage.
    float mul = (float)model->cell_voltage_total_mV * 0.001f * sample_count / (float)(-raw_val);

    if(mul > MIN_CALIBRATION_MUL && mul < MAX_CALIBRATION_MUL) {
        info_printf("Link voltage calibration complete: raw=%ld mul=%f\n", raw_val, mul);
        model->link_voltage_mul = mul;
        return true;
    } else {
        error_printf("Link voltage calibration out of range: %f\n", mul);
        return false;
    }
}

bool calibrate_pos_contactor(bms_model_t *model, int32_t raw_val, float sample_count) {
    // Rev1 measures directly across the pos contactor, which reads ~0V while
    // the contactors are closed, so calibrate the offset (like the neg
    // contactor). Must run after battery and link voltage calibration.

    // lazily split the difference for now, not quite right but it'll do
    model->pos_contactor_mul = (model->battery_voltage_mul + model->link_voltage_mul) / 2.0f;

    // calculate the offset
    float pos_mV = (float)raw_val * model->pos_contactor_mul * 1000.0f / sample_count;

    debug_printf("Raw poscal=%ld pos_mV=%f, pos_contactor_mul=%f\n", raw_val, pos_mV, model->pos_contactor_mul);

    if(pos_mV > MIN_POS_CONTACTOR_OFFSET_MV && pos_mV < MAX_POS_CONTACTOR_OFFSET_MV) {
        info_printf("Pos contactor calibration complete: raw=%ld pos_mV=%f\n", raw_val, pos_mV);
        model->pos_contactor_offset_mV = -pos_mV;
        return true;
    } else {
        error_printf("Pos contactor calibration out of range: %f mV\n", pos_mV);
        return false;
    }
}

#else

bool calibrate_pos_contactor(bms_model_t *model, int32_t raw_val, float sample_count) {
    float mul = (float)model->cell_voltage_total_mV * 0.001f * sample_count / (float)raw_val;

    if(mul > MIN_CALIBRATION_MUL && mul < MAX_CALIBRATION_MUL) {
        info_printf("Pos contactor calibration complete: raw=%ld mul=%f\n", raw_val, mul);
        model->pos_contactor_mul = mul;
        return true;
    } else {
        error_printf("Pos contactor calibration out of range: %f\n", mul);
        return false;
    }
}

#endif

bool calibrate_current_offset(bms_model_t *model, int32_t raw_offset) {
    if(raw_offset > MIN_CURRENT_OFFSET && raw_offset < MAX_CURRENT_OFFSET) {
        info_printf("Current calibration complete: offset=%ld\n", raw_offset);
        model->current_offset = raw_offset;
        return true;
    } else {
        error_printf("Current calibration out of range: %ld\n", raw_offset);
        return false;
    }
}
