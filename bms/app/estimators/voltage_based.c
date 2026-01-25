#include "estimators.h"
#include "../model.h"
#include "../../config/limits.h"

#include <stdint.h>

static float internal_resistance = 0.02f; // ohms

uint16_t voltage_based_soc_estimate(bms_model_t *model) {
    // Open-circuit voltage (OCV) approximation

    // calculate representative cell voltage
    uint16_t mean_voltage = (model->cell_voltage_total_mV / NUM_CELLS);
    float soc_estimate = nmc_ocv_to_soc((float)mean_voltage / 1000.0f);
    // Use min voltage for low SoC, max voltage for high SoC
    uint16_t representative_voltage_mV = (
        model->cell_voltage_min_mV + (model->cell_voltage_max_mV - model->cell_voltage_min_mV) * soc_estimate
    );

    //float cell_voltage_mV

    float ocv = (representative_voltage_mV + model->current_mA * internal_resistance) / 1000.0f;
    float soc = nmc_ocv_to_soc(ocv);
    return (uint16_t)(soc * 10000.0f); // in 0.01% units
}
