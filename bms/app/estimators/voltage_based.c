#include "estimators.h"
#include "ekf.h"
#include "../model.h"
#include "../../config/limits.h"

#include <stdint.h>

// SOC read straight off the voltage, scaled to the working limits as the EKF
// does: 0% at the working min and 100% at the working max, both looked up on the
// factory table (not a learned one), so the two estimates agree where 0% and
// 100% are.
uint16_t voltage_based_soc_estimate(const bms_model_t *model) {
    const float *table = ekf_factory_ocv_table();

    // calculate representative cell voltage
    uint16_t mean_voltage = (model->cell_voltage_total_mV / NUM_CELLS);
    float soc_estimate = table_soc(table, (float)mean_voltage / 1000.0f);
    // Use min voltage for low SoC, max voltage for high SoC
    uint16_t representative_voltage_mV = (
        model->cell_voltage_min_mV + (model->cell_voltage_max_mV - model->cell_voltage_min_mV) * soc_estimate
    );

    // Open-circuit voltage: charging (positive current) lifts the terminal
    // voltage above it by I * R, discharging pulls it below
    float ocv = (representative_voltage_mV - model->current_mA * EKF_R_OHM) / 1000.0f;
    float soc = table_soc(table, ocv);

    float soc_min = table_soc(table, get_cell_voltage_working_min_mV(model) / 1000.0f);
    float soc_max = table_soc(table, get_cell_voltage_working_max_mV(model) / 1000.0f);
    soc = (soc - soc_min) / (soc_max - soc_min);
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;
    return (uint16_t)(soc * 10000.0f + 0.5f); // in 0.01% units
}
