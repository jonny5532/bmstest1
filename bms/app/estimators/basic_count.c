#include <stdint.h>
#include "../model.h"
#include "../../config/limits.h"
#include "../../drivers/sensors/ina228.h"

// Counts downwards from the top of charge, in raw units
static int64_t last_charge_raw = 0;

static float charge_counter_mC;

static bool initialized = false;

float ocv_to_soc(float ocv);

uint16_t basic_count_soc_estimate(bms_model_t *model) {
    int64_t charge_delta_raw = model->charge_raw - last_charge_raw;

    float charge = (float)charge_delta_raw * (1.0f / INA228_CHARGE_RAW_PER_MC);
    // Recorded charge is negative when discharging, so we invert
    charge_counter_mC -= charge;
    if(charge_counter_mC < 0.0f) {
        charge_counter_mC = 0.0f;
    } else if(charge_counter_mC > get_actual_capacity_mC(model)) {
        // maybe expand capacity? or record measured capacity?
        //charge_counter_mC = capacity_mC;
    }
    last_charge_raw = model->charge_raw;

    if(!initialized && timestep() > 200 && model->high_voltages.battery > 0) {
        // Initialize SOC estimate based on OCV
        float soc = ocv_to_soc(model->high_voltages.battery / NUM_CELLS);

        charge_counter_mC = (1.0f - soc) * get_actual_capacity_mC(model);

        initialized = true;
    }

    float soc = 1.0f - (charge_counter_mC / (float)get_actual_capacity_mC(model));
    if(soc < 0.0f) {
        soc = 0.0f;
    } else if(soc > 1.0f) {
        soc = 1.0f;
    }
    return (uint16_t)(soc * 10000.0f);
}
