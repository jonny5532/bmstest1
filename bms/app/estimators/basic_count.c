#include <stdint.h>
#include "../model.h"
#include "../../config/limits.h"

// Counts downwards from the top of charge, in raw units
static int32_t last_charge_raw = 0;

static float charge_counter_mC;

static bool initialized = false;

float nmc_ocv_to_soc(float ocv);

// we can measure this, with extremely slow averaging
static const float INA228_SAMPLING_PERIOD_S = 0.530940f;
static const float INA228_CURRENT_LSB_mA = 0.25f;
static const float INA228_CHARGE_LSB_mC = INA228_CURRENT_LSB_mA * INA228_SAMPLING_PERIOD_S;

uint16_t basic_count_soc_estimate(bms_model_t *model) {
    int32_t charge_delta_raw = model->charge_raw - last_charge_raw;

    float charge = (float)charge_delta_raw * INA228_CHARGE_LSB_mC;
    // Recorded charge is negative when discharging, so we invert
    charge_counter_mC -= charge;
    if(charge_counter_mC < 0.0f) {
        charge_counter_mC = 0.0f;
    } else if(charge_counter_mC > model->capacity_mC) {
        // maybe expand capacity? or record measured capacity?
        //charge_counter_mC = capacity_mC;
    }
    last_charge_raw = model->charge_raw;

    if(!initialized && timestep() > 200 && model->battery_voltage_mV > 0) {
        // Initialize SOC estimate based on OCV
        float soc = nmc_ocv_to_soc((float)model->battery_voltage_mV / (NUM_CELLS * 1000.0f));

        charge_counter_mC = (1.0f - soc) * model->capacity_mC;

        initialized = true;
    }

    float soc = 1.0f - (charge_counter_mC / (float)model->capacity_mC);
    if(soc < 0.0f) {
        soc = 0.0f;
    } else if(soc > 1.0f) {
        soc = 1.0f;
    }
    return (uint16_t)(soc * 10000.0f);
}
