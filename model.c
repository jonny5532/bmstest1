#include "model.h"

#include "../limits.h"
#include "../lib/math.h"

bms_model_t model = {0};

static void model_calculate_temperatures(bms_model_t *model) {
    model->temperature_min_dC = model->module_temperatures_dC[0];
    model->temperature_max_dC = model->module_temperatures_dC[0];
    for(int i=1; i<8; i++) {
        int16_t temp = model->module_temperatures_dC[i];
        if(temp < model->temperature_min_dC) {
            model->temperature_min_dC = temp;
        }
        if(temp > model->temperature_max_dC) {
            model->temperature_max_dC = temp;
        }   
    }
    model->temperature_millis = model->module_temperatures_millis;
}

static void model_calculate_cell_current_limits(bms_model_t *model) {
    model->cell_voltage_charge_current_limit_dA = 0xFFFF;
    model->cell_voltage_discharge_current_limit_dA = 0xFFFF;

    if(model->cell_voltage_max_mV > CELL_VOLTAGE_SOFT_MAX_MV) {
        // Above max cell voltage, stop charging and limit discharge
        model->cell_voltage_charge_current_limit_dA = 0;
        model->cell_voltage_discharge_current_limit_dA = OVERCHARGE_DISCHARGE_CURRENT_LIMIT_DA;
    }
    if(model->cell_voltage_min_mV < CELL_VOLTAGE_SOFT_MIN_MV) {
        // Hit min cell voltage, stop discharging and limit charge
        model->cell_voltage_discharge_current_limit_dA = 0;
        model->cell_voltage_charge_current_limit_dA = OVERDISCHARGE_CHARGE_CURRENT_LIMIT_DA;
    }
}

static void model_apply_current_limits(bms_model_t *model) {
    uint16_t charge_limit = 0xFFFF;
    uint16_t discharge_limit = 0xFFFF;

    // Temperature limits
    if(charge_limit > model->temp_charge_current_limit_dA) {
        charge_limit = model->temp_charge_current_limit_dA;
    }
    if(discharge_limit > model->temp_discharge_current_limit_dA) {
        discharge_limit = model->temp_discharge_current_limit_dA;
    }

    // Pack voltage limits
    if(charge_limit > model->pack_voltage_charge_current_limit_dA) {
        charge_limit = model->pack_voltage_charge_current_limit_dA;
    }
    if(discharge_limit > model->pack_voltage_discharge_current_limit_dA) {
        discharge_limit = model->pack_voltage_discharge_current_limit_dA;
    }

    // Cell voltage limits
    if(charge_limit > model->cell_voltage_charge_current_limit_dA) {
        charge_limit = model->cell_voltage_charge_current_limit_dA;
    }
    if(discharge_limit > model->cell_voltage_discharge_current_limit_dA) {
        discharge_limit = model->cell_voltage_discharge_current_limit_dA;
    }

    // User limits
    if(charge_limit > model->user_charge_current_limit_dA) {
        charge_limit = model->user_charge_current_limit_dA;
    }
    if(discharge_limit > model->user_discharge_current_limit_dA) {
        discharge_limit = model->user_discharge_current_limit_dA;
    }

    model->charge_current_limit_dA = charge_limit;
    model->discharge_current_limit_dA = discharge_limit;
}

// Check if the current exceeds the limits, and accumulate excess
// charge/discharge into separate buffers, so that we can cut off the battery if
// it goes on for too long. This is to protect the battery from excessively high
// charge or discharge currents.
static void model_accumulate_overcurrent(bms_model_t *model) {
    if(model->current_mA > 0) {
        // We are charging. Work out the excess current above the limit.
        int32_t excess_dA = (model->current_mA / 100) - model->charge_current_limit_dA - CURRENT_LIMIT_ERROR_MARGIN_DA;
        // Accumulate into the charge buffer if there is excess
        model->excess_charge_buffer_dC = sadd_i32(model->excess_charge_buffer_dC, 
                                                  (excess_dA > 0) ? excess_dA : 0);
    } else if(model->current_mA < 0) {
        // Discharging
        int32_t excess_dA = (-model->current_mA / 100) - model->discharge_current_limit_dA - CURRENT_LIMIT_ERROR_MARGIN_DA;
        model->excess_discharge_buffer_dC = sadd_i32(model->excess_discharge_buffer_dC,
                                                     (excess_dA > 0) ? excess_dA : 0);
    }
}

// Check if we are in the soft limit region, and accumulate excess
// charge/discharge into a single buffer. We can then tolerate a limited amount
// of charge/discharge (eg, whilst the inverter starts up), but can disconnect
// the battery if it continues for too long. This is to protect the battery from
// overcharge/overdischarge.
static void model_accumulate_soft_limit_overcurrent(bms_model_t *model) {
    if(model->cell_voltage_max_mV < CELL_VOLTAGE_SOFT_MAX_MV &&
       model->cell_voltage_min_mV > CELL_VOLTAGE_SOFT_MIN_MV) {
        // Not in soft limit region, reset buffer
        model->soft_limit_charge_buffer_dC = 0;
        return;
    }

    if(model->current_mA > 0) {
        // Charging
        int32_t excess_dA = (model->current_mA / 100) - model->charge_current_limit_dA;
        model->soft_limit_charge_buffer_dC = sadd_i32(model->soft_limit_charge_buffer_dC,
                                                      (excess_dA > 0) ? excess_dA : 0);
    } else if(model->current_mA < 0) {
        // Discharging
        int32_t excess_dA = (-model->current_mA / 100) - model->discharge_current_limit_dA;
        model->soft_limit_charge_buffer_dC = ssub_i32(model->soft_limit_charge_buffer_dC,
                                                        (excess_dA > 0) ? excess_dA : 0);
    }
}

static void model_check_overcurrent_accumulation(bms_model_t *model) {
    if(model->excess_charge_buffer_dC > OVERCURRENT_BUFFER_LIMIT_DC) {
        // Too much excess charge, cut off charging
    } else if(model->excess_discharge_buffer_dC > OVERCURRENT_BUFFER_LIMIT_DC) {
        // Too much excess discharge, cut off discharging
    }
}

static void model_check_soft_limit_charge_accumulation(bms_model_t *model) {
    if(model->soft_limit_charge_buffer_dC > OVERCHARGE_BUFFER_LIMIT_DC) {
        // Too much excess charge in soft limit region, cut off charging
    } else if(model->soft_limit_charge_buffer_dC < -OVERDISCHARGE_BUFFER_LIMIT_DC) {
        // Too much excess discharge in soft limit region, cut off discharging
    }
}


void model_tick(bms_model_t *model) {
    model_calculate_temperatures(model);

    model_calculate_cell_current_limits(model);
    model_apply_current_limits(model);
    model_accumulate_overcurrent(model);
    model_accumulate_soft_limit_overcurrent(model);

    model_check_overcurrent_accumulation(model);
    model_check_soft_limit_charge_accumulation(model);
}
