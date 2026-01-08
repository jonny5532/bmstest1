#include "model.h"

#include "../limits.h"
#include "../lib/math.h"

bms_model_t model = {0};

static void model_process_temperatures(bms_model_t *model) {
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

static void model_process_cell_voltages(bms_model_t *model) {
    model->cell_voltage_min_mV = model->cell_voltages_mV[0];
    model->cell_voltage_max_mV = model->cell_voltages_mV[0];
    for(int i=1; i<NUM_CELLS; i++) {
        int32_t volt = model->cell_voltages_mV[i];

        // TODO - decide on how to handle missing cells
        if(volt < 0) {
            continue;
        }

        if(volt < model->cell_voltage_min_mV) {
            model->cell_voltage_min_mV = volt;
        }
        if(volt > model->cell_voltage_max_mV) {
            model->cell_voltage_max_mV = volt;
        }   
    }
}

static void model_calculate_cell_current_limits(bms_model_t *model) {
    model->cell_voltage_charge_current_limit_dA = 0xFFFF;
    model->cell_voltage_discharge_current_limit_dA = 0xFFFF;

    if(model->cell_voltage_max_mV > CELL_VOLTAGE_SOFT_MAX_mV) {
        // Above max cell voltage, stop charging and limit discharge
        model->cell_voltage_charge_current_limit_dA = 0;
        model->cell_voltage_discharge_current_limit_dA = OVERCHARGE_DISCHARGE_CURRENT_LIMIT_dA;
    }
    if(model->cell_voltage_min_mV < CELL_VOLTAGE_SOFT_MIN_mV) {
        // Hit min cell voltage, stop discharging and limit charge
        model->cell_voltage_discharge_current_limit_dA = 0;
        model->cell_voltage_charge_current_limit_dA = OVERDISCHARGE_CHARGE_CURRENT_LIMIT_dA;
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
        int32_t excess_dA = (model->current_mA / 100) - model->charge_current_limit_dA - CURRENT_LIMIT_ERROR_MARGIN_dA;
        // Accumulate into the charge buffer if there is excess
        model->excess_charge_buffer_dC = sadd_i32(model->excess_charge_buffer_dC, 
                                                  (excess_dA > 0) ? excess_dA : 0);
    } else if(model->current_mA < 0) {
        // Discharging
        int32_t excess_dA = (-model->current_mA / 100) - model->discharge_current_limit_dA - CURRENT_LIMIT_ERROR_MARGIN_dA;
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
    if(model->cell_voltage_max_mV < CELL_VOLTAGE_SOFT_MAX_mV &&
       model->cell_voltage_min_mV > CELL_VOLTAGE_SOFT_MIN_mV) {
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
    if(model->excess_charge_buffer_dC > OVERCURRENT_BUFFER_LIMIT_dC) {
        // Too much excess charge, cut off charging
    } else if(model->excess_discharge_buffer_dC > OVERCURRENT_BUFFER_LIMIT_dC) {
        // Too much excess discharge, cut off discharging
    }
}

static void model_check_soft_limit_charge_accumulation(bms_model_t *model) {
    if(model->soft_limit_charge_buffer_dC > OVERCHARGE_BUFFER_LIMIT_dC) {
        // Too much excess charge in soft limit region, cut off charging
    } else if(model->soft_limit_charge_buffer_dC < -OVERDISCHARGE_BUFFER_LIMIT_dC) {
        // Too much excess discharge in soft limit region, cut off discharging
    }
}


static void model_calculate_temperature_current_limits(bms_model_t *model) {
    // Charge current limit derating
    if(model->temperature_max_dC >= MAX_CHARGE_TEMPERATURE_LIMIT_dC || model->temperature_min_dC <= MIN_CHARGE_TEMPERATURE_LIMIT_dC) {
        model->temp_charge_current_limit_dA = 0;
    } else {
        uint16_t upper_charge_limit = (MAX_CHARGE_TEMPERATURE_LIMIT_dC - model->temperature_max_dC) * CHARGE_TEMPERATURE_DERATE_dA_PER_dC;
        uint16_t lower_charge_limit = (model->temperature_min_dC - MIN_CHARGE_TEMPERATURE_LIMIT_dC) * CHARGE_TEMPERATURE_DERATE_dA_PER_dC;
        model->temp_charge_current_limit_dA = (upper_charge_limit < lower_charge_limit) ? upper_charge_limit : lower_charge_limit;
    }

    // Discharge current limit derating
    if(model->temperature_max_dC >= MAX_DISCHARGE_TEMPERATURE_LIMIT_dC || model->temperature_min_dC <= MIN_DISCHARGE_TEMPERATURE_LIMIT_dC) {
        model->temp_discharge_current_limit_dA = 0;
    } else {
        uint16_t upper_discharge_limit = (MAX_DISCHARGE_TEMPERATURE_LIMIT_dC - model->temperature_max_dC) * DISCHARGE_TEMPERATURE_DERATE_dA_PER_dC;
        uint16_t lower_discharge_limit = (model->temperature_min_dC - MIN_DISCHARGE_TEMPERATURE_LIMIT_dC) * DISCHARGE_TEMPERATURE_DERATE_dA_PER_dC;
        model->temp_discharge_current_limit_dA = (upper_discharge_limit < lower_discharge_limit) ? upper_discharge_limit : lower_discharge_limit;
    }
}



void model_tick(bms_model_t *model) {
    model_process_temperatures(model);
    model_process_cell_voltages(model);

    model_calculate_cell_current_limits(model);
    model_calculate_temperature_current_limits(model);

    model_apply_current_limits(model);
    model_accumulate_overcurrent(model);
    model_accumulate_soft_limit_overcurrent(model);

    model_check_overcurrent_accumulation(model);
    model_check_soft_limit_charge_accumulation(model);
}
