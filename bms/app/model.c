#include "model.h"

#include "battery/current_limits.h"
#include "../config/limits.h"
#include "../lib/math.h"

bms_model_t model = {0};

static void model_process_temperatures(bms_model_t *model) {
    model->temperature_min_dC = model->module_temperatures_dC[0];
    model->temperature_max_dC = model->module_temperatures_dC[0];
    for(int i=1; i<NUM_MODULE_TEMPS; i++) {
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
    if(model->cell_voltages_millis == 0) {
        // No valid data yet
        return;
    }
    model->cell_voltage_min_mV = model->cell_voltages_mV[0];
    model->cell_voltage_max_mV = model->cell_voltages_mV[0];
    model->cell_voltage_total_mV = model->cell_voltages_mV[0];
    for(int i=1; i<NUM_CELLS; i++) {
        int32_t volt = model->cell_voltages_mV[i];

        // TODO - decide on how to handle missing cells
        if(volt < 0) {
            continue;
        }

        model->cell_voltage_total_mV += volt;

        if(volt < model->cell_voltage_min_mV) {
            model->cell_voltage_min_mV = volt;
        }
        if(volt > model->cell_voltage_max_mV) {
            model->cell_voltage_max_mV = volt;
        }
    }
    model->cell_voltage_millis = model->cell_voltages_millis;
}

static void model_calculate_cell_current_limits(bms_model_t *model) {
    model->cell_voltage_charge_current_limit_dA =calculate_cell_voltage_charge_current_limit(
        model->cell_voltage_min_mV,
        model->cell_voltage_max_mV
    );
    model->cell_voltage_discharge_current_limit_dA = calculate_cell_voltage_discharge_current_limit(
        model->cell_voltage_min_mV,
        model->cell_voltage_max_mV
    );
}

static void model_calculate_voltage_limits(bms_model_t *model) {
    // Calculate pack voltage limits based on cell voltages

    // TODO - add error compensation based on measured current?

    model->max_voltage_limit_dV = (CELL_VOLTAGE_SOFT_MAX_mV * NUM_CELLS) / 100; // in 0.1V units
    model->min_voltage_limit_dV = (CELL_VOLTAGE_SOFT_MIN_mV * NUM_CELLS) / 100; // in 0.1V units
}



static void model_apply_current_limits(bms_model_t *model) {
    uint16_t charge_limit = CHARGE_MAX_CURRENT_dA;
    uint16_t discharge_limit = DISCHARGE_MAX_CURRENT_dA;

    if(!model->contactor_sm.enable_current) {
        // Contactor state machine disallows current flow
        charge_limit = 0;
        discharge_limit = 0;
    }

    // Temperature limits
    if(charge_limit > model->temp_charge_current_limit_dA) {
        charge_limit = model->temp_charge_current_limit_dA;
    }
    if(discharge_limit > model->temp_discharge_current_limit_dA) {
        discharge_limit = model->temp_discharge_current_limit_dA;
    }

    // Pack voltage limits
    // if(charge_limit > model->pack_voltage_charge_current_limit_dA) {
    //     charge_limit = model->pack_voltage_charge_current_limit_dA;
    // }
    // if(discharge_limit > model->pack_voltage_discharge_current_limit_dA) {
    //     discharge_limit = model->pack_voltage_discharge_current_limit_dA;
    // }

    // Cell voltage limits
    if(charge_limit > model->cell_voltage_charge_current_limit_dA) {
        charge_limit = model->cell_voltage_charge_current_limit_dA;
    }
    if(discharge_limit > model->cell_voltage_discharge_current_limit_dA) {
        discharge_limit = model->cell_voltage_discharge_current_limit_dA;
    }

    // User limits
    // if(charge_limit > model->user_charge_current_limit_dA) {
    //     charge_limit = model->user_charge_current_limit_dA;
    // }
    // if(discharge_limit > model->user_discharge_current_limit_dA) {
    //     discharge_limit = model->user_discharge_current_limit_dA;
    // }

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
    model->temp_charge_current_limit_dA = calculate_temperature_charge_current_limit(
        model->temperature_min_dC,
        model->temperature_max_dC
    );
    model->temp_discharge_current_limit_dA = calculate_temperature_discharge_current_limit(
        model->temperature_min_dC,
        model->temperature_max_dC
    );
}



void model_tick(bms_model_t *model) {
    model_process_temperatures(model);
    model_process_cell_voltages(model);

    model_calculate_cell_current_limits(model);
    model_calculate_temperature_current_limits(model);
    model_calculate_voltage_limits(model);

    model_apply_current_limits(model);
    model_accumulate_overcurrent(model);
    model_accumulate_soft_limit_overcurrent(model);

    model_check_overcurrent_accumulation(model);
    model_check_soft_limit_charge_accumulation(model);
}
