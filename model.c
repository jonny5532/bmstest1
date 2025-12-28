#include "model.h"

bms_model_t model = {0};

static void model_check_temperatures(bms_model_t *model) {
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

static void model_calculate_current_limits(bms_model_t *model) {
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

void model_tick(bms_model_t *model) {
    model_check_temperatures(model);
    model_calculate_current_limits(model);
}
