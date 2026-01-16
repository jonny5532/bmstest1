#include "../limits.h"

#include <stdint.h>

float nmc_ocv_to_soc(float ocv);

uint16_t calculate_cell_voltage_charge_current_limit(uint32_t cell_voltage_min_mV, uint32_t cell_voltage_max_mV) {
    uint16_t charge_limit = 0xFFFF;

    if(cell_voltage_max_mV > 4000) {
        // We're onto the steeper part of the curve now, so SoC estimation is more accurate
        float soc = nmc_ocv_to_soc(cell_voltage_max_mV / 1000.0f);
        int32_t derate_dA = (int32_t)((1.0f - soc) * 100.0f * CHARGE_CELL_VOLTAGE_DERATE_dA_PER_SoC);
        if(derate_dA < charge_limit) {
            charge_limit = derate_dA;
        }
    }

    if(cell_voltage_max_mV > CELL_VOLTAGE_SOFT_MAX_mV) {
        // Above max cell voltage, stop charging
        charge_limit = 0;
    }
    if(cell_voltage_min_mV < CELL_VOLTAGE_SOFT_MIN_mV) {
        // Below min cell voltage, limit charge current
        if(charge_limit > OVERDISCHARGE_CHARGE_CURRENT_LIMIT_dA) {
            charge_limit = OVERDISCHARGE_CHARGE_CURRENT_LIMIT_dA;
        }
    }

    return charge_limit;
}

uint16_t calculate_cell_voltage_discharge_current_limit(uint32_t cell_voltage_min_mV, uint32_t cell_voltage_max_mV) {
    uint16_t discharge_limit = 0xFFFF;

    if(cell_voltage_min_mV < 3500) {
        float soc = nmc_ocv_to_soc(cell_voltage_min_mV / 1000.0f);
        int32_t derate_dA = (int32_t)(soc * 100.0f * DISCHARGE_CELL_VOLTAGE_DERATE_dA_PER_SoC);
        if(derate_dA < discharge_limit) {
            discharge_limit = derate_dA;
        }
    }

    if(cell_voltage_min_mV < CELL_VOLTAGE_SOFT_MIN_mV) {
        // Hit min cell voltage, stop discharging
        discharge_limit = 0;
    }
    if(cell_voltage_max_mV > CELL_VOLTAGE_SOFT_MAX_mV) {
        // Above max cell voltage, limit discharge current
        if(discharge_limit > OVERCHARGE_DISCHARGE_CURRENT_LIMIT_dA) {
            discharge_limit = OVERCHARGE_DISCHARGE_CURRENT_LIMIT_dA;
        }
    }

    return discharge_limit;
}

uint16_t calculate_temperature_charge_current_limit(int16_t temperature_min_dC, int16_t temperature_max_dC) {
    if(temperature_max_dC >= MAX_CHARGE_TEMPERATURE_LIMIT_dC || temperature_min_dC <= MIN_CHARGE_TEMPERATURE_LIMIT_dC) {
        return 0;
    } else {
        uint16_t upper_charge_limit = (MAX_CHARGE_TEMPERATURE_LIMIT_dC - temperature_max_dC) * CHARGE_TEMPERATURE_DERATE_dA_PER_dC;
        uint16_t lower_charge_limit = (temperature_min_dC - MIN_CHARGE_TEMPERATURE_LIMIT_dC) * CHARGE_TEMPERATURE_DERATE_dA_PER_dC;
        return (upper_charge_limit < lower_charge_limit) ? upper_charge_limit : lower_charge_limit;
    }
    return 0xFFFF;
}

uint16_t calculate_temperature_discharge_current_limit(int16_t temperature_min_dC, int16_t temperature_max_dC) {
    if(temperature_max_dC >= MAX_DISCHARGE_TEMPERATURE_LIMIT_dC || temperature_min_dC <= MIN_DISCHARGE_TEMPERATURE_LIMIT_dC) {
        return 0;
    } else {
        uint16_t upper_discharge_limit = (MAX_DISCHARGE_TEMPERATURE_LIMIT_dC - temperature_max_dC) * DISCHARGE_TEMPERATURE_DERATE_dA_PER_dC;
        uint16_t lower_discharge_limit = (temperature_min_dC - MIN_DISCHARGE_TEMPERATURE_LIMIT_dC) * DISCHARGE_TEMPERATURE_DERATE_dA_PER_dC;
        return (upper_discharge_limit < lower_discharge_limit) ? upper_discharge_limit : lower_discharge_limit;
    }
    return 0xFFFF;
}

