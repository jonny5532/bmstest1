#include "model.h"

#include "battery/current_limits.h"
#include "../config/limits.h"
#include "../lib/math.h"
#include "../lib/aema.h"

#include <string.h>

#include "sys/logging/logging.h"
#include "sys/events/events.h"

bms_model_t model = {0};

static void model_process_temperatures(bms_model_t *model) {
    model->temperature_min = model->module_temperatures[0];
    model->temperature_max = model->module_temperatures[0];
    for(int i=1; i<NUM_MODULE_TEMPS; i++) {
        int16_t temp = model->module_temperatures[i];
        if(temp < model->temperature_min) {
            model->temperature_min = temp;
        }
        if(temp > model->temperature_max) {
            model->temperature_max = temp;
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
        int32_t voltage = model->cell_voltages_mV[i];

        // TODO - decide on how to handle missing cells
        if(voltage < 0) {
            continue;
        }

        model->cell_voltage_total_mV += voltage;

        if(voltage < model->cell_voltage_min_mV) {
            model->cell_voltage_min_mV = voltage;
        }
        if(voltage > model->cell_voltage_max_mV) {
            model->cell_voltage_max_mV = voltage;
        }
    }
    model->cell_voltage_millis = model->cell_voltages_millis;
}

void store_cell_voltage(uint8_t logical_index, int16_t voltage_mV) {
    if(logical_index >= NUM_CELLS) {
        return;
    }

    int16_t previous_voltage = model.cell_voltages_mV[logical_index];

    if(previous_voltage > 0) {
        int16_t delta = voltage_mV - previous_voltage;
        if(delta < 0) delta = -delta;

        if(delta >= CELL_VOLTAGE_GLITCH_THRESHOLD_mV) {
            uint64_t event_data = ((uint64_t)logical_index << 32) |
                                  ((uint16_t)previous_voltage << 16) |
                                  (uint16_t)voltage_mV;
            count_bms_event(ERR_CELL_VOLTAGE_GLITCH, event_data);
        }
    }

    model.cell_voltages_mV[logical_index] = voltage_mV;
}

void store_module_temperature(uint8_t module_index, int16_t raw_temp_dC) {
    if(module_index >= NUM_MODULE_TEMPS) {
        return;
    }

    int16_t previous_raw_temp = model.module_temperatures_raw_dC[module_index];

    if(previous_raw_temp != 0) {
        int16_t delta = raw_temp_dC - previous_raw_temp;
        if(delta < 0) delta = -delta;

        if(delta >= MODULE_TEMPERATURE_GLITCH_THRESHOLD_dC) {
            uint64_t event_data = ((uint64_t)module_index << 32) |
                                  ((uint32_t)(uint16_t)previous_raw_temp << 16) |
                                  (uint16_t)raw_temp_dC;
            count_bms_event(ERR_MODULE_TEMPERATURE_GLITCH, event_data);
        }
    }

    model.module_temperatures_raw_dC[module_index] = raw_temp_dC;

    aema_update(
        &model.module_temperatures[module_index],
        NULL,
        raw_temp_dC * 0.1f,
        0.05f,  // slow alpha
        0.75f,  // fast alpha
        2.0f,   // slow threshold
        10.0f   // fast threshold
    );
}

static void model_calculate_cell_current_limits(bms_model_t *model) {
    model->cell_voltage_charge_current_limit_dA = calculate_cell_voltage_charge_current_limit(
        model
    );
    model->cell_voltage_discharge_current_limit_dA = calculate_cell_voltage_discharge_current_limit(
        model
    );
    
    model->working_charge_current_limit_dA = calculate_working_charge_current_limit(model);
    model->delta_charge_current_limit_dA = calculate_delta_charge_current_limit(model);
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

    if(charge_limit > model->working_charge_current_limit_dA) {
        charge_limit = model->working_charge_current_limit_dA;
    }

    if(charge_limit > model->delta_charge_current_limit_dA) {
        charge_limit = model->delta_charge_current_limit_dA;
    }

    // User limits
    if(charge_limit > model->user_charge_current_limit_dA && model->user_charge_current_limit_dA > 0) {
        charge_limit = model->user_charge_current_limit_dA - 1;
    }
    if(discharge_limit > model->user_discharge_current_limit_dA && model->user_discharge_current_limit_dA > 0) {
        discharge_limit = model->user_discharge_current_limit_dA - 1;
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
                                                  max(excess_dA, 0));
    } else if(model->current_mA < 0) {
        // Discharging
        int32_t excess_dA = (-model->current_mA / 100) - model->discharge_current_limit_dA - CURRENT_LIMIT_ERROR_MARGIN_dA;
        // Accumulate into the discharge buffer if there is excess
        model->excess_discharge_buffer_dC = sadd_i32(model->excess_discharge_buffer_dC,
                                                     max(excess_dA, 0));
    }

    if(model->excess_charge_buffer_dC > 0) {
        // Decay the excess charge buffer slowly
        model->excess_charge_buffer_dC--;
    }
    if(model->excess_discharge_buffer_dC > 0) {
        // Decay the excess discharge buffer slowly
        model->excess_discharge_buffer_dC--;
    }
}

// Check if we are in the soft limit region, and accumulate excess
// charge/discharge into a single buffer. We can then tolerate a limited amount
// of charge/discharge (eg, whilst the inverter starts up), but can disconnect
// the battery if it continues for too long. This is to protect the battery from
// overcharge/overdischarge.
static void model_accumulate_soft_limit_overcurrent(bms_model_t *model) {
    if(model->cell_voltage_max_mV < get_cell_voltage_soft_max_mV(model) &&
       model->cell_voltage_min_mV > get_cell_voltage_soft_min_mV(model)) {
        // Not in soft limit region, reset buffer
        model->soft_limit_charge_buffer_dC = 0;
        return;
    }

    if(model->current_mA > 0) {
        // Charging
        int32_t excess_dA = (model->current_mA / 100) - model->charge_current_limit_dA;
        model->soft_limit_charge_buffer_dC = sadd_i32(model->soft_limit_charge_buffer_dC,
                                                      max(excess_dA, 0));
    } else if(model->current_mA < 0) {
        // Discharging
        int32_t excess_dA = (-model->current_mA / 100) - model->discharge_current_limit_dA;
        model->soft_limit_charge_buffer_dC = ssub_i32(model->soft_limit_charge_buffer_dC,
                                                        max(excess_dA, 0));
    }
}

static void model_check_overcurrent_accumulation(bms_model_t *model) {
    // TODO - do we actually want to cut off the battery, or just raise events?

    if(model->excess_charge_buffer_dC > OVERCURRENT_BUFFER_LIMIT_dC) {
        // Too much excess charge, cut off charging
    } else if(model->excess_discharge_buffer_dC > OVERCURRENT_BUFFER_LIMIT_dC) {
        // Too much excess discharge, cut off discharging
    }
}

static void model_calculate_temperature_current_limits(bms_model_t *model) {
    model->temp_charge_current_limit_dA = calculate_temperature_charge_current_limit(
        model->temperature_min,
        model->temperature_max
    );
    model->temp_discharge_current_limit_dA = calculate_temperature_discharge_current_limit(
        model->temperature_min,
        model->temperature_max
    );
}

static void model_calculate_inverter_voltage_limits(const bms_model_t *model, inverter_outputs_t *out) {
    // For inverters that allow absorption charging to continue at the voltage
    // limit.

    uint16_t cell_voltage_working_max_mV = get_cell_voltage_working_max_mV(model);
    uint16_t cell_voltage_working_min_mV = get_cell_voltage_working_min_mV(model);

    int32_t max_voltage_limit_dV = (cell_voltage_working_max_mV * NUM_CELLS) / 100; // in 0.1V units
    int32_t min_voltage_limit_dV = (cell_voltage_working_min_mV * NUM_CELLS) / 100; // in 0.1V units

    int32_t mean_cell_voltage_mV = model->cell_voltage_total_mV / NUM_CELLS;

    // How far is the highest cell above the mean?
    int32_t deviation_max_mV = model->cell_voltage_max_mV - mean_cell_voltage_mV;
    // Calculate how much to reduce the voltage limit by to account for this
    // deviation, to avoid overcharging the highest cell.
    int32_t deviation_reduction_mV = deviation_max_mV * NUM_CELLS;
    if(deviation_reduction_mV > 0) {
        max_voltage_limit_dV -= deviation_reduction_mV / 100;
    }

    // How far is the lowest cell below the mean?
    int32_t deviation_min_mV = mean_cell_voltage_mV - model->cell_voltage_min_mV;
    // Calculate how much to increase the voltage limit by to account for this
    // deviation, to avoid overdischarging the lowest cell.
    int32_t deviation_increase_mV = deviation_min_mV * NUM_CELLS;
    if(deviation_increase_mV > 0) {
        min_voltage_limit_dV += deviation_increase_mV / 100;
    }

    // Apply user-configured offsets to account for errors in the inverter
    // voltage reading.
    max_voltage_limit_dV += model->pack_voltage_limit_upper_offset_dV;
    min_voltage_limit_dV += model->pack_voltage_limit_lower_offset_dV;

    // Guard against a single wildly-deviating cell (e.g. a failed/disconnected
    // tap reported as ~0V) pushing the per-cell deviation term far enough to
    // invert the limits relative to each other.
    if(max_voltage_limit_dV < 0) {
        max_voltage_limit_dV = 0;
    }
    if(min_voltage_limit_dV > max_voltage_limit_dV) {
        min_voltage_limit_dV = max_voltage_limit_dV;
    }
    if(min_voltage_limit_dV < 0) {
        min_voltage_limit_dV = 0;
    }

    out->max_voltage_limit_dV = max_voltage_limit_dV;
    out->min_voltage_limit_dV = min_voltage_limit_dV;
}

static void model_calculate_inverter_soc_and_capacity(const bms_model_t *model, inverter_outputs_t *out) {
    // Calculate the SoC and capacity to send to the inverter, applying scaling and limits.

    int16_t divisor = model->soc_scaling_max - model->soc_scaling_min;
    if(divisor <= 0) {
        divisor = 10000; // default to no scaling
    }

    int32_t scaled_soc = (int32_t)(model->soc - model->soc_scaling_min) * 10000 / divisor;
    if(scaled_soc > 10000) scaled_soc = 10000;
    if(scaled_soc < 0) scaled_soc = 0;

    if(model->discharge_current_limit_dA == 0) {
        // Force to 0% to stop discharge
        scaled_soc = 0;
    } else if(model->charge_current_limit_dA == 0) {
        // Force to 100% to stop charge
        scaled_soc = 10000;
    }

    out->soc = scaled_soc;
    out->soc_millis = model->soc_millis;

    // TODO - use floating point instead of the int64_t math here?

    // so if scaling min/max is 50 to 75, we're only using 25% of the working capacity

    uint32_t scaled_working_capacity_mC = ((uint64_t)model->working_capacity_mC * divisor) / 10000;

    out->remaining_capacity_dAh = ((uint64_t)scaled_working_capacity_mC * scaled_soc) / ((uint64_t)10000 * 3600 * 100);
    out->full_capacity_dAh = (scaled_working_capacity_mC / (3600 * 100));
}

static void model_calculate_inverter_outputs(const bms_model_t *model, inverter_outputs_t *out) {
    // Calculate the values to send to the inverter, applying any necessary scaling or limits.

    memset(out, 0, sizeof(*out));
    
    model_calculate_inverter_voltage_limits(model, out);
    model_calculate_inverter_soc_and_capacity(model, out);

    // Copy other values acruss from the model
    out->battery_voltage = model->high_voltages.battery;
    out->battery_voltage_millis = model->high_voltages.battery_millis;
    out->current_mA = model->current_mA;
    out->current_millis = model->current_millis;
    out->temperature_min = model->temperature_min;
    out->temperature_max = model->temperature_max;
    out->temperature_millis = model->temperature_millis;
    out->charge_current_limit_dA = model->charge_current_limit_dA;
    out->discharge_current_limit_dA = model->discharge_current_limit_dA;
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

    model_calculate_inverter_outputs(model, &model->inverter_outputs);
}
