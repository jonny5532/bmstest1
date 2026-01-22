#pragma once

#include "hw/chip/time.h"
#include "calibration/offline.h"
#include "battery/balancing.h"
#include "state_machines/contactors.h"
#include "state_machines/system.h"

#include <stdint.h>
#include <stdio.h>

// This entire structure will be zero-initialized at startup
typedef struct bms_model {
    // Positive current means battery is charging
    int32_t current_mA;
    millis_t current_millis;
    int64_t charge_raw;
    millis_t charge_millis;

    int16_t temperature_min_dC;
    int16_t temperature_max_dC;
    millis_t temperature_millis;

    int32_t battery_voltage_mV;
    millis_t battery_voltage_millis;
    int32_t battery_voltage_range_mV;
    int32_t output_voltage_mV;
    millis_t output_voltage_millis;
    int32_t output_voltage_range_mV;

    int32_t pos_contactor_voltage_mV;
    millis_t pos_contactor_voltage_millis;
    int32_t pos_contactor_voltage_range_mV;
    int32_t neg_contactor_voltage_mV;
    millis_t neg_contactor_voltage_millis;
    int32_t neg_contactor_voltage_range_mV;

    uint16_t soc; // state of charge in 0.01% units (0=0%, 10000=100.00%)
    millis_t soc_millis;
    uint16_t soh; // state of health in 0.01% units (0=0%, 10000=100.00%)

    uint16_t soc_ekf;
    uint16_t soc_voltage_based;
    uint16_t soc_basic_count;
    uint16_t soc_fancy_count;

    uint32_t capacity_mC; // battery capacity in mC

    system_sm_t system_sm;
    system_requests_t system_req;

    contactors_sm_t contactor_sm;
    contactors_requests_t contactor_req;

    balancing_sm_t balancing_sm;

    offline_calibration_sm_t offline_calibration_sm;
    offline_calibration_requests_t offline_calibration_req;

    // Battery
    int16_t module_temperatures_dC[8];
    millis_t module_temperatures_millis;
    int16_t cell_voltage_min_mV;
    int16_t cell_voltage_max_mV;
    int32_t cell_voltage_total_mV;
    millis_t cell_voltage_millis; // aggregated cell voltages
    int16_t cell_voltages_mV[120];
    millis_t cell_voltages_millis; // individial cell voltages
    bool cell_voltages_unstable; // set when cell voltages are changing rapidly
    bool cell_voltage_slow_mode; // only request BMB data infrequently

    // The calculated pack voltage limits
    uint16_t max_voltage_limit_dV;
    uint16_t min_voltage_limit_dV;

    // Current limits (the lower of these limits will be used)
    uint16_t temp_charge_current_limit_dA; // in 0.1A units
    uint16_t temp_discharge_current_limit_dA; // in 0.1A units
    uint16_t pack_voltage_charge_current_limit_dA; // in 0.1A units
    uint16_t pack_voltage_discharge_current_limit_dA; // in 0.1A units
    uint16_t cell_voltage_charge_current_limit_dA; // in 0.1A units
    uint16_t cell_voltage_discharge_current_limit_dA; // in 0.1A units
    uint16_t user_charge_current_limit_dA; // in 0.1A units
    uint16_t user_discharge_current_limit_dA; // in 0.1A units
    // The calculated final current limits
    uint16_t charge_current_limit_dA; // in 0.1A units
    uint16_t discharge_current_limit_dA; // in 0.1A units

    // The amount of charge that has passed into the battery in excess of the
    // charge/discharge limits in the soft-limit region (slightly under or
    // overcharged). If this gets too large, we can cut off the battery to avoid
    // the problem worsening. We need a buffer to give some leeway during
    // inverter startup.
    int32_t soft_limit_charge_buffer_dC; // in 0.1C units

    // The amount of charge that has gone into/out of the battery beyond the
    // current limits.
    int32_t excess_charge_buffer_dC; // in 0.1C units
    int32_t excess_discharge_buffer_dC; // in 0.1C units

    int32_t supply_voltage_3V3_mV;
    millis_t supply_voltage_3V3_millis;
    int32_t supply_voltage_5V_mV;
    millis_t supply_voltage_5V_millis;
    int32_t supply_voltage_12V_mV;
    millis_t supply_voltage_12V_millis;
    int32_t supply_voltage_contactor_mV;
    millis_t supply_voltage_contactor_millis;

    // Calibration constants
    int32_t battery_voltage_mul; // convert raw ADC to mV (/4096)
    int32_t output_voltage_mul; // convert raw ADC to mV (/4096)
    int32_t neg_contactor_mul;
    int32_t neg_contactor_offset_mV;
    int32_t pos_contactor_mul; // actually only the bat+ to out- part
    int32_t current_offset;

    bool balancing_enabled;
    bool balancing_active; // whether balancing was requested this BMB cycle
    int16_t balancing_voltage_threshold_mV; // Only balance cells above this voltage

    bool estop_pressed;
    // Detected via the aux contacts (note that this is actually the precharge bypass contactor)
    bool precharge_closed; 
    // Whether we should ignore a potential loop overrun at the end of this tick
    // (eg, due to a slow flash write). This is reset each tick.
    bool ignore_missed_deadline;
  
} bms_model_t;

extern bms_model_t model;

void model_tick(bms_model_t *model);
