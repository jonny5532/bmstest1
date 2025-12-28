#include "hw/time.h"
#include "battery/balancing.h"
#include "state_machines/charging.h"
#include "state_machines/contactors.h"

#include <stdint.h>

// This entire structure will be zero-initialized at startup
typedef struct {
    int32_t current_mA;
    millis_t current_millis;

    int32_t temperature_min_dC;
    int32_t temperature_max_dC;
    millis_t temperature_millis;

    int32_t battery_voltage_mv;
    millis_t battery_voltage_millis;
    int32_t output_voltage_mv;
    millis_t output_voltage_millis;

    uint32_t soc; // state of charge in 0.01% units (0=0%, 10000=100.00%)
    uint32_t soh; // state of health in 0.01% units (0=0%, 10000=100.00%)

    contactors_sm_t contactor_sm;
    contactors_requests_t contactor_req;

    charging_sm_t charging_sm;
    

    balancing_state_t balancing;

    // Battery
    int16_t module_temperatures_dC[8];
    millis_t module_temperatures_millis;
    int16_t cell_voltages_mV[120];

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


  
} bms_model_t;

extern bms_model_t model;

void model_tick(bms_model_t *model);
