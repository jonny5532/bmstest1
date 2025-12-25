#include "hw/time.h"
#include "battery/balancing.h"
#include "state_machines/charging.h"
#include "state_machines/contactors.h"

#include <stdint.h>

typedef struct {
    int32_t current_mA;
    millis_t current_millis;

    int32_t battery_voltage_mv;
    millis_t battery_voltage_millis;
    int32_t output_voltage_mv;
    millis_t output_voltage_millis;

    contactors_sm_t contactor_sm;
    contactors_requests_t contactor_req;

    charging_sm_t charging_sm;
    

    balancing_state_t balancing;

    // Battery
    int16_t module_temperatures_dc[8];
    int16_t cell_voltages_mV[120];


  
} bms_model_t;

extern bms_model_t model;