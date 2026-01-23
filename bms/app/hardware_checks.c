#include "../config/limits.h"
#include "model.h"
#include "../sys/events/events.h"

void confirm_hardware_integrity(bms_model_t *model) {
    if(model->supply_voltage_contactor_millis==0) {
        // This is the last voltage read, so once set all others should be available
        return;
    }

    if(confirm(
        millis_recent_enough(model->supply_voltage_3V3_millis, SUPPLY_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_SUPPLY_VOLTAGE_STALE,
        0x1000000000000000
    )) {
        confirm(
            model->supply_voltage_3V3_mV >= SUPPLY_VOLTAGE_3V3_MIN_MV,
            ERR_SUPPLY_VOLTAGE_3V3_LOW,
            model->supply_voltage_3V3_mV
        );
        confirm(
            model->supply_voltage_3V3_mV <= SUPPLY_VOLTAGE_3V3_MAX_MV,
            ERR_SUPPLY_VOLTAGE_3V3_HIGH,
            model->supply_voltage_3V3_mV
        );
    }

    if(confirm(
        millis_recent_enough(model->supply_voltage_5V_millis, SUPPLY_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_SUPPLY_VOLTAGE_STALE,
        0x2000000000000000
    )) {
        confirm(
            model->supply_voltage_5V_mV >= SUPPLY_VOLTAGE_5V_MIN_MV,
            ERR_SUPPLY_VOLTAGE_5V_LOW,
            model->supply_voltage_5V_mV
        );
        confirm(
            model->supply_voltage_5V_mV <= SUPPLY_VOLTAGE_5V_MAX_MV,
            ERR_SUPPLY_VOLTAGE_5V_HIGH,
            model->supply_voltage_5V_mV
        );
    }

    if(confirm(
        millis_recent_enough(model->supply_voltage_12V_millis, SUPPLY_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_SUPPLY_VOLTAGE_STALE,
        0x3000000000000000
    )) {
        confirm(
            model->supply_voltage_12V_mV >= SUPPLY_VOLTAGE_12V_MIN_MV,
            ERR_SUPPLY_VOLTAGE_12V_LOW,
            model->supply_voltage_12V_mV
        );
        confirm(
            model->supply_voltage_12V_mV <= SUPPLY_VOLTAGE_12V_MAX_MV,
            ERR_SUPPLY_VOLTAGE_12V_HIGH,
            model->supply_voltage_12V_mV
        );
    }

    if(confirm(
        millis_recent_enough(model->supply_voltage_contactor_millis, SUPPLY_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_SUPPLY_VOLTAGE_STALE,
        0x4000000000000000
    )) {
        confirm(
            model->supply_voltage_contactor_mV >= SUPPLY_VOLTAGE_CONTACTOR_SOFT_MIN_MV,
            ERR_SUPPLY_VOLTAGE_CONTACTOR_LOW,
            model->supply_voltage_contactor_mV
        );
        confirm(
            model->supply_voltage_contactor_mV >= SUPPLY_VOLTAGE_CONTACTOR_HARD_MIN_MV,
            ERR_SUPPLY_VOLTAGE_CONTACTOR_VERY_LOW,
            model->supply_voltage_contactor_mV
        );
        confirm(
            model->supply_voltage_contactor_mV <= SUPPLY_VOLTAGE_CONTACTOR_MAX_MV,
            ERR_SUPPLY_VOLTAGE_CONTACTOR_HIGH,
            model->supply_voltage_contactor_mV
        );
    }
}
