#include "../model.h"
#include "../limits.h"
#include "../debug/events.h"

void confirm_battery_safety(bms_model_t *model) {
    if(confirm(
        millis_recent_enough(model->battery_voltage_millis, STALENESS_THRESHOLD_MS),
        ERR_BATTERY_VOLTAGE_STALE,
        LEVEL_CRITICAL,
        0x1000000000000000
    )) {
        confirm(
            model->battery_voltage_mV <= BATTERY_VOLTAGE_SOFT_MAX_mV,
            ERR_BATTERY_VOLTAGE_HIGH,
            LEVEL_CRITICAL,
            model->battery_voltage_mV
        );
        confirm(
            model->battery_voltage_mV <= BATTERY_VOLTAGE_HARD_MAX_mV,
            ERR_BATTERY_VOLTAGE_VERY_HIGH,
            LEVEL_CRITICAL,
            model->battery_voltage_mV
        );
        confirm(
            model->battery_voltage_mV >= BATTERY_VOLTAGE_SOFT_MIN_mV,
            ERR_BATTERY_VOLTAGE_LOW,
            LEVEL_CRITICAL,
            model->battery_voltage_mV
        );
        confirm(
            model->battery_voltage_mV >= BATTERY_VOLTAGE_HARD_MIN_mV,
            ERR_BATTERY_VOLTAGE_VERY_LOW,
            LEVEL_CRITICAL,
            model->battery_voltage_mV
        );
    }



}