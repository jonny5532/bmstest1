#include "../model.h"
#include "../limits.h"
#include "../monitoring/events.h"

void confirm_battery_safety(bms_model_t *model) {
    bool not_fully_initialized = (
       model->system_sm.state == SYSTEM_STATE_UNINITIALIZED ||
       model->system_sm.state == SYSTEM_STATE_INITIALIZING
    );

    if(check_or_confirm(
        millis_recent_enough(model->battery_voltage_millis, BATTERY_VOLTAGE_STALE_THRESHOLD_MS),
        // Don't raise faults if we're still initializing
        !not_fully_initialized,
        ERR_BATTERY_VOLTAGE_STALE,
        0x1000000000000000
    )) {
        confirm(
            model->battery_voltage_mV <= BATTERY_VOLTAGE_SOFT_MAX_mV,
            ERR_BATTERY_VOLTAGE_HIGH,
            model->battery_voltage_mV
        );
        confirm(
            model->battery_voltage_mV <= BATTERY_VOLTAGE_HARD_MAX_mV,
            ERR_BATTERY_VOLTAGE_VERY_HIGH,
            model->battery_voltage_mV
        );
        confirm(
            model->battery_voltage_mV >= BATTERY_VOLTAGE_SOFT_MIN_mV,
            ERR_BATTERY_VOLTAGE_LOW,
            model->battery_voltage_mV
        );
        confirm(
            model->battery_voltage_mV >= BATTERY_VOLTAGE_HARD_MIN_mV,
            ERR_BATTERY_VOLTAGE_VERY_LOW,
            model->battery_voltage_mV
        );
    }

    if(check_or_confirm(
        millis_recent_enough(model->cell_voltage_millis, 
            model->cell_voltage_slow_mode ?
                CELL_VOLTAGE_STALE_THRESHOLD_SLOW_MS
                : CELL_VOLTAGE_STALE_THRESHOLD_MS
        ),
        // Don't raise faults if we're still initializing
        !not_fully_initialized,
        ERR_CELL_VOLTAGES_STALE,
        0x1000000000000000
    )) {
        confirm(
            model->cell_voltage_max_mV <= CELL_VOLTAGE_SOFT_MAX_mV,
            ERR_CELL_VOLTAGE_HIGH,
            model->cell_voltage_max_mV
        );
        confirm(
            model->cell_voltage_max_mV <= CELL_VOLTAGE_HARD_MAX_mV,
            ERR_CELL_VOLTAGE_VERY_HIGH,
            model->cell_voltage_max_mV
        );
        confirm(
            model->cell_voltage_min_mV >= CELL_VOLTAGE_SOFT_MIN_mV,
            ERR_CELL_VOLTAGE_LOW,
            model->cell_voltage_min_mV
        );
        confirm(
            model->cell_voltage_min_mV >= CELL_VOLTAGE_HARD_MIN_mV,
            ERR_CELL_VOLTAGE_VERY_LOW,
            model->cell_voltage_min_mV
        );
    }

    if(check_or_confirm(
        millis_recent_enough(model->temperature_millis, 
            model->cell_voltage_slow_mode ?
                CELL_TEMPERATURE_STALE_THRESHOLD_SLOW_MS
                : CELL_TEMPERATURE_STALE_THRESHOLD_MS
        ),
        // Don't raise faults if we're still initializing
        !not_fully_initialized,
        ERR_BATTERY_TEMPERATURE_STALE,
        0x1000000000000000
    )) {
        confirm(
            model->temperature_max_dC <= TEMPERATURE_SOFT_MAX_dC,
            ERR_BATTERY_TEMPERATURE_HIGH,
            model->temperature_max_dC
        );
        confirm(
            model->temperature_max_dC <= TEMPERATURE_HARD_MAX_dC,
            ERR_BATTERY_TEMPERATURE_VERY_HIGH,
            model->temperature_max_dC
        );
        confirm(
            model->temperature_min_dC >= TEMPERATURE_SOFT_MIN_dC,
            ERR_BATTERY_TEMPERATURE_LOW,
            model->temperature_min_dC
        );
        confirm(
            model->temperature_min_dC >= TEMPERATURE_HARD_MIN_dC,
            ERR_BATTERY_TEMPERATURE_VERY_LOW,
            model->temperature_min_dC
        );
    }
}