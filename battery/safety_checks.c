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
            TEMPERATURE_STALE_THRESHOLD_MS(model)
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

    // TODO: Is this the right place for this?

    if(model->battery_voltage_millis > 0 && model->cell_voltage_millis > 0 && (model->battery_voltage_millis - model->cell_voltage_millis) < 1000) {
        // If we have a recent cell voltage reading, compare total to battery
        // voltage. We may be sampling the cell voltages very infrequently, so
        // only do this check if the voltage readings are close in time.
        int32_t expected_total = model->battery_voltage_mV;
        int32_t voltage_diff = model->cell_voltage_total_mV - expected_total;
        confirm(
            voltage_diff > -VOLTAGE_MISMATCH_THRESHOLD_mV && voltage_diff < VOLTAGE_MISMATCH_THRESHOLD_mV,
            ERR_VOLTAGE_MISMATCH,
            ((uint64_t)model->cell_voltage_total_mV << 32) | (uint32_t)expected_total
        );
    }

    // TODO: definitely not the right place for this
    confirm(
        !model->estop_pressed,
        ERR_ESTOP_PRESSED,
        0
    );
}
