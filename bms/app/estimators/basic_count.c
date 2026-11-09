#include <stdint.h>
#include "ekf.h"
#include "../model.h"
#include "../../config/limits.h"
#include "../../drivers/sensors/ina228.h"

// A plain coulomb count down from the top of charge, as a simple alternative
// to the EKF. It resets to full when the highest cell has been within
// BASIC_COUNT_FULL_MARGIN_mV of the working max, with the current settled near
// zero, for BASIC_COUNT_FULL_SETTLE_MS. Like the EKF, 0% is at the working min:
// crudely, the usable charge is the actual capacity times the share of the
// factory OCV table between the working limits.

// The inverter regulates around zero at the top, with readings swinging +-2A
// either side, so it's the averaged current that has to settle
#define BASIC_COUNT_CURRENT_TAU_MS 60000
#define BASIC_COUNT_FULL_CURRENT_mA 1000.0f
#define BASIC_COUNT_FULL_SETTLE_MS 600000

// model->charge_raw at the last full charge: what's been used since is the
// difference, so nothing accumulates (or rounds away) between ticks
static int64_t full_charge_raw;
static bool initialized = false;

static float current_avg_mA;
static millis_t last_tick_millis;
static millis_t near_full_since_millis;
static bool near_full = false;

// Factory table SOC (0-1) at a cell voltage
static float factory_soc(float cell_V) {
    return table_soc(ekf_factory_ocv_table(), cell_V);
}

// Charge from full (at the working max) down to this cell voltage at rest
static int64_t raw_below_full(const bms_model_t *model, float cell_V) {
    float share = factory_soc(get_cell_voltage_working_max_mV(model) / 1000.0f) - factory_soc(cell_V);
    return (int64_t)(share * (float)get_actual_capacity_mC(model)) * INA228_CHARGE_RAW_PER_MC;
}

// Near the top, settled, for long enough
static bool settled_at_full(const bms_model_t *model) {
    millis_t now = millis();
    millis_t dt = now - last_tick_millis;
    last_tick_millis = now;
    if (dt > BASIC_COUNT_CURRENT_TAU_MS) dt = BASIC_COUNT_CURRENT_TAU_MS;
    current_avg_mA += ((float)model->current_mA - current_avg_mA) * (float)dt / BASIC_COUNT_CURRENT_TAU_MS;

    bool near = model->cell_voltage_max_mV >= get_cell_voltage_working_max_mV(model) - BASIC_COUNT_FULL_MARGIN_mV
             && current_avg_mA < BASIC_COUNT_FULL_CURRENT_mA && current_avg_mA > -BASIC_COUNT_FULL_CURRENT_mA;
    if (near && !near_full) {
        near_full_since_millis = now;
    }
    near_full = near;
    return near && (now - near_full_since_millis) >= BASIC_COUNT_FULL_SETTLE_MS;
}

uint16_t basic_count_soc_estimate(bms_model_t *model) {
    if(!initialized && timestep() > 200 && model->high_voltages.battery > 0) {
        // Until the first full charge, start from the voltage
        full_charge_raw = model->charge_raw + raw_below_full(model, model->high_voltages.battery / NUM_CELLS);
        current_avg_mA = (float)model->current_mA;
        last_tick_millis = millis();
        initialized = true;
    }

    // Charge counted past full means full was really higher
    if(settled_at_full(model) || model->charge_raw > full_charge_raw) {
        full_charge_raw = model->charge_raw;
    }

    int64_t working_raw = raw_below_full(model, get_cell_voltage_working_min_mV(model) / 1000.0f);
    if(working_raw <= 0) {
        return 0; // working limits both off the same end of the table
    }
    float soc = 1.0f - (float)(full_charge_raw - model->charge_raw) / (float)working_raw;
    if(soc < 0.0f) {
        soc = 0.0f;
    } else if(soc > 1.0f) {
        soc = 1.0f;
    }
    return (uint16_t)(soc * 10000.0f + 0.5f);
}
