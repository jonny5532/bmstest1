#include "config/limits.h"
#include "app/model.h"
#include "lib/aema.h"

#include <stdint.h>
#include <stdio.h>
#include <math.h>

uint16_t calculate_cell_voltage_charge_current_limit(const bms_model_t *model) {
    uint16_t charge_limit = 0xFFFF;

#if CHEMISTRY == LFP
    if(model->cell_voltage_max_mV >= 3570) {
        charge_limit = 0;
    } else if(model->cell_voltage_max_mV > 3350) {
        // The LFP knee is very sharp - use a quadratic curve to model it
        uint32_t delta_from_top = 3570 - model->cell_voltage_max_mV;
        charge_limit = (((71*CHARGE_MAX_CURRENT_dA)/500) * delta_from_top * delta_from_top + ((3014*CHARGE_MAX_CURRENT_dA)/500) * delta_from_top) / 8192;
    }
#elif CHEMISTRY == NMC
    if(model->cell_voltage_max_mV >= 4200) {
        charge_limit = 0;
    } else {
        // Linear derate using CHARGE_VOLTAGE_DERATE_dA_PER_mV
        int32_t delta_from_max_mV = 4200 - model->cell_voltage_max_mV;
        int32_t derate_dA = delta_from_max_mV * CHARGE_VOLTAGE_DERATE_dA_PER_mV;
        if(derate_dA >= 0) {
            charge_limit = derate_dA;
        }
    }
#endif

    if(model->cell_voltage_max_mV > get_cell_voltage_soft_max_mV(model)) {
        // Above max cell voltage, stop charging
        charge_limit = 0;
    }
    if(model->cell_voltage_min_mV < CELL_VOLTAGE_HARD_MIN_mV) {
        // Below hard min cell voltage, stop charging
        charge_limit = 0;
    }
    if(model->cell_voltage_min_mV < get_cell_voltage_soft_min_mV(model)) {
        // Below min cell voltage, limit charge current
        if(charge_limit > OVERDISCHARGE_CHARGE_CURRENT_LIMIT_dA) {
            charge_limit = OVERDISCHARGE_CHARGE_CURRENT_LIMIT_dA;
        }
    }

    return charge_limit;
}

uint16_t calculate_cell_voltage_discharge_current_limit(bms_model_t *model) {
    uint16_t discharge_limit = 0xFFFF;

#if CHEMISTRY == LFP
    if(model->cell_voltage_min_mV <= 2770) {
        discharge_limit = 0;
    } else if(model->cell_voltage_min_mV < 3300) {
        int32_t delta_from_min_mV = model->cell_voltage_min_mV - 2700;
        int32_t numerator = delta_from_min_mV * (((1545*DISCHARGE_MAX_CURRENT_dA)/500) * delta_from_min_mV - ((112537*DISCHARGE_MAX_CURRENT_dA)/500));

        if (numerator < 0) {
            discharge_limit = 0;
        } else {
            discharge_limit = numerator >> 19;
        }
    }
#elif CHEMISTRY == NMC
    if(model->cell_voltage_min_mV <= 2900) {
        discharge_limit = 0;
    } else {
        // Linear derate using DISCHARGE_VOLTAGE_DERATE_dA_PER_mV
        int32_t delta_from_min_mV = model->cell_voltage_min_mV - 2900;
        int32_t derate_dA = delta_from_min_mV * DISCHARGE_VOLTAGE_DERATE_dA_PER_mV;
        if(derate_dA >= 0) {
            discharge_limit = derate_dA;
        }
    }
#endif

    if(model->cell_voltage_min_mV < get_cell_voltage_soft_min_mV(model)) {
        // Hit min cell voltage, stop discharging
        discharge_limit = 0;
    }
    if(model->cell_voltage_max_mV > get_cell_voltage_soft_max_mV(model)) {
        // Above max cell voltage, limit discharge current
        if(discharge_limit > OVERCHARGE_DISCHARGE_CURRENT_LIMIT_dA) {
            discharge_limit = OVERCHARGE_DISCHARGE_CURRENT_LIMIT_dA;
        }
    }

    // Working range lower limit (simple 100mV hysteresis)
    int32_t working_min_with_hysteresis_mV = get_cell_voltage_working_min_mV(model) + ((model->below_working_min) ? 100 : 0);
    if(model->cell_voltage_min_mV < working_min_with_hysteresis_mV) {
        discharge_limit = 0;
        model->below_working_min = true;
    } else {
        model->below_working_min = false;
    }

    return discharge_limit;
}

uint16_t calculate_delta_charge_current_limit(bms_model_t *model) {
    uint16_t charge_limit = UINT16_MAX;

    int16_t cell_voltage_delta_mV = model->cell_voltage_max_mV - model->cell_voltage_min_mV;
    int16_t delta_threshold_mV = (model->above_delta_threshold) ? CELL_VOLTAGE_DELTA_THRESHOLD_DOWN_mV : CELL_VOLTAGE_DELTA_THRESHOLD_UP_mV;
    if(cell_voltage_delta_mV > delta_threshold_mV) {
        if(charge_limit > CELL_VOLTAGE_DELTA_CURRENT_LIMIT_dA) {
            charge_limit = CELL_VOLTAGE_DELTA_CURRENT_LIMIT_dA;
        }
        model->above_delta_threshold = true;
    } else {
        model->above_delta_threshold = false;
    }

    return charge_limit;
}

uint16_t calculate_temperature_charge_current_limit(float temperature_min, float temperature_max) {
    if(isnan(temperature_min) || isnan(temperature_max) || temperature_max >= MAX_CHARGE_TEMPERATURE_LIMIT || temperature_min <= MIN_CHARGE_TEMPERATURE_LIMIT) {
        return 0;
    } else {
        // Limit is in dA
        uint16_t upper_charge_limit = 10.0f * (MAX_CHARGE_TEMPERATURE_LIMIT - temperature_max) * CHARGE_TEMPERATURE_DERATE_A_PER_C;
        uint16_t lower_charge_limit = 10.0f * (temperature_min - MIN_CHARGE_TEMPERATURE_LIMIT) * CHARGE_TEMPERATURE_DERATE_A_PER_C;
        return min(upper_charge_limit, lower_charge_limit);
    }
    return 0xFFFF;
}

uint16_t calculate_temperature_discharge_current_limit(float temperature_min, float temperature_max) {
    if(isnan(temperature_min) || isnan(temperature_max) || temperature_max >= MAX_DISCHARGE_TEMPERATURE_LIMIT || temperature_min <= MIN_DISCHARGE_TEMPERATURE_LIMIT) {
        return 0;
    } else {
        // Limit is in dA
        uint16_t upper_discharge_limit = 10.0f * (MAX_DISCHARGE_TEMPERATURE_LIMIT - temperature_max) * DISCHARGE_TEMPERATURE_DERATE_A_PER_C;
        uint16_t lower_discharge_limit = 10.0f * (temperature_min - MIN_DISCHARGE_TEMPERATURE_LIMIT) * DISCHARGE_TEMPERATURE_DERATE_A_PER_C;
        return min(upper_discharge_limit, lower_discharge_limit);
    }
    return 0xFFFF;
}

float working_charge_internal_resistance_uR = WORKING_LIMIT_INTERNAL_RESISTANCE_uR;
float working_charge_ceiling_dA = 30.0f; // 3A tethered ceiling
float working_charge_slow_alpha = 0.0005f; // ~10s time constant for approaching new target when far away
float working_charge_fast_alpha = 0.005f; // ~1s time constant for approaching new target when close
float working_charge_slow_threshold_dA = 15.0f; // 0.1A difference or less is considered close
float working_charge_fast_threshold_dA = 20.0f; // 1A difference


uint16_t calculate_working_charge_current_limit(bms_model_t *model) {
    // Current in dA (0.1A units)
    float current_dA = (float)model->current_mA / 100.0f;
    uint16_t working_max_mV = get_cell_voltage_working_max_mV(model);
    int32_t target_ocv_uV = (int32_t)working_max_mV * 1000;

    // 1. Estimate current cell OCV
    // V_ocv = V_meas - (I_mA * R_uR)/1000 (uV)
    int32_t guessed_ocv_uV = (int32_t)model->cell_voltage_max_mV * 1000 - (model->current_mA * WORKING_LIMIT_INTERNAL_RESISTANCE_uR) / 1000;

    // 2. Calculate target limit to maintain OCV at working max
    // I_limit_dA = (V_target - V_ocv) / R * 10.
    float target_limit_dA = 0.0f;
    if (guessed_ocv_uV < target_ocv_uV) {
        int32_t delta_uV = target_ocv_uV - guessed_ocv_uV;
        target_limit_dA = (float)delta_uV * 10.0f / WORKING_LIMIT_INTERNAL_RESISTANCE_uR;
    } else {
        target_limit_dA = 0.0f;
    }

    // Clip to absolute max charge
    if (target_limit_dA > CHARGE_MAX_CURRENT_dA) {
        target_limit_dA = (float)CHARGE_MAX_CURRENT_dA;
    }

    // 3. Apply Tapered Tethered Ceiling
    // To prevent rapid ramp-up of current when far from the target limit, we tether the 
    // allowed limit to the current being drawn. This tether is tightest when near 
    // the working voltage limit to prevent overshoot.
    // In the "near-max" region (within 20mV of working max), we use the fixed cushion.
    // Between 20mV and 100mV below max, the cushion tapers up to the global limit.
    float current_clamped_dA = max(current_dA, 0.0f);
    float delta_to_working_max_mV = (float)working_max_mV - (float)model->cell_voltage_max_mV;
    float cushion_dA = (float)CHARGE_MAX_CURRENT_dA; // Default to NO tether

    if (delta_to_working_max_mV <= 10.0f) {
        // Flat cushion within 10mV of working max
        cushion_dA = working_charge_ceiling_dA;
    } else if (delta_to_working_max_mV < 50.0f) {
        // Linear taper to CHARGE_MAX_CURRENT_dA (at 50mV delta)
        float taper_factor = (delta_to_working_max_mV - 10.0f) / (50.0f - 10.0f);
        cushion_dA = working_charge_ceiling_dA + taper_factor * ((float)CHARGE_MAX_CURRENT_dA - working_charge_ceiling_dA);
    }

    float tethered_ceiling_dA = current_clamped_dA + cushion_dA;
    if (target_limit_dA > tethered_ceiling_dA) {
        target_limit_dA = tethered_ceiling_dA;
    }

    if (target_limit_dA < 0.0f) {
        target_limit_dA = 0.0f;
    }

    // 4. Update smoothed value via AEMA
    // Use low smoothing (alpha_min) when close to the target, high smoothing (alpha_max) 
    // for larger deviations.
    // dA Units: alpha_min=0.01, alpha_max=0.2, noise_threshold=1 dA (0.1A), change_threshold=10 dA (1A)


    // At 50 samples/sec, a=0.002 gives a time constant of ~10s

    aema_update(
        &model->working_charge_current_limit_filtered_dA,
        NULL,
        target_limit_dA,
        working_charge_slow_alpha,
        working_charge_fast_alpha,
        working_charge_slow_threshold_dA,
        working_charge_fast_threshold_dA
    );

    return (uint16_t)model->working_charge_current_limit_filtered_dA;
}