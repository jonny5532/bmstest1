#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct bms_model bms_model_t;

// The filter runs in double precision. After days of coulomb counting on the
// LFP plateau the SoC and bias errors become almost perfectly correlated, and
// in float the rounding of the tiny per-second covariance increments pushes
// that correlation past 1. The next voltage update then makes the variances
// negative and the bias runs away (seen replaying a reboot onto the plateau).
typedef struct {
    // --- State Vector x ---
    // x[0] = Ah_used (Consumed Charge in Ah)
    // x[1] = V_off, voltage the OCV curve and resistance don't explain (V)
    // x[2] = Current sensor bias in A (measured = true + bias). Only weakly
    //        observable; it is learned mainly when the voltage pins the SoC at
    //        the top of charge, from how far the coulomb count has drifted.
    double x[3];

    // --- Covariance Matrix P (3x3) ---
    double P[3][3];

    // --- Tuning Parameters ---
    double Q[3];    // Process Noise Variances (Diagonal)
    double R;       // Measurement Noise Variance
    double min_ocv_slope; // Skip the voltage update where |dOCV/dSOC| (V per unit SOC) is below this...
    double knee_ocv_slope; // ...unless the measured voltage puts us on a part of the curve this steep

    // --- Model Parameters ---
    // Capacity (Ah), which maps Ah_used to SOC. Not a state: as a state it soaked
    // up OCV curve error rather than measuring capacity. Fade is taken up by the
    // learned OCV table instead (ocv_learn.c).
    double cap;
    double R_ohm;         // Cell resistance to a sustained current
    double offset_tau_s;  // Time constant V_off decays back to zero with

    // --- Output Scaling Cache ---
    uint16_t prev_min_mV;
    uint16_t prev_max_mV;
    float prev_soc_min;
    float prev_soc_mul;

    bool initialized;
} ekf_t;

// The HMI reads x and P as HMI_ARRAY_EKF_STATE and HMI_ARRAY_EKF_COV
_Static_assert(sizeof(((ekf_t *)0)->x) == 3 * sizeof(double), "update HMI_EKF_STATE_* to match");

// Initialize the filter
void ekf_init(ekf_t *ekf, double initial_soc, double initial_capacity, double initial_bias_A);

// Run one iteration of the filter
// charge_Ah, current_amps: as measured (including any sensor bias).
// Positive = Charge, Negative = Discharge
// voltage_measured: Terminal voltage
void ekf_step(ekf_t *ekf, double charge_Ah, double current_amps, double voltage_measured);

// Helper to get current SOC
double ekf_get_soc(ekf_t *ekf);
void ekf_set_soc(bms_model_t *model, uint16_t soc);
// Set the actual capacity (mC; zero for the profile's) and use it from now on
void ekf_set_capacity(bms_model_t *model, uint32_t capacity_mC);
// Forget the learned current bias, e.g. because the sensor offset was just recalibrated
void ekf_reset_bias(bms_model_t *model);

// The learned current sensor bias (A).
double ekf_bias_A(const bms_model_t *model);

uint32_t ekf_tick(bms_model_t *model, float charge_Ah, int32_t current_mA, int32_t voltage_mV);

// The chemistry's factory OCV table (101 points, index = SOC in percent)
const float *ekf_factory_ocv_table(void);

// Lookups on a 101-point OCV table (index = SOC in percent)
float table_soc(const float *table, float ocv);
double table_ocv(const float *table, double soc);

void ekf_print_state(ekf_t *ekf);
