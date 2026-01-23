#pragma once

#include <math.h>
#include <stdint.h>

typedef struct {
    // --- State Vector x ---
    // x[0] = Ah_used (Consumed Charge in Ah)
    // x[1] = V_c1 (Polarization Voltage in V)
    // x[2] = Capacity (Total Battery Capacity in Ah)
    float x[3];

    // --- Covariance Matrix P (3x3) ---
    float P[3][3];

    // --- Tuning Parameters ---
    float Q[3];    // Process Noise Variances (Diagonal)
    float R;       // Measurement Noise Variance
    
    // --- Model Parameters ---
    float R0;      // Ohmic Resistance
    float R1;      // Polarization Resistance
    float C1;      // Polarization Capacitance

} EKF;

// Initialize the filter
void ekf_init(EKF *ekf, float initial_soc, float initial_capacity);

// Run one iteration of the filter
// current_amps: Positive = Discharge, Negative = Charge
// voltage_measured: Terminal voltage
void ekf_step(EKF *ekf, float charge_Ah, float current_amps, float voltage_measured);

// Helper to get current SOC
float ekf_get_soc(EKF *ekf);

uint32_t ekf_tick(int32_t charge_mC, int32_t current_mA, int32_t voltage_mV);
