#pragma once

// Sanity checks on the selected pack profile, so that a typo or a bad tweak in
// a profile causes a build failure. Included by config/limits.h straight after
// the profile.

// Settings every profile must provide

#ifndef CHEMISTRY
#error "Profile must define CHEMISTRY (include chemistry/lfp.h or chemistry/nmc.h)"
#endif
#ifndef CELL_PRESENCE_WORDS
#error "Profile must define CELL_PRESENCE_WORDS"
#endif
#ifndef NUM_CELLS
#error "Profile must define NUM_CELLS"
#endif
#ifndef NUM_MODULES
#error "Profile must define NUM_MODULES"
#endif
#ifndef NUM_MODULE_TEMPS
#error "Profile must define NUM_MODULE_TEMPS"
#endif
#ifndef NAMEPLATE_CAPACITY_AH
#error "Profile must define NAMEPLATE_CAPACITY_AH"
#endif
// The capacity the SOC estimator uses until one is saved over the HMI. A new
// pack starts at the label's, so that's the default.
#ifndef ACTUAL_CAPACITY_AH
#define ACTUAL_CAPACITY_AH NAMEPLATE_CAPACITY_AH
#endif
#ifndef INA228_SHUNT_CAL
#error "Profile must define INA228_SHUNT_CAL"
#endif
#ifndef CHARGE_MAX_CURRENT_dA
#error "Profile must define CHARGE_MAX_CURRENT_dA"
#endif
#ifndef DISCHARGE_MAX_CURRENT_dA
#error "Profile must define DISCHARGE_MAX_CURRENT_dA"
#endif

#if !defined(EKF_P0_AH) || !defined(EKF_P0_AH_VOLTAGE) || !defined(EKF_P0_OFFSET) || !defined(EKF_P0_BIAS) \
    || !defined(EKF_Q_AH) || !defined(EKF_Q_OFFSET) || !defined(EKF_Q_BIAS) || !defined(EKF_R_V) \
    || !defined(EKF_MIN_OCV_SLOPE) || !defined(EKF_KNEE_OCV_SLOPE) \
    || !defined(EKF_R_OHM) || !defined(EKF_OFFSET_TAU_S) \
    || !defined(OCV_DIFF_HALF_WIDTH) || !defined(OCV_LEARN_V_REF)
#error "Profile is missing an SOC estimation setting (see chemistry/lfp.h or chemistry/nmc.h)"
#endif
#if defined(OCV_LEARN_TABLE) && !defined(OCV_LEARN)
#error "OCV_LEARN_TABLE needs OCV_LEARN"
#endif

#if CHEMISTRY != LFP && CHEMISTRY != NMC
#error "CHEMISTRY must be LFP or NMC"
#endif

// Voltage limits must nest: hard outside soft, soft outside working

#if !(CELL_VOLTAGE_HARD_MIN_mV <= DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV)
#error "DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV is below CELL_VOLTAGE_HARD_MIN_mV"
#endif
#if !(DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV <= DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV)
#error "DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV is below DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV"
#endif
#if !(DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV < DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV)
#error "DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV must be below DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV"
#endif
#if !(DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV <= DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV)
#error "DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV is above DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV"
#endif
#if !(DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV <= CELL_VOLTAGE_HARD_MAX_mV)
#error "DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV is above CELL_VOLTAGE_HARD_MAX_mV"
#endif

// Temperature limits likewise, with the charge and discharge cut-offs inside
// the hard limits

_Static_assert(TEMPERATURE_HARD_MIN <= TEMPERATURE_SOFT_MIN, "TEMPERATURE_SOFT_MIN is below TEMPERATURE_HARD_MIN");
_Static_assert(TEMPERATURE_SOFT_MIN < TEMPERATURE_SOFT_MAX, "TEMPERATURE_SOFT_MIN must be below TEMPERATURE_SOFT_MAX");
_Static_assert(TEMPERATURE_SOFT_MAX <= TEMPERATURE_HARD_MAX, "TEMPERATURE_SOFT_MAX is above TEMPERATURE_HARD_MAX");
_Static_assert(TEMPERATURE_HARD_MIN <= MIN_CHARGE_TEMPERATURE_LIMIT
            && MIN_CHARGE_TEMPERATURE_LIMIT < MAX_CHARGE_TEMPERATURE_LIMIT
            && MAX_CHARGE_TEMPERATURE_LIMIT <= TEMPERATURE_HARD_MAX,
               "Charge temperature limits must be ordered and inside the hard limits");
_Static_assert(TEMPERATURE_HARD_MIN <= MIN_DISCHARGE_TEMPERATURE_LIMIT
            && MIN_DISCHARGE_TEMPERATURE_LIMIT < MAX_DISCHARGE_TEMPERATURE_LIMIT
            && MAX_DISCHARGE_TEMPERATURE_LIMIT <= TEMPERATURE_HARD_MAX,
               "Discharge temperature limits must be ordered and inside the hard limits");

// Pack layout

#define PROFILE_POPCOUNT4_(a, b, c, d) \
    (__builtin_popcount(a) + __builtin_popcount(b) + __builtin_popcount(c) + __builtin_popcount(d))
#define PROFILE_POPCOUNT4(...) PROFILE_POPCOUNT4_(__VA_ARGS__)
_Static_assert(PROFILE_POPCOUNT4(CELL_PRESENCE_WORDS) == NUM_CELLS,
               "CELL_PRESENCE_WORDS must have exactly NUM_CELLS bits set");

_Static_assert(NUM_MODULE_TEMPS >= 1, "NUM_MODULE_TEMPS must be at least 1");
_Static_assert(NAMEPLATE_CAPACITY_AH > 0, "NAMEPLATE_CAPACITY_AH must be positive");
_Static_assert(ACTUAL_CAPACITY_AH >= 1 && ACTUAL_CAPACITY_AH <= 1000, "ACTUAL_CAPACITY_AH must be 1 to 1000Ah");
_Static_assert(CHARGE_MAX_CURRENT_dA > 0 && DISCHARGE_MAX_CURRENT_dA > 0, "Max currents must be positive");

// SOC estimation

_Static_assert(EKF_R_OHM >= 0, "EKF_R_OHM must not be negative");
_Static_assert(EKF_OFFSET_TAU_S > 0, "EKF_OFFSET_TAU_S must be positive");
_Static_assert(EKF_R_V > 0, "EKF_R_V must be positive");
_Static_assert(OCV_DIFF_HALF_WIDTH >= 1, "OCV_DIFF_HALF_WIDTH must be at least 1");
