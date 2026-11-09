#pragma once

// Base settings for NMC cells. Pack profiles include this and then add their
// own pack-specific settings (see profiles/README.md).

#include "chemistry/chemistry.h"

#define CHEMISTRY NMC

// TODO - derate voltage limits based on temperature

// VOLTAGE

// Hard voltage limits, beyond which the battery will be cut off
#define CELL_VOLTAGE_HARD_MIN_mV 2700
#define CELL_VOLTAGE_HARD_MAX_mV 4250
// Soft voltage limits, beyond which the battery will only allow a low-current
// restoration charge/discharge.
#define DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV 3000
#define DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV 4200
// Working voltage range, which defines 0% and 100% SoC. This may be
// overridden by user settings.
#define DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV 3300
#define DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV 4050

#define CELL_VOLTAGE_DELTA_THRESHOLD_UP_mV 100
#define CELL_VOLTAGE_DELTA_THRESHOLD_DOWN_mV 50
#define CELL_VOLTAGE_DELTA_CURRENT_LIMIT_dA 10

#define CHARGE_CELL_VOLTAGE_DERATE_dA_PER_SoC 40 // in 0.1A per percent SoC
#define DISCHARGE_CELL_VOLTAGE_DERATE_dA_PER_SoC 20 // in 0.1A per percent SoC

// Is voltage derating of current limits feasible, given the steepness of the
// voltage curves at top of charge? Probably not?

// So we stop at 4200mV. 90% SoC is about 4150mV, so we have about 50mV of
//
// TODO - do voltage-based-SoC-linearisation at the extremes (should be
// reasonably accurate) and limit by SoC

#define CHARGE_VOLTAGE_DERATE_dA_PER_mV 5
#define DISCHARGE_VOLTAGE_DERATE_dA_PER_mV 5

#define DEFAULT_MINIMUM_BALANCING_VOLTAGE_mV 3770
//3830

// TEMPERATURE

// Temperatures by which charge power limit falls to zero
#define MAX_CHARGE_TEMPERATURE_LIMIT  50.0f
#define MIN_CHARGE_TEMPERATURE_LIMIT   0.0f
// Charge current-per-temp linear derating (near min and max limits)
#define CHARGE_TEMPERATURE_DERATE_A_PER_C 2.0f

// Temperatures by which discharge power limit falls to zero
#define MAX_DISCHARGE_TEMPERATURE_LIMIT  55.0f
#define MIN_DISCHARGE_TEMPERATURE_LIMIT  -5.0f
// Discharge current-per-temp linear derating (near min and max limits)
#define DISCHARGE_TEMPERATURE_DERATE_A_PER_C 2.0f

#define TEMPERATURE_HARD_MAX  60.0f
#define TEMPERATURE_HARD_MIN -10.0f

#define TEMPERATURE_SOFT_MAX  50.0f
#define TEMPERATURE_SOFT_MIN   0.0f

// STATE OF CHARGE ESTIMATION (see bms/app/estimators/ekf.c for what each setting does)

// The basic coulomb count (bms/app/estimators/basic_count.c) resets to full when
// the highest cell has settled within this of the working max. Generous, as the
// inverter can stop short; near the top, ~10mV is about 1% SOC, but only a pack
// that has settled there counts as full.
#define BASIC_COUNT_FULL_MARGIN_mV 25

#define OCV_LEARN       // learn a per-pack correction to the OCV table as we go...
#define OCV_LEARN_TABLE // ...and use it, with the SOC pinned at full charge
// Where the learned capacity is read off: steep, and somewhere packs actually rest
#define OCV_LEARN_V_REF 3.50f
#define OCV_DIFF_HALF_WIDTH 1

// Cell resistance, and how quickly the voltage offset decays. Tuned by replaying
// 10 days of 1s data from an aged 96S pack through ekf_tick (kalman/replay, with
// --tick and a warm-up pass to train the OCV learner). NMC's sloped curve leaves
// little for the offset to do, so it's nearly off, and the resistance is what a
// sustained current sees: the pack's ~1mOhm within 20s grows to ~2.4mOhm over a
// few minutes. 2-3mOhm did best; at the old 1.3mOhm with Q_OFFSET 1e-6, an 8%
// starting error took 45 minutes rather than 2 to clear, and an injected 150mA
// current bias pushed the worst error from 3.8% to 4.3%.
#define EKF_R_OHM 0.0025
#define EKF_OFFSET_TAU_S 81.0

// From replaying ~94 days of a 96S BLUETESLA pack, plus synthetic fresher
// packs driven the same way. NMC's sloped curve keeps the voltage
// informative everywhere, so no gate. Q_AH:Q_OFFSET trades trusting the count
// (better when the curve is off) against trusting voltage (better when the
// capacity or shunt offset is off); this ratio was the best worst case.
// Q_OFFSET used to be 1e-2, which let the offset soak up every voltage residual and
// left a 15% starting error two days to clear; now it's minutes. Anything from 0
// to 1e-7 does as well as 1e-8.
#define EKF_P0_AH   100.0f  // (10Ah)^2
#define EKF_P0_AH_VOLTAGE EKF_P0_AH // a voltage start is good on a sloped curve
#define EKF_P0_OFFSET 1e-4f
#define EKF_P0_BIAS 0.0f    // Off: with a mismatched curve it learns curve error too
#define EKF_Q_AH    5e-8f
#define EKF_Q_OFFSET  1e-8f
#define EKF_Q_BIAS  0.0f
#define EKF_R_V     1e-2f
#define EKF_MIN_OCV_SLOPE  0.0f
#define EKF_KNEE_OCV_SLOPE 0.0f
