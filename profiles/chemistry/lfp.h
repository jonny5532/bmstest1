#pragma once

// Base settings for LFP cells. Pack profiles include this and then add their
// own pack-specific settings (see profiles/README.md).

#include "chemistry/chemistry.h"

#define CHEMISTRY LFP

// TODO - derate voltage limits based on temperature

// VOLTAGE

// Hard voltage limits, beyond which the battery will be cut off
#define CELL_VOLTAGE_HARD_MIN_mV 2500
#define CELL_VOLTAGE_HARD_MAX_mV 3650
// Soft voltage limits, beyond which the battery will only allow a low-current
// restoration charge/discharge
#define DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV 2800
#define DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV 3400
// Working voltage range, which defines 0% and 100% SoC. This may be
// overridden by user settings.
#define DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV 2900
#define DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV 3350

#define CELL_VOLTAGE_DELTA_THRESHOLD_UP_mV 100
#define CELL_VOLTAGE_DELTA_THRESHOLD_DOWN_mV 50
#define CELL_VOLTAGE_DELTA_CURRENT_LIMIT_dA 10

// TODO - this is the NMC value, which is above CELL_VOLTAGE_HARD_MAX_mV, so LFP
// packs never balance unless the user setting overrides it.
#define DEFAULT_MINIMUM_BALANCING_VOLTAGE_mV 3770

// TEMPERATURE

// Temperatures at which charge power limit falls to zero
#define MAX_CHARGE_TEMPERATURE_LIMIT 50.0f
#define MIN_CHARGE_TEMPERATURE_LIMIT  0.0f
// Charge current-per-temp linear derating (near min and max limits)
#define CHARGE_TEMPERATURE_DERATE_A_PER_C 2.0f

// Temperatures at which discharge power limit falls to zero
#define MAX_DISCHARGE_TEMPERATURE_LIMIT  55.0f
#define MIN_DISCHARGE_TEMPERATURE_LIMIT -15.0f
// Discharge current-per-temp linear derating (near min and max limits)
#define DISCHARGE_TEMPERATURE_DERATE_A_PER_C 2.0f

// Hard temperature limits, beyond which the battery will be cut off
#define TEMPERATURE_HARD_MAX 60.0f
#define TEMPERATURE_HARD_MIN -20.0f
// Soft temperature limits, beyond which the battery will warn
#define TEMPERATURE_SOFT_MAX 55.0f
#define TEMPERATURE_SOFT_MIN -15.0f

// STATE OF CHARGE ESTIMATION (see bms/app/estimators/ekf.c for what each setting does)

// The basic coulomb count (bms/app/estimators/basic_count.c) resets to full when
// the highest cell has settled within this of the working max. Generous, as the
// inverter can stop short (one set to 3.45V settles at 3.42-3.43V, with a voltage
// limit offset), but keep the threshold above ~3.40V: an LFP pack rests at
// 3.32-3.37V across most of its range.
#define BASIC_COUNT_FULL_MARGIN_mV 25

// Learn the OCV curve only to report capacity, e.g. after a deliberate deep
// discharge. The flat middle carries no SOC information and hysteresis would
// bias a learned table, so the filter keeps the factory one (no OCV_LEARN_TABLE).
#define OCV_LEARN
// Where the learned capacity is read off: steep, and somewhere packs actually rest
#define OCV_LEARN_V_REF 3.25f // the bottom knee
#define OCV_DIFF_HALF_WIDTH 4

// Cell resistance, and how quickly the voltage offset decays. Tuned by replaying
// 10 days of 1s data from a 108S pack (kalman/replay, 27 Sep - 7 Oct 2026, with a
// deep discharge) after moving to the LFP_A midpoint curve: 1.2 to 2.5mOhm all did
// better than the old 0.3mOhm, which is the resistance a sustained current sees
// (~0.3mOhm instantly, ~2mOhm after a few minutes). At 1.8mOhm, the worst error
// from a correct start fell from 3.3% to 2.1%, from a 15% high start from 6.7% to
// 3.0%, and with a +150mA injected bias from 7.4% to 5.7%. The RMS error from a
// correct start is ~0.5% either way, and -150mA stays ~6.6%.
#define EKF_R_OHM 0.0018
#define EKF_OFFSET_TAU_S 81.0

// From replaying ~10 days of logged pack data through the filter (scripts in
// rpipico/kalman, next to this repo).
#define EKF_P0_AH   100.0f  // (10Ah)^2 when starting from the SOC stored in NVM. Too
                            // small and the first knee blames the starting error on the bias.
#define EKF_P0_AH_VOLTAGE 900.0f // (30Ah)^2 when starting from a voltage lookup instead
                            // (e.g. a reboot with a stale stored SOC): on the plateau that
                            // can be 15-20% out. Replaying the car's reboots, this let the
                            // next knee correct the SOC rather than the bias.
#define EKF_P0_OFFSET 1e-4f
#define EKF_P0_BIAS 1e-2f   // (100mA)^2
#define EKF_Q_AH    1e-7f
#define EKF_Q_OFFSET  1e-4f  // Q * tau sets the offset's spread; 3e-3 to 2e-2 all did well
#define EKF_Q_BIAS  1e-10f  // bias can wander ~3mA/day (e.g. with temperature)
#define EKF_R_V     1e-2f
#define EKF_MIN_OCV_SLOPE  1.5f
#define EKF_KNEE_OCV_SLOPE 5.0f
