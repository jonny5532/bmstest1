#pragma once

#define LFP 1
#define NMC 2
//#define CHEMISTRY NMC

#define BMS_DESK 1
#define BMS_BLUETESLA 2
#define BMS_LFP1 3

#ifndef BMS_PROFILE
#define BMS_PROFILE BMS_BLUETESLA
#endif


#if BMS_PROFILE == BMS_DESK
#define CHEMISTRY NMC
#define CELL_PRESENCE_MASK { 0x7FFF, 0, 0, 0 }
#define NUM_CELLS 15
#define NUM_MODULE_VOLTAGES 1
#define NUM_MODULE_TEMPS 1
#define NAMEPLATE_CAPACITY_AH 1
#define INA228_SHUNT_CAL 3088
    
#elif BMS_PROFILE == BMS_BLUETESLA
    // for the 96-cell NMC pack:
#define CHEMISTRY NMC
#define CELL_PRESENCE_MASK { 0xc7ff8f7f, 0xf3ffe3ff, 0xfcfff8ff, 0x1ffe3d }
#define NUM_CELLS 96
#define NUM_MODULE_VOLTAGES 8
#define NUM_MODULE_TEMPS 8
#define NAMEPLATE_CAPACITY_AH 147
#define INA228_SHUNT_CAL 332
#define INVERTER_MODEL_STRING "CellKeeper Tesla NMC 1\x00\x00\x00\x00\x00";

#elif BMS_PROFILE == BMS_LFP1

#define CHEMISTRY LFP
#define CELL_PRESENCE_MASK { 0x87ffbfff, 0xffffefff, 0xf9fffbff, 0x3ffeff }
#define NUM_CELLS 108
#define NUM_MODULE_VOLTAGES 8
#define NUM_MODULE_TEMPS 8
#define NAMEPLATE_CAPACITY_AH 163
#define INA228_SHUNT_CAL 332
#define PRECHARGE_ON_NEGATIVE 1
#define INVERTER_MODEL_STRING "CellKeeper Tesla LFP 1\x00\x00\x00\x00\x00";

#else
    #error "Unsupported BMS_PROFILE"
#endif


// PCB revision selection (orthogonal to the pack profile above)

#define BMS_BOARD_REV0 1
#define BMS_BOARD_REV1 2

#ifndef BMS_BOARD
#define BMS_BOARD BMS_BOARD_REV0
#endif

#if BMS_BOARD == BMS_BOARD_REV1
// Rev1 adds a second ADS1115 (0x49) sensing the Link and Drive Unit rails
#define HAS_ADS1115_SECONDARY 1
// Rev1's default contactor arrangement precharges via the FC negative contactor
#ifndef PRECHARGE_ON_NEGATIVE
#define PRECHARGE_ON_NEGATIVE 1
#endif
// Rev1 uses 501:1 dividers (12M/24k): 501 * 1.024/32768 V per count
#define ADS1115_DEFAULT_MUL 0.015656f
#elif BMS_BOARD == BMS_BOARD_REV0
// Rev0 divider ratio is ~416:1
#define ADS1115_DEFAULT_MUL 0.013f
#else
    #error "Unsupported BMS_BOARD"
#endif


// TODO - derate voltage limits based on temperature

#if CHEMISTRY == LFP
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

    #define CHARGE_MAX_CURRENT_dA 500 // 50A
    #define DISCHARGE_MAX_CURRENT_dA 500 // 50A

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

#elif CHEMISTRY == NMC
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

    // Absolute maximum current limits
    #define CHARGE_MAX_CURRENT_dA 300 // 30A
    #define DISCHARGE_MAX_CURRENT_dA 300 // 30A

    #define CHARGE_CELL_VOLTAGE_DERATE_dA_PER_SoC 40 // in 0.1A per percent SoC
    #define DISCHARGE_CELL_VOLTAGE_DERATE_dA_PER_SoC 20 // in 0.1A per percent SoC

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
#else
    #error "Unsupported CHEMISTRY"
#endif

// Cell presence bitmask (1 = present, 0 = not present, 32-bit groups, LSB = cell 0)
// #define CELL_PRESENCE_MASK { 0x7FFF, 0, 0, 0 }
// #define NUM_CELLS 15
// #define NUM_MODULE_VOLTAGES 1
// #define NUM_MODULE_TEMPS 1


// for the 108-cell LFP pack:
// #define CELL_PRESENCE_MASK { 0x87ffbfff, 0xffffefff, 0xf9fffbff, 0x3ffeff }
// #define NUM_CELLS 108
// #define NUM_MODULE_VOLTAGES 8
// #define NUM_MODULE_TEMPS 8
// #define NAMEPLATE_CAPACITY_AH 163


// A deliberate overestimate of internal resistance of an individual cell, for working current limits
#define WORKING_LIMIT_INTERNAL_RESISTANCE_uR 50


// Options: 
// - we could load in the cellvoltages as they come in, and use the presence mask for decoding
// - or use the presence mask to load in the cellvoltages (and modify the balance mask)
// i guess we check them more often (every model tick) vs BMB comms every 1s... so second option


#define BATTERY_VOLTAGE_HARD_MAX_mV (NUM_CELLS * CELL_VOLTAGE_HARD_MAX_mV)
#define BATTERY_VOLTAGE_HARD_MIN_mV (NUM_CELLS * CELL_VOLTAGE_HARD_MIN_mV)

// Max discrepancy between BMB cell voltages and measured terminal voltage
// (which should be calibrated away at zero current). If in slow mode there
// could be a considerable time delay between cellvoltage and pack samples.
#define VOLTAGE_MISMATCH_THRESHOLD_mV 20000

#define DEFAULT_MINIMUM_BALANCING_VOLTAGE_mV 3770
//3830

// Is voltage derating of current limits feasible, given the steepness of the
// voltage curves at top of charge? Probably not?



// So we stop at 4200mV. 90% SoC is about 4150mV, so we have about 50mV of
//
// TODO - do voltage-based-SoC-linearisation at the extremes (should be
// reasonably accurate) and limit by SoC

#define CHARGE_VOLTAGE_DERATE_dA_PER_mV 5
#define DISCHARGE_VOLTAGE_DERATE_dA_PER_mV 5

// Current limits to apply in the soft-limit region
#define OVERCHARGE_DISCHARGE_CURRENT_LIMIT_dA 50 // in 0.1A units
#define OVERDISCHARGE_CHARGE_CURRENT_LIMIT_dA 50 // in 0.1A units

// How much over the normal-region current limits we consider excessive.
#define CURRENT_LIMIT_ERROR_MARGIN_dA 10 // in 0.1A units

// How much excess normal-region charge/discharge (beyond current limits) we allow before
// cutting off the battery to protect it.
#define OVERCURRENT_BUFFER_LIMIT_dC 1000 // in 0.1Coulomb units

// How much excess charge/discharge we allow in the soft-limit region before
// cutting off the battery to protect it.

// We dont't tolerate much overcharge (1A for 200 seconds)
#define OVERCHARGE_BUFFER_LIMIT_dC 2000 // in 0.1Coulomb units

// 5000 dC (50C) is 200mA (quiescent current for a big inverter) for ~40 minutes,
// which should be long enough to get the inverter [dis]charging.
#define OVERDISCHARGE_BUFFER_LIMIT_dC 5000 // in 0.1Coulomb units










// How recent readings must be to be considered valid.

// Battery voltage samples every ~100ms
#define BATTERY_VOLTAGE_STALE_THRESHOLD_MS 1000
#define OUTPUT_VOLTAGE_STALE_THRESHOLD_MS 1000
#define CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS 1000

// The current samples every ~531ms, so this should be sufficient. Precharge
// must wait this long to ensure it isn't using pre-precharge readings.
#define CURRENT_STALE_THRESHOLD_MS 800

#define TEMPERATURE_STALE_THRESHOLD_MS(model) (  \
    (model)->cell_voltage_slow_mode ?              \
        CELL_TEMPERATURE_STALE_THRESHOLD_SLOW_MS  \
        : CELL_TEMPERATURE_STALE_THRESHOLD_MS )

// We only sample every 1.28s, and could have isospi/CRC issues, so be generous
// (also allow long enough for stable readings during balancing)
#define CELL_VOLTAGE_STALE_THRESHOLD_MS 15000
// In slow mode we only sample every 81.92s, allow two bad reads
#define CELL_VOLTAGE_STALE_THRESHOLD_SLOW_MS 270000
#define CELL_TEMPERATURE_STALE_THRESHOLD_MS 5000
#define CELL_TEMPERATURE_STALE_THRESHOLD_SLOW_MS 270000

// Threshold for detecting a cell voltage glitch (sudden jump in one cycle)
#define CELL_VOLTAGE_GLITCH_THRESHOLD_mV 100

// Threshold for detecting a module temperature glitch (sudden jump in one cycle)
#define MODULE_TEMPERATURE_GLITCH_THRESHOLD_dC 100

// The 3V3 measurement is really just measuring the divider resistor tolerances...
#define SUPPLY_VOLTAGE_3V3_MIN_MV 3200
#define SUPPLY_VOLTAGE_3V3_MAX_MV 3600
#define SUPPLY_VOLTAGE_5V_MIN_MV 4400
#define SUPPLY_VOLTAGE_5V_MAX_MV 5600
#define SUPPLY_VOLTAGE_12V_MIN_MV 11000
#define SUPPLY_VOLTAGE_12V_MAX_MV 16000
#define SUPPLY_VOLTAGE_CONTACTOR_HARD_MIN_MV 10000 // Shut down if below this (risk of contactors opening)
#define SUPPLY_VOLTAGE_CONTACTOR_SOFT_MIN_MV 12000
#define SUPPLY_VOLTAGE_CONTACTOR_MAX_MV 16000
#define SUPPLY_VOLTAGE_STALE_THRESHOLD_MS 2000


// CONTACTOR BEHAVIOUR THRESHOLDS

// Contactor weld test thresholds

// How long to wait for voltage to settle during contactor tests
#define CONTACTORS_TEST_WAIT_MS 1000
// How long to wait after a failed precharge (for the PTCs to cool down)
#define CONTACTORS_FAILED_PRECHARGE_TIMEOUT_MS 20000
// How long to wait after a failed contactor test (to avoid rapid cycling)
#define CONTACTORS_FAILED_TEST_TIMEOUT_MS 20000
// Max voltage across a contactor to consider it closed
#define CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV 2000
// Min voltage across a contactor to consider it open
#define CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV 3000 // was 5000
#if BMS_BOARD == BMS_BOARD_REV1
// Rev1 measures directly across the pos (Link Positive) contactor, so the
// tight generic thresholds apply
#define CONTACTORS_POS_CLOSED_VOLTAGE_THRESHOLD_MV 2000
#define CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV 3000
// Max discrepancy between battery and link voltage while closed (measured by
// two different ADCs which may be independently [un]calibrated)
#define CONTACTORS_LINK_MISMATCH_THRESHOLD_MV 16000
// Max sustained drop across the F4 fuse/jumper (while current is flowing)
// before warning that it may have blown
#define FUSE_DROP_WARNING_THRESHOLD_MV 10000
#else
// Pos contactor has wider tolerances due to the way it is measured
#define CONTACTORS_POS_CLOSED_VOLTAGE_THRESHOLD_MV 5000
#define CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV 10000
#endif


// Precharge thresholds

// It is difficult to get a good reading across the positive contactor due to
// the ADC arrangement, so we have to tolerate a wider precharge voltage
// differential than we'd like.

// Max steady-state PTC current (TDK C1451 in 2S3P) will be at 450mA at 16V.

// Thus we can close with a current < 225mA and voltage < 16V, since hot PTCs
// would result in at least 25V or higher.

// Largest allowable pack current to consider precharge successful
#define PRECHARGE_SUCCESS_MAX_MA 225
// Maximum voltage difference to consider precharge successful (allowing for uncalibrated contactors)
#define PRECHARGE_SUCCESS_MAX_MV 16000
#if BMS_BOARD == BMS_BOARD_REV1
// Rev1 also senses the link rail directly, so the voltage across the PTC + FC
// negative contactor path (link vs output) can be held to a tighter limit
#define PRECHARGE_SUCCESS_LINK_MAX_MV 8000
#endif


// Contactor opening thresholds

// Current below which we can instantly open contactors
#define CONTACTORS_INSTANT_OPEN_MA 1000
// Current below which we will begrudgingly open contactors after the wait is up
#define CONTACTORS_DELAYED_OPEN_MA 5000
// How long we wait for the current to fall before failing to open contactors.
// NOTE: Some inverters (Deye!) take a long time to react to current changes (20
// seconds!).
#define CONTACTORS_OPEN_TIMEOUT_MS 30000
// How long we wait when force-opening contactors
#define CONTACTORS_FORCE_OPEN_TIMEOUT_MS 2000
