#define LFP 1
#define NMC 2
#define CHEMISTRY NMC

// TODO - derate voltage limits based on temperature

#if CHEMISTRY == LFP
    // VOLTAGE

    // Hard voltage limits, beyond which the battery will be cut off
    #define CELL_VOLTAGE_HARD_MIN_mV 2500
    #define CELL_VOLTAGE_HARD_MAX_mV 3650
    // Soft voltage limits, beyond which the battery will only allow a low-current
    // restoration charge/discharge
    #define CELL_VOLTAGE_SOFT_MIN_mV 2800
    #define CELL_VOLTAGE_SOFT_MAX_mV 3350

    // TEMPERATURE

    // Temperatures at which charge power limit falls to zero
    #define MAX_CHARGE_TEMPERATURE_LIMIT_dC 500    // in 0.1C units
    #define MIN_CHARGE_TEMPERATURE_LIMIT_dC   0    // in 0.1C units
    // Charge current-per-temp linear derating (near min and max limits)
    #define CHARGE_TEMPERATURE_DERATE_dA_PER_dC 2 // in 0.1A per 0.1C units

    // Temperatures at which discharge power limit falls to zero
    #define MAX_DISCHARGE_TEMPERATURE_LIMIT_dC  550   // in 0.1C units
    #define MIN_DISCHARGE_TEMPERATURE_LIMIT_dC -150   // in 0.1C units
    // Discharge current-per-temp linear derating (near min and max limits)
    #define DISCHARGE_TEMPERATURE_DERATE_dA_PER_dC 2 // in 0.1A per 0.1C units

    // Hard temperature limits, beyond which the battery will be cut off
    #define TEMPERATURE_HARD_MAX_dC 600
    #define TEMPERATURE_HARD_MIN_dC -200
    // Soft temperature limits, beyond which the battery will warn
    #define TEMPERATURE_SOFT_MAX_dC 550
    #define TEMPERATURE_SOFT_MIN_dC -150

#elif CHEMISTRY == NMC
    // Hard voltage limits, beyond which the battery will be cut off
    #define CELL_VOLTAGE_HARD_MIN_mV 2800
    #define CELL_VOLTAGE_HARD_MAX_mV 4250
    // Soft voltage limits, beyond which the battery will only allow a low-current
    // restoration charge/discharge.
    #define CELL_VOLTAGE_SOFT_MIN_mV 3200
    #define CELL_VOLTAGE_SOFT_MAX_mV 4100

    // Absolute maximum current limits
    #define CHARGE_MAX_CURRENT_dA 500 // 50A
    #define DISCHARGE_MAX_CURRENT_dA 500 // 50A

    #define CHARGE_CELL_VOLTAGE_DERATE_dA_PER_SoC 40 // in 0.1A per percent SoC
    #define DISCHARGE_CELL_VOLTAGE_DERATE_dA_PER_SoC 20 // in 0.1A per percent SoC

    // Temperatures by which charge power limit falls to zero
    #define MAX_CHARGE_TEMPERATURE_LIMIT_dC  500    // in 0.1C units
    #define MIN_CHARGE_TEMPERATURE_LIMIT_dC 0       // in 0.1C units
    // Charge current-per-temp linear derating (near min and max limits)
    #define CHARGE_TEMPERATURE_DERATE_dA_PER_dC 2 // in 0.1A per 0.1C units

    // Temperatures by which discharge power limit falls to zero
    #define MAX_DISCHARGE_TEMPERATURE_LIMIT_dC  550   // in 0.1C units
    #define MIN_DISCHARGE_TEMPERATURE_LIMIT_dC -50    // in 0.1C units
    // Discharge current-per-temp linear derating (near min and max limits)
    #define DISCHARGE_TEMPERATURE_DERATE_dA_PER_dC 2 // in 0.1A per 0.1C units

    #define TEMPERATURE_HARD_MAX_dC 600
    #define TEMPERATURE_HARD_MIN_dC -100

    #define TEMPERATURE_SOFT_MAX_dC 500
    #define TEMPERATURE_SOFT_MIN_dC 0
#else
    #error "Unsupported CHEMISTRY"
#endif


// Cell presence bitmask (1 = present, 0 = not present, 32-bit groups, LSB = cell 0)
// #define CELL_PRESENCE_MASK { 0x7FFF, 0, 0, 0 }
// #define NUM_CELLS 15
// #define NUM_MODULE_VOLTAGES 1
// #define NUM_MODULE_TEMPS 1

// for the 96-cell NMC pack:
#define CELL_PRESENCE_MASK { 0xc7ff8f7f, 0xf3ffe3ff, 0xfcfff8ff, 0x1ffe3d }
#define NUM_CELLS 96
#define NUM_MODULE_VOLTAGES 8
#define NUM_MODULE_TEMPS 8
#define BATTERY_CAPACITY_AH 200

// for the 108-cell LFP pack:
// #define CELL_PRESENCE_MASK { 0x87ffbfff, 0xffffefff, 0xf9fffbff, 0x3ffeff }
// #define NUM_CELLS 108
// #define NUM_MODULE_VOLTAGES 8
// #define NUM_MODULE_TEMPS 8
// #define BATTERY_CAPACITY_AH 163



// Options: 
// - we could load in the cellvoltages as they come in, and use the presence mask for decoding
// - or use the presence mask to load in the cellvoltages (and modify the balance mask)
// i guess we check them more often (every model tick) vs BMB comms every 1s... so second option


#define BATTERY_VOLTAGE_HARD_MAX_mV (NUM_CELLS * CELL_VOLTAGE_HARD_MAX_mV)
#define BATTERY_VOLTAGE_HARD_MIN_mV (NUM_CELLS * CELL_VOLTAGE_HARD_MIN_mV)
#define BATTERY_VOLTAGE_SOFT_MAX_mV (NUM_CELLS * CELL_VOLTAGE_SOFT_MAX_mV)
#define BATTERY_VOLTAGE_SOFT_MIN_mV (NUM_CELLS * CELL_VOLTAGE_SOFT_MIN_mV)

// Max discrepancy between BMB cell voltages and measured terminal voltage
// (which should be calibrated away at zero current).
#define VOLTAGE_MISMATCH_THRESHOLD_mV 5000

#define MINIMUM_BALANCE_VOLTAGE_mV 3500

// Is voltage derating of current limits feasible, given the steepness of the
// voltage curves at top of charge? Probably not?



// So we stop at 4200mV. 90% SoC is about 4150mV, so we have about 50mV of
//
// TODO - do voltage-based-SoC-linearisation at the extremes (should be
// reasonably accurate) and limit by SoC

#define CHARGE_VOLTAGE_DERATE_dA_PER_mV 2
#define DISCHARGE_VOLTAGE_DERATE_dA_PER_mV 2

// Current limits to apply in the soft-limit region
#define OVERCHARGE_DISCHARGE_CURRENT_LIMIT_dA 10 // in 0.1A units
#define OVERDISCHARGE_CHARGE_CURRENT_LIMIT_dA 10 // in 0.1A units

// How much over the normal-region current limits we consider excessive.
#define CURRENT_LIMIT_ERROR_MARGIN_dA 5 // in 0.1A units

// How much excess normal-region charge/discharge (beyond current limits) we allow before
// cutting off the battery to protect it.
#define OVERCURRENT_BUFFER_LIMIT_dC 1000 // in 0.1Coulomb units

// How much excess charge/discharge we allow in the soft-limit region before
// cutting off the battery to protect it.
#define OVERCHARGE_BUFFER_LIMIT_dC 500 // in 0.1Coulomb units
#define OVERDISCHARGE_BUFFER_LIMIT_dC 500 // in 0.1Coulomb units










// How recent a reading must be to be considered valid. The slowest sensors are 10Hz so this should be fine.
#define STALENESS_THRESHOLD_MS 200

// Battery voltage samples every 200ms (plus jitter)
#define BATTERY_VOLTAGE_STALE_THRESHOLD_MS 300
#define OUTPUT_VOLTAGE_STALE_THRESHOLD_MS 300
#define CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS 300

// The current samples every ~530ms, so this should be sufficient
#define CURRENT_STALE_THRESHOLD_MS 1000

#define TEMPERATURE_STALE_THRESHOLD_MS(model) (  \
    model->cell_voltage_slow_mode ?              \
        CELL_TEMPERATURE_STALE_THRESHOLD_SLOW_MS  \
        : CELL_TEMPERATURE_STALE_THRESHOLD_MS )

// We only sample every 1.28s, and could have isospi/CRC issues, so be generous
// (also allow long enough for stable readings during balancing, which pauses every 12.8s)
#define CELL_VOLTAGE_STALE_THRESHOLD_MS 15000
// In slow mode we only sample every 81.92s, allow two bad reads
#define CELL_VOLTAGE_STALE_THRESHOLD_SLOW_MS 270000
#define CELL_TEMPERATURE_STALE_THRESHOLD_MS 5000
#define CELL_TEMPERATURE_STALE_THRESHOLD_SLOW_MS 270000


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
