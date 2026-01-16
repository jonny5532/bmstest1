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
    // restoration charge/discharge
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
#define CELL_PRESENCE_MASK {0, 0, 0, 0x7FFF}
// Number of 1s in the above bitmask
#define NUM_CELLS 15
// Number of modules of voltages to read
#define NUM_MODULE_VOLTAGES 1
// How many modules of temps to read
#define NUM_MODULE_TEMPS 1

// Options: 
// - we could load in the cellvoltages as they come in, and use the presence mask for decoding
// - or use the presence mask to load in the cellvoltages (and modify the balance mask)
// i guess we check them more often (every model tick) vs BMB comms every 1s... so second option


#define BATTERY_VOLTAGE_HARD_MAX_mV (NUM_CELLS * CELL_VOLTAGE_HARD_MAX_mV)
#define BATTERY_VOLTAGE_HARD_MIN_mV (NUM_CELLS * CELL_VOLTAGE_HARD_MIN_mV)
#define BATTERY_VOLTAGE_SOFT_MAX_mV (NUM_CELLS * CELL_VOLTAGE_SOFT_MAX_mV)
#define BATTERY_VOLTAGE_SOFT_MIN_mV (NUM_CELLS * CELL_VOLTAGE_SOFT_MIN_mV)

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

// We only sample every 1.28s, and could have isospi/CRC issues, so be generous
#define CELL_VOLTAGE_STALE_THRESHOLD_MS 5000

// In slow mode we only sample every 81.92s, allow two bad reads
#define CELL_VOLTAGE_STALE_THRESHOLD_SLOW_MS 270000

// Current samples every ~530ms?
#define CURRENT_STALE_THRESHOLD_MS 1000

#define CELL_TEMPERATURE_STALE_THRESHOLD_MS 5000
#define CELL_TEMPERATURE_STALE_THRESHOLD_SLOW_MS 270000