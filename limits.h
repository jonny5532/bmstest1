// Hard voltage limits, beyond which the battery will be cut off
#define CELL_VOLTAGE_HARD_MIN_mV 2500
#define CELL_VOLTAGE_HARD_MAX_mV 4300
// Soft voltage limits, beyond which the battery will only allow a low-current
// restoration charge/discharge
#define CELL_VOLTAGE_SOFT_MIN_mV 3000
#define CELL_VOLTAGE_SOFT_MAX_mV 4200

// Cell presence bitmask (1 = present, 0 = not present, 32-bit groups, LSB = cell 0)
#define CELL_PRESENCE_MASK {0, 0, 0, 0x7FFF}
// Number of 1s in the above bitmask
#define NUM_CELLS 15

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

#define CHARGE_VOLTAGE_DERATE_dA_PER_mV 2
#define DISCHARGE_VOLTAGE_DERATE_dA_PER_mV 2

// Current limits to apply in the soft-limit region
#define OVERCHARGE_DISCHARGE_CURRENT_LIMIT_dA 10 // in 0.1A units
#define OVERDISCHARGE_CHARGE_CURRENT_LIMIT_dA 10 // in 0.1A units

// How much over the normal-region current limits we consider excessive.
#define CURRENT_LIMIT_ERROR_MARGIN_dA 5 // in 0.1A units

// How much excess normal-region charge/discharge (beyond current limits) we allow before
// cutting off the battery to protect it.
#define OVERCURRENT_BUFFER_LIMIT_dC 1000 // in 0.1C units

// How much excess charge/discharge we allow in the soft-limit region before
// cutting off the battery to protect it.
#define OVERCHARGE_BUFFER_LIMIT_dC 500 // in 0.1C units
#define OVERDISCHARGE_BUFFER_LIMIT_dC 500 // in 0.1C units

// Simple linear temperature derating of current limits

// Temperatures by which charge power limit falls to zero
#define MAX_CHARGE_TEMPERATURE_LIMIT_dC  450    // in 0.1C units
#define MIN_CHARGE_TEMPERATURE_LIMIT_dC -100    // in 0.1C units
// Charge current-per-temp derating rate
#define CHARGE_TEMPERATURE_DERATE_dA_PER_dC 2 // in 0.1A per 0.1C units

// Temperatures by which discharge power limit falls to zero
#define MAX_DISCHARGE_TEMPERATURE_LIMIT_dC  500   // in 0.1C units
#define MIN_DISCHARGE_TEMPERATURE_LIMIT_dC -100   // in 0.1C units
// Discharge current-per-temp derating rate
#define DISCHARGE_TEMPERATURE_DERATE_dA_PER_dC 2 // in 0.1A per 0.1C units











// How recent a reading must be to be considered valid. The slowest sensors are 10Hz so this should be fine.
#define STALENESS_THRESHOLD_MS 200

