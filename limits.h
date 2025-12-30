// Hard voltage limits, beyond which the battery will be cut off
#define CELL_VOLTAGE_HARD_MIN_MV 2500
#define CELL_VOLTAGE_HARD_MAX_MV 4300
// Soft voltage limits, beyond which the battery will only allow a low-current
// restoration charge/discharge
#define CELL_VOLTAGE_SOFT_MIN_MV 3000
#define CELL_VOLTAGE_SOFT_MAX_MV 4200

// Current limits to apply in the soft-limit region
#define OVERCHARGE_DISCHARGE_CURRENT_LIMIT_DA 10 // in 0.1A units
#define OVERDISCHARGE_CHARGE_CURRENT_LIMIT_DA 10 // in 0.1A units

// How much over the normal-region current limits we consider excessive.
#define CURRENT_LIMIT_ERROR_MARGIN_DA 5 // in 0.1A units

// How much excess normal-region charge/discharge (beyond current limits) we allow before
// cutting off the battery to protect it.
#define OVERCURRENT_BUFFER_LIMIT_DC 1000 // in 0.1C units

// How much excess charge/discharge we allow in the soft-limit region before
// cutting off the battery to protect it.
#define OVERCHARGE_BUFFER_LIMIT_DC 500 // in 0.1C units
#define OVERDISCHARGE_BUFFER_LIMIT_DC 500 // in 0.1C units
