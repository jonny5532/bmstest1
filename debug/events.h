// Options:

// fixed length array with a slot for each event
//  - count number of occurences
//  - timestamp of last occurence
//  - adjustable level?
// a bit memory heavy, only stores one timestamp per event but does count
// repeats reliably, can have an external flag for current max level. harder
// to store event-specific data as it'll sit empty most of the time.

// ring buffer of events
//  - type
//  - timestamp
//  - level
//  - extra info
// more flexible, can store extra data per event, but events will eventually be rotated out.

// separate ring buffers for different event levels
//  - means old criticals won't get lost

// separate ring buffer for event timestamps+data?

// hmm, the fixed-event-list approach does require a sort, which is unideal.
// maybe we can do a lazy n^2 selection-sort.
// so could iterate list to start with, then selection-sort the remainder? (based on timestamp)
// should we ensure no two events have the same timestamp?

#include "../hw/chip/time.h"

#include <stdint.h>

typedef struct {
    millis_t timestamp;
    union {
        uint8_t data8[8];
        uint16_t data16[4];
        uint32_t data32[2];
        uint64_t data64;
    };
    uint32_t count;
    uint16_t level;
} bms_event_slot_t;

typedef enum {
    LEVEL_NONE = 0,
    LEVEL_INFO = 1,
    LEVEL_WARNING = 2,
    LEVEL_CRITICAL = 3,
} bms_event_level_t;

typedef enum {
    ERR_CONTACTOR_POS_STUCK_OPEN,
    ERR_CONTACTOR_POS_STUCK_CLOSED,
    ERR_CONTACTOR_NEG_STUCK_OPEN,
    ERR_CONTACTOR_NEG_STUCK_CLOSED,
    ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH,
    ERR_CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH,
    ERR_CONTACTOR_PRECHARGE_NEG_OPEN,
    ERR_BATTERY_VOLTAGE_STALE,
    ERR_BATTERY_VOLTAGE_OUT_OF_RANGE,
    ERR_BATTERY_TEMPERATURE_STALE,
    ERR_BATTERY_TEMPERATURE_OUT_OF_RANGE,
    ERR_CURRENT_STALE,
    ERR_HIGHEST,
} bms_event_type_t;
