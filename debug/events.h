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

#include "../hw/time.h"

#include <stdint.h>

typedef struct {
    millis_t timestamp;
    uint64_t data;
    uint32_t count;
    uint16_t level;
} bms_event_slot_t;

enum bms_event_levels {
    BMS_EVENT_LEVEL_INFO = 0,
    BMS_EVENT_LEVEL_WARNING = 1,
    BMS_EVENT_LEVEL_CRITICAL = 2,
};

enum bms_event_types {
    BMS_EVENT_TYPE_CONTACTOR_FAULT = 0,
    BMS_EVENT_TYPE_OVERVOLTAGE = 1,
    BMS_EVENT_TYPE_UNDERVOLTAGE = 2,
    BMS_EVENT_TYPE_OVERTEMPERATURE = 3,
    BMS_EVENT_TYPE_UNDERTEMPERATURE = 4,
    BMS_EVENT_TYPE_CURRENT_SENSOR_FAULT = 5,
    BMS_EVENT_TYPE_COMMUNICATION_FAULT = 6,
    BMS_EVENT_TYPE_WATCHDOG_RESET = 7,
    BMS_EVENT_TYPE_SOFTWARE_FAULT = 8,
    BMS_EVENT_TYPE_HARDWARE_FAULT = 9,
    BMS_EVENT_TYPE_COUNT,
};
