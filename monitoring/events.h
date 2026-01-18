#pragma once

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
    uint16_t count;
    uint16_t level;
    uint16_t accumulator;
} bms_event_slot_t;

typedef enum {
    // Not in an error state
    LEVEL_NONE = 0,
    // Informational
    LEVEL_INFO = 1,
    // A cause for concern, but doesn't affect operation
    LEVEL_WARNING = 2,
    // Serious problem affecting operation, will trigger a fault state soon
    LEVEL_CRITICAL = 3,
    // In a fault state, operation ceases
    LEVEL_FATAL = 4
} bms_event_level_t;

// Pass a macro in as X to render the event list in different ways
//
// Format: X(name, default_level, escalate_after)
//
// where:
//  - name is the event name (ERR_ is prepended to the enum value)
//  - default_level is the default level for the event (although can be
//    overridden)
//  - escalate_after has different functions depending on level:
//    - for WARNING level events, it's the number of occurrences before
//      escalating to CRITICAL (zero means never escalate)
//    - for CRITICAL level events, it's the active time in milliseconds before
//      escalating to FATAL (zero means instant). Should be a multiple of 100ms.
//      Maximum 6553s.

// Todo - also have a max-warnings-before-critical counter?

#define EVENT_TYPES(X)                                          \
    X(CONTACTOR_POS_STUCK_OPEN, LEVEL_CRITICAL, 0)              \
    X(CONTACTOR_POS_STUCK_CLOSED, LEVEL_CRITICAL, 0)            \
    X(CONTACTOR_NEG_STUCK_OPEN, LEVEL_CRITICAL, 0)              \
    X(CONTACTOR_NEG_STUCK_CLOSED, LEVEL_CRITICAL, 0)            \
    X(CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH, LEVEL_CRITICAL, 0)  \
    X(CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH, LEVEL_CRITICAL, 0)  \
    X(CONTACTOR_POS_UNEXPECTED_OPEN, LEVEL_WARNING, 0)          \
    X(CONTACTOR_NEG_UNEXPECTED_OPEN, LEVEL_WARNING, 0)          \
    X(CONTACTOR_CLOSING_FAILED, LEVEL_WARNING, 20)               \
    X(SUPPLY_VOLTAGE_STALE, LEVEL_CRITICAL, 2000)               \
    X(BATTERY_VOLTAGE_STALE, LEVEL_CRITICAL, 2000)              \
    X(BATTERY_TEMPERATURE_STALE, LEVEL_CRITICAL, 20000)         \
    X(CURRENT_STALE, LEVEL_CRITICAL, 2000)                      \
    X(CELL_VOLTAGES_STALE, LEVEL_CRITICAL, 1000)                \
    X(SUPPLY_VOLTAGE_3V3_LOW, LEVEL_WARNING, 0)                 \
    X(SUPPLY_VOLTAGE_3V3_HIGH, LEVEL_WARNING, 0)                \
    X(SUPPLY_VOLTAGE_5V_LOW, LEVEL_WARNING, 0)                  \
    X(SUPPLY_VOLTAGE_5V_HIGH, LEVEL_WARNING, 0)                 \
    X(SUPPLY_VOLTAGE_12V_LOW, LEVEL_WARNING, 0)                 \
    X(SUPPLY_VOLTAGE_12V_HIGH, LEVEL_WARNING, 0)                \
    X(SUPPLY_VOLTAGE_CONTACTOR_LOW, LEVEL_WARNING, 0)           \
    X(SUPPLY_VOLTAGE_CONTACTOR_VERY_LOW, LEVEL_CRITICAL, 5000)  \
    X(SUPPLY_VOLTAGE_CONTACTOR_HIGH, LEVEL_WARNING, 0)          \
    X(BATTERY_VOLTAGE_HIGH, LEVEL_WARNING, 0)                   \
    X(BATTERY_VOLTAGE_VERY_HIGH, LEVEL_CRITICAL, 1000)          \
    /* Low voltage escalates after an hour, to stop a tiny  */  \
    /* discharge eventually depleting the pack.             */  \
    X(BATTERY_VOLTAGE_LOW, LEVEL_CRITICAL, 3600000)             \
    X(BATTERY_VOLTAGE_VERY_LOW, LEVEL_CRITICAL, 1000)           \
    X(CELL_VOLTAGE_HIGH, LEVEL_WARNING, 0)                      \
    X(CELL_VOLTAGE_VERY_HIGH, LEVEL_CRITICAL, 1000)             \
    X(CELL_VOLTAGE_LOW, LEVEL_WARNING, 0)                       \
    X(CELL_VOLTAGE_VERY_LOW, LEVEL_CRITICAL, 1000)              \
    X(BATTERY_TEMPERATURE_HIGH, LEVEL_WARNING, 0)               \
    X(BATTERY_TEMPERATURE_VERY_HIGH, LEVEL_CRITICAL, 1000)      \
    X(BATTERY_TEMPERATURE_LOW, LEVEL_WARNING, 0)                \
    X(BATTERY_TEMPERATURE_VERY_LOW, LEVEL_CRITICAL, 1000)       \
    X(RESTARTING, LEVEL_FATAL, 0)

typedef enum {
#define X(name, _1, _2) ERR_##name,
    EVENT_TYPES(X)
#undef X
    ERR_HIGHEST
} bms_event_type_t;


void record_bms_event(bms_event_type_t event_type, uint64_t data, bool repeat);
void clear_bms_event(bms_event_type_t type);
void print_bms_events();
uint16_t get_highest_event_level();
void events_tick();

static inline int16_t get_event_count(bms_event_type_t type) {
    extern bms_event_slot_t bms_event_slots[];
    if(type >= ERR_HIGHEST) return 0;
    return bms_event_slots[type].count;
}

// Logs an event of the specified type with the provided data. Once raised,
// subsequent calls will not increase the count.
static inline void raise_bms_event(bms_event_type_t type, uint64_t data) {
    record_bms_event(type, data, false);
}

// Logs an event of the specified type with the provided data. Each call
// will increase the count.
static inline void count_bms_event(bms_event_type_t type, uint64_t data) {
    record_bms_event(type, data, true);
}

// Checks whether the first argument is true.
// If not, logs an event of the specified type and level, with the provided data.
// If the condition is true, clears any existing event of that type.
// Returns the value of the condition.
static inline bool confirm(bool success, bms_event_type_t event_type, uint64_t data) {
    if(!success) {
        raise_bms_event(
            event_type,
            data
        );
    } else {
        clear_bms_event(event_type);
    }

    return success;
}

// Like confirm(), but only logs/clears the event if do_confirm is true.
static inline bool check_or_confirm(bool success, bool do_confirm, bms_event_type_t event_type, uint64_t data) {
    if(do_confirm) return confirm(success, event_type, data);
    return success;
}
