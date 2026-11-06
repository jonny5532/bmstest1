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

#include "../../config/limits.h"
#include "../../sys/time/time.h"

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
// Format: X(name, level, escalate_after)
//
// where:
//  - name is the event name (ERR_ is prepended to the enum value)
//  - level is the initial event severity (although can escalate)
//  - escalate_after has different functions depending on level:
//    - for WARNING level events, it's the number of occurrences before
//      escalating to FATAL (zero means never escalate)
//    - for CRITICAL level events, it's the active time in milliseconds before
//      escalating to FATAL (zero means instant). Should be a multiple of 100ms.
//      Maximum 6553s.

#define EVENT_TYPES(X)                                          \
    X(CONTACTOR_POS_STUCK_OPEN, LEVEL_WARNING, 20)              \
    X(CONTACTOR_POS_STUCK_CLOSED, LEVEL_CRITICAL, 0)            \
    X(CONTACTOR_NEG_STUCK_OPEN, LEVEL_WARNING, 20)              \
    X(CONTACTOR_NEG_STUCK_CLOSED, LEVEL_CRITICAL, 0)            \
    X(CONTACTOR_PRE_STUCK_OPEN, LEVEL_CRITICAL, 0)              \
    X(CONTACTOR_PRE_STUCK_CLOSED, LEVEL_CRITICAL, 0)            \
    X(CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH, LEVEL_CRITICAL, 0)  \
    X(CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH, LEVEL_CRITICAL, 0)  \
    X(CONTACTOR_POS_UNEXPECTED_OPEN, LEVEL_WARNING, 0)          \
    X(CONTACTOR_NEG_UNEXPECTED_OPEN, LEVEL_WARNING, 0)          \
    X(CONTACTOR_CLOSING_FAILED, LEVEL_WARNING, 20)              \
    X(SUPPLY_VOLTAGE_STALE, LEVEL_CRITICAL, 2000)               \
    X(BATTERY_VOLTAGE_STALE, LEVEL_CRITICAL, 2000)              \
    X(BATTERY_TEMPERATURE_STALE, LEVEL_CRITICAL, 20000)         \
    X(CURRENT_STALE, LEVEL_CRITICAL, 2000)                      \
    X(CELL_VOLTAGES_STALE, LEVEL_CRITICAL, CELL_VOLTAGES_STALE_ESCALATION_MS) \
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
    X(CELL_VOLTAGE_VERY_HIGH, LEVEL_CRITICAL, CELL_VOLTAGE_FAULT_ESCALATION_MS) \
    X(CELL_VOLTAGE_LOW, LEVEL_WARNING, 0)                       \
    X(CELL_VOLTAGE_VERY_LOW, LEVEL_CRITICAL, CELL_VOLTAGE_FAULT_ESCALATION_MS) \
    X(SOFT_CHARGE_BUFFER_EXCEEDED, LEVEL_CRITICAL, 1000)        \
    X(OVERCURRENT_CHARGING, LEVEL_WARNING, 0 /*LEVEL_CRITICAL, 1000*/ )               \
    X(OVERCURRENT_DISCHARGING, LEVEL_WARNING, 0 /*LEVEL_CRITICAL, 1000*/ )            \
    X(BATTERY_TEMPERATURE_HIGH, LEVEL_WARNING, 0)               \
    X(BATTERY_TEMPERATURE_VERY_HIGH, LEVEL_CRITICAL, 1000)      \
    X(BATTERY_TEMPERATURE_LOW, LEVEL_WARNING, 0)                \
    X(BATTERY_TEMPERATURE_VERY_LOW, LEVEL_CRITICAL, 1000)       \
    X(VOLTAGE_MISMATCH, LEVEL_CRITICAL, 1000)                   \
                                                                \
    X(BMB_READ_ERROR, LEVEL_WARNING, 0)                         \
    X(BMB_CRC_MISMATCH, LEVEL_WARNING, 0)                       \
    X(CELL_VOLTAGE_GLITCH, LEVEL_WARNING, 0)                    \
    X(MODULE_TEMPERATURE_GLITCH, LEVEL_WARNING, 0)              \
                                                                \
    X(INVERTER_DETECTED, LEVEL_INFO, 0)                         \
                                                                \
    X(ESTOP_PRESSED, LEVEL_CRITICAL, 500)                       \
                                                                \
    X(BOOT_NORMAL, LEVEL_INFO, 0)                               \
    X(BOOT_WATCHDOG, LEVEL_WARNING, 0)                          \
    X(LOOP_OVERRUN, LEVEL_WARNING, 0)                           \
    X(RESTARTING, LEVEL_FATAL, 0)                               \
                                                                \
    /* Rev1 board events. New events must be appended at the */ \
    /* end of this list: the numeric event IDs are consumed  */ \
    /* by the HMI, so inserting mid-list breaks its decoding */ \
    X(LINK_VOLTAGE_MISMATCH, LEVEL_WARNING, 0)                  \
    X(FUSE_VOLTAGE_DROP_HIGH, LEVEL_WARNING, 0)                 \
    /* Upstream's (jonny5532) event 54: the cell presence    */ \
    /* seen on the BMBs doesn't match the cell map. Not      */ \
    /* raised here yet; listed so both firmwares number      */ \
    /* their events alike and one HMI decodes both           */ \
    X(BMB_PACK_MISMATCH, LEVEL_WARNING, 0)                      \
    X(ISOLATION_ASYMMETRY, LEVEL_WARNING, 0)                    \
    X(CONTACTOR_PORT_ENERGIZED, LEVEL_INFO, 0)                  \
    /* PCB revision strap doesn't match the BMS_BOARD build */  \
    X(BOARD_MISMATCH, LEVEL_FATAL, 0)                           \
    /* Closing held off as the supervisor isn't enabling the */ \
    /* contactor drivers (data: state, latch, reasons)       */ \
    X(SUPERVISOR_NOT_ENABLED, LEVEL_WARNING, 0)                 \
    /* The supervisor cut the contactor drivers while the    */ \
    /* contactors were being driven                          */ \
    X(SUPERVISOR_DISABLED_CONTACTORS, LEVEL_CRITICAL, 0)        \
    /* BMB module ID failed its CRC or check bits            */ \
    /* (data: failed slot mask, first bad raw word)          */ \
    X(BMB_MODULE_ID_CHECK_FAILED, LEVEL_WARNING, 0)             \
    /* A module ID isn't in the pack type table (data: ID)   */ \
    X(BMB_MODULE_ID_UNKNOWN, LEVEL_INFO, 0)                     \
    /* Known module IDs, but no pack type has them all and   */ \
    /* fits the profile: the BMS is built for another pack.  */ \
    /* (data: up to four IDs). See CUSTOM_PACK_MODULE_IDS.   */ \
    X(BMB_MODULE_ID_CONFLICT, LEVEL_FATAL, 0)

// What each event's 8 bytes of data hold: the reading that raised its latest occurrence (kept
// until the next occurrence). The HMI passes them to Home Assistant as hex, and the dashboard
// decodes them with these lines (CellKeeper-HA's tools/make_events.py reads them), so keep one
// for every event and change it with the call site.
//
//   // data NAME: <field>; <field>...       the fields, each <type>[@bit] <unit> <name>
//   // data NAME = 0x<value>: <meaning>     a fixed value instead (a stale reading...)
//
// Types: i8 u8 i16 u16 i32 u32 i64 u64, at bit 0 unless @bit is given; hex16, hex32 and hex64
// print as hex. Units: mV, mA, dA (0.1 A), dC (0.1 coulomb), dDegC (0.1 C), C (whole C), ms,
// index (0-based: shown from 1), count, id, or - for a plain number.
//
// data CONTACTOR_POS_STUCK_OPEN: i32 mV voltage across it
// data CONTACTOR_POS_STUCK_OPEN = 0x1000000000000000: its voltage reading is stale
// data CONTACTOR_POS_STUCK_CLOSED: i32 mV voltage across it
// data CONTACTOR_POS_STUCK_CLOSED = 0x1000000000000000: its voltage reading is stale
// data CONTACTOR_NEG_STUCK_OPEN: i32 mV voltage across it
// data CONTACTOR_NEG_STUCK_OPEN = 0x1000000000000000: its voltage reading is stale
// data CONTACTOR_NEG_STUCK_CLOSED: i32 mV voltage across it
// data CONTACTOR_NEG_STUCK_CLOSED = 0x1000000000000000: its voltage reading is stale
// data CONTACTOR_PRE_STUCK_OPEN: none
// data CONTACTOR_PRE_STUCK_CLOSED: none
// data CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH: i32 mV output; i32@32 mV pack or link
// data CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH = 0x2000000000000000: the output voltage reading is stale
// data CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH = 0x5000000000000000: the link voltage reading is stale
// data CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH: i32 mA current rise
// data CONTACTOR_POS_UNEXPECTED_OPEN: i32 mV voltage across it
// data CONTACTOR_POS_UNEXPECTED_OPEN = 0x1000000000000000: its voltage reading is stale
// data CONTACTOR_NEG_UNEXPECTED_OPEN: i32 mV voltage across it
// data CONTACTOR_NEG_UNEXPECTED_OPEN = 0x2000000000000000: its voltage reading is stale
// data CONTACTOR_NEG_UNEXPECTED_OPEN = 0x3000000000000000: its voltage reading is stale (closing)
// data CONTACTOR_CLOSING_FAILED = 0x1000000000000000: precharge failed
// data CONTACTOR_CLOSING_FAILED = 0x2000000000000000: weld test: something conducts with both open
// data CONTACTOR_CLOSING_FAILED = 0x3000000000000000: the negative contactor didn't close
// data CONTACTOR_CLOSING_FAILED = 0x4000000000000000: weld test: the positive conducts with only the negative closed
// data CONTACTOR_CLOSING_FAILED = 0x5000000000000000: the positive contactor didn't close
// data SUPPLY_VOLTAGE_STALE = 0x1000000000000000: the 3.3 V reading is stale
// data SUPPLY_VOLTAGE_STALE = 0x2000000000000000: the 5 V reading is stale
// data SUPPLY_VOLTAGE_STALE = 0x3000000000000000: the 12 V reading is stale
// data SUPPLY_VOLTAGE_STALE = 0x4000000000000000: the contactor supply reading is stale
// data BATTERY_VOLTAGE_STALE = 0x1000000000000000: no fresh pack voltage
// data BATTERY_TEMPERATURE_STALE = 0x1000000000000000: no fresh temperatures
// data CURRENT_STALE = 0x1000000000000000: no fresh current
// data CURRENT_STALE = 0x4000000000000000: no fresh current (contactors)
// data CELL_VOLTAGES_STALE = 0x1000000000000000: no fresh cell voltages
// data SUPPLY_VOLTAGE_3V3_LOW: u16 mV 3.3 V supply
// data SUPPLY_VOLTAGE_3V3_HIGH: u16 mV 3.3 V supply
// data SUPPLY_VOLTAGE_5V_LOW: u16 mV 5 V supply
// data SUPPLY_VOLTAGE_5V_HIGH: u16 mV 5 V supply
// data SUPPLY_VOLTAGE_12V_LOW: u16 mV 12 V supply
// data SUPPLY_VOLTAGE_12V_HIGH: u16 mV 12 V supply
// data SUPPLY_VOLTAGE_CONTACTOR_LOW: u16 mV contactor supply
// data SUPPLY_VOLTAGE_CONTACTOR_VERY_LOW: u16 mV contactor supply
// data SUPPLY_VOLTAGE_CONTACTOR_HIGH: u16 mV contactor supply
// data BATTERY_VOLTAGE_HIGH: i32 mV pack
// data BATTERY_VOLTAGE_VERY_HIGH: i32 mV pack
// data BATTERY_VOLTAGE_LOW: i32 mV pack
// data BATTERY_VOLTAGE_VERY_LOW: i32 mV pack
// data CELL_VOLTAGE_HIGH: i16 mV highest cell
// data CELL_VOLTAGE_VERY_HIGH: i16 mV highest cell
// data CELL_VOLTAGE_LOW: i16 mV lowest cell
// data CELL_VOLTAGE_VERY_LOW: i16 mV lowest cell
// data SOFT_CHARGE_BUFFER_EXCEEDED: i32 dC charge past the soft limits
// data OVERCURRENT_CHARGING: i32 mA current; u16@32 dA limit
// data OVERCURRENT_DISCHARGING: i32 mA current; u16@32 dA limit
// data BATTERY_TEMPERATURE_HIGH: i32 dDegC hottest module
// data BATTERY_TEMPERATURE_VERY_HIGH: i32 dDegC hottest module
// data BATTERY_TEMPERATURE_LOW: i32 dDegC coldest module
// data BATTERY_TEMPERATURE_VERY_LOW: i32 dDegC coldest module
// data VOLTAGE_MISMATCH: u32 mV pack; u32@32 mV sum of the cells
// data BMB_READ_ERROR: u8@56 - read (1 cells, 2 temperatures, 3 module IDs); u8 index bank
// data BMB_CRC_MISMATCH: u8@56 - read (1 cells, 2 temperatures, 3 module IDs); hex16@16 - received; hex16 - calculated
// data CELL_VOLTAGE_GLITCH: u16 mV new; u16@16 mV before; u16@32 index cell
// data MODULE_TEMPERATURE_GLITCH: i16 dDegC new; i16@16 dDegC before; u16@32 index module
// data INVERTER_DETECTED: hex32 id CAN frame
// data ESTOP_PRESSED: none
// data BOOT_NORMAL: none
// data BOOT_WATCHDOG: none
// data LOOP_OVERRUN: i32 ms loop took
// data RESTARTING: none
// data LINK_VOLTAGE_MISMATCH: u32 mV link; u32@32 mV pack
// data LINK_VOLTAGE_MISMATCH = 0x3000000000000000: the link or pack voltage reading is stale
// data FUSE_VOLTAGE_DROP_HIGH: i32 mV drop
// data BMB_PACK_MISMATCH: u32 count channels that differ from the cell map
// data ISOLATION_ASYMMETRY: i32 count midpoint offset
// data CONTACTOR_PORT_ENERGIZED: i32 mV output
// data BOARD_MISMATCH: u16 - built for; u16@16 - strap reads
// data SUPERVISOR_NOT_ENABLED: hex16 - reasons; hex16@16 - latched reasons; u8@32 - state
// data SUPERVISOR_DISABLED_CONTACTORS: hex16 - reasons; hex16@16 - latched reasons; u8@32 - state
// data BMB_MODULE_ID_CHECK_FAILED: hex16 - first bad word; hex16@16 - failed slots
// data BMB_MODULE_ID_UNKNOWN: hex16 id module ID
// data BMB_MODULE_ID_CONFLICT: hex16 id ID 1; hex16@16 id ID 2; hex16@32 id ID 3; hex16@48 id ID 4

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

extern bms_event_slot_t bms_event_slots[];

static inline int16_t get_event_count(bms_event_type_t type) {
    if(type >= ERR_HIGHEST) return 0;
    return bms_event_slots[type].count;
}

// Gets the current level of the specified event type.
static inline bms_event_level_t get_event_level(bms_event_type_t type) {
    if(type >= ERR_HIGHEST) return LEVEL_NONE;
    return (bms_event_level_t)bms_event_slots[type].level;
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
