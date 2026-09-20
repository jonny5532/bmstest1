#pragma once

/*
 * Custom multi-battery CAN protocol - shared definitions
 * =======================================================
 *
 * This header is the single source of truth for the wire format. It is
 * deliberately self-contained (only <stdint.h>/<stdbool.h>) so that it can be
 * dropped verbatim into a controller implementation (e.g. Battery Emulator).
 *
 * The full specification, including controller-side guidance, lives in
 * docs/custom_can_protocol.md. Short version:
 *
 *  - All frames use 29-bit extended identifiers, laid out as
 *
 *        bits 28..14   prefix   (15 bits)  chosen by the controller
 *        bits 13..8    type     ( 6 bits)  fixed by this protocol
 *        bits  7..0    node     ( 8 bits)  chosen by the controller
 *
 *    Because the type field sits above the node field, CAN arbitration
 *    prioritises by message type first: a STATUS frame from any battery beats
 *    a cell-voltage page from any other battery.
 *
 *  - Only the discovery block (prefix CUSTOM_CAN_DISCOVERY_PREFIX) is fixed.
 *    Batteries ANNOUNCE there; the controller replies with ASSIGN, giving the
 *    battery the prefix and node it should use from then on.
 *
 *  - All multi-byte fields are big-endian. Unused/reserved bytes are sent as
 *    zero and must be ignored on receipt. Unknown message types must be
 *    ignored. int16 telemetry values of CUSTOM_CAN_INT16_NA mean "no data".
 */

#include <stdbool.h>
#include <stdint.h>

#define CUSTOM_CAN_PROTOCOL_VERSION 1

/* ------------------------------------------------------------------------ */
/* Identifier layout                                                         */
/* ------------------------------------------------------------------------ */

#define CUSTOM_CAN_ID_MASK      0x1FFFFFFFu
#define CUSTOM_CAN_PREFIX_SHIFT 14
#define CUSTOM_CAN_PREFIX_MAX   0x7FFFu
#define CUSTOM_CAN_TYPE_SHIFT   8
#define CUSTOM_CAN_TYPE_MASK    0x3Fu
#define CUSTOM_CAN_NODE_MASK    0xFFu

// Node id 0 is "unassigned" and is never used on the wire for a battery.
// Node id 0xFF addresses every battery sharing the prefix (controller->battery
// frames only). Batteries are assigned 1..254.
#define CUSTOM_CAN_NODE_NONE 0x00u
#define CUSTOM_CAN_NODE_ALL  0xFFu
#define CUSTOM_CAN_NODE_MIN  0x01u
#define CUSTOM_CAN_NODE_MAX  0xFEu

static inline uint32_t custom_can_make_id(uint16_t prefix, uint8_t type, uint8_t node) {
    return ((uint32_t)(prefix & CUSTOM_CAN_PREFIX_MAX) << CUSTOM_CAN_PREFIX_SHIFT)
         | ((uint32_t)(type & CUSTOM_CAN_TYPE_MASK) << CUSTOM_CAN_TYPE_SHIFT)
         | (uint32_t)node;
}

static inline uint16_t custom_can_id_prefix(uint32_t id) {
    return (uint16_t)((id & CUSTOM_CAN_ID_MASK) >> CUSTOM_CAN_PREFIX_SHIFT);
}

static inline uint8_t custom_can_id_type(uint32_t id) {
    return (uint8_t)((id >> CUSTOM_CAN_TYPE_SHIFT) & CUSTOM_CAN_TYPE_MASK);
}

static inline uint8_t custom_can_id_node(uint32_t id) {
    return (uint8_t)(id & CUSTOM_CAN_NODE_MASK);
}

/* ------------------------------------------------------------------------ */
/* Discovery block (the only fixed identifiers)                              */
/* ------------------------------------------------------------------------ */

// Prefix 0x7384 == identifiers 0x1CE10000..0x1CE13FFF. Controllers must never
// assign this prefix to a battery.
#define CUSTOM_CAN_DISCOVERY_PREFIX 0x7384u

// Battery -> everyone. Low byte of the identifier is the low byte of the
// battery's serial, so that several batteries announcing at once still have
// distinct identifiers (identical identifiers with different payloads are a
// CAN protocol violation). Filter with mask 0x1FFFFF00.
#define CUSTOM_CAN_TYPE_ANNOUNCE 0x00u
#define CUSTOM_CAN_ID_ANNOUNCE_BASE 0x1CE10000u

// Controller -> one battery (selected by serial in the payload).
#define CUSTOM_CAN_TYPE_ASSIGN 0x01u
#define CUSTOM_CAN_ID_ASSIGN 0x1CE10100u

// ANNOUNCE payload (DLC 8):
//   0     bits 7..4 protocol version, bits 3..0 link state (custom_can_link_t)
//   1     currently assigned node (0 if unassigned)
//   2..5  serial32 (see custom_can_serial32())
//   6..7  currently assigned prefix (0 if unassigned)
typedef enum {
    CUSTOM_CAN_LINK_UNASSIGNED = 0,
    CUSTOM_CAN_LINK_ASSIGNED = 1,
} custom_can_link_t;

// ASSIGN payload (DLC 8):
//   0..3  serial32 of the battery being addressed (must match exactly)
//   4..5  prefix to use (0..0x7FFF, not CUSTOM_CAN_DISCOVERY_PREFIX)
//   6     node to use (1..254). 0 releases the battery back to unassigned.
//   7     flags (CUSTOM_CAN_ASSIGN_FLAG_*)
//
// Send ASSIGN once per ANNOUNCE you want to act on, not periodically.
// Re-sending an identical assignment is harmless.
#define CUSTOM_CAN_ASSIGN_FLAG_STOP_ON_TIMEOUT 0x01u // request STOP if the controller goes quiet
#define CUSTOM_CAN_ASSIGN_RATE_SHIFT_MASK      0x30u // bits 5..4: slow non-essential telemetry by 2^n
#define CUSTOM_CAN_ASSIGN_RATE_SHIFT_POS       4

/* ------------------------------------------------------------------------ */
/* Message types (within an assigned prefix)                                 */
/* ------------------------------------------------------------------------ */

// Controller -> battery: 0x00..0x0F (highest priority)
#define CUSTOM_CAN_TYPE_COMMAND 0x00u
#define CUSTOM_CAN_TYPE_REQUEST 0x01u
// 0x02..0x0F reserved

// Battery -> controller: 0x10..0x3F, roughly ordered by importance
#define CUSTOM_CAN_TYPE_STATUS        0x10u // liveness + state, 200 ms
#define CUSTOM_CAN_TYPE_LIMITS        0x11u // voltage & current limits, 200 ms
#define CUSTOM_CAN_TYPE_MEASUREMENTS  0x12u // pack V / I / SoC, 200 ms
#define CUSTOM_CAN_TYPE_ENERGY        0x13u // SoH, capacities, 1 s
#define CUSTOM_CAN_TYPE_CELL_STATS    0x14u // min/max/mean cell voltage, 1 s
#define CUSTOM_CAN_TYPE_TEMPERATURES  0x15u // min/max temperature, 1 s
#define CUSTOM_CAN_TYPE_HV_VOLTAGES   0x16u // output & contactor voltages, 1 s
#define CUSTOM_CAN_TYPE_SUPPLY        0x17u // internal supply rails, 2 s
#define CUSTOM_CAN_TYPE_BALANCING     0x18u // balancing summary, 1 s
#define CUSTOM_CAN_TYPE_EVENT         0x19u // rolling event table, 250 ms
#define CUSTOM_CAN_TYPE_ESTIMATORS    0x1Au // alternative SoC estimates, 2 s
#define CUSTOM_CAN_TYPE_CONFIG        0x1Bu // static configuration, 5 s
#define CUSTOM_CAN_TYPE_SERIAL        0x1Cu // full 64-bit board id, 5 s
#define CUSTOM_CAN_TYPE_SETTINGS      0x1Du // user settings, 5 s
#define CUSTOM_CAN_TYPE_CELL_LIMITS   0x1Eu // cell voltage thresholds, 5 s
// 0x1F reserved
#define CUSTOM_CAN_TYPE_MODULE_TEMPS  0x20u // paged, 250 ms per page
#define CUSTOM_CAN_TYPE_CELL_VOLTAGES 0x21u // paged, 100 ms per page
// 0x22..0x2F reserved for future paged data
// 0x30..0x3F reserved for experimental / vendor use

#define CUSTOM_CAN_TYPE_CONTROLLER_MAX 0x0Fu

static inline bool custom_can_type_is_from_controller(uint8_t type) {
    return type <= CUSTOM_CAN_TYPE_CONTROLLER_MAX;
}

/* ------------------------------------------------------------------------ */
/* Controller -> battery payloads                                            */
/* ------------------------------------------------------------------------ */

// COMMAND (DLC >= 4). Send at least every CUSTOM_CAN_COMMAND_MAX_PERIOD_MS to
// every assigned node (or to node 0xFF). This is the controller's heartbeat.
//   0     desired state (custom_can_desired_state_t)
//   1     flags, reserved (0)
//   2..3  sequence number, incremented per frame; echoed in STATUS
//   4..7  reserved (0)
//
// The desired state is a level, not an edge: repeat it in every frame. The
// battery acts when the received value differs from the last value it
// received, so a lost frame is harmless and a local operator override is not
// fought by the next heartbeat. To re-assert RUN after a local stop, send
// STOP then RUN.
typedef enum {
    CUSTOM_CAN_DESIRED_NONE = 0, // heartbeat only, leave run state alone
    CUSTOM_CAN_DESIRED_STOP = 1, // open contactors (gracefully, once current has fallen)
    CUSTOM_CAN_DESIRED_RUN = 2,  // close contactors and allow current
} custom_can_desired_state_t;

// REQUEST (DLC >= 1): ask for a battery->controller frame to be sent now.
//   0     message type (0x10..0x3F)
//   1     page (paged types only, optional; 0xFF or absent = current page)
// Bounded to one frame per request; use it for snapshots, not polling.

/* ------------------------------------------------------------------------ */
/* Battery -> controller payloads                                            */
/* ------------------------------------------------------------------------ */

// STATUS (DLC 8):
//   0     system state (custom_can_system_state_t)
//   1     contactor state (custom_can_contactor_state_t)
//   2     flags (CUSTOM_CAN_STATUS_FLAG_*)
//   3     bits 2..0 highest event level (custom_can_event_level_t)
//         bits 5..4 commanded state (custom_can_desired_state_t, last received)
//   4..5  last received COMMAND sequence number
//   6     event type with the highest level, 0xFF if none
//   7     reserved
// Sent every 200 ms, and immediately after a COMMAND is received or the
// system/contactor state changes.
#define CUSTOM_CAN_STATUS_FLAG_CURRENT_ENABLED  0x01u // contactors closed and current allowed
#define CUSTOM_CAN_STATUS_FLAG_BALANCING        0x02u // balancing resistors active this cycle
#define CUSTOM_CAN_STATUS_FLAG_ESTOP            0x04u // local emergency stop input active
#define CUSTOM_CAN_STATUS_FLAG_OPERATING        0x08u // persisted "resume operating on boot"
#define CUSTOM_CAN_STATUS_FLAG_COMMAND_PENDING  0x10u // commanded state not yet reached
#define CUSTOM_CAN_STATUS_FLAG_SLOW_MODE        0x20u // cell data sampled infrequently
#define CUSTOM_CAN_STATUS_FLAG_PRECHARGE_CLOSED 0x40u // precharge/bypass aux contact

// Mirrors enum system_states in app/state_machines/system.h
typedef enum {
    CUSTOM_CAN_SYSTEM_UNINITIALIZED = 0,
    CUSTOM_CAN_SYSTEM_INITIALIZING = 1,
    CUSTOM_CAN_SYSTEM_CALIBRATING = 2,
    CUSTOM_CAN_SYSTEM_INACTIVE = 3,  // contactors open, ready to RUN
    CUSTOM_CAN_SYSTEM_OPERATING = 4, // contactors closing/closed
    CUSTOM_CAN_SYSTEM_FAULT = 5,     // latched; needs local intervention
} custom_can_system_state_t;

// Mirrors CONTACTORS_STATES in app/state_machines/contactors.h. Only the
// values a controller normally cares about are named here; treat anything
// else as "in transition".
typedef enum {
    CUSTOM_CAN_CONTACTORS_OPEN = 0,
    CUSTOM_CAN_CONTACTORS_PRECHARGING = 7,
    CUSTOM_CAN_CONTACTORS_CLOSED = 8,
    CUSTOM_CAN_CONTACTORS_AWAITING_OPEN = 9, // waiting for current to fall
    CUSTOM_CAN_CONTACTORS_PRECHARGE_FAILED = 16,
    CUSTOM_CAN_CONTACTORS_TESTING_FAILED = 17,
} custom_can_contactor_state_t;

typedef enum {
    CUSTOM_CAN_LEVEL_NONE = 0,
    CUSTOM_CAN_LEVEL_INFO = 1,
    CUSTOM_CAN_LEVEL_WARNING = 2,
    CUSTOM_CAN_LEVEL_CRITICAL = 3, // contactors will open soon unless it clears
    CUSTOM_CAN_LEVEL_FATAL = 4,    // contactors opening/open, latched
} custom_can_event_level_t;

// LIMITS (DLC 8):
//   0..1  max pack voltage limit, 0.1 V, int16
//   2..3  min pack voltage limit, 0.1 V, int16
//   4..5  charge current limit, 0.1 A, uint16 (0 = no charging)
//   6..7  discharge current limit, 0.1 A, uint16 (0 = no discharging)

// MEASUREMENTS (DLC 8):
//   0..1  pack (battery side) voltage, 0.01 V, uint16
//   2..5  current, mA, int32, positive = charging
//   6..7  state of charge as presented to the inverter, 0.01 %, uint16

// ENERGY (DLC 8):
//   0..1  state of health, 0.01 %, uint16
//   2..3  remaining capacity, 0.1 Ah, uint16
//   4..5  full capacity, 0.1 Ah, uint16
//   6..7  charge used since full, 0.1 Ah, int16

// CELL_STATS (DLC 8):
//   0..1  min cell voltage, mV, int16
//   2..3  max cell voltage, mV, int16
//   4..5  mean cell voltage, mV, int16
//   6     index of min cell
//   7     index of max cell

// TEMPERATURES (DLC 8):
//   0..1  min module temperature, 0.1 C, int16
//   2..3  max module temperature, 0.1 C, int16
//   4     index of coldest module
//   5     index of hottest module
//   6..7  reserved

// HV_VOLTAGES (DLC 8):
//   0..1  output (inverter side) voltage, 0.01 V, uint16
//   2..3  voltage across positive contactor, 0.1 V, int16
//   4..5  voltage across negative contactor, 0.1 V, int16
//   6..7  battery minus output voltage, 0.1 V, int16

// SUPPLY (DLC 8): 3V3, 5V, 12V, contactor supply; mV, uint16 each

// BALANCING (DLC 8):
//   0     balancing state (0 idle, 1 active)
//   1     number of cells being balanced this cycle
//   2..3  longest remaining balance time of any cell, in 1.28 s periods, int16
//   4..5  pause counter
//   6     flags: bit0 balancing active this BMB cycle, bit1 pause cycle,
//                bit2 auto-balancing enabled
//   7     reserved

// EVENT (DLC 8): one recorded event per frame, cycling through all recorded
// events; a frame with type 0xFF means "nothing recorded".
//   0     event type (index into the firmware's event table, see docs)
//   1     current level (custom_can_event_level_t)
//   2..3  occurrence count
//   4..5  seconds since last occurrence, saturating
//   6     number of event types defined by this firmware
//   7     number of events currently active (level > NONE)

// ESTIMATORS (DLC 8): voltage-based, basic-count, fancy-count and EKF SoC
// estimates, 0.01 %, uint16 each (unscaled, for diagnostics).

// CONFIG (DLC 8):
//   0     protocol version
//   1     chemistry (1 = LFP, 2 = NMC)
//   2     number of cells
//   3     number of module temperature sensors
//   4..5  nameplate capacity, 0.1 Ah, uint16
//   6..7  working (usable) capacity, 0.1 Ah, uint16

// SERIAL (DLC 8): the 64-bit board id, byte 0 first. serial32 is derived from
// it with custom_can_serial32().

// SETTINGS (DLC 8):
//   0..1  user charge current limit, 0.1 A, uint16 (0 = none)
//   2..3  user discharge current limit, 0.1 A, uint16 (0 = none)
//   4..5  working (0 %) cell voltage, mV
//   6..7  working (100 %) cell voltage, mV

// CELL_LIMITS (DLC 8): soft min, soft max, hard min, hard max cell voltage; mV

// MODULE_TEMPS (DLC 8, paged):
//   0     index of first sensor in this frame
//   1     total number of sensors
//   2..7  three temperatures, 0.1 C, int16 (CUSTOM_CAN_INT16_NA if missing)

// CELL_VOLTAGES (DLC 8, paged):
//   0     index of first cell in this frame
//   1     bits 2..0: cell i is being balanced; bit 7: voltages may be
//         disturbed by balancing this cycle
//   2..7  three cell voltages, mV, int16 (CUSTOM_CAN_INT16_NA if missing)
// Total cell count is in CONFIG.

#define CUSTOM_CAN_INT16_NA 0x7FFF

/* ------------------------------------------------------------------------ */
/* Timing                                                                    */
/* ------------------------------------------------------------------------ */

#define CUSTOM_CAN_ANNOUNCE_PERIOD_UNASSIGNED_MS 1000
#define CUSTOM_CAN_ANNOUNCE_PERIOD_ASSIGNED_MS   5000
#define CUSTOM_CAN_STATUS_PERIOD_MS              200
// The battery drops its assignment if it hears nothing from the controller
// for this long. Controllers should send COMMAND at least twice as often.
#define CUSTOM_CAN_CONTROLLER_TIMEOUT_MS         5000
#define CUSTOM_CAN_COMMAND_MAX_PERIOD_MS         1000
#define CUSTOM_CAN_COMMAND_RECOMMENDED_PERIOD_MS 500
// Controllers should consider a battery lost after this long without STATUS.
#define CUSTOM_CAN_BATTERY_TIMEOUT_MS            2000

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */

// 32-bit FNV-1a of the 64-bit board id. Used in ANNOUNCE/ASSIGN and as the low
// byte of the ANNOUNCE identifier.
static inline uint32_t custom_can_serial32(const uint8_t board_id[8]) {
    uint32_t h = 0x811C9DC5u;
    for (int i = 0; i < 8; i++) {
        h ^= board_id[i];
        h *= 0x01000193u;
    }
    return h;
}

static inline void custom_can_put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static inline void custom_can_put_i16(uint8_t *p, int16_t v) {
    custom_can_put_u16(p, (uint16_t)v);
}

static inline void custom_can_put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static inline void custom_can_put_i32(uint8_t *p, int32_t v) {
    custom_can_put_u32(p, (uint32_t)v);
}

static inline uint16_t custom_can_get_u16(const uint8_t *p) {
    return (uint16_t)((p[0] << 8) | p[1]);
}

static inline int16_t custom_can_get_i16(const uint8_t *p) {
    return (int16_t)custom_can_get_u16(p);
}

static inline uint32_t custom_can_get_u32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static inline int32_t custom_can_get_i32(const uint8_t *p) {
    return (int32_t)custom_can_get_u32(p);
}
