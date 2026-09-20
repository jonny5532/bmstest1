/*
 * Custom multi-battery CAN protocol - battery side
 * =================================================
 *
 * Several batteries share one DC bus and one CAN bus with an upstream
 * controller (a CellKeeper in master mode, see can_master.c, or e.g. Battery
 * Emulator) which talks to the actual inverter. This module implements the
 * battery end of the protocol defined in custom_can.h and documented in
 * docs/custom_can_protocol.md. It runs on the inter-BMS bus (CAN2).
 *
 * Lifecycle
 * ---------
 *
 *   UNASSIGNED ── ASSIGN (serial matches) ──▶ ASSIGNED
 *       ▲                                        │
 *       └── controller timeout / release / ──────┘
 *           address conflict
 *
 *  - UNASSIGNED: broadcast ANNOUNCE every second (staggered per battery).
 *    Nothing else is transmitted; local control (HMI/CLI/persisted state)
 *    remains fully functional.
 *  - ASSIGNED: transmit telemetry on prefix/node given by the controller,
 *    accept COMMAND/REQUEST frames for our node (or node 0xFF), keep
 *    announcing every 5 s so a restarted controller finds us quickly.
 *
 * Restarts: a rebooted battery simply starts announcing again. A rebooted
 * controller either re-assigns us immediately (we accept ASSIGN at any time)
 * or, if it takes longer than CUSTOM_CAN_CONTROLLER_TIMEOUT_MS, we fall back
 * to UNASSIGNED and are rediscovered from scratch.
 *
 * Control: the controller repeats its desired run state in every COMMAND.
 * We act on changes of that value, translating them into system requests.
 * All contactor sequencing (waiting for current to fall before opening, etc.)
 * stays with the system/contactor state machines - this module never touches
 * contactors directly.
 *
 * Bus load: one scheduler sends at most MAX_TX_PER_TICK frames per 20 ms tick,
 * highest-priority message first, so the transmit queue can never overflow
 * and heavy paged data (cell voltages) trickles out behind the important
 * frames. Steady state is roughly 40 frames/s per battery (~1 % of 500 kbps).
 *
 * Concurrency: the can2040 callback runs in interrupt context. It only filters
 * and copies frames into a small ring buffer; all protocol handling and every
 * access to the model happens in inverter_tick() on the main loop.
 */

#include "protocols/inverter/custom_can.h"
#include "protocols/inverter/inverter.h"
#include "protocols/inverter/can.h"

#include "app/model.h"
#include "config/limits.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "sys/time/time.h"

#include "can2040.h"
#include "pico/unique_id.h"

#include <math.h>
#include <string.h>

// The fleet protocol normally runs on the dedicated inter-BMS bus (CAN2,
// PIN_INTERCAN_*), leaving the inverter bus free.
#ifndef CUSTOM_CAN_BUS
#define CUSTOM_CAN_BUS CAN_BUS_INTER
#endif

#define MAX_TX_PER_TICK 3
#define RX_RING_SIZE 8
// Minimum interval between repeated system requests for a pending command
#define REQUEST_RETRY_MS 1000
// Spread announcements of different batteries over this many ticks
#define ANNOUNCE_JITTER_TICKS 25

#define MS_TO_TICKS(ms) ((ms) / TIMESTEP_PERIOD_MS)

/* ------------------------------------------------------------------------ */
/* State                                                                     */
/* ------------------------------------------------------------------------ */

typedef enum {
    UNASSIGN_RELEASED,
    UNASSIGN_TIMEOUT,
    UNASSIGN_CONFLICT,
} unassign_reason_t;

static struct {
    pico_unique_board_id_t board_id;
    uint32_t serial32;

    custom_can_link_t state;
    uint16_t prefix;
    uint8_t node;
    uint8_t assign_flags;
    uint8_t rate_shift;

    millis_t last_controller_millis;
    uint16_t last_cmd_seq;
    // Last non-NONE desired state received from the controller
    uint8_t commanded_state;
    // Desired state we have not yet achieved (NONE if satisfied)
    uint8_t pending_state;
    uint32_t next_request_timestep;

    uint32_t next_announce_timestep;

    // For expediting STATUS on state changes
    uint16_t last_status_system_state;
    uint16_t last_status_contactor_state;

    // Diagnostics
    uint32_t tx_failed;
    uint32_t conflicts;
} link;

/* ------------------------------------------------------------------------ */
/* Receive ring (interrupt -> main loop)                                     */
/* ------------------------------------------------------------------------ */

static struct can2040_msg rx_ring[RX_RING_SIZE];
static volatile uint8_t rx_head; // written by the interrupt
static volatile uint8_t rx_tail; // written by the main loop
static volatile uint32_t rx_overflow;

static bool frame_is_interesting(uint32_t id) {
    if(id == CUSTOM_CAN_ID_ASSIGN) return true;
    if(link.state != CUSTOM_CAN_LINK_ASSIGNED) return false;
    if(custom_can_id_prefix(id) != link.prefix) return false;

    uint8_t node = custom_can_id_node(id);
    if(custom_can_type_is_from_controller(custom_can_id_type(id))) {
        return node == link.node || node == CUSTOM_CAN_NODE_ALL;
    }
    // A battery-type frame carrying our node id: somebody else is using our
    // address. Let the main loop deal with it.
    return node == link.node;
}

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    (void)cd;
    if(notify != CAN2040_NOTIFY_RX) return;
    if(!(msg->id & CAN2040_ID_EFF) || (msg->id & CAN2040_ID_RTR)) return;

    uint32_t id = msg->id & CUSTOM_CAN_ID_MASK;
    if(!frame_is_interesting(id)) return;

    uint8_t next = (uint8_t)((rx_head + 1) % RX_RING_SIZE);
    if(next == rx_tail) {
        rx_overflow++;
        return;
    }
    rx_ring[rx_head] = *msg;
    rx_ring[rx_head].id = id;
    rx_head = next;
}

/* ------------------------------------------------------------------------ */
/* Encoding helpers                                                          */
/* ------------------------------------------------------------------------ */

static int16_t float_to_i16(float value, float scale) {
    if(isnan(value)) return CUSTOM_CAN_INT16_NA;
    float scaled = value * scale;
    if(scaled >= 32766.0f) return 32766;
    if(scaled <= -32767.0f) return -32767;
    return (int16_t)lroundf(scaled);
}

static uint16_t float_to_u16(float value, float scale) {
    if(isnan(value) || value <= 0.0f) return 0;
    float scaled = value * scale;
    if(scaled >= 65535.0f) return 65535;
    return (uint16_t)lroundf(scaled);
}

static uint16_t clamp_u16(int32_t value) {
    if(value < 0) return 0;
    if(value > 65535) return 65535;
    return (uint16_t)value;
}

static int16_t clamp_i16(int32_t value) {
    if(value > 32766) return 32766;
    if(value < -32767) return -32767;
    return (int16_t)value;
}

static int16_t cell_mV_or_na(int16_t voltage_mV) {
    return voltage_mV > 0 ? voltage_mV : CUSTOM_CAN_INT16_NA;
}

static int send_frame(uint32_t id, const uint8_t data[8], uint8_t dlc) {
    struct can2040_msg msg = {0};
    msg.id = CAN2040_ID_EFF | id;
    msg.dlc = dlc;
    memcpy(msg.data, data, dlc);
    int ret = can_bus_transmit(CUSTOM_CAN_BUS, &msg);
    if(ret < 0) link.tx_failed++;
    return ret;
}

/* ------------------------------------------------------------------------ */
/* Frame builders (battery -> controller)                                    */
/* ------------------------------------------------------------------------ */

// Each builder fills 8 bytes and returns false if the underlying data is not
// valid yet, in which case the frame is withheld until its next period.

static uint8_t highest_event_type(uint16_t level) {
    if(level == LEVEL_NONE) return 0xFF;
    for(int i = 0; i < ERR_HIGHEST; i++) {
        if(get_event_level((bms_event_type_t)i) == level) return (uint8_t)i;
    }
    return 0xFF;
}

static bool build_status(uint8_t d[8], uint8_t page) {
    (void)page;
    const bms_model_t *m = &model;

    link.last_status_system_state = m->system_sm.state;
    link.last_status_contactor_state = m->contactor_sm.state;

    d[0] = (uint8_t)m->system_sm.state;
    d[1] = (uint8_t)m->contactor_sm.state;

    uint8_t flags = 0;
    if(m->contactor_sm.enable_current) flags |= CUSTOM_CAN_STATUS_FLAG_CURRENT_ENABLED;
    if(m->balancing_active) flags |= CUSTOM_CAN_STATUS_FLAG_BALANCING;
    if(m->estop_pressed) flags |= CUSTOM_CAN_STATUS_FLAG_ESTOP;
    if(m->operating) flags |= CUSTOM_CAN_STATUS_FLAG_OPERATING;
    if(link.pending_state != CUSTOM_CAN_DESIRED_NONE) flags |= CUSTOM_CAN_STATUS_FLAG_COMMAND_PENDING;
    if(m->cell_voltage_slow_mode) flags |= CUSTOM_CAN_STATUS_FLAG_SLOW_MODE;
    if(m->precharge_closed) flags |= CUSTOM_CAN_STATUS_FLAG_PRECHARGE_CLOSED;
    d[2] = flags;

    uint16_t level = get_highest_event_level();
    d[3] = (uint8_t)((level & 0x07) | ((link.commanded_state & 0x03) << 4));
    custom_can_put_u16(&d[4], link.last_cmd_seq);
    d[6] = highest_event_type(level);
    d[7] = 0;
    return true;
}

static bool build_limits(uint8_t d[8], uint8_t page) {
    (void)page;
    const inverter_outputs_t *out = &model.inverter_outputs;
    custom_can_put_i16(&d[0], out->max_voltage_limit_dV);
    custom_can_put_i16(&d[2], out->min_voltage_limit_dV);
    custom_can_put_u16(&d[4], out->charge_current_limit_dA);
    custom_can_put_u16(&d[6], out->discharge_current_limit_dA);
    return true;
}

static bool build_measurements(uint8_t d[8], uint8_t page) {
    (void)page;
    const inverter_outputs_t *out = &model.inverter_outputs;
    if(out->battery_voltage_millis == 0 || out->current_millis == 0) return false;
    custom_can_put_u16(&d[0], float_to_u16(out->battery_voltage, 100.0f));
    custom_can_put_i32(&d[2], out->current_mA);
    custom_can_put_u16(&d[6], clamp_u16(out->soc));
    return true;
}

static bool build_energy(uint8_t d[8], uint8_t page) {
    (void)page;
    const bms_model_t *m = &model;
    custom_can_put_u16(&d[0], m->soh > 0 ? m->soh : 10000);
    custom_can_put_u16(&d[2], clamp_u16(m->inverter_outputs.remaining_capacity_dAh));
    custom_can_put_u16(&d[4], clamp_u16(m->inverter_outputs.full_capacity_dAh));
    custom_can_put_i16(&d[6], float_to_i16(m->charge_used_Ah, 10.0f));
    return true;
}

static bool build_cell_stats(uint8_t d[8], uint8_t page) {
    (void)page;
    const bms_model_t *m = &model;
    if(m->cell_voltage_millis == 0) return false;

    uint8_t idx_min = 0, idx_max = 0;
    for(int i = 1; i < NUM_CELLS; i++) {
        if(m->cell_voltages_mV[i] <= 0) continue;
        if(m->cell_voltages_mV[idx_min] <= 0 || m->cell_voltages_mV[i] < m->cell_voltages_mV[idx_min]) idx_min = (uint8_t)i;
        if(m->cell_voltages_mV[i] > m->cell_voltages_mV[idx_max]) idx_max = (uint8_t)i;
    }

    custom_can_put_i16(&d[0], m->cell_voltage_min_mV);
    custom_can_put_i16(&d[2], m->cell_voltage_max_mV);
    custom_can_put_i16(&d[4], clamp_i16(m->cell_voltage_total_mV / NUM_CELLS));
    d[6] = idx_min;
    d[7] = idx_max;
    return true;
}

static bool build_temperatures(uint8_t d[8], uint8_t page) {
    (void)page;
    const bms_model_t *m = &model;
    if(m->temperature_millis == 0) return false;

    uint8_t idx_min = 0, idx_max = 0;
    bool have_ref = false;
    for(int i = 0; i < NUM_MODULE_TEMPS; i++) {
        float t = m->module_temperatures[i];
        if(isnan(t)) continue;
        if(!have_ref) {
            idx_min = idx_max = (uint8_t)i;
            have_ref = true;
            continue;
        }
        if(t < m->module_temperatures[idx_min]) idx_min = (uint8_t)i;
        if(t > m->module_temperatures[idx_max]) idx_max = (uint8_t)i;
    }

    custom_can_put_i16(&d[0], float_to_i16(m->temperature_min, 10.0f));
    custom_can_put_i16(&d[2], float_to_i16(m->temperature_max, 10.0f));
    d[4] = idx_min;
    d[5] = idx_max;
    d[6] = 0;
    d[7] = 0;
    return true;
}

static bool build_hv_voltages(uint8_t d[8], uint8_t page) {
    (void)page;
    const high_voltages_t *hv = &model.high_voltages;
    if(hv->battery_millis == 0 || hv->output_millis == 0) return false;
    custom_can_put_u16(&d[0], float_to_u16(hv->output, 100.0f));
    custom_can_put_i16(&d[2], float_to_i16(hv->pos_contactor, 10.0f));
    custom_can_put_i16(&d[4], float_to_i16(hv->neg_contactor, 10.0f));
    custom_can_put_i16(&d[6], float_to_i16(hv->battery - hv->output, 10.0f));
    return true;
}

static bool build_supply(uint8_t d[8], uint8_t page) {
    (void)page;
    const supply_voltages_t *sv = &model.supply_voltages;
    custom_can_put_u16(&d[0], clamp_u16(sv->voltage_3V3_mV));
    custom_can_put_u16(&d[2], clamp_u16(sv->voltage_5V_mV));
    custom_can_put_u16(&d[4], clamp_u16(sv->voltage_12V_mV));
    custom_can_put_u16(&d[6], clamp_u16(sv->voltage_contactor_mV));
    return true;
}

static bool build_balancing(uint8_t d[8], uint8_t page) {
    (void)page;
    const bms_model_t *m = &model;
    const balancing_sm_t *b = &m->balancing_sm;

    unsigned cells = 0;
    for(int i = 0; i < 4; i++) {
        cells += (unsigned)__builtin_popcount(b->balance_request_mask[i]);
    }
    int16_t max_remaining = 0;
    for(int i = 0; i < NUM_CELLS; i++) {
        if(b->balance_time_remaining[i] > max_remaining) max_remaining = b->balance_time_remaining[i];
    }

    d[0] = (uint8_t)b->state;
    d[1] = (uint8_t)(cells > 255 ? 255 : cells);
    custom_can_put_i16(&d[2], max_remaining);
    custom_can_put_u16(&d[4], b->pause_counter);
    uint8_t flags = 0;
    if(m->balancing_active) flags |= 0x01;
    if(b->is_pause_cycle) flags |= 0x02;
    if(m->auto_balancing_period_ms > 0) flags |= 0x04;
    d[6] = flags;
    d[7] = 0;
    return true;
}

static bool build_event(uint8_t d[8], uint8_t page) {
    (void)page;
    static uint8_t cursor = 0;

    uint8_t n_active = 0;
    bool any_recorded = false;
    for(int i = 0; i < ERR_HIGHEST; i++) {
        if(bms_event_slots[i].count > 0) any_recorded = true;
        if(bms_event_slots[i].level > LEVEL_NONE) n_active++;
    }

    memset(d, 0, 8);
    d[6] = (uint8_t)ERR_HIGHEST;
    d[7] = n_active;

    if(!any_recorded) {
        d[0] = 0xFF;
        return true;
    }

    for(int k = 0; k < ERR_HIGHEST; k++) {
        uint8_t i = (uint8_t)((cursor + k) % ERR_HIGHEST);
        const bms_event_slot_t *slot = &bms_event_slots[i];
        if(slot->count == 0) continue;

        uint32_t age_s = (millis() - slot->timestamp) / 1000;
        d[0] = i;
        d[1] = (uint8_t)slot->level;
        custom_can_put_u16(&d[2], slot->count);
        custom_can_put_u16(&d[4], age_s > 65535 ? 65535 : (uint16_t)age_s);
        cursor = (uint8_t)((i + 1) % ERR_HIGHEST);
        return true;
    }
    return true;
}

static bool build_estimators(uint8_t d[8], uint8_t page) {
    (void)page;
    const bms_model_t *m = &model;
    custom_can_put_u16(&d[0], m->soc_voltage_based);
    custom_can_put_u16(&d[2], m->soc_basic_count);
    custom_can_put_u16(&d[4], m->soc_fancy_count);
    custom_can_put_u16(&d[6], m->soc);
    return true;
}

static bool build_config(uint8_t d[8], uint8_t page) {
    (void)page;
    const bms_model_t *m = &model;
    d[0] = CUSTOM_CAN_PROTOCOL_VERSION;
    d[1] = CHEMISTRY;
    d[2] = NUM_CELLS;
    d[3] = NUM_MODULE_TEMPS;
    // mC -> 0.1 Ah: 1 dAh = 0.1 * 3600 * 1000 mC
    custom_can_put_u16(&d[4], clamp_u16((int32_t)(m->nameplate_capacity_mC / 360000u)));
    custom_can_put_u16(&d[6], clamp_u16((int32_t)(m->working_capacity_mC / 360000u)));
    return true;
}

static bool build_serial(uint8_t d[8], uint8_t page) {
    (void)page;
    memcpy(d, link.board_id.id, 8);
    return true;
}

static bool build_settings(uint8_t d[8], uint8_t page) {
    (void)page;
    const bms_model_t *m = &model;
    custom_can_put_u16(&d[0], m->user_charge_current_limit_dA > 0 ? m->user_charge_current_limit_dA - 1 : 0);
    custom_can_put_u16(&d[2], m->user_discharge_current_limit_dA > 0 ? m->user_discharge_current_limit_dA - 1 : 0);
    custom_can_put_u16(&d[4], get_cell_voltage_working_min_mV(m));
    custom_can_put_u16(&d[6], get_cell_voltage_working_max_mV(m));
    return true;
}

static bool build_cell_limits(uint8_t d[8], uint8_t page) {
    (void)page;
    const bms_model_t *m = &model;
    custom_can_put_u16(&d[0], get_cell_voltage_soft_min_mV(m));
    custom_can_put_u16(&d[2], get_cell_voltage_soft_max_mV(m));
    custom_can_put_u16(&d[4], CELL_VOLTAGE_HARD_MIN_mV);
    custom_can_put_u16(&d[6], CELL_VOLTAGE_HARD_MAX_mV);
    return true;
}

static bool build_module_temps(uint8_t d[8], uint8_t page) {
    const bms_model_t *m = &model;
    int first = page * 3;
    d[0] = (uint8_t)first;
    d[1] = NUM_MODULE_TEMPS;
    for(int j = 0; j < 3; j++) {
        int idx = first + j;
        int16_t value = CUSTOM_CAN_INT16_NA;
        if(idx < NUM_MODULE_TEMPS && m->module_temperatures_millis != 0) {
            value = float_to_i16(m->module_temperatures[idx], 10.0f);
        }
        custom_can_put_i16(&d[2 + 2 * j], value);
    }
    return true;
}

static bool build_cell_voltages(uint8_t d[8], uint8_t page) {
    const bms_model_t *m = &model;
    int first = page * 3;
    d[0] = (uint8_t)first;
    uint8_t flags = m->balancing_active ? 0x80 : 0x00;
    for(int j = 0; j < 3; j++) {
        int idx = first + j;
        int16_t value = CUSTOM_CAN_INT16_NA;
        if(idx < NUM_CELLS) {
            value = cell_mV_or_na(m->cell_voltages_mV[idx]);
            if((m->balancing_sm.balance_request_mask[idx / 32] >> (idx % 32)) & 1u) {
                flags |= (uint8_t)(1u << j);
            }
        }
        custom_can_put_i16(&d[2 + 2 * j], value);
    }
    d[1] = flags;
    return true;
}

/* ------------------------------------------------------------------------ */
/* Transmit schedule                                                         */
/* ------------------------------------------------------------------------ */

typedef bool (*build_fn_t)(uint8_t d[8], uint8_t page);

typedef struct {
    uint8_t type;
    uint16_t period_ms;
    // Essential frames are not slowed down by the controller's rate shift
    bool essential;
    build_fn_t build;
    uint8_t pages;

    // Runtime
    uint8_t page;
    uint32_t next_timestep;
    bool due_now;
} sched_entry_t;

#define PAGES_FOR(n) (((n) + 2) / 3)

// Ordered by priority: when several frames are due in the same tick, earlier
// entries go first.
static sched_entry_t schedule[] = {
    { .type = CUSTOM_CAN_TYPE_STATUS,        .period_ms = CUSTOM_CAN_STATUS_PERIOD_MS, .essential = true, .build = build_status, .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_LIMITS,        .period_ms = 200,  .essential = true,  .build = build_limits,        .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_MEASUREMENTS,  .period_ms = 200,  .essential = true,  .build = build_measurements,  .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_ENERGY,        .period_ms = 1000, .essential = false, .build = build_energy,        .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_CELL_STATS,    .period_ms = 1000, .essential = false, .build = build_cell_stats,    .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_TEMPERATURES,  .period_ms = 1000, .essential = false, .build = build_temperatures,  .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_HV_VOLTAGES,   .period_ms = 1000, .essential = false, .build = build_hv_voltages,   .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_SUPPLY,        .period_ms = 2000, .essential = false, .build = build_supply,        .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_BALANCING,     .period_ms = 1000, .essential = false, .build = build_balancing,     .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_EVENT,         .period_ms = 250,  .essential = false, .build = build_event,         .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_ESTIMATORS,    .period_ms = 2000, .essential = false, .build = build_estimators,    .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_CONFIG,        .period_ms = 5000, .essential = false, .build = build_config,        .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_SERIAL,        .period_ms = 5000, .essential = false, .build = build_serial,        .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_SETTINGS,      .period_ms = 5000, .essential = false, .build = build_settings,      .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_CELL_LIMITS,   .period_ms = 5000, .essential = false, .build = build_cell_limits,   .pages = 1 },
    { .type = CUSTOM_CAN_TYPE_MODULE_TEMPS,  .period_ms = 250,  .essential = false, .build = build_module_temps,  .pages = PAGES_FOR(NUM_MODULE_TEMPS) },
    { .type = CUSTOM_CAN_TYPE_CELL_VOLTAGES, .period_ms = 100,  .essential = false, .build = build_cell_voltages, .pages = PAGES_FOR(NUM_CELLS) },
};
#define SCHEDULE_LEN (sizeof(schedule) / sizeof(schedule[0]))

static sched_entry_t *schedule_find(uint8_t type) {
    for(unsigned i = 0; i < SCHEDULE_LEN; i++) {
        if(schedule[i].type == type) return &schedule[i];
    }
    return NULL;
}

static void schedule_expedite(uint8_t type) {
    sched_entry_t *e = schedule_find(type);
    if(e) e->due_now = true;
}

// Make everything due immediately (in priority order) and restart paging.
static void schedule_reset(void) {
    for(unsigned i = 0; i < SCHEDULE_LEN; i++) {
        schedule[i].page = 0;
        schedule[i].next_timestep = timestep();
        schedule[i].due_now = false;
    }
}

static uint32_t effective_period_ticks(const sched_entry_t *e) {
    uint32_t ticks = MS_TO_TICKS(e->period_ms);
    if(!e->essential) ticks <<= link.rate_shift;
    return ticks > 0 ? ticks : 1;
}

static void schedule_tick(void) {
    uint32_t now = timestep();
    int sent = 0;

    for(unsigned i = 0; i < SCHEDULE_LEN && sent < MAX_TX_PER_TICK; i++) {
        sched_entry_t *e = &schedule[i];
        if(!e->due_now && (int32_t)(now - e->next_timestep) < 0) continue;

        uint8_t data[8] = {0};
        uint32_t period_ticks = effective_period_ticks(e);

        if(!e->build(data, e->page)) {
            // Data not valid yet, try again next period
            e->due_now = false;
            e->next_timestep = now + period_ticks;
            continue;
        }

        if(send_frame(custom_can_make_id(link.prefix, e->type, link.node), data, 8) < 0) {
            // Transmit queue full, retry next tick (entry stays due)
            break;
        }

        sent++;
        e->due_now = false;
        e->next_timestep = now + period_ticks;
        if(e->pages > 1) e->page = (uint8_t)((e->page + 1) % e->pages);
    }
}

/* ------------------------------------------------------------------------ */
/* Discovery                                                                 */
/* ------------------------------------------------------------------------ */

static int send_announce(void) {
    bool assigned = link.state == CUSTOM_CAN_LINK_ASSIGNED;
    uint8_t d[8];
    d[0] = (uint8_t)((CUSTOM_CAN_PROTOCOL_VERSION << 4) | (link.state & 0x0F));
    d[1] = assigned ? link.node : CUSTOM_CAN_NODE_NONE;
    custom_can_put_u32(&d[2], link.serial32);
    custom_can_put_u16(&d[6], assigned ? link.prefix : 0);
    return send_frame(CUSTOM_CAN_ID_ANNOUNCE_BASE | (link.serial32 & 0xFF), d, 8);
}

static void announce_tick(void) {
    if((int32_t)(timestep() - link.next_announce_timestep) < 0) return;
    if(send_announce() < 0) return; // retry next tick

    uint32_t period_ms = link.state == CUSTOM_CAN_LINK_ASSIGNED
        ? CUSTOM_CAN_ANNOUNCE_PERIOD_ASSIGNED_MS
        : CUSTOM_CAN_ANNOUNCE_PERIOD_UNASSIGNED_MS;
    link.next_announce_timestep = timestep() + MS_TO_TICKS(period_ms);
}

static void become_assigned(uint16_t prefix, uint8_t node, uint8_t flags) {
    link.state = CUSTOM_CAN_LINK_ASSIGNED;
    link.prefix = prefix;
    link.node = node;
    link.assign_flags = flags;
    link.rate_shift = (uint8_t)((flags & CUSTOM_CAN_ASSIGN_RATE_SHIFT_MASK) >> CUSTOM_CAN_ASSIGN_RATE_SHIFT_POS);
    link.last_controller_millis = millis();
    link.last_cmd_seq = 0;
    link.commanded_state = CUSTOM_CAN_DESIRED_NONE;
    link.pending_state = CUSTOM_CAN_DESIRED_NONE;

    schedule_reset();
    // Announce again soon so monitors see the new assignment
    link.next_announce_timestep = timestep() + MS_TO_TICKS(500);

    info_printf("Custom CAN: assigned prefix 0x%04X node %u (flags 0x%02X), base id 0x%08lX\n",
        prefix, node, flags, (unsigned long)custom_can_make_id(prefix, 0, 0));
    raise_bms_event(ERR_INVERTER_DETECTED, ((uint64_t)prefix << 8) | node);
}

static void unassign(unassign_reason_t reason) {
    const char *why = reason == UNASSIGN_TIMEOUT ? "controller timeout"
                    : reason == UNASSIGN_CONFLICT ? "address conflict"
                    : "released by controller";
    info_printf("Custom CAN: unassigned (%s), was prefix 0x%04X node %u\n", why, link.prefix, link.node);

    if(reason == UNASSIGN_TIMEOUT && (link.assign_flags & CUSTOM_CAN_ASSIGN_FLAG_STOP_ON_TIMEOUT)) {
        warning_printf("Custom CAN: controller lost, requesting STOP as configured\n");
        link.pending_state = CUSTOM_CAN_DESIRED_STOP;
        link.next_request_timestep = timestep();
    } else {
        link.pending_state = CUSTOM_CAN_DESIRED_NONE;
    }

    link.state = CUSTOM_CAN_LINK_UNASSIGNED;
    link.prefix = 0;
    link.node = CUSTOM_CAN_NODE_NONE;
    link.assign_flags = 0;
    link.rate_shift = 0;
    link.commanded_state = CUSTOM_CAN_DESIRED_NONE;
    link.last_cmd_seq = 0;
    link.next_announce_timestep = timestep();
}

static void handle_assign(const struct can2040_msg *msg) {
    if(msg->dlc != 8) return;
    if(custom_can_get_u32(&msg->data[0]) != link.serial32) return; // for another battery

    uint16_t prefix = custom_can_get_u16(&msg->data[4]);
    uint8_t node = msg->data[6];
    uint8_t flags = msg->data[7];

    if(node == CUSTOM_CAN_NODE_NONE) {
        if(link.state == CUSTOM_CAN_LINK_ASSIGNED) unassign(UNASSIGN_RELEASED);
        return;
    }

    if(prefix > CUSTOM_CAN_PREFIX_MAX || prefix == CUSTOM_CAN_DISCOVERY_PREFIX || node == CUSTOM_CAN_NODE_ALL) {
        warning_printf("Custom CAN: rejected invalid assignment prefix 0x%04X node %u\n", prefix, node);
        return;
    }

    if(link.state == CUSTOM_CAN_LINK_ASSIGNED && link.prefix == prefix && link.node == node) {
        // Idempotent re-assignment (e.g. controller restarted quickly). Keep
        // command tracking so a repeated desired state is not re-applied.
        link.assign_flags = flags;
        link.rate_shift = (uint8_t)((flags & CUSTOM_CAN_ASSIGN_RATE_SHIFT_MASK) >> CUSTOM_CAN_ASSIGN_RATE_SHIFT_POS);
        link.last_controller_millis = millis();
        schedule_expedite(CUSTOM_CAN_TYPE_STATUS);
        schedule_expedite(CUSTOM_CAN_TYPE_CONFIG);
        debug_printf("Custom CAN: re-assignment confirmed\n");
        return;
    }

    become_assigned(prefix, node, flags);
}

/* ------------------------------------------------------------------------ */
/* Control                                                                   */
/* ------------------------------------------------------------------------ */

static void handle_command(const struct can2040_msg *msg) {
    if(msg->dlc < 4) return;

    link.last_controller_millis = millis();
    link.last_cmd_seq = custom_can_get_u16(&msg->data[2]);

    uint8_t desired = msg->data[0];
    if(desired != CUSTOM_CAN_DESIRED_STOP && desired != CUSTOM_CAN_DESIRED_RUN) {
        return; // heartbeat only (or unknown value, which we treat the same)
    }

    if(desired != link.commanded_state) {
        info_printf("Custom CAN: controller requests %s (seq %u)\n",
            desired == CUSTOM_CAN_DESIRED_RUN ? "RUN" : "STOP", link.last_cmd_seq);
        link.commanded_state = desired;
        link.pending_state = desired;
        link.next_request_timestep = timestep();
        schedule_expedite(CUSTOM_CAN_TYPE_STATUS);
    }
}

static void handle_request(const struct can2040_msg *msg) {
    if(msg->dlc < 1) return;
    link.last_controller_millis = millis();

    sched_entry_t *e = schedule_find(msg->data[0]);
    if(!e) return;
    if(msg->dlc >= 2 && e->pages > 1 && msg->data[1] < e->pages) {
        e->page = msg->data[1];
    }
    e->due_now = true;
}

static void handle_conflict(const struct can2040_msg *msg) {
    link.conflicts++;
    warning_printf("Custom CAN: another node is transmitting on our address (id 0x%08lX), dropping assignment\n",
        (unsigned long)msg->id);
    unassign(UNASSIGN_CONFLICT);
}

static void dispatch(const struct can2040_msg *msg) {
    if(msg->id == CUSTOM_CAN_ID_ASSIGN) {
        handle_assign(msg);
        return;
    }

    // Frames may have been queued just before an unassignment
    if(link.state != CUSTOM_CAN_LINK_ASSIGNED || custom_can_id_prefix(msg->id) != link.prefix) return;

    uint8_t type = custom_can_id_type(msg->id);
    uint8_t node = custom_can_id_node(msg->id);

    if(custom_can_type_is_from_controller(type)) {
        if(node != link.node && node != CUSTOM_CAN_NODE_ALL) return;
        switch(type) {
            case CUSTOM_CAN_TYPE_COMMAND: handle_command(msg); break;
            case CUSTOM_CAN_TYPE_REQUEST: handle_request(msg); break;
            default: break; // unknown controller frame: ignore (forward compatibility)
        }
    } else if(node == link.node) {
        handle_conflict(msg);
    }
}

static void process_rx(void) {
    while(rx_tail != rx_head) {
        struct can2040_msg msg = rx_ring[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1) % RX_RING_SIZE);
        dispatch(&msg);
    }
}

static void check_controller_timeout(void) {
    if(link.state != CUSTOM_CAN_LINK_ASSIGNED) return;
    if(millis() - link.last_controller_millis <= CUSTOM_CAN_CONTROLLER_TIMEOUT_MS) return;
    warning_printf("Custom CAN: no controller frames for %lu ms\n",
        (unsigned long)(millis() - link.last_controller_millis));
    unassign(UNASSIGN_TIMEOUT);
}

// Translate the pending desired state into system requests. The system state
// machine decides what is actually possible (e.g. nothing leaves FAULT), and
// the contactor state machine handles the safe opening sequence.
static void apply_pending(void) {
    if(link.pending_state == CUSTOM_CAN_DESIRED_NONE) return;

    uint16_t sys = model.system_sm.state;
    bool satisfied = false;
    system_requests_t request = SYSTEM_REQUEST_NULL;

    if(link.pending_state == CUSTOM_CAN_DESIRED_RUN) {
        if(sys == SYSTEM_STATE_OPERATING) satisfied = true;
        else if(sys == SYSTEM_STATE_INACTIVE) request = SYSTEM_REQUEST_RUN;
    } else {
        if(sys == SYSTEM_STATE_INACTIVE || sys == SYSTEM_STATE_FAULT) satisfied = true;
        else if(sys == SYSTEM_STATE_OPERATING) request = SYSTEM_REQUEST_STOP;
    }

    if(satisfied) {
        link.pending_state = CUSTOM_CAN_DESIRED_NONE;
        schedule_expedite(CUSTOM_CAN_TYPE_STATUS);
        return;
    }

    if(request != SYSTEM_REQUEST_NULL
       && model.system_req == SYSTEM_REQUEST_NULL
       && (int32_t)(timestep() - link.next_request_timestep) >= 0) {
        model.system_req = request;
        link.next_request_timestep = timestep() + MS_TO_TICKS(REQUEST_RETRY_MS);
        info_printf("Custom CAN: issuing system request %s\n", request == SYSTEM_REQUEST_RUN ? "RUN" : "STOP");
    }
}

/* ------------------------------------------------------------------------ */
/* Public API (inverter.h)                                                   */
/* ------------------------------------------------------------------------ */

void init_inverter() {
    memset(&link, 0, sizeof(link));
    rx_head = rx_tail = 0;
    rx_overflow = 0;

    pico_get_unique_board_id(&link.board_id);
    link.serial32 = custom_can_serial32(link.board_id.id);
    link.state = CUSTOM_CAN_LINK_UNASSIGNED;
    link.node = CUSTOM_CAN_NODE_NONE;
    // Stagger announcements between batteries so identical-serial-low-byte
    // collisions are unlikely to be simultaneous
    link.next_announce_timestep = timestep() + (link.serial32 % ANNOUNCE_JITTER_TICKS);

    can_bus_init(CUSTOM_CAN_BUS, can2040_cb);
    info_printf("Custom CAN: initialised, serial 0x%08lX, announcing on 0x%08lX\n",
        (unsigned long)link.serial32,
        (unsigned long)(CUSTOM_CAN_ID_ANNOUNCE_BASE | (link.serial32 & 0xFF)));
}

void inverter_tick(inverter_outputs_t *outputs) {
    // Everything we report comes from the global model (which contains
    // outputs); the parameter is kept for interface compatibility.
    (void)outputs;

    process_rx();
    check_controller_timeout();
    apply_pending();

    if(link.state == CUSTOM_CAN_LINK_ASSIGNED
       && (model.system_sm.state != link.last_status_system_state
           || model.contactor_sm.state != link.last_status_contactor_state)) {
        schedule_expedite(CUSTOM_CAN_TYPE_STATUS);
    }

    announce_tick();

    if(link.state == CUSTOM_CAN_LINK_ASSIGNED) {
        schedule_tick();
    }
}
