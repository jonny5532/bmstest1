/*
 * Master mode - fleet controller
 * ==============================
 *
 * Plays the controller role of the custom multi-battery protocol
 * (custom_can.h, docs/custom_can_protocol.md) on the inter-BMS bus, while the
 * inverter protocol carries on unchanged on the inverter bus:
 *
 *  - discovers slave BMSs from their ANNOUNCE frames and assigns them a node
 *    on CAN_MASTER_PREFIX (adopting existing assignments after a master
 *    reboot so the slaves never notice),
 *  - sends the COMMAND heartbeat carrying this battery's run intent
 *    (model->operating), so the operator's RUN/STOP on the master applies to
 *    the whole fleet, and a FATAL fault on the master stops it,
 *  - tracks slave liveness, state and the few telemetry frames needed to
 *    aggregate the fleet: STATUS, LIMITS, MEASUREMENTS, ENERGY, TEMPERATURES,
 *  - fills model->fleet_outputs, which the inverter protocol sends instead of
 *    the local inverter_outputs.
 *
 * Aggregation rules for a parallel HV bus:
 *  - a member "contributes" when it is online, has current enabled and has no
 *    CRITICAL/FATAL event; only contributing members count towards current
 *    limits and capacity (a lost slave is simply subtracted),
 *  - current limits and capacities are sums, voltage limits are the tightest
 *    (min of maxima, max of minima), SoC is capacity weighted, temperatures
 *    are the extremes, current is the sum, voltage is the local measurement,
 *  - like model.c, SoC is forced to 0 %/100 % when the fleet cannot
 *    discharge/charge, so inverters that only look at SoC also stop.
 *
 * Transmit budget: at most 2 frames per tick, so the 6-deep can2040 queue can
 * never overflow even with a burst of announcements. Anything that does not
 * fit is retried next tick; slaves re-announce until assigned.
 */

#include "protocols/inverter/can_master.h"
#include "protocols/inverter/can.h"

#include "app/model.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "sys/time/time.h"

#include "can2040.h"

#include <string.h>

#define RX_RING_SIZE 32
#define MAX_TX_PER_TICK 2
#define MS_TO_TICKS(ms) ((ms) / TIMESTEP_PERIOD_MS)

typedef struct {
    uint32_t serial;      // 0 = empty slot
    uint8_t node;         // 0 = no node assigned
    millis_t first_seen_millis;
    millis_t last_announce_millis;
    millis_t last_status_millis; // 0 = never heard
    millis_t offline_since_millis;
    bool online;
    uint16_t assign_count;

    // STATUS
    uint8_t system_state, contactor_state, status_flags, level, commanded, top_event;
    uint16_t last_seq;
    // LIMITS
    bool limits_valid;
    int16_t max_voltage_dV, min_voltage_dV;
    uint16_t charge_dA, discharge_dA;
    // MEASUREMENTS
    bool meas_valid;
    uint16_t voltage_cV;
    int32_t current_mA;
    uint16_t soc;
    // ENERGY
    bool energy_valid;
    uint16_t soh, remaining_dAh, full_dAh;
    // TEMPERATURES
    bool temps_valid;
    int16_t temp_min_dC, temp_max_dC;

    // Command tracking
    uint16_t last_seq_sent;
    millis_t seq_sent_millis;
    uint16_t rtt_ms;
    uint32_t next_heartbeat_timestep;
} slave_t;

static slave_t slaves[CAN_MASTER_MAX_SLAVES];
static uint16_t cmd_seq;
static int tx_this_tick;
static uint32_t tx_tick;
static bool warned_other_controller;

/* ------------------------------------------------------------------------ */
/* Receive ring (interrupt -> main loop)                                     */
/* ------------------------------------------------------------------------ */

static struct can2040_msg rx_ring[RX_RING_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint32_t rx_overflow;

static bool frame_is_interesting(uint32_t id) {
    if((id & 0x1FFFFF00u) == CUSTOM_CAN_ID_ANNOUNCE_BASE) return true;
    if(custom_can_id_prefix(id) != CAN_MASTER_PREFIX) return false;
    switch(custom_can_id_type(id)) {
        case CUSTOM_CAN_TYPE_STATUS:
        case CUSTOM_CAN_TYPE_LIMITS:
        case CUSTOM_CAN_TYPE_MEASUREMENTS:
        case CUSTOM_CAN_TYPE_ENERGY:
        case CUSTOM_CAN_TYPE_TEMPERATURES:
            return true;
        default:
            // Controller-type frames on our prefix mean a second master
            return custom_can_type_is_from_controller(custom_can_id_type(id));
    }
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
/* Transmit                                                                  */
/* ------------------------------------------------------------------------ */

static bool tx_budget_available(void) {
    if(tx_tick != timestep()) {
        tx_tick = timestep();
        tx_this_tick = 0;
    }
    return tx_this_tick < MAX_TX_PER_TICK;
}

static bool send_frame(uint32_t id, const uint8_t data[8], uint8_t dlc) {
    if(!tx_budget_available()) return false;
    struct can2040_msg msg = {0};
    msg.id = CAN2040_ID_EFF | id;
    msg.dlc = dlc;
    memcpy(msg.data, data, dlc);
    if(can_bus_transmit(CAN_MASTER_BUS, &msg) < 0) return false;
    tx_this_tick++;
    return true;
}

static bool send_assign(slave_t *s, uint8_t node, uint8_t flags) {
    uint8_t d[8];
    custom_can_put_u32(&d[0], s->serial);
    custom_can_put_u16(&d[4], CAN_MASTER_PREFIX);
    d[6] = node;
    d[7] = flags;
    return send_frame(CUSTOM_CAN_ID_ASSIGN, d, 8);
}

static bool send_command(slave_t *s, uint8_t desired) {
    uint8_t d[8] = {0};
    uint16_t seq = (uint16_t)(cmd_seq + 1);
    d[0] = desired;
    custom_can_put_u16(&d[2], seq);
    if(!send_frame(custom_can_make_id(CAN_MASTER_PREFIX, CUSTOM_CAN_TYPE_COMMAND, s->node), d, 8)) return false;
    cmd_seq = seq;
    s->last_seq_sent = seq;
    s->seq_sent_millis = millis();
    return true;
}

/* ------------------------------------------------------------------------ */
/* Slave table                                                               */
/* ------------------------------------------------------------------------ */

static slave_t *find_slave(uint32_t serial) {
    for(int i = 0; i < CAN_MASTER_MAX_SLAVES; i++) {
        if(slaves[i].serial == serial) return &slaves[i];
    }
    return NULL;
}

static slave_t *slave_for_node(uint8_t node) {
    if(node == CUSTOM_CAN_NODE_NONE) return NULL;
    for(int i = 0; i < CAN_MASTER_MAX_SLAVES; i++) {
        if(slaves[i].serial != 0 && slaves[i].node == node) return &slaves[i];
    }
    return NULL;
}

static slave_t *find_or_create_slave(uint32_t serial) {
    slave_t *s = find_slave(serial);
    if(s) return s;

    // Free slot, or evict the longest-forgotten slave without a node
    slave_t *victim = NULL;
    for(int i = 0; i < CAN_MASTER_MAX_SLAVES; i++) {
        if(slaves[i].serial == 0) {
            victim = &slaves[i];
            break;
        }
        if(slaves[i].node == 0 && (victim == NULL ||
            (int32_t)(slaves[i].last_announce_millis - victim->last_announce_millis) < 0)) {
            victim = &slaves[i];
        }
    }
    if(victim == NULL) {
        warning_printf("CAN master: no room for slave %08lX\n", (unsigned long)serial);
        return NULL;
    }
    if(victim->serial != 0) {
        info_printf("CAN master: forgetting slave %08lX\n", (unsigned long)victim->serial);
    }
    memset(victim, 0, sizeof(*victim));
    victim->serial = serial;
    victim->first_seen_millis = millis();
    info_printf("CAN master: discovered slave %08lX\n", (unsigned long)serial);
    return victim;
}

static uint8_t allocate_node(void) {
    for(uint8_t n = 1; n <= CAN_MASTER_MAX_SLAVES && n <= CUSTOM_CAN_NODE_MAX; n++) {
        if(slave_for_node(n) == NULL) return n;
    }
    return CUSTOM_CAN_NODE_NONE;
}

static void bind_node(slave_t *s, uint8_t node) {
    s->node = node;
    // Stagger heartbeats between slaves: start on a different tick each
    s->next_heartbeat_timestep = timestep() + (uint32_t)(s - slaves);
}

/* ------------------------------------------------------------------------ */
/* Receive handling                                                          */
/* ------------------------------------------------------------------------ */

static void on_announce(const struct can2040_msg *msg) {
    if(msg->dlc < 8) return;
    uint8_t version = msg->data[0] >> 4;
    bool assigned = (msg->data[0] & 0x0F) == CUSTOM_CAN_LINK_ASSIGNED;
    uint8_t their_node = msg->data[1];
    uint32_t serial = custom_can_get_u32(&msg->data[2]);
    uint16_t their_prefix = custom_can_get_u16(&msg->data[6]);

    slave_t *s = find_or_create_slave(serial);
    if(s == NULL) return;
    s->last_announce_millis = millis();

    if(version != CUSTOM_CAN_PROTOCOL_VERSION) {
        warning_printf("CAN master: slave %08lX speaks protocol v%u, we speak v%u\n",
            (unsigned long)serial, version, CUSTOM_CAN_PROTOCOL_VERSION);
    }

    // Slave still holds an assignment on our prefix (we probably rebooted):
    // adopt its node if it is free, so it never has to drop the link.
    if(assigned && their_prefix == CAN_MASTER_PREFIX && s->node == CUSTOM_CAN_NODE_NONE
       && their_node >= CUSTOM_CAN_NODE_MIN && their_node <= CAN_MASTER_MAX_SLAVES
       && slave_for_node(their_node) == NULL) {
        info_printf("CAN master: slave %08lX adopting existing node %u\n", (unsigned long)serial, their_node);
        bind_node(s, their_node);
    }

    if(s->node == CUSTOM_CAN_NODE_NONE) {
        uint8_t node = allocate_node();
        if(node == CUSTOM_CAN_NODE_NONE) {
            warning_printf("CAN master: no free node for slave %08lX\n", (unsigned long)serial);
            return;
        }
        bind_node(s, node);
    }

    bool consistent = assigned && their_prefix == CAN_MASTER_PREFIX && their_node == s->node;
    if(!consistent) {
        // If the budget is exhausted the slave will announce again shortly
        if(send_assign(s, s->node, CAN_MASTER_ASSIGN_FLAGS)) {
            s->assign_count++;
            info_printf("CAN master: slave %08lX -> node %u\n", (unsigned long)serial, s->node);
        }
    }
}

static void on_slave_frame(const struct can2040_msg *msg) {
    uint8_t type = custom_can_id_type(msg->id);
    uint8_t node = custom_can_id_node(msg->id);

    if(custom_can_type_is_from_controller(type)) {
        if(!warned_other_controller) {
            warned_other_controller = true;
            warning_printf("CAN master: another controller is transmitting on prefix 0x%04X (id 0x%08lX)\n",
                CAN_MASTER_PREFIX, (unsigned long)msg->id);
        }
        return;
    }

    slave_t *s = slave_for_node(node);
    if(s == NULL || msg->dlc < 8) return;
    const uint8_t *d = msg->data;

    switch(type) {
        case CUSTOM_CAN_TYPE_STATUS:
            s->system_state = d[0];
            s->contactor_state = d[1];
            s->status_flags = d[2];
            s->level = d[3] & 0x07;
            s->commanded = (d[3] >> 4) & 0x03;
            s->last_seq = custom_can_get_u16(&d[4]);
            s->top_event = d[6];
            s->last_status_millis = millis();
            if(s->last_seq == s->last_seq_sent && s->seq_sent_millis != 0) {
                s->rtt_ms = (uint16_t)(millis() - s->seq_sent_millis);
                s->seq_sent_millis = 0;
            }
            break;
        case CUSTOM_CAN_TYPE_LIMITS:
            s->max_voltage_dV = custom_can_get_i16(&d[0]);
            s->min_voltage_dV = custom_can_get_i16(&d[2]);
            s->charge_dA = custom_can_get_u16(&d[4]);
            s->discharge_dA = custom_can_get_u16(&d[6]);
            s->limits_valid = true;
            break;
        case CUSTOM_CAN_TYPE_MEASUREMENTS:
            s->voltage_cV = custom_can_get_u16(&d[0]);
            s->current_mA = custom_can_get_i32(&d[2]);
            s->soc = custom_can_get_u16(&d[6]);
            s->meas_valid = true;
            break;
        case CUSTOM_CAN_TYPE_ENERGY:
            s->soh = custom_can_get_u16(&d[0]);
            s->remaining_dAh = custom_can_get_u16(&d[2]);
            s->full_dAh = custom_can_get_u16(&d[4]);
            s->energy_valid = true;
            break;
        case CUSTOM_CAN_TYPE_TEMPERATURES:
            s->temp_min_dC = custom_can_get_i16(&d[0]);
            s->temp_max_dC = custom_can_get_i16(&d[2]);
            s->temps_valid = true;
            break;
        default:
            break;
    }
}

static void process_rx(void) {
    while(rx_tail != rx_head) {
        struct can2040_msg msg = rx_ring[rx_tail];
        rx_tail = (uint8_t)((rx_tail + 1) % RX_RING_SIZE);
        if((msg.id & 0x1FFFFF00u) == CUSTOM_CAN_ID_ANNOUNCE_BASE) {
            on_announce(&msg);
        } else {
            on_slave_frame(&msg);
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Liveness and heartbeat                                                    */
/* ------------------------------------------------------------------------ */

static bool slave_contributing(const slave_t *s) {
    return s->online && s->limits_valid
        && (s->status_flags & CUSTOM_CAN_STATUS_FLAG_CURRENT_ENABLED)
        && s->level < CUSTOM_CAN_LEVEL_CRITICAL;
}

static void update_liveness(void) {
    millis_t now = millis();
    bool any_lost = false;
    bool any_fault = false;

    for(int i = 0; i < CAN_MASTER_MAX_SLAVES; i++) {
        slave_t *s = &slaves[i];
        if(s->serial == 0) continue;

        bool online = s->last_status_millis != 0 && (now - s->last_status_millis) <= CUSTOM_CAN_BATTERY_TIMEOUT_MS;
        if(online != s->online) {
            s->online = online;
            if(online) {
                info_printf("CAN master: slave %08lX (node %u) online\n", (unsigned long)s->serial, s->node);
            } else {
                s->offline_since_millis = now;
                warning_printf("CAN master: slave %08lX (node %u) LOST\n", (unsigned long)s->serial, s->node);
                count_bms_event(ERR_CAN_SLAVE_LOST, s->serial);
            }
        }

        if(!online && s->node != CUSTOM_CAN_NODE_NONE) {
            // Never heard from at all, or gone for a long time: free the node
            if(s->last_status_millis != 0 && (now - s->offline_since_millis) > CAN_MASTER_RETIRE_MS) {
                info_printf("CAN master: slave %08lX retired from node %u\n", (unsigned long)s->serial, s->node);
                s->node = CUSTOM_CAN_NODE_NONE;
            } else if(s->last_status_millis != 0) {
                any_lost = true;
            }
        }
        if(online && s->level >= CUSTOM_CAN_LEVEL_FATAL) any_fault = true;
    }

    if(!any_lost) clear_bms_event(ERR_CAN_SLAVE_LOST);
    if(any_fault) {
        raise_bms_event(ERR_CAN_SLAVE_FAULT, 0);
    } else {
        clear_bms_event(ERR_CAN_SLAVE_FAULT);
    }
}

static void send_heartbeats(const bms_model_t *model) {
    // The operator's intent for this battery is the intent for the fleet.
    // (model->operating is cleared by the system state machine on FATAL.)
    uint8_t desired = model->operating ? CUSTOM_CAN_DESIRED_RUN : CUSTOM_CAN_DESIRED_STOP;

    for(int i = 0; i < CAN_MASTER_MAX_SLAVES; i++) {
        slave_t *s = &slaves[i];
        if(s->serial == 0 || s->node == CUSTOM_CAN_NODE_NONE) continue;
        if((int32_t)(timestep() - s->next_heartbeat_timestep) < 0) continue;
        if(!send_command(s, desired)) return; // budget/queue exhausted, retry next tick
        s->next_heartbeat_timestep = timestep() + MS_TO_TICKS(CAN_MASTER_HEARTBEAT_MS);
    }
}

/* ------------------------------------------------------------------------ */
/* Aggregation                                                               */
/* ------------------------------------------------------------------------ */

static uint16_t sat_u16(uint32_t v) {
    return v > 65535u ? 65535u : (uint16_t)v;
}

static int16_t sat_i16(int32_t v) {
    if(v > 32767) return 32767;
    if(v < -32768) return -32768;
    return (int16_t)v;
}

static void aggregate(bms_model_t *model) {
    const inverter_outputs_t *local = &model->inverter_outputs;
    inverter_outputs_t out = *local; // voltage, timestamps and fallbacks come from the local battery
    fleet_summary_t fleet = {0};
    millis_t now = millis();

    bool local_contributing = model->contactor_sm.enable_current && get_highest_event_level() < LEVEL_CRITICAL;

    uint32_t charge_dA = 0, discharge_dA = 0, remaining_dAh = 0, full_dAh = 0;
    uint64_t soc_weighted = 0;
    uint32_t soc_weight = 0;
    int32_t max_voltage_dV = 0, min_voltage_dV = 0;
    bool have_voltage_limits = false;
    int32_t current_mA = local->current_millis != 0 ? local->current_mA : 0;
    float temp_min = local->temperature_min, temp_max = local->temperature_max;
    bool have_temps = local->temperature_millis != 0;
    uint32_t slave_voltage_sum_cV = 0;
    int slave_voltage_count = 0;

    if(local_contributing) {
        charge_dA += local->charge_current_limit_dA;
        discharge_dA += local->discharge_current_limit_dA;
        if(local->remaining_capacity_dAh > 0) remaining_dAh += (uint32_t)local->remaining_capacity_dAh;
        if(local->full_capacity_dAh > 0) {
            full_dAh += (uint32_t)local->full_capacity_dAh;
            soc_weighted += (uint64_t)(local->soc > 0 ? local->soc : 0) * (uint32_t)local->full_capacity_dAh;
            soc_weight += (uint32_t)local->full_capacity_dAh;
        }
        max_voltage_dV = local->max_voltage_limit_dV;
        min_voltage_dV = local->min_voltage_limit_dV;
        have_voltage_limits = true;
    }

    for(int i = 0; i < CAN_MASTER_MAX_SLAVES; i++) {
        const slave_t *s = &slaves[i];
        if(s->serial == 0) continue;
        fleet.slaves_known++;
        if(!s->online) continue;
        fleet.slaves_online++;
        if(s->level > fleet.worst_slave_level) fleet.worst_slave_level = s->level;

        if(s->meas_valid) {
            current_mA += s->current_mA;
            slave_voltage_sum_cV += s->voltage_cV;
            slave_voltage_count++;
        }
        if(s->temps_valid) {
            float tmin = s->temp_min_dC * 0.1f, tmax = s->temp_max_dC * 0.1f;
            if(!have_temps) {
                temp_min = tmin;
                temp_max = tmax;
                have_temps = true;
            } else {
                if(tmin < temp_min) temp_min = tmin;
                if(tmax > temp_max) temp_max = tmax;
            }
        }

        if(!slave_contributing(s)) continue;
        fleet.slaves_contributing++;

        charge_dA += s->charge_dA;
        discharge_dA += s->discharge_dA;
        if(!have_voltage_limits) {
            max_voltage_dV = s->max_voltage_dV;
            min_voltage_dV = s->min_voltage_dV;
            have_voltage_limits = true;
        } else {
            if(s->max_voltage_dV < max_voltage_dV) max_voltage_dV = s->max_voltage_dV;
            if(s->min_voltage_dV > min_voltage_dV) min_voltage_dV = s->min_voltage_dV;
        }
        if(s->energy_valid) {
            remaining_dAh += s->remaining_dAh;
            full_dAh += s->full_dAh;
            if(s->meas_valid && s->full_dAh > 0) {
                soc_weighted += (uint64_t)s->soc * s->full_dAh;
                soc_weight += s->full_dAh;
            }
        }
    }

    out.charge_current_limit_dA = sat_u16(charge_dA);
    out.discharge_current_limit_dA = sat_u16(discharge_dA);
    if(have_voltage_limits) {
        out.max_voltage_limit_dV = sat_i16(max_voltage_dV);
        out.min_voltage_limit_dV = sat_i16(min_voltage_dV);
    }
    out.remaining_capacity_dAh = sat_i16((int32_t)(remaining_dAh > 32767u ? 32767u : remaining_dAh));
    out.full_capacity_dAh = sat_i16((int32_t)(full_dAh > 32767u ? 32767u : full_dAh));

    if(soc_weight > 0) {
        out.soc = (int16_t)(soc_weighted / soc_weight);
        if(out.soc_millis == 0) out.soc_millis = now;
    }
    // Stop inverters that only look at SoC, as model.c does for one battery
    if(out.discharge_current_limit_dA == 0) {
        out.soc = 0;
    } else if(out.charge_current_limit_dA == 0) {
        out.soc = 10000;
    }

    out.current_mA = current_mA;
    if(out.current_millis == 0 && slave_voltage_count > 0) out.current_millis = now;
    if(out.battery_voltage_millis == 0 && slave_voltage_count > 0) {
        out.battery_voltage = (slave_voltage_sum_cV / (float)slave_voltage_count) * 0.01f;
        out.battery_voltage_millis = now;
    }
    if(have_temps) {
        out.temperature_min = temp_min;
        out.temperature_max = temp_max;
        if(out.temperature_millis == 0) out.temperature_millis = now;
    }

    model->fleet_outputs = out;
    model->fleet = fleet;
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

void init_can_master(void) {
    memset(slaves, 0, sizeof(slaves));
    cmd_seq = 0;
    tx_this_tick = 0;
    tx_tick = 0;
    warned_other_controller = false;
    rx_head = rx_tail = 0;
    rx_overflow = 0;
    can_bus_init(CAN_MASTER_BUS, can2040_cb);
    info_printf("CAN master: fleet prefix 0x%04X (base id 0x%08lX), up to %d slaves\n",
        CAN_MASTER_PREFIX, (unsigned long)custom_can_make_id(CAN_MASTER_PREFIX, 0, 0), CAN_MASTER_MAX_SLAVES);
}

void can_master_tick(bms_model_t *model) {
    process_rx();
    update_liveness();
    send_heartbeats(model);
    aggregate(model);
}

void can_master_print_status(void) {
    debug_printf("Fleet: %u known, %u online, %u contributing | limits chg %u dA dis %u dA | I %ld mA | SoC %d | rx overflow %lu\n",
        model.fleet.slaves_known, model.fleet.slaves_online, model.fleet.slaves_contributing,
        model.fleet_outputs.charge_current_limit_dA, model.fleet_outputs.discharge_current_limit_dA,
        (long)model.fleet_outputs.current_mA, model.fleet_outputs.soc, (unsigned long)rx_overflow);
    for(int i = 0; i < CAN_MASTER_MAX_SLAVES; i++) {
        const slave_t *s = &slaves[i];
        if(s->serial == 0) continue;
        debug_printf("  %08lX n%u %s sys %u ctr %u flags %02X lvl %u cmd %u | %u.%02u V %ld mA soc %u | chg %u dis %u | rtt %u ms\n",
            (unsigned long)s->serial, s->node, s->online ? "ONLINE " : "offline",
            s->system_state, s->contactor_state, s->status_flags, s->level, s->commanded,
            s->voltage_cV / 100, s->voltage_cV % 100, (long)s->current_mA, s->soc,
            s->charge_dA, s->discharge_dA, s->rtt_ms);
    }
}
