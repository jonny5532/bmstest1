// Host-side tests for master mode (protocols/inverter/can_master.c).
//
// Slave batteries are simulated by injecting the frames they would send
// (built with the shared custom_can.h encoders) and the master's transmissions
// are captured from the can2040 stub.

#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app/model.h"
#include "protocols/inverter/can.h"
#include "protocols/inverter/can_master.h"
#include "protocols/inverter/custom_can.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "can2040.h"

/* ---------------------------------------------------------------- stubs -- */

static bool verbose = false;

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    if(!verbose) return;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;
uint32_t stored_timestep = 0;

#define TX_LOG_SIZE 4096
static struct can2040_msg tx_log[TX_LOG_SIZE];
static uint32_t tx_step[TX_LOG_SIZE];
static int tx_count = 0;

int can2040_transmit(struct can2040 *cd, const struct can2040_msg *msg) {
    (void)cd;
    assert_true(tx_count < TX_LOG_SIZE);
    tx_log[tx_count] = *msg;
    tx_step[tx_count] = stored_timestep;
    tx_count++;
    return 0;
}

static can2040_rx_cb rx_cb = NULL;
void can2040_callback_config(struct can2040 *cd, can2040_rx_cb cb) { (void)cd; rx_cb = cb; }
void can2040_setup(struct can2040 *cd, uint32_t pio_num) { (void)cd; (void)pio_num; }
void can2040_start(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate, uint32_t gpio_rx, uint32_t gpio_tx) {
    (void)cd; (void)sys_clock; (void)bitrate; (void)gpio_rx; (void)gpio_tx;
}
void can2040_pio_irq_handler(struct can2040 *cd) { (void)cd; }

/* -------------------------------------------------------------- helpers -- */

#define SERIAL_A 0x11111111u
#define SERIAL_B 0x22222222u
#define SERIAL_C 0x33333333u

static void clear_tx(void) { tx_count = 0; }

static void tick(int n) {
    for(int i = 0; i < n; i++) {
        can_master_tick(&model);
        stored_timestep++;
        stored_millis += TIMESTEP_PERIOD_MS;
        stored_millis64 += TIMESTEP_PERIOD_MS;
    }
}

static void rx(uint32_t id29, const uint8_t *data, uint32_t dlc) {
    struct can2040_msg msg = {0};
    msg.id = CAN2040_ID_EFF | id29;
    msg.dlc = dlc;
    memcpy(msg.data, data, dlc);
    rx_cb(NULL, CAN2040_NOTIFY_RX, &msg);
}

static int count_id(uint32_t id29) {
    int n = 0;
    for(int i = 0; i < tx_count; i++) {
        if((tx_log[i].id & CAN2040_ID_EFF) && (tx_log[i].id & CUSTOM_CAN_ID_MASK) == id29) n++;
    }
    return n;
}

static const struct can2040_msg *last_id(uint32_t id29) {
    for(int i = tx_count - 1; i >= 0; i--) {
        if((tx_log[i].id & CUSTOM_CAN_ID_MASK) == id29) return &tx_log[i];
    }
    return NULL;
}

// Most recent ASSIGN addressed to a serial, or NULL
static const struct can2040_msg *last_assign_for(uint32_t serial) {
    for(int i = tx_count - 1; i >= 0; i--) {
        if((tx_log[i].id & CUSTOM_CAN_ID_MASK) == CUSTOM_CAN_ID_ASSIGN
           && custom_can_get_u32(&tx_log[i].data[0]) == serial) return &tx_log[i];
    }
    return NULL;
}

static int count_assigns_for(uint32_t serial) {
    int n = 0;
    for(int i = 0; i < tx_count; i++) {
        if((tx_log[i].id & CUSTOM_CAN_ID_MASK) == CUSTOM_CAN_ID_ASSIGN
           && custom_can_get_u32(&tx_log[i].data[0]) == serial) n++;
    }
    return n;
}

static uint32_t cmd_id(uint8_t node) {
    return custom_can_make_id(CAN_MASTER_PREFIX, CUSTOM_CAN_TYPE_COMMAND, node);
}

/* ---- frames a slave would send ---- */

static void slave_announce(uint32_t serial, bool assigned, uint8_t node, uint16_t prefix) {
    uint8_t d[8];
    d[0] = (uint8_t)((CUSTOM_CAN_PROTOCOL_VERSION << 4) | (assigned ? CUSTOM_CAN_LINK_ASSIGNED : CUSTOM_CAN_LINK_UNASSIGNED));
    d[1] = assigned ? node : 0;
    custom_can_put_u32(&d[2], serial);
    custom_can_put_u16(&d[6], assigned ? prefix : 0);
    rx(CUSTOM_CAN_ID_ANNOUNCE_BASE | (serial & 0xFF), d, 8);
}

static void slave_status(uint8_t node, uint8_t sys, uint8_t ctr, uint8_t flags, uint8_t level, uint8_t commanded, uint16_t seq) {
    uint8_t d[8] = {0};
    d[0] = sys;
    d[1] = ctr;
    d[2] = flags;
    d[3] = (uint8_t)((level & 0x07) | ((commanded & 0x03) << 4));
    custom_can_put_u16(&d[4], seq);
    d[6] = 0xFF;
    rx(custom_can_make_id(CAN_MASTER_PREFIX, CUSTOM_CAN_TYPE_STATUS, node), d, 8);
}

static void slave_limits(uint8_t node, int16_t maxv, int16_t minv, uint16_t chg, uint16_t dis) {
    uint8_t d[8];
    custom_can_put_i16(&d[0], maxv);
    custom_can_put_i16(&d[2], minv);
    custom_can_put_u16(&d[4], chg);
    custom_can_put_u16(&d[6], dis);
    rx(custom_can_make_id(CAN_MASTER_PREFIX, CUSTOM_CAN_TYPE_LIMITS, node), d, 8);
}

static void slave_measurements(uint8_t node, uint16_t v_cV, int32_t i_mA, uint16_t soc) {
    uint8_t d[8];
    custom_can_put_u16(&d[0], v_cV);
    custom_can_put_i32(&d[2], i_mA);
    custom_can_put_u16(&d[6], soc);
    rx(custom_can_make_id(CAN_MASTER_PREFIX, CUSTOM_CAN_TYPE_MEASUREMENTS, node), d, 8);
}

static void slave_energy(uint8_t node, uint16_t soh, uint16_t remaining, uint16_t full) {
    uint8_t d[8] = {0};
    custom_can_put_u16(&d[0], soh);
    custom_can_put_u16(&d[2], remaining);
    custom_can_put_u16(&d[4], full);
    rx(custom_can_make_id(CAN_MASTER_PREFIX, CUSTOM_CAN_TYPE_ENERGY, node), d, 8);
}

static void slave_temperatures(uint8_t node, int16_t min_dC, int16_t max_dC) {
    uint8_t d[8] = {0};
    custom_can_put_i16(&d[0], min_dC);
    custom_can_put_i16(&d[2], max_dC);
    rx(custom_can_make_id(CAN_MASTER_PREFIX, CUSTOM_CAN_TYPE_TEMPERATURES, node), d, 8);
}

typedef struct {
    uint8_t node;
    bool enabled;
    uint8_t level;
    uint16_t chg, dis;
    int16_t maxv, minv;
    uint16_t v_cV;
    int32_t i_mA;
    uint16_t soc, remaining, full;
    int16_t tmin, tmax;
} fake_slave_t;

// One full telemetry set, as a healthy slave would emit within a second
static void slave_report(const fake_slave_t *s) {
    slave_status(s->node, CUSTOM_CAN_SYSTEM_OPERATING, CUSTOM_CAN_CONTACTORS_CLOSED,
                 s->enabled ? CUSTOM_CAN_STATUS_FLAG_CURRENT_ENABLED : 0, s->level, CUSTOM_CAN_DESIRED_RUN, 0);
    slave_limits(s->node, s->maxv, s->minv, s->chg, s->dis);
    slave_measurements(s->node, s->v_cV, s->i_mA, s->soc);
    slave_energy(s->node, 9900, s->remaining, s->full);
    slave_temperatures(s->node, s->tmin, s->tmax);
}

static fake_slave_t SLAVE1 = { .node = 1, .enabled = true, .level = 0, .chg = 200, .dis = 150, .maxv = 3880, .minv = 3250,
                               .v_cV = 37000, .i_mA = -2000, .soc = 7000, .remaining = 700, .full = 1000, .tmin = 150, .tmax = 350 };
static fake_slave_t SLAVE2 = { .node = 2, .enabled = true, .level = 0, .chg = 50, .dis = 60, .maxv = 3950, .minv = 3100,
                               .v_cV = 36900, .i_mA = 500, .soc = 3000, .remaining = 900, .full = 3000, .tmin = 200, .tmax = 250 };

static void setup(void) {
    memset(&model, 0, sizeof(model));
    for(int i = 0; i < ERR_HIGHEST; i++) clear_bms_event((bms_event_type_t)i);
    memset(bms_event_slots, 0, sizeof(bms_event_slot_t) * ERR_HIGHEST);
    stored_millis = stored_millis64 = 100000;
    stored_timestep = 5000;
    clear_tx();

    // A healthy local battery, operating with current enabled
    model.operating = true;
    model.contactor_sm.enable_current = true;
    inverter_outputs_t *o = &model.inverter_outputs;
    o->charge_current_limit_dA = 100;
    o->discharge_current_limit_dA = 120;
    o->max_voltage_limit_dV = 3900;
    o->min_voltage_limit_dV = 3200;
    o->soc = 5000;
    o->soc_millis = stored_millis;
    o->remaining_capacity_dAh = 500;
    o->full_capacity_dAh = 1000;
    o->battery_voltage = 371.5f;
    o->battery_voltage_millis = stored_millis;
    o->current_mA = 1000;
    o->current_millis = stored_millis;
    o->temperature_min = 20.0f;
    o->temperature_max = 30.0f;
    o->temperature_millis = stored_millis;

    init_can_master();
}

// Discover and assign a slave, and have it report once
static void bring_up(fake_slave_t *s, uint32_t serial) {
    slave_announce(serial, false, 0, 0);
    tick(1);
    const struct can2040_msg *a = last_assign_for(serial);
    assert_non_null(a);
    s->node = a->data[6];
    slave_announce(serial, true, s->node, CAN_MASTER_PREFIX);
    slave_report(s);
    tick(1);
}

/* ---------------------------------------------------------------- tests -- */

static void test_discovery_assigns_distinct_nodes(void **state) {
    (void)state;
    setup();

    slave_announce(SERIAL_A, false, 0, 0);
    slave_announce(SERIAL_B, false, 0, 0);
    tick(1);

    const struct can2040_msg *a = last_assign_for(SERIAL_A);
    const struct can2040_msg *b = last_assign_for(SERIAL_B);
    assert_non_null(a);
    assert_non_null(b);
    assert_int_equal(custom_can_get_u16(&a->data[4]), CAN_MASTER_PREFIX);
    assert_int_equal(a->data[6], 1);
    assert_int_equal(b->data[6], 2);
    assert_int_equal(a->data[7], CAN_MASTER_ASSIGN_FLAGS);

    // Still unassigned (ASSIGN lost?) -> same node again; consistent -> quiet
    slave_announce(SERIAL_A, false, 0, 0);
    tick(1);
    assert_int_equal(count_assigns_for(SERIAL_A), 2);
    assert_int_equal(last_assign_for(SERIAL_A)->data[6], 1);
    slave_announce(SERIAL_A, true, 1, CAN_MASTER_PREFIX);
    tick(1);
    assert_int_equal(count_assigns_for(SERIAL_A), 2);

    // Wrong prefix (someone else's controller): claimed for ours
    slave_announce(SERIAL_A, true, 1, CAN_MASTER_PREFIX + 1);
    tick(1);
    assert_int_equal(count_assigns_for(SERIAL_A), 3);
}

static void test_adopts_existing_assignment_after_reboot(void **state) {
    (void)state;
    setup();

    // Slave still thinks it is node 3 on our prefix (we just rebooted)
    slave_announce(SERIAL_C, true, 3, CAN_MASTER_PREFIX);
    tick(1);
    assert_int_equal(count_assigns_for(SERIAL_C), 0); // nothing to fix
    clear_tx();
    tick(30);
    assert_true(count_id(cmd_id(3)) >= 1); // heartbeats go to its node

    // A newcomer must not get node 3
    slave_announce(SERIAL_A, false, 0, 0);
    tick(1);
    assert_int_not_equal(last_assign_for(SERIAL_A)->data[6], 3);
}

static void test_heartbeat_follows_operating_intent(void **state) {
    (void)state;
    setup();
    bring_up(&SLAVE1, SERIAL_A);
    bring_up(&SLAVE2, SERIAL_B);
    clear_tx();

    model.operating = true;
    tick(100); // 2 s
    int n1 = count_id(cmd_id(SLAVE1.node));
    int n2 = count_id(cmd_id(SLAVE2.node));
    assert_true(n1 >= 3 && n1 <= 5); // every 500 ms
    assert_true(n2 >= 3 && n2 <= 5);
    assert_int_equal(last_id(cmd_id(SLAVE1.node))->data[0], CUSTOM_CAN_DESIRED_RUN);

    model.operating = false; // operator stop, or FATAL on the master
    clear_tx();
    tick(30);
    assert_int_equal(last_id(cmd_id(SLAVE1.node))->data[0], CUSTOM_CAN_DESIRED_STOP);
    assert_int_equal(last_id(cmd_id(SLAVE2.node))->data[0], CUSTOM_CAN_DESIRED_STOP);

    // Sequence numbers advance and are echoed for RTT
    uint16_t seq = custom_can_get_u16(&last_id(cmd_id(SLAVE1.node))->data[2]);
    assert_true(seq > 0);

    // Never more than the budget per tick
    int per_tick = 1;
    for(int i = 1; i < tx_count; i++) {
        per_tick = (tx_step[i] == tx_step[i - 1]) ? per_tick + 1 : 1;
        assert_true(per_tick <= 2);
    }
}

static void test_aggregation(void **state) {
    (void)state;
    setup();
    bring_up(&SLAVE1, SERIAL_A);
    tick(1);

    const inverter_outputs_t *f = &model.fleet_outputs;
    assert_int_equal(model.fleet.slaves_known, 1);
    assert_int_equal(model.fleet.slaves_online, 1);
    assert_int_equal(model.fleet.slaves_contributing, 1);
    assert_int_equal(f->charge_current_limit_dA, 100 + 200);
    assert_int_equal(f->discharge_current_limit_dA, 120 + 150);
    assert_int_equal(f->max_voltage_limit_dV, 3880); // tightest of 3900/3880
    assert_int_equal(f->min_voltage_limit_dV, 3250); // tightest of 3200/3250
    assert_int_equal(f->current_mA, 1000 - 2000);
    assert_int_equal(f->soc, (5000 * 1000 + 7000 * 1000) / 2000);
    assert_int_equal(f->remaining_capacity_dAh, 500 + 700);
    assert_int_equal(f->full_capacity_dAh, 1000 + 1000);
    assert_float_equal(f->temperature_min, 15.0f, 0.01f);
    assert_float_equal(f->temperature_max, 35.0f, 0.01f);
    assert_float_equal(f->battery_voltage, 371.5f, 0.01f); // local measurement
    assert_int_equal(f->soc_millis, model.inverter_outputs.soc_millis);

    // Second slave with a bigger pack shifts the weighted SoC
    bring_up(&SLAVE2, SERIAL_B);
    tick(1);
    assert_int_equal(model.fleet.slaves_contributing, 2);
    assert_int_equal(f->charge_current_limit_dA, 100 + 200 + 50);
    assert_int_equal(f->min_voltage_limit_dV, 3250);
    assert_int_equal(f->max_voltage_limit_dV, 3880);
    assert_int_equal(f->soc, (5000 * 1000 + 7000 * 1000 + 3000 * 3000) / 5000);
    assert_int_equal(f->full_capacity_dAh, 5000);

    // Slave 1 disables current (e.g. it is stopping): it still counts for
    // current and temperature, but not for limits or capacity
    SLAVE1.enabled = false;
    slave_report(&SLAVE1);
    tick(1);
    assert_int_equal(model.fleet.slaves_online, 2);
    assert_int_equal(model.fleet.slaves_contributing, 1);
    assert_int_equal(f->charge_current_limit_dA, 100 + 50);
    assert_int_equal(f->current_mA, 1000 - 2000 + 500);
    assert_int_equal(f->max_voltage_limit_dV, 3900); // slave 1's limits no longer apply
    SLAVE1.enabled = true;

    // A CRITICAL slave is excluded too; FATAL also raises a warning here
    SLAVE1.level = CUSTOM_CAN_LEVEL_CRITICAL;
    slave_report(&SLAVE1);
    tick(1);
    assert_int_equal(model.fleet.slaves_contributing, 1);
    assert_int_equal(model.fleet.worst_slave_level, CUSTOM_CAN_LEVEL_CRITICAL);
    assert_int_equal(get_event_level(ERR_CAN_SLAVE_FAULT), LEVEL_NONE);
    SLAVE1.level = CUSTOM_CAN_LEVEL_FATAL;
    slave_report(&SLAVE1);
    tick(1);
    assert_int_equal(get_event_level(ERR_CAN_SLAVE_FAULT), LEVEL_WARNING);
    SLAVE1.level = 0;
    slave_report(&SLAVE1);
    tick(1);
    assert_int_equal(get_event_level(ERR_CAN_SLAVE_FAULT), LEVEL_NONE);
    assert_int_equal(model.fleet.slaves_contributing, 2);
}

static void test_local_battery_not_contributing(void **state) {
    (void)state;
    setup();
    bring_up(&SLAVE1, SERIAL_A);

    model.contactor_sm.enable_current = false; // local contactors open/opening
    tick(1);
    const inverter_outputs_t *f = &model.fleet_outputs;
    assert_int_equal(f->charge_current_limit_dA, 200);
    assert_int_equal(f->discharge_current_limit_dA, 150);
    assert_int_equal(f->max_voltage_limit_dV, 3880); // only the slave's limits apply
    assert_int_equal(f->min_voltage_limit_dV, 3250);
    assert_int_equal(f->soc, 7000);
    assert_int_equal(f->full_capacity_dAh, 1000);
    assert_int_equal(f->current_mA, 1000 - 2000); // measured current still includes us

    // A critical event on the master excludes it as well
    model.contactor_sm.enable_current = true;
    raise_bms_event(ERR_CELL_VOLTAGE_VERY_HIGH, 0);
    tick(1);
    assert_int_equal(f->charge_current_limit_dA, 200);
    clear_bms_event(ERR_CELL_VOLTAGE_VERY_HIGH);
    tick(1);
    assert_int_equal(f->charge_current_limit_dA, 300);
}

static void test_soc_forced_when_fleet_cannot_flow(void **state) {
    (void)state;
    setup();
    bring_up(&SLAVE1, SERIAL_A);

    model.inverter_outputs.discharge_current_limit_dA = 0;
    SLAVE1.dis = 0;
    slave_report(&SLAVE1);
    tick(1);
    assert_int_equal(model.fleet_outputs.discharge_current_limit_dA, 0);
    assert_int_equal(model.fleet_outputs.soc, 0);

    model.inverter_outputs.discharge_current_limit_dA = 120;
    model.inverter_outputs.charge_current_limit_dA = 0;
    SLAVE1.dis = 150;
    SLAVE1.chg = 0;
    slave_report(&SLAVE1);
    tick(1);
    assert_int_equal(model.fleet_outputs.charge_current_limit_dA, 0);
    assert_int_equal(model.fleet_outputs.soc, 10000);
    SLAVE1.chg = 200;

    // Nobody contributing at all -> everything zero, SoC 0
    model.contactor_sm.enable_current = false;
    SLAVE1.enabled = false;
    slave_report(&SLAVE1);
    tick(1);
    assert_int_equal(model.fleet_outputs.charge_current_limit_dA, 0);
    assert_int_equal(model.fleet_outputs.discharge_current_limit_dA, 0);
    assert_int_equal(model.fleet_outputs.soc, 0);
    SLAVE1.enabled = true;
}

static void test_slave_lost_and_recovered(void **state) {
    (void)state;
    setup();
    bring_up(&SLAVE1, SERIAL_A);
    bring_up(&SLAVE2, SERIAL_B);
    tick(1);
    assert_int_equal(model.fleet.slaves_online, 2);
    assert_int_equal(get_event_level(ERR_CAN_SLAVE_LOST), LEVEL_NONE);

    // Slave 2 keeps reporting, slave 1 goes silent
    for(int i = 0; i < 6; i++) {
        slave_report(&SLAVE2);
        tick(25); // 500 ms
    }
    assert_int_equal(model.fleet.slaves_known, 2);
    assert_int_equal(model.fleet.slaves_online, 1);
    assert_int_equal(model.fleet.slaves_contributing, 1);
    assert_int_equal(model.fleet_outputs.charge_current_limit_dA, 100 + 50);
    assert_int_equal(get_event_level(ERR_CAN_SLAVE_LOST), LEVEL_WARNING);
    assert_int_equal(get_event_count(ERR_CAN_SLAVE_LOST), 1);
    assert_int_equal(bms_event_slots[ERR_CAN_SLAVE_LOST].data32[0], SERIAL_A);

    // It comes back (same node, no re-assignment needed)
    slave_report(&SLAVE1);
    tick(1);
    assert_int_equal(model.fleet.slaves_online, 2);
    assert_int_equal(get_event_level(ERR_CAN_SLAVE_LOST), LEVEL_NONE);
    assert_int_equal(model.fleet_outputs.charge_current_limit_dA, 350);
}

static void test_retired_node_is_reused(void **state) {
    (void)state;
    setup();
    bring_up(&SLAVE1, SERIAL_A);
    uint8_t node_a = SLAVE1.node;

    // Gone for good: after the retire timeout its node is free
    tick((CAN_MASTER_RETIRE_MS + CUSTOM_CAN_BATTERY_TIMEOUT_MS) / TIMESTEP_PERIOD_MS + 100);
    clear_tx();
    tick(50);
    assert_int_equal(count_id(cmd_id(node_a)), 0); // no more heartbeats to it

    slave_announce(SERIAL_B, false, 0, 0);
    tick(1);
    assert_int_equal(last_assign_for(SERIAL_B)->data[6], node_a);

    // ...and if A returns it is simply assigned another node
    slave_announce(SERIAL_A, false, 0, 0);
    tick(1);
    assert_int_not_equal(last_assign_for(SERIAL_A)->data[6], node_a);
}

static void test_local_sensor_fallback(void **state) {
    (void)state;
    setup();
    bring_up(&SLAVE1, SERIAL_A);

    // Master's own pack voltage/current/temperature unavailable: report the fleet's
    model.inverter_outputs.battery_voltage_millis = 0;
    model.inverter_outputs.current_millis = 0;
    model.inverter_outputs.temperature_millis = 0;
    model.inverter_outputs.soc_millis = 0;
    tick(1);
    const inverter_outputs_t *f = &model.fleet_outputs;
    assert_float_equal(f->battery_voltage, 370.0f, 0.01f);
    assert_true(f->battery_voltage_millis != 0);
    assert_int_equal(f->current_mA, -2000);
    assert_true(f->current_millis != 0);
    assert_float_equal(f->temperature_min, 15.0f, 0.01f);
    assert_true(f->temperature_millis != 0);
    assert_true(f->soc_millis != 0);
}

static void test_ignores_foreign_and_malformed_frames(void **state) {
    (void)state;
    setup();
    bring_up(&SLAVE1, SERIAL_A);
    clear_tx();

    // Another prefix, a standard-id frame, an unknown node, and a short frame
    uint8_t d[8] = {0};
    rx(custom_can_make_id(CAN_MASTER_PREFIX + 1, CUSTOM_CAN_TYPE_STATUS, SLAVE1.node), d, 8);
    struct can2040_msg std = { .id = 0x110, .dlc = 8 };
    rx_cb(NULL, CAN2040_NOTIFY_RX, &std);
    slave_limits(200, 1, 1, 1, 1);
    rx(custom_can_make_id(CAN_MASTER_PREFIX, CUSTOM_CAN_TYPE_LIMITS, SLAVE1.node), d, 4);
    tick(1);

    assert_int_equal(model.fleet_outputs.charge_current_limit_dA, 300);
    assert_int_equal(model.fleet.slaves_known, 1);
}

int main(void) {
    verbose = getenv("VERBOSE") != NULL;
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_discovery_assigns_distinct_nodes),
        cmocka_unit_test(test_adopts_existing_assignment_after_reboot),
        cmocka_unit_test(test_heartbeat_follows_operating_intent),
        cmocka_unit_test(test_aggregation),
        cmocka_unit_test(test_local_battery_not_contributing),
        cmocka_unit_test(test_soc_forced_when_fleet_cannot_flow),
        cmocka_unit_test(test_slave_lost_and_recovered),
        cmocka_unit_test(test_retired_node_is_reused),
        cmocka_unit_test(test_local_sensor_fallback),
        cmocka_unit_test(test_ignores_foreign_and_malformed_frames),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
