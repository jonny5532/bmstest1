// Host-side tests for the custom multi-battery CAN protocol (battery side).
//
// The protocol module is driven purely through its public surface: frames are
// injected via the can2040 receive callback and captured from the transmit
// stub, and the model is manipulated as the state machines would.

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
#include "protocols/inverter/inverter.h"
#include "protocols/inverter/custom_can.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "can2040.h"
#include "pico/unique_id.h"

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

static const uint8_t BOARD_ID[8] = {0xE6, 0x61, 0x41, 0x04, 0x03, 0x2B, 0x9C, 0x27};

void pico_get_unique_board_id(pico_unique_board_id_t *id_out) {
    memcpy(id_out->id, BOARD_ID, 8);
}

#define TX_LOG_SIZE 4096
static struct can2040_msg tx_log[TX_LOG_SIZE];
static uint32_t tx_step[TX_LOG_SIZE];
static int tx_count = 0;
static int tx_fail_remaining = 0;

int can2040_transmit(struct can2040 *cd, const struct can2040_msg *msg) {
    (void)cd;
    if(tx_fail_remaining > 0) {
        tx_fail_remaining--;
        return -1;
    }
    assert_true(tx_count < TX_LOG_SIZE);
    tx_log[tx_count] = *msg;
    tx_step[tx_count] = stored_timestep;
    tx_count++;
    return 0;
}

static can2040_rx_cb rx_cb = NULL;

void can2040_callback_config(struct can2040 *cd, can2040_rx_cb cb) {
    (void)cd;
    rx_cb = cb;
}
void can2040_setup(struct can2040 *cd, uint32_t pio_num) { (void)cd; (void)pio_num; }
void can2040_start(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate, uint32_t gpio_rx, uint32_t gpio_tx) {
    (void)cd; (void)sys_clock; (void)bitrate; (void)gpio_rx; (void)gpio_tx;
}
void can2040_pio_irq_handler(struct can2040 *cd) { (void)cd; }

/* -------------------------------------------------------------- helpers -- */

#define PREFIX 0x0300
#define NODE 5

static uint32_t serial(void) {
    return custom_can_serial32(BOARD_ID);
}

static uint32_t announce_id(void) {
    return CUSTOM_CAN_ID_ANNOUNCE_BASE | (serial() & 0xFF);
}

static void clear_tx(void) {
    tx_count = 0;
}

static void tick(int n) {
    for(int i = 0; i < n; i++) {
        inverter_tick(&model.inverter_outputs);
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

static int count_type(uint8_t type) {
    return count_id(custom_can_make_id(PREFIX, type, NODE));
}

static const struct can2040_msg *last_id(uint32_t id29) {
    for(int i = tx_count - 1; i >= 0; i--) {
        if((tx_log[i].id & CUSTOM_CAN_ID_MASK) == id29) return &tx_log[i];
    }
    return NULL;
}

static const struct can2040_msg *last_type(uint8_t type) {
    return last_id(custom_can_make_id(PREFIX, type, NODE));
}

static void send_assign(uint32_t serial32, uint16_t prefix, uint8_t node, uint8_t flags) {
    uint8_t d[8];
    custom_can_put_u32(&d[0], serial32);
    custom_can_put_u16(&d[4], prefix);
    d[6] = node;
    d[7] = flags;
    rx(CUSTOM_CAN_ID_ASSIGN, d, 8);
}

static void send_command_to(uint8_t node, uint8_t desired, uint16_t seq) {
    uint8_t d[8] = {0};
    d[0] = desired;
    custom_can_put_u16(&d[2], seq);
    rx(custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_COMMAND, node), d, 8);
}

static void send_command(uint8_t desired, uint16_t seq) {
    send_command_to(NODE, desired, seq);
}

// Tick while behaving like a live controller (heartbeat every 500 ms)
static void tick_with_heartbeat(int n) {
    static uint16_t seq = 1000;
    for(int i = 0; i < n; i++) {
        if(i % 25 == 0) send_command(CUSTOM_CAN_DESIRED_NONE, seq++);
        tick(1);
    }
}

static void setup_model(void) {
    memset(&model, 0, sizeof(model));
    memset(bms_event_slots, 0, sizeof(bms_event_slot_t) * ERR_HIGHEST);
    stored_millis = 10000;
    stored_millis64 = 10000;
    stored_timestep = 500;

    model.nameplate_capacity_mC = NAMEPLATE_CAPACITY_AH * 3600 * 1000;
    model.system_sm.state = SYSTEM_STATE_INACTIVE;
    model.contactor_sm.state = CONTACTORS_STATE_OPEN;

    model.high_voltages.battery = 3.7f * NUM_CELLS;
    model.high_voltages.battery_millis = stored_millis;
    model.high_voltages.output = 3.7f * NUM_CELLS - 1.0f;
    model.high_voltages.output_millis = stored_millis;
    model.current_mA = -12345;
    model.current_millis = stored_millis;
    model.soc = 5500;
    model.soc_millis = stored_millis;
    for(int i = 0; i < NUM_CELLS; i++) model.cell_voltages_mV[i] = 3700;
    model.cell_voltages_millis = stored_millis;
    for(int i = 0; i < NUM_MODULE_TEMPS; i++) model.module_temperatures[i] = 25.0f;
    model.module_temperatures_millis = stored_millis;

    model_tick(&model);
}

static void setup_assigned(void) {
    setup_model();
    init_inverter();
    clear_tx();
    send_assign(serial(), PREFIX, NODE, 0);
    tick(1);
}

/* ---------------------------------------------------------------- tests -- */

static void test_unassigned_only_announces(void **state) {
    (void)state;
    setup_model();
    init_inverter();
    clear_tx();

    tick(150); // 3 s

    assert_true(tx_count >= 2 && tx_count <= 4);
    for(int i = 0; i < tx_count; i++) {
        assert_true(tx_log[i].id & CAN2040_ID_EFF);
        assert_int_equal(tx_log[i].id & CUSTOM_CAN_ID_MASK, announce_id());
        assert_int_equal(tx_log[i].dlc, 8);
        assert_int_equal(tx_log[i].data[0], (CUSTOM_CAN_PROTOCOL_VERSION << 4) | CUSTOM_CAN_LINK_UNASSIGNED);
        assert_int_equal(tx_log[i].data[1], CUSTOM_CAN_NODE_NONE);
        assert_int_equal(custom_can_get_u32(&tx_log[i].data[2]), serial());
        assert_int_equal(custom_can_get_u16(&tx_log[i].data[6]), 0);
    }
}

static void test_assign_validation(void **state) {
    (void)state;
    setup_model();
    init_inverter();
    clear_tx();

    send_assign(serial() + 1, PREFIX, NODE, 0);              // not our serial
    send_assign(serial(), CUSTOM_CAN_DISCOVERY_PREFIX, NODE, 0); // reserved prefix
    send_assign(serial(), 0x8000, NODE, 0);                  // prefix out of range
    send_assign(serial(), PREFIX, CUSTOM_CAN_NODE_ALL, 0);   // broadcast node
    tick(50);

    for(int i = 0; i < tx_count; i++) {
        assert_int_equal(tx_log[i].id & CUSTOM_CAN_ID_MASK, announce_id());
    }
}

static void test_assign_starts_telemetry(void **state) {
    (void)state;
    setup_assigned();

    // First tick after assignment: highest priority frames first
    assert_int_equal(tx_log[0].id & CUSTOM_CAN_ID_MASK, custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_STATUS, NODE));
    assert_int_equal(tx_log[1].id & CUSTOM_CAN_ID_MASK, custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_LIMITS, NODE));
    assert_int_equal(tx_log[2].id & CUSTOM_CAN_ID_MASK, custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_MEASUREMENTS, NODE));

    tick_with_heartbeat(300); // 6 s

    // Every defined message type shows up
    for(uint8_t type = CUSTOM_CAN_TYPE_STATUS; type <= CUSTOM_CAN_TYPE_CELL_LIMITS; type++) {
        assert_true(count_type(type) >= 1);
    }
    assert_true(count_type(CUSTOM_CAN_TYPE_MODULE_TEMPS) >= 1);
    assert_true(count_type(CUSTOM_CAN_TYPE_CELL_VOLTAGES) >= 1);

    // STATUS at ~5 Hz
    int status = count_type(CUSTOM_CAN_TYPE_STATUS);
    assert_true(status >= 28 && status <= 34);

    // Never more than the scheduler limit plus an announce in one tick
    int per_tick = 0;
    for(int i = 1; i < tx_count; i++) {
        per_tick = (tx_step[i] == tx_step[i - 1]) ? per_tick + 1 : 0;
        assert_true(per_tick < 4);
    }

    // Announce now reports the assignment
    const struct can2040_msg *ann = last_id(announce_id());
    assert_non_null(ann);
    assert_int_equal(ann->data[0], (CUSTOM_CAN_PROTOCOL_VERSION << 4) | CUSTOM_CAN_LINK_ASSIGNED);
    assert_int_equal(ann->data[1], NODE);
    assert_int_equal(custom_can_get_u16(&ann->data[6]), PREFIX);

    // Spot-check payloads
    const struct can2040_msg *cfg = last_type(CUSTOM_CAN_TYPE_CONFIG);
    assert_int_equal(cfg->data[0], CUSTOM_CAN_PROTOCOL_VERSION);
    assert_int_equal(cfg->data[1], CHEMISTRY);
    assert_int_equal(cfg->data[2], NUM_CELLS);
    assert_int_equal(cfg->data[3], NUM_MODULE_TEMPS);
    assert_int_equal(custom_can_get_u16(&cfg->data[4]), NAMEPLATE_CAPACITY_AH * 10);

    const struct can2040_msg *ser = last_type(CUSTOM_CAN_TYPE_SERIAL);
    assert_memory_equal(ser->data, BOARD_ID, 8);

    const struct can2040_msg *meas = last_type(CUSTOM_CAN_TYPE_MEASUREMENTS);
    assert_int_equal(custom_can_get_u16(&meas->data[0]), (uint16_t)(3.7f * NUM_CELLS * 100.0f + 0.5f));
    assert_int_equal(custom_can_get_i32(&meas->data[2]), -12345);

    const struct can2040_msg *lim = last_type(CUSTOM_CAN_TYPE_LIMITS);
    assert_int_equal(custom_can_get_i16(&lim->data[0]), model.inverter_outputs.max_voltage_limit_dV);
    assert_int_equal(custom_can_get_u16(&lim->data[6]), model.inverter_outputs.discharge_current_limit_dA);

    const struct can2040_msg *st = last_type(CUSTOM_CAN_TYPE_STATUS);
    assert_int_equal(st->data[0], SYSTEM_STATE_INACTIVE);
    assert_int_equal(st->data[1], CONTACTORS_STATE_OPEN);
    // Assignment raises the INFO-level INVERTER_DETECTED event
    assert_int_equal(st->data[3] & 0x07, LEVEL_INFO);
    assert_int_equal(st->data[6], ERR_INVERTER_DETECTED);
}

static void test_measurements_withheld_until_valid(void **state) {
    (void)state;
    setup_model();
    model.high_voltages.battery_millis = 0;
    model_tick(&model);
    init_inverter();
    clear_tx();
    send_assign(serial(), PREFIX, NODE, 0);
    tick(30);

    assert_int_equal(count_type(CUSTOM_CAN_TYPE_MEASUREMENTS), 0);
    assert_true(count_type(CUSTOM_CAN_TYPE_STATUS) > 0);
}

static void test_command_run_and_stop(void **state) {
    (void)state;
    setup_assigned();
    clear_tx();

    // RUN while inactive -> system request issued, STATUS acknowledges
    send_command(CUSTOM_CAN_DESIRED_RUN, 7);
    tick(1);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_RUN);
    const struct can2040_msg *st = last_type(CUSTOM_CAN_TYPE_STATUS);
    assert_non_null(st);
    assert_int_equal(custom_can_get_u16(&st->data[4]), 7);
    assert_int_equal((st->data[3] >> 4) & 0x03, CUSTOM_CAN_DESIRED_RUN);
    assert_true(st->data[2] & CUSTOM_CAN_STATUS_FLAG_COMMAND_PENDING);

    // System state machine acts
    model.system_req = SYSTEM_REQUEST_NULL;
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    clear_tx();
    tick(1);
    st = last_type(CUSTOM_CAN_TYPE_STATUS); // expedited by state change
    assert_non_null(st);
    assert_int_equal(st->data[0], SYSTEM_STATE_OPERATING);
    assert_false(st->data[2] & CUSTOM_CAN_STATUS_FLAG_COMMAND_PENDING);

    // Repeated RUN is a no-op
    send_command(CUSTOM_CAN_DESIRED_RUN, 8);
    tick(5);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_NULL);

    // Local operator stops the battery; controller still says RUN -> not fought
    model.system_sm.state = SYSTEM_STATE_INACTIVE;
    send_command(CUSTOM_CAN_DESIRED_RUN, 9);
    tick(5);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_NULL);

    // Controller toggles STOP -> RUN to re-assert
    send_command(CUSTOM_CAN_DESIRED_STOP, 10);
    tick(1);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_NULL); // already inactive: satisfied
    send_command(CUSTOM_CAN_DESIRED_RUN, 11);
    tick(1);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_RUN);
    model.system_req = SYSTEM_REQUEST_NULL;
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    tick(1);

    // STOP while operating
    send_command(CUSTOM_CAN_DESIRED_STOP, 12);
    tick(1);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_STOP);
    st = last_type(CUSTOM_CAN_TYPE_STATUS);
    assert_int_equal((st->data[3] >> 4) & 0x03, CUSTOM_CAN_DESIRED_STOP);
    assert_int_equal(custom_can_get_u16(&st->data[4]), 12);
}

static void test_command_pending_retries_and_fault(void **state) {
    (void)state;
    setup_assigned();

    // RUN cannot be applied while initialising; it is applied once inactive
    model.system_sm.state = SYSTEM_STATE_INITIALIZING;
    send_command(CUSTOM_CAN_DESIRED_RUN, 1);
    tick(10);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_NULL);
    model.system_sm.state = SYSTEM_STATE_INACTIVE;
    tick(1);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_RUN);

    // If the request is swallowed without effect, it is retried after a second
    model.system_req = SYSTEM_REQUEST_NULL;
    tick(40);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_NULL);
    tick(15);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_RUN);

    // In FAULT, STOP is trivially satisfied and RUN stays pending
    model.system_req = SYSTEM_REQUEST_NULL;
    model.system_sm.state = SYSTEM_STATE_FAULT;
    tick(100);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_NULL);
    const struct can2040_msg *st = last_type(CUSTOM_CAN_TYPE_STATUS);
    assert_true(st->data[2] & CUSTOM_CAN_STATUS_FLAG_COMMAND_PENDING);
    assert_int_equal(st->data[0], SYSTEM_STATE_FAULT);
}

static void test_broadcast_and_foreign_node(void **state) {
    (void)state;
    setup_assigned();
    clear_tx();

    send_command_to(NODE + 1, CUSTOM_CAN_DESIRED_RUN, 100);
    tick(10);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_NULL);
    const struct can2040_msg *st = last_type(CUSTOM_CAN_TYPE_STATUS);
    assert_non_null(st);
    assert_int_equal(custom_can_get_u16(&st->data[4]), 0);

    send_command_to(CUSTOM_CAN_NODE_ALL, CUSTOM_CAN_DESIRED_RUN, 101);
    tick(1);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_RUN);
    st = last_type(CUSTOM_CAN_TYPE_STATUS);
    assert_int_equal(custom_can_get_u16(&st->data[4]), 101);

    // Frames on another prefix are ignored entirely
    uint8_t d[8] = {CUSTOM_CAN_DESIRED_STOP, 0, 0, 55, 0, 0, 0, 0};
    rx(custom_can_make_id(PREFIX + 1, CUSTOM_CAN_TYPE_COMMAND, NODE), d, 8);
    tick(1);
    st = last_type(CUSTOM_CAN_TYPE_STATUS);
    assert_int_equal(custom_can_get_u16(&st->data[4]), 101);
}

static void test_controller_timeout(void **state) {
    (void)state;
    setup_assigned();
    model.system_sm.state = SYSTEM_STATE_OPERATING;

    // Heartbeats keep the link alive
    for(int i = 0; i < 10; i++) {
        send_command(CUSTOM_CAN_DESIRED_NONE, (uint16_t)i);
        tick(25); // 500 ms
    }
    clear_tx();
    tick(10);
    assert_true(count_type(CUSTOM_CAN_TYPE_STATUS) > 0);

    // Silence -> unassigned after the timeout, telemetry stops
    tick(CUSTOM_CAN_CONTROLLER_TIMEOUT_MS / TIMESTEP_PERIOD_MS + 10);
    clear_tx();
    tick(100);
    assert_int_equal(count_type(CUSTOM_CAN_TYPE_STATUS), 0);
    const struct can2040_msg *ann = last_id(announce_id());
    assert_non_null(ann);
    assert_int_equal(ann->data[0] & 0x0F, CUSTOM_CAN_LINK_UNASSIGNED);
    // Without the flag the battery keeps running
    assert_int_equal(model.system_req, SYSTEM_REQUEST_NULL);

    // Re-discovery works
    clear_tx();
    send_assign(serial(), PREFIX, NODE, 0);
    tick(1);
    assert_int_equal(count_type(CUSTOM_CAN_TYPE_STATUS), 1);
}

static void test_controller_timeout_stop_flag(void **state) {
    (void)state;
    setup_model();
    init_inverter();
    send_assign(serial(), PREFIX, NODE, CUSTOM_CAN_ASSIGN_FLAG_STOP_ON_TIMEOUT);
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    tick(1);

    tick(CUSTOM_CAN_CONTROLLER_TIMEOUT_MS / TIMESTEP_PERIOD_MS + 10);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_STOP);
}

static void test_release(void **state) {
    (void)state;
    setup_assigned();
    send_assign(serial(), PREFIX, CUSTOM_CAN_NODE_NONE, 0);
    clear_tx();
    tick(60);
    assert_int_equal(count_type(CUSTOM_CAN_TYPE_STATUS), 0);
    assert_true(count_id(announce_id()) >= 1);
}

static void test_reassignment_semantics(void **state) {
    (void)state;
    setup_assigned();

    send_command(CUSTOM_CAN_DESIRED_RUN, 1);
    tick(1);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_RUN);
    model.system_req = SYSTEM_REQUEST_NULL;
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    tick(1);
    model.system_sm.state = SYSTEM_STATE_INACTIVE; // local stop

    // Identical re-assignment (quick controller restart) keeps command tracking
    send_assign(serial(), PREFIX, NODE, 0);
    send_command(CUSTOM_CAN_DESIRED_RUN, 2);
    tick(5);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_NULL);

    // A different assignment is a fresh start: desired state is applied.
    // (The receive filter only accepts the new address once the ASSIGN has
    // been processed by the main loop, hence the tick in between.)
    send_assign(serial(), PREFIX, NODE + 1, 0);
    tick(1);
    uint8_t d[8] = {CUSTOM_CAN_DESIRED_RUN, 0, 0, 3, 0, 0, 0, 0};
    rx(custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_COMMAND, NODE + 1), d, 8);
    tick(1);
    assert_int_equal(model.system_req, SYSTEM_REQUEST_RUN);
    assert_true(count_id(custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_STATUS, NODE + 1)) > 0);
}

static void test_request(void **state) {
    (void)state;
    setup_assigned();
    tick(20); // let the initial burst drain
    clear_tx();

    uint8_t d[2] = {CUSTOM_CAN_TYPE_CONFIG, 0xFF};
    rx(custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_REQUEST, NODE), d, 2);
    tick(1);
    assert_int_equal(count_type(CUSTOM_CAN_TYPE_CONFIG), 1);

    // Specific page of a paged type
    uint8_t page = (NUM_CELLS - 1) / 3;
    uint8_t r[2] = {CUSTOM_CAN_TYPE_CELL_VOLTAGES, page};
    clear_tx();
    rx(custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_REQUEST, NODE), r, 2);
    tick(1);
    const struct can2040_msg *cv = last_type(CUSTOM_CAN_TYPE_CELL_VOLTAGES);
    assert_non_null(cv);
    assert_int_equal(cv->data[0], page * 3);
}

static void test_cell_voltage_pages(void **state) {
    (void)state;
    setup_model();
    for(int i = 0; i < NUM_CELLS; i++) model.cell_voltages_mV[i] = (int16_t)(3000 + i);
    model.cell_voltages_mV[2] = 0; // not read yet
    model.balancing_sm.balance_request_mask[4 / 32] |= 1u << (4 % 32);
    model.balancing_active = true;
    model_tick(&model);
    init_inverter();
    send_assign(serial(), PREFIX, NODE, 0);
    clear_tx();

    tick_with_heartbeat(500); // 10 s, comfortably more than one full cycle

    bool seen[128] = {false};
    for(int i = 0; i < tx_count; i++) {
        if((tx_log[i].id & CUSTOM_CAN_ID_MASK) != custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_CELL_VOLTAGES, NODE)) continue;
        const uint8_t *d = tx_log[i].data;
        int first = d[0];
        assert_true(d[1] & 0x80);
        for(int j = 0; j < 3; j++) {
            int idx = first + j;
            int16_t v = custom_can_get_i16(&d[2 + 2 * j]);
            if(idx >= NUM_CELLS) {
                assert_int_equal(v, CUSTOM_CAN_INT16_NA);
                continue;
            }
            seen[idx] = true;
            if(idx == 2) {
                assert_int_equal(v, CUSTOM_CAN_INT16_NA);
            } else {
                assert_int_equal(v, 3000 + idx);
            }
            bool balancing = (d[1] >> j) & 1;
            assert_int_equal(balancing, idx == 4);
        }
    }
    for(int i = 0; i < NUM_CELLS; i++) assert_true(seen[i]);

    const struct can2040_msg *stats = last_type(CUSTOM_CAN_TYPE_CELL_STATS);
    assert_non_null(stats);
    assert_int_equal(stats->data[7], NUM_CELLS - 1); // index of max cell

    const struct can2040_msg *bal = last_type(CUSTOM_CAN_TYPE_BALANCING);
    assert_non_null(bal);
    assert_int_equal(bal->data[1], 1);
    assert_true(bal->data[6] & 0x01);
}

static void test_module_temp_pages(void **state) {
    (void)state;
    setup_model();
    for(int i = 0; i < NUM_MODULE_TEMPS; i++) model.module_temperatures[i] = 20.0f + i;
    model_tick(&model);
    init_inverter();
    send_assign(serial(), PREFIX, NODE, 0);
    clear_tx();
    tick(150);

    bool seen[64] = {false};
    for(int i = 0; i < tx_count; i++) {
        if((tx_log[i].id & CUSTOM_CAN_ID_MASK) != custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_MODULE_TEMPS, NODE)) continue;
        const uint8_t *d = tx_log[i].data;
        assert_int_equal(d[1], NUM_MODULE_TEMPS);
        for(int j = 0; j < 3; j++) {
            int idx = d[0] + j;
            int16_t v = custom_can_get_i16(&d[2 + 2 * j]);
            if(idx >= NUM_MODULE_TEMPS) {
                assert_int_equal(v, CUSTOM_CAN_INT16_NA);
            } else {
                seen[idx] = true;
                assert_int_equal(v, (20 + idx) * 10);
            }
        }
    }
    for(int i = 0; i < NUM_MODULE_TEMPS; i++) assert_true(seen[i]);

    const struct can2040_msg *t = last_type(CUSTOM_CAN_TYPE_TEMPERATURES);
    assert_non_null(t);
    assert_int_equal(custom_can_get_i16(&t->data[0]), 200);
    assert_int_equal(custom_can_get_i16(&t->data[2]), (20 + NUM_MODULE_TEMPS - 1) * 10);
    assert_int_equal(t->data[4], 0);
    assert_int_equal(t->data[5], NUM_MODULE_TEMPS - 1);
}

static void test_events_reported(void **state) {
    (void)state;
    setup_assigned();
    raise_bms_event(ERR_CELL_VOLTAGE_HIGH, 0);
    clear_tx();
    tick(100);

    const struct can2040_msg *st = last_type(CUSTOM_CAN_TYPE_STATUS);
    assert_int_equal(st->data[3] & 0x07, LEVEL_WARNING);
    assert_int_equal(st->data[6], ERR_CELL_VOLTAGE_HIGH);

    bool found = false;
    for(int i = 0; i < tx_count; i++) {
        if((tx_log[i].id & CUSTOM_CAN_ID_MASK) != custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_EVENT, NODE)) continue;
        const uint8_t *d = tx_log[i].data;
        assert_int_equal(d[6], ERR_HIGHEST);
        if(d[0] == ERR_CELL_VOLTAGE_HIGH) {
            found = true;
            assert_int_equal(d[1], LEVEL_WARNING);
            assert_int_equal(custom_can_get_u16(&d[2]), 1);
        }
    }
    assert_true(found);
}

static void test_tx_queue_full_retries(void **state) {
    (void)state;
    setup_assigned();
    tick(20);
    clear_tx();

    tx_fail_remaining = 3; // everything fails this tick
    tick(10);              // 200 ms: STATUS/LIMITS/MEASUREMENTS due again at some point
    assert_int_equal(tx_fail_remaining, 0);
    // The frames blocked by a full queue were not lost, just delayed
    assert_true(count_type(CUSTOM_CAN_TYPE_STATUS) >= 1);
}

static void test_address_conflict(void **state) {
    (void)state;
    setup_assigned();

    // Another node transmits a battery-type frame with our address
    uint8_t d[8] = {0};
    rx(custom_can_make_id(PREFIX, CUSTOM_CAN_TYPE_STATUS, NODE), d, 8);
    tick(1);
    clear_tx();
    tick(60);
    assert_int_equal(count_type(CUSTOM_CAN_TYPE_STATUS), 0);
    const struct can2040_msg *ann = last_id(announce_id());
    assert_non_null(ann);
    assert_int_equal(ann->data[0] & 0x0F, CUSTOM_CAN_LINK_UNASSIGNED);
}

static void test_rate_shift(void **state) {
    (void)state;
    setup_model();
    init_inverter();
    send_assign(serial(), PREFIX, NODE, (uint8_t)(2 << CUSTOM_CAN_ASSIGN_RATE_SHIFT_POS)); // 4x slower
    tick(1);
    clear_tx();
    tick_with_heartbeat(500); // 10 s

    int status = count_type(CUSTOM_CAN_TYPE_STATUS);
    int energy = count_type(CUSTOM_CAN_TYPE_ENERGY);
    assert_true(status >= 45 && status <= 55); // essential: unchanged, 5 Hz
    assert_true(energy >= 2 && energy <= 3);   // 1 Hz / 4
}

static void test_id_helpers(void **state) {
    (void)state;
    uint32_t id = custom_can_make_id(0x7FFF, 0x3F, 0xFE);
    assert_int_equal(id, 0x1FFFFFFE);
    assert_int_equal(custom_can_id_prefix(id), 0x7FFF);
    assert_int_equal(custom_can_id_type(id), 0x3F);
    assert_int_equal(custom_can_id_node(id), 0xFE);
    assert_int_equal(custom_can_id_prefix(CUSTOM_CAN_ID_ANNOUNCE_BASE), CUSTOM_CAN_DISCOVERY_PREFIX);
    assert_int_equal(custom_can_id_type(CUSTOM_CAN_ID_ANNOUNCE_BASE), CUSTOM_CAN_TYPE_ANNOUNCE);
    assert_int_equal(custom_can_id_prefix(CUSTOM_CAN_ID_ASSIGN), CUSTOM_CAN_DISCOVERY_PREFIX);
    assert_int_equal(custom_can_id_type(CUSTOM_CAN_ID_ASSIGN), CUSTOM_CAN_TYPE_ASSIGN);
}

int main(void) {
    verbose = getenv("VERBOSE") != NULL;
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_id_helpers),
        cmocka_unit_test(test_unassigned_only_announces),
        cmocka_unit_test(test_assign_validation),
        cmocka_unit_test(test_assign_starts_telemetry),
        cmocka_unit_test(test_measurements_withheld_until_valid),
        cmocka_unit_test(test_command_run_and_stop),
        cmocka_unit_test(test_command_pending_retries_and_fault),
        cmocka_unit_test(test_broadcast_and_foreign_node),
        cmocka_unit_test(test_controller_timeout),
        cmocka_unit_test(test_controller_timeout_stop_flag),
        cmocka_unit_test(test_release),
        cmocka_unit_test(test_reassignment_semantics),
        cmocka_unit_test(test_request),
        cmocka_unit_test(test_cell_voltage_pages),
        cmocka_unit_test(test_module_temp_pages),
        cmocka_unit_test(test_events_reported),
        cmocka_unit_test(test_tx_queue_full_retries),
        cmocka_unit_test(test_address_conflict),
        cmocka_unit_test(test_rate_shift),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
