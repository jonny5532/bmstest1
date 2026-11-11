#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/model.h"
#include "protocols/inverter/inverter.h"
#include "protocols/inverter/can.h"
#include "can2040.h"
#include "sys/logging/logging.h"

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;
uint32_t stored_timestep = 0;

// Record of every transmitted CAN id, in order
#define TX_LOG_SIZE 64
uint32_t tx_log[TX_LOG_SIZE];
int tx_log_count = 0;
int can2040_stop_count = 0;

static void tx_log_reset(void) {
    tx_log_count = 0;
}

int can2040_transmit(struct can2040 *cd, const struct can2040_msg *msg) {
    (void)cd;
    if(tx_log_count < TX_LOG_SIZE) {
        tx_log[tx_log_count] = msg->id;
    }
    tx_log_count++;

    return 0;
}

void can2040_stop(struct can2040 *cd) { (void)cd; can2040_stop_count++; }
// What can2040 reports; cleared by can2040_setup, as the real one's are
static struct can2040_stats mock_stats;
void can2040_get_statistics(struct can2040 *cd, struct can2040_stats *stats) {
    (void)cd;
    *stats = mock_stats;
}

// Current callback function
void (*can2040_cb)(struct can2040 *, uint32_t, struct can2040_msg *) = NULL;

void can2040_callback_config(struct can2040 *cd, void (*cb)(struct can2040 *, uint32_t, struct can2040_msg *)) {
    (void)cd;
    // Store callback so we can invoke it in tests
    can2040_cb = cb;
}

void can2040_setup(struct can2040 *cd, uint32_t pio_num) {
    (void)cd; (void)pio_num;
    memset(&mock_stats, 0, sizeof(mock_stats));
}
void can2040_start(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate, uint32_t gpio_rx, uint32_t gpio_tx) { (void)cd; (void)sys_clock; (void)bitrate; (void)gpio_rx; (void)gpio_tx; }
void can2040_pio_irq_handler(struct can2040 *cd) { (void)cd; }

// Mock pico/stdlib.h functions
void gpio_init(uint32_t gpio) { (void)gpio; }
void gpio_set_dir(uint32_t gpio, bool out) { (void)gpio; (void)out; }
void gpio_set_function(uint32_t gpio, uint32_t fn) { (void)gpio; (void)fn; }

// What the model would hand the inverter: valid and fresh, so every regular
// frame is sent
static inverter_outputs_t outputs;

static void set_valid_outputs(void) {
    memset(&outputs, 0, sizeof(outputs));
    outputs.soc = 5000;
    outputs.soc_millis = stored_millis;
    outputs.battery_voltage = 3.7f * NUM_CELLS;
    outputs.battery_voltage_millis = stored_millis;
    outputs.current_millis = stored_millis;
    outputs.temperature_min = 25.0f;
    outputs.temperature_max = 25.0f;
    outputs.temperature_millis = stored_millis;
}

// Advance one main-loop tick (20 ms) and run the inverter
static void inverter_step(bool ack) {
    stored_millis += 20;
    stored_millis64 += 20;
    stored_timestep += 1;
    inverter_tick(&outputs);
    if (ack) {
        struct can2040_msg ack_msg = {0};
        can2040_cb(0, CAN2040_NOTIFY_TX, &ack_msg);
    }
}

static void inverter_rx(uint32_t id, uint8_t byte0) {
    struct can2040_msg msg = {0};
    msg.id = id;
    msg.dlc = 8;
    msg.data[0] = byte0;
    can2040_cb(0, CAN2040_NOTIFY_RX, &msg);
}

static const uint32_t IDENT_SEQUENCE[] = {0x250, 0x290, 0x2D0, 0x3D0, 0x3D0, 0x3D0, 0x3D0};

static void test_inverter_restart_reidentifies(void **state) {
    (void) state;

    stored_millis = 100000;
    stored_millis64 = 100000;
    stored_timestep = 5000;
    set_valid_outputs();

    init_inverter();

    // The inverter driver keeps static state across tests; clear any pending
    // transmit left over from a previous test by acknowledging it.
    {
        struct can2040_msg ack_msg = {0};
        can2040_cb(0, CAN2040_NOTIFY_TX, &ack_msg);
    }
    can2040_stop_count = 0;

    // Inverter is up and asks us to identify; we should reply and then run
    inverter_rx(0x151, 0x01);
    tx_log_reset();
    for (int i = 0; i < 10; i++) inverter_step(true);
    assert_true(tx_log_count >= 7);
    for (int i = 0; i < 7; i++) {
        assert_int_equal(tx_log[i], IDENT_SEQUENCE[i]);
    }

    // Steady state: regular frames flowing, all acknowledged
    tx_log_reset();
    for (int i = 0; i < 50; i++) inverter_step(true);
    assert_true(tx_log_count > 0);
    assert_int_equal(can2040_stop_count, 0);

    // Inverter goes down: nothing acknowledges our frames any more. Within a
    // few seconds we must notice, reset the CAN stack and go quiet.
    for (int i = 0; i < 150; i++) inverter_step(false); // 3 s
    assert_int_equal(can2040_stop_count, 1);

    tx_log_reset();
    for (int i = 0; i < 100; i++) inverter_step(false); // 2 s of silence
    assert_int_equal(tx_log_count, 0);

    // Inverter comes back and speaks. The very next frames out of us must be
    // the identification sequence, in order, before any regular frame.
    inverter_rx(0x151, 0x01);
    tx_log_reset();
    inverter_step(true);
    assert_int_equal(tx_log_count, 7);
    for (int i = 0; i < 7; i++) {
        assert_int_equal(tx_log[i], IDENT_SEQUENCE[i]);
    }

    // ...and then regular frames resume
    tx_log_reset();
    for (int i = 0; i < 10; i++) inverter_step(true);
    assert_true(tx_log_count > 0);
    assert_int_equal(tx_log[0], 0x110);
    // 0x190 goes out with the first 0x110 rather than up to 60 s later: the
    // Deye only counts the battery as identified once it has seen it
    assert_true(tx_log_count >= 2);
    assert_int_equal(tx_log[1], 0x190);

    // RX silence path: frames are acknowledged (someone is on the bus) but the
    // inverter says nothing for longer than the RX timeout.
    for (int i = 0; i < 520; i++) inverter_step(true); // 10.4 s
    assert_int_equal(can2040_stop_count, 2);
    tx_log_reset();
    for (int i = 0; i < 50; i++) inverter_step(true);
    assert_int_equal(tx_log_count, 0);

    // A regular inverter frame (not a reinit request) also triggers
    // identification after a loss, same as a BMS cold start.
    inverter_rx(0x91, 0x00);
    tx_log_reset();
    inverter_step(true);
    assert_int_equal(tx_log_count, 7);
    assert_int_equal(tx_log[0], 0x250);
}

// can2040's statistics start again from zero when the bus is restarted; the
// frame counts the HMI reads must not
static void test_frame_counts_carry_across_a_bus_restart(void **state) {
    (void)state;

    uint32_t sent_before = inverter_can_frames_sent();
    uint32_t received_before = inverter_can_frames_received();

    mock_stats.tx_total = 5;
    mock_stats.rx_total = 3;
    assert_int_equal(inverter_can_frames_sent() - sent_before, 5);
    assert_int_equal(inverter_can_frames_received() - received_before, 3);

    inverter_can_reset();
    assert_int_equal(mock_stats.tx_total, 0);
    assert_int_equal(inverter_can_frames_sent() - sent_before, 5);
    assert_int_equal(inverter_can_frames_received() - received_before, 3);

    mock_stats.tx_total = 2;
    mock_stats.rx_total = 1;
    assert_int_equal(inverter_can_frames_sent() - sent_before, 7);
    assert_int_equal(inverter_can_frames_received() - received_before, 4);

    inverter_can_reset();
    assert_int_equal(inverter_can_frames_sent() - sent_before, 7);
    assert_int_equal(inverter_can_frames_received() - received_before, 4);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_inverter_restart_reidentifies),
        cmocka_unit_test(test_frame_counts_carry_across_a_bus_restart),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
