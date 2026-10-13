#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "app/model.h"
#include "app/estimators/ekf.h"
#include "protocols/inverter/inverter.h"
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

struct can2040_msg last_150;
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
    if(msg->id == 0x150) {
        last_150 = *msg;
    }
    if(tx_log_count < TX_LOG_SIZE) {
        tx_log[tx_log_count] = msg->id;
    }
    tx_log_count++;

    return 0;
}

void can2040_stop(struct can2040 *cd) { (void)cd; can2040_stop_count++; }
void can2040_get_statistics(struct can2040 *cd, struct can2040_stats *stats) {
    (void)cd;
    memset(stats, 0, sizeof(*stats));
}

// Current callback function
void (*can2040_cb)(struct can2040 *, uint32_t, struct can2040_msg *) = NULL;

void can2040_callback_config(struct can2040 *cd, void (*cb)(struct can2040 *, uint32_t, struct can2040_msg *)) { 
    (void)cd; 
    // Store callback so we can invoke it in tests
    can2040_cb = cb;
}

void can2040_setup(struct can2040 *cd, uint32_t pio_num) { (void)cd; (void)pio_num; }
void can2040_start(struct can2040 *cd, uint32_t sys_clock, uint32_t bitrate, uint32_t gpio_rx, uint32_t gpio_tx) { (void)cd; (void)sys_clock; (void)bitrate; (void)gpio_rx; (void)gpio_tx; }
void can2040_pio_irq_handler(struct can2040 *cd) { (void)cd; }


// Mock pico/stdlib.h functions
void gpio_init(uint32_t gpio) { (void)gpio; }
void gpio_set_dir(uint32_t gpio, bool out) { (void)gpio; (void)out; }
void gpio_set_function(uint32_t gpio, uint32_t fn) { (void)gpio; (void)fn; }
uint32_t stdio_getchar_timeout_us(uint32_t us) { (void)us; return 0xFF; }

// External model from model.c
extern bms_model_t model;

static void test_ekf_soc_scaling_midrange(void **state) {
    (void) state;

    // Reset model
    memset(&model, 0, sizeof(bms_model_t));
    model.nameplate_capacity_mC = 100 * 3600 * 1000; // 100Ah

    // Use upper half of the SoC range
    model.cell_voltage_working_min_mV = 3690; // ~50% SoC in NMC curve
    model.cell_voltage_working_max_mV = 4100; // ~90% SoC in NMC curve

    // Set voltage near the lower end
    uint32_t soc_out = ekf_tick(&model, 0, 0, 3750); 

    // Current EKF scaling puts this around ~12%
    assert_true(soc_out > 900);  // >9.00%
    assert_true(soc_out < 1600); // <16.00%
}

static void test_ekf_soc_scaling_top(void **state) {
    (void) state;
    
    // Reset model
    memset(&model, 0, sizeof(bms_model_t));
    model.nameplate_capacity_mC = 100 * 3600 * 1000; // 100Ah

    // Use lower half of the SoC range
    model.cell_voltage_working_min_mV = 3200; // ~10% SoC in NMC curve
    model.cell_voltage_working_max_mV = 3690; // ~50% SoC in NMC curve

    // Set voltage at the upper limit
    uint32_t soc_out = ekf_tick(&model, 0, 0, 3690);

    assert_true(soc_out == 10000); // 100.00%
}

void print_bms_events();

static void test_ekf_init_full_charge(void **state) {
    (void)state;

    memset(&model, 0, sizeof(bms_model_t));
    model.nameplate_capacity_mC = 100 * 3600 * 1000; // 100 Ah

    // Working range that reaches near 100% absolute SoC on the NMC curve.
    // working_max 4175 mV is the ceiling after soft-limit clamping (soft_max=4200, -25).
    model.cell_voltage_working_min_mV = 3300;
    model.cell_voltage_working_max_mV = 4175;

    // Stored: 0 Ah used = 100% SoC. The old code skipped this because of
    // the `!= 0.0f` guard; the new code uses it because diff vs voltage is ~2.6%.
    model.charge_used_Ah = 0.0f;

    // Voltage at 4150 mV/cell — 97.4% absolute SoC on NMC curve, inside working range.
    uint32_t soc_out = ekf_tick(&model, 0, 0, 4150);

    // With the fix: initialises from stored (100%), result clamped to 100.00% → 10000.
    // Without the fix: initialises from voltage (~97.4%), result ≈ 98.8% → ~9880.
    assert_true(soc_out > 9950);
}
 
static void test_ekf_init_uninitialized_falls_back_to_voltage(void **state) {
    (void)state;
 
    memset(&model, 0, sizeof(bms_model_t));
    model.nameplate_capacity_mC = 100 * 3600 * 1000; // 100 Ah
 
    // Wide working range covering 0-100%
    model.cell_voltage_working_min_mV = 3300;
    model.cell_voltage_working_max_mV = 4175;
 
    // Uninitialized struct: charge_used_Ah = 0.0f (stored would say 100%),
    // but voltage says ~50% NMC. Diff = ~50% >> 10% tolerance → must use voltage.
    model.charge_used_Ah = 0.0f;
 
    uint32_t soc_out = ekf_tick(&model, 0, 0, 3700); // ~50% NMC cell voltage
 
    // Must NOT latch to 100% from the uninitialized stored value
    assert_true(soc_out < 6000);
}
 
static void test_inverter_soc_scaling(void **state) {
    (void) state;
    
    // Reset model
    memset(&model, 0, sizeof(bms_model_t));

    stored_millis = 1000;
    model.soc_millis = stored_millis;
    
    // Initialize inverter

    init_inverter();

    struct can2040_msg msg = {0};
    msg.id = 0x151;
    msg.dlc = 8;
    can2040_cb(0, CAN2040_NOTIFY_RX, &msg);

    // Set up model so model_tick computes inverter_soc from soc_scaling
    model.system_sm.state = SYSTEM_STATE_OPERATING;
    model.contactor_sm.enable_current = true;
    model.soc_millis = stored_millis;
    model.high_voltages.battery_millis = stored_millis;
    model.cell_voltage_millis = stored_millis;
    model.cell_voltages_millis = stored_millis;
    model.temperature_millis = stored_millis;
    model.module_temperatures_millis = stored_millis;
    model.current_millis = stored_millis;
    model.temperature_min = 25.0f;
    model.temperature_max = 25.0f;
    for (int i = 0; i < NUM_CELLS; i++) {
        model.cell_voltages_mV[i] = 3700;
    }
    for (int i = 0; i < NUM_MODULE_TEMPS; i++) {
        model.module_temperatures_raw_dC[i] = 250;
        model.module_temperatures[i] = 25.0f;
    }
    model.high_voltages.battery = 3700.0f * NUM_CELLS * 0.001f;

    // Run init sequence to completion (one stage per tick)
    for (int i = 0; i < 10; i++) {
        inverter_tick(&model.inverter_outputs);
    }

    // Test 55% SoC with scaling 50% to 100%

    model.soc = 5500; // 50.00% absolute SoC
    model.soc_scaling_min = 5000;
    model.soc_scaling_max = 10000;

    model_tick(&model);

    last_150.id = 0;
    stored_timestep += 1000;
    inverter_tick(&model.inverter_outputs);

    // Expect 10% SoC reported
    assert_int_equal(last_150.id, 0x150);
    uint16_t sent_soc = (last_150.data[0] << 8) | last_150.data[1];
    assert_int_equal(sent_soc, 1000);

    // Test 75% SoC with scaling 50% to 75%

    model.soc = 7500; // 75.00% absolute SoC
    model.soc_scaling_min = 5000;
    model.soc_scaling_max = 7500;

    model_tick(&model);

    stored_timestep += 1000;
    inverter_tick(&model.inverter_outputs);

    // Expect 100% SoC reported
    assert_int_equal(last_150.id, 0x150);
    sent_soc = (last_150.data[0] << 8) | last_150.data[1];
    assert_int_equal(sent_soc, 10000);
}

// Advance one main-loop tick (20 ms) and run the inverter
static void inverter_step(bool ack) {
    stored_millis += 20;
    stored_millis64 += 20;
    stored_timestep += 1;
    inverter_tick(&model.inverter_outputs);
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

    memset(&model, 0, sizeof(bms_model_t));
    stored_millis = 100000;
    stored_millis64 = 100000;
    stored_timestep = 5000;

    model.system_sm.state = SYSTEM_STATE_OPERATING;
    model.contactor_sm.enable_current = true;
    model.soc = 5000;
    model.soc_scaling_min = 0;
    model.soc_scaling_max = 10000;
    model.soc_millis = stored_millis;
    model.high_voltages.battery_millis = stored_millis;
    model.cell_voltage_millis = stored_millis;
    model.cell_voltages_millis = stored_millis;
    model.temperature_millis = stored_millis;
    model.module_temperatures_millis = stored_millis;
    model.current_millis = stored_millis;
    model.temperature_min = 25.0f;
    model.temperature_max = 25.0f;
    for (int i = 0; i < NUM_CELLS; i++) {
        model.cell_voltages_mV[i] = 3700;
    }
    model.high_voltages.battery = 3700.0f * NUM_CELLS * 0.001f;
    model_tick(&model);

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

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_ekf_soc_scaling_midrange),
        cmocka_unit_test(test_ekf_soc_scaling_top),
        cmocka_unit_test(test_ekf_init_full_charge),
        cmocka_unit_test(test_ekf_init_uninitialized_falls_back_to_voltage),
        cmocka_unit_test(test_inverter_soc_scaling),
        cmocka_unit_test(test_inverter_restart_reidentifies),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
