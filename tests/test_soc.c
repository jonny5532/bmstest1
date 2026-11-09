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
#include "app/estimators/estimators.h"
#include "app/estimators/ocv_learn.h"
#include "protocols/inverter/inverter.h"
#include "can2040.h"
#include "sys/logging/logging.h"

// Everything logged since the test last cleared it, for asserting on decisions
static char log_text[4096];
static size_t log_len;

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    if (log_len < sizeof(log_text) - 1) {
        va_start(args, format);
        int n = vsnprintf(log_text + log_len, sizeof(log_text) - log_len, format, args);
        va_end(args);
        if (n > 0) log_len += (size_t)n < sizeof(log_text) - log_len ? (size_t)n : sizeof(log_text) - 1 - log_len;
    }
}

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;
uint32_t stored_timestep = 0;

struct can2040_msg last_150;

int can2040_transmit(struct can2040 *cd, const struct can2040_msg *msg) {
    (void)cd;
    if(msg->id == 0x150) {
        last_150 = *msg;
    }

    return 0;
}

void can2040_stop(struct can2040 *cd) { (void)cd; }
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
    model.actual_capacity_mC = 100 * 3600 * 1000; // 100Ah

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
    model.actual_capacity_mC = 100 * 3600 * 1000; // 100Ah

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
    model.actual_capacity_mC = 100 * 3600 * 1000; // 100 Ah

    // Working range that reaches near 100% absolute SoC on the NMC curve.
    // working_max 4175 mV is the ceiling after soft-limit clamping (soft_max=4200, -25).
    model.cell_voltage_working_min_mV = 3300;
    model.cell_voltage_working_max_mV = 4175;

    // Stored: 0 Ah used = 100% SoC. The old code skipped this because of
    // the `!= 0.0f` guard; the new code uses it because diff vs voltage is ~2.6%.
    model.charge_used_Ah = 0.0f;

    // Voltage at 4150 mV/cell — 97.4% absolute SoC on NMC curve, inside working range.
    log_len = 0;
    log_text[0] = '\0';
    uint32_t soc_out = ekf_tick(&model, 0, 0, 4150);

    // With the fix it initialises from the stored 100%; without it, from voltage.
    // The first update then weighs that start against the voltage by P0_AH, so
    // check the decision itself rather than the filtered output.
    assert_non_null(strstr(log_text, "from NVM"));
    assert_true(soc_out > 9500);
}
 
static void test_ekf_init_uninitialized_falls_back_to_voltage(void **state) {
    (void)state;
 
    memset(&model, 0, sizeof(bms_model_t));
    model.actual_capacity_mC = 100 * 3600 * 1000; // 100 Ah
 
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

static void test_ekf_pinned_at_full_charge(void **state) {
    (void)state;

    memset(&model, 0, sizeof(bms_model_t));
    model.actual_capacity_mC = 100 * 3600 * 1000; // 100 Ah
    model.cell_voltage_working_min_mV = 3300;
    model.cell_voltage_working_max_mV = 4100;
    model.charge_used_Ah = 12.0f; // a stale count, close enough to voltage to be used

    // Held at the charge voltage with no current: after 10 minutes it counts as
    // full, and the filter is set there rather than reading the held voltage as OCV
    uint32_t soc_out = 0;
    for (int i = 0; i < 300; i++) soc_out = ekf_tick(&model, 0, 0, 4100);
    assert_true(model.ekf.P[0][0] != 1.0);
    for (int i = 0; i < 400; i++) soc_out = ekf_tick(&model, 0, 0, 4100);
    assert_float_equal(model.ekf.P[0][0], 1.0, 1e-9);
    assert_float_equal(model.ekf.x[0], (1.0 - model.ocv_learn.persistent.s_full) * 100.0, 1e-6);
    assert_int_equal(soc_out, 10000);
}

static void test_actual_capacity_default_and_saved(void **state) {
    (void)state;

    // Nothing saved (as on older NVM): the profile's
    memset(&model, 0, sizeof(bms_model_t));
    assert_int_equal(get_actual_capacity_mC(&model), (uint32_t)(ACTUAL_CAPACITY_AH * 3600000.0 + 0.5));

    model.actual_capacity_mC = 150 * 3600 * 1000;
    assert_int_equal(get_actual_capacity_mC(&model), 150u * 3600 * 1000);
}

static void test_ekf_set_capacity_keeps_ah_used(void **state) {
    (void)state;

    // Voltages from the factory table, so this holds for either chemistry
    const float *ocv = ekf_factory_ocv_table();
    memset(&model, 0, sizeof(bms_model_t));
    model.actual_capacity_mC = 100 * 3600 * 1000; // 100 Ah
    model.cell_voltage_working_min_mV = (uint16_t)lroundf(ocv[5] * 1000.0f);
    model.cell_voltage_working_max_mV = (uint16_t)lroundf(ocv[95] * 1000.0f);
    ekf_tick(&model, 0, 0, (int32_t)lroundf(ocv[30] * 1000.0f)); // starts from the voltage, at ~30%
    double ah_used = model.ekf.x[0];
    uint32_t working_mC = model.working_capacity_mC;
    assert_true(ah_used > 10.0 && working_mC > 0);

    // The Ah used since full was measured, so it stays and the SOC follows the new capacity
    ekf_set_capacity(&model, 80 * 3600 * 1000);
    assert_int_equal(model.actual_capacity_mC, 80u * 3600 * 1000);
    assert_float_equal(model.ekf.cap, 80.0, 1e-9);
    assert_float_equal(model.ekf.x[0], ah_used, 1e-9);
    assert_float_equal(ekf_get_soc(&model.ekf), 1.0 - ah_used / 80.0, 1e-9);
    ekf_tick(&model, 0, 0, 3750);
    assert_float_equal((double)model.working_capacity_mC, working_mC * 0.8, working_mC * 1e-4);

    // Zero goes back to the profile's
    ekf_set_capacity(&model, 0);
    assert_int_equal(model.actual_capacity_mC, 0);
    assert_float_equal(model.ekf.cap, ACTUAL_CAPACITY_AH, 1e-6);
}

// Pack whose cells rest 30mV above a linear factory table, cycled from full to
// 48.75Ah below (between bins 6 and 7, 7.5Ah apart) and back with a 6h rest there
static ocv_learn_t learn;
static float learn_factory[101];

static bool learn_run(int seconds, float current_A, float voltage_V) {
    bool changed = false;
    for (int i = 0; i < seconds; i++) {
        changed |= ocv_learn_tick(&learn, learn_factory, 100.0, 4.08f, current_A / 3600.0, current_A, voltage_V);
    }
    return changed;
}

static void learn_cycle(void) {
    assert_false(learn_run(1200, 0.0f, 4.08f));             // settled full: the anchor
    learn_run(7020, -25.0f, 3.5f);                            // discharge, too much current to read
    learn_run(6 * 3600, 0.0f, 3.0f + 1.2f * 0.4125f + 0.03f); // rest at 41.25% SOC
    learn_run(7020, 25.0f, 3.9f);                             // charge back
}

static void test_ocv_learn_cycles(void **state) {
    (void)state;

    memset(&learn, 0, sizeof(learn));
    for (int i = 0; i <= 100; i++) learn_factory[i] = 3.0f + 1.2f * i / 100.0f;
    // Charge voltage 4.08V puts full at 90% on the factory curve

    learn_cycle();
    assert_true(learn_run(1200, 0.0f, 4.08f)); // next full charge: the rest is learned
    assert_true(learn.persistent.corr[6] > 0.02 && learn.persistent.corr[6] < 0.04);
    assert_true(learn.persistent.corr[7] > 0.02 && learn.persistent.corr[7] < 0.04);
    assert_float_equal(learn.table[41], 3.0f + 1.2f * 0.41f + 0.03f, 0.01f);
    assert_true(isnan(learn.capacity_ref_Ah)); // one cycle isn't enough to trust
    assert_int_equal(learn.capacity_state, OCV_LEARN_CAPACITY_NO_DATA);
    assert_true(isnan(learn.capacity_Ah));

    learn_cycle();
    learn_run(1200, 0.0f, 4.08f);
    // Full (90%) down to the 3.50V reference: 3.0 + 1.2 * (0.9 - a / 100) + 0.03 = 3.5 at a = 50.8Ah
    assert_float_equal(learn.capacity_ref_Ah, 50.8f, 1.5f);

    // Bins 6 and 7 (45 and 52.5Ah below full) read 30mV high, which the factory curve
    // puts 2.5% shallower: 45 / 0.425 = 105.9Ah and 52.5 / 0.5 = 105.0Ah. Half way
    // down, the curve's error alone is ~5% (20mV at the bin and at full, over 1.2V of
    // curve, divided by the depth), so it's preliminary.
    assert_int_equal(learn.capacity_state, OCV_LEARN_CAPACITY_PRELIMINARY);
    assert_float_equal(learn.capacity_Ah, 105.4f, 1.5f);
    assert_true(learn.capacity_sd_Ah > 0.04f * learn.capacity_Ah && learn.capacity_sd_Ah < 0.08f * learn.capacity_Ah);
    assert_float_equal(learn.capacity_depth, 0.5f, 0.02f);                       // bin 7
    assert_float_equal(learn.capacity_depth_V, 3.0f + 1.2f * 0.375f + 0.03f, 0.01f);
    assert_true(learn.capacity_depth_cycles >= 1.5f);
}

static void test_ocv_learn_reanchors_when_charge_voltage_moves(void **state) {
    (void)state;

    memset(&learn, 0, sizeof(learn));
    for (int i = 0; i <= 100; i++) learn_factory[i] = 3.0f + 1.2f * i / 100.0f;
    learn_cycle();
    learn_run(1200, 0.0f, 4.08f);
    learn_cycle();
    learn_run(1200, 0.0f, 4.08f);
    assert_int_equal(learn.capacity_state, OCV_LEARN_CAPACITY_PRELIMINARY);
    double corr6 = learn.persistent.corr[6];

    // A few mV (0.4% of the curve) isn't a move
    assert_false(ocv_learn_tick(&learn, learn_factory, 100.0, 4.085f, 0.0, 0.0f, 4.0f));
    assert_float_equal(learn.persistent.s_full, 0.9f, 1e-6f);

    // A lower working max: full is re-anchored at 85%, and the bins earn their trust again
    assert_true(ocv_learn_tick(&learn, learn_factory, 100.0, 4.02f, 0.0, 0.0f, 4.0f));
    assert_float_equal(learn.persistent.s_full, 0.85f, 1e-4f);
    for (int i = 0; i < OCV_LEARN_BINS; i++) assert_true(learn.persistent.cycles[i] == 0.0f);
    assert_false(learn.have_anchor);
    assert_int_equal(learn.capacity_state, OCV_LEARN_CAPACITY_NO_DATA);
    assert_true(isnan(learn.capacity_Ah));
    assert_float_equal(learn.persistent.corr[6], corr6, 1e-12); // kept, see ocv_learn_tick
}

// All cells at the same voltage, with this current
static uint16_t voltage_soc_at(int32_t cell_mV, int32_t current_mA) {
    model.cell_voltage_min_mV = cell_mV;
    model.cell_voltage_max_mV = cell_mV;
    model.cell_voltage_total_mV = cell_mV * NUM_CELLS;
    model.current_mA = current_mA;
    return voltage_based_soc_estimate(&model);
}

static void test_voltage_soc_scaled_to_working_limits(void **state) {
    (void)state;

    // Working limits at 10% and 90% on the factory table
    const float *ocv = ekf_factory_ocv_table();
    memset(&model, 0, sizeof(bms_model_t));
    int32_t min_mV = (int32_t)lroundf(ocv[10] * 1000.0f);
    int32_t max_mV = (int32_t)lroundf(ocv[90] * 1000.0f);
    model.cell_voltage_working_min_mV = min_mV;
    model.cell_voltage_working_max_mV = max_mV;
    model.cell_voltage_soft_min_mV = min_mV - 100;
    model.cell_voltage_soft_max_mV = max_mV + 100;

    assert_int_equal(voltage_soc_at(min_mV, 0), 0);
    assert_int_equal(voltage_soc_at(max_mV, 0), 10000);
    assert_int_equal(voltage_soc_at(min_mV - 50, 0), 0);
    assert_int_equal(voltage_soc_at(max_mV + 50, 0), 10000);

    // Halfway on the table is halfway between the limits (to the table's mV)
    int32_t mid_mV = (int32_t)lroundf(ocv[50] * 1000.0f);
    float s = table_soc(ocv, mid_mV / 1000.0f), s_min = table_soc(ocv, min_mV / 1000.0f),
          s_max = table_soc(ocv, max_mV / 1000.0f);
    uint16_t mid = voltage_soc_at(mid_mV, 0);
    assert_int_equal(mid, (uint16_t)((s - s_min) / (s_max - s_min) * 10000.0f + 0.5f));
    assert_true(mid > 4500 && mid < 5500);

    // Charging lifts the terminal voltage by I * R, discharging drops it: either
    // way the SOC is the same as at rest
    int32_t ir_mV = (int32_t)lround(50000 * EKF_R_OHM); // 50A
    assert_int_equal(voltage_soc_at(mid_mV + ir_mV, 50000), mid);
    assert_int_equal(voltage_soc_at(mid_mV - ir_mV, -50000), mid);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_ekf_soc_scaling_midrange),
        cmocka_unit_test(test_ekf_soc_scaling_top),
        cmocka_unit_test(test_ekf_init_full_charge),
        cmocka_unit_test(test_ekf_init_uninitialized_falls_back_to_voltage),
        cmocka_unit_test(test_inverter_soc_scaling),
        cmocka_unit_test(test_ocv_learn_cycles),
        cmocka_unit_test(test_ekf_pinned_at_full_charge),
        cmocka_unit_test(test_actual_capacity_default_and_saved),
        cmocka_unit_test(test_ekf_set_capacity_keeps_ah_used),
        cmocka_unit_test(test_ocv_learn_reanchors_when_charge_voltage_moves),
        cmocka_unit_test(test_voltage_soc_scaled_to_working_limits),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
