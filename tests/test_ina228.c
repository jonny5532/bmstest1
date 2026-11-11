#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app/model.h"
#include "config/allocations.h"
#include "drivers/chip/i2c.h"
#include "drivers/sensors/ina228.h"
#include "sys/time/time.h"

// Coulomb counting through ina228_read_current_async, against a simulated
// INA228 on a 1ms timeline: continuous conversions on the chip's own clock,
// interrupted by the driver's temperature measurements, polled by the 160ms
// timer over a fake I2C bus, and picked up by a 20ms main loop.

// Mock globals
millis64_t stored_millis64 = 0;
millis_t stored_millis = 0;
uint32_t stored_timestep = 0;

bms_model_t model;

// Simulated time. time_us_32 wraps during the tests, as it does every ~71 minutes.
static uint64_t now_us = 4194000000ull; // ~101s before the wrap

uint32_t time_us_32(void) { return (uint32_t)now_us; }
void sleep_ms(uint32_t ms) { (void)ms; }

#include "sys/logging/logging.h"
// Everything logged since the test last cleared it
static char log_text[4096];
static size_t log_len;

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    if (log_len < sizeof(log_text) - 1) {
        va_list args;
        va_start(args, format);
        int n = vsnprintf(log_text + log_len, sizeof(log_text) - log_len, format, args);
        va_end(args);
        if (n > 0) log_len += (size_t)n < sizeof(log_text) - log_len ? (size_t)n : sizeof(log_text) - 1 - log_len;
    }
}

// ============================================================================
// Current through the shunt, in CURRENT register LSBs (0.25mA)
// ============================================================================

static int32_t true_current_raw = 0;
// Exact integral of true_current_raw over time, in raw x us (charge_raw units)
static int64_t true_charge_raw = 0;

// ============================================================================
// Simulated INA228
// ============================================================================

// Conversion period on the chip's own oscillator: 0.12% short of the nominal
// 530.944ms, as measured against the RP2350's crystal on a real board
#define INA_PERIOD_US 530307
// The driver's offset correction at 25C (0.5uV at the die temperature it
// assumes, and that the fake reports); the CURRENT register reads this high
#define INA_VOS_COUNTS 2
#define INA_DIETEMP_25C 3200 // 7.8125 m degrees C per LSB
#define INA_VBUS_NTC 13824   // ~2.7V on the NTC divider

// Which chip is fitted. An INA237 or INA238 has 16-bit CURRENT and VBUS
// registers with steps 16 times the INA228's, and a 12-bit DIETEMP.
static uint16_t ina_chip = INA_CHIP_INA228;
static uint16_t ina_device_id = 0x2281;
static bool ina_16bit(void) {
    return ina_chip == INA_CHIP_INA237 || ina_chip == INA_CHIP_INA238;
}
// How many bytes the driver last read from each result register
static size_t current_read_len, vbus_read_len;

static struct {
    bool continuous;
    uint64_t conv_start_us;      // start of the current conversion (continuous)
    int64_t conv_charge;         // integral over the current conversion so far
    int32_t current_reg;         // average over the last completed conversion
    bool cnvrf;                  // conversion ready flag, cleared by reading DIAG_ALRT
    bool triggered;              // a triggered (temperature) conversion is running
    uint64_t trig_done_us;
    int triggered_count;
} ina;

static void ina_write_adc_config(uint16_t value) {
    uint16_t mode = value >> 12;
    if (mode == 0xA) {
        // Continuous shunt conversions, starting now
        ina.continuous = true;
        ina.triggered = false;
        ina.conv_start_us = now_us;
        ina.conv_charge = 0;
    } else {
        // Triggered single shot: abandons the conversion in progress
        ina.continuous = false;
        ina.triggered = true;
        ina.trig_done_us = now_us + 132700;
        ina.triggered_count++;
    }
}

// Advance the chip over the last us microseconds (now_us already moved on)
static void ina_advance(uint32_t us) {
    if (ina.continuous) {
        ina.conv_charge += (int64_t)true_current_raw * us;
        uint64_t conv_end_us = ina.conv_start_us + INA_PERIOD_US;
        if (now_us >= conv_end_us) {
            // Split the step at the end of the conversion
            int64_t overshoot = (int64_t)true_current_raw * (int64_t)(now_us - conv_end_us);
            // The driver only corrects an INA228's offset
            ina.current_reg = (int32_t)((ina.conv_charge - overshoot) / INA_PERIOD_US)
                + (ina_chip == INA_CHIP_INA228 ? INA_VOS_COUNTS : 0);
            ina.cnvrf = true;
            ina.conv_start_us = conv_end_us;
            ina.conv_charge = overshoot;
        }
    }
    if (ina.triggered && now_us >= ina.trig_done_us) {
        // A triggered conversion sets the flag too
        ina.triggered = false;
        ina.cnvrf = true;
    }
}

static void put_be(uint8_t *buf, uint32_t value, size_t len) {
    for (size_t i = 0; i < len; i++) {
        buf[i] = (uint8_t)(value >> (8 * (len - 1 - i)));
    }
}

static void ina_read_reg(uint8_t reg, uint8_t *buf, size_t len) {
    switch (reg) {
        case INA228_REG_DIAG_ALRT:
            put_be(buf, ina.cnvrf ? 0x0002 : 0, len);
            ina.cnvrf = false;
            break;
        case INA228_REG_CURRENT:
            current_read_len = len;
            if (ina_16bit()) {
                // 4mA steps, rounded down
                put_be(buf, (uint32_t)(ina.current_reg >> 4) & 0xFFFF, len);
            } else {
                put_be(buf, ((uint32_t)ina.current_reg & 0xFFFFF) << 4, len);
            }
            break;
        case INA228_REG_DIETEMP:
            // The 16-bit chips' low four bits are reserved; the driver must ignore them
            put_be(buf, ina_16bit() ? (INA_DIETEMP_25C | 0x5) : INA_DIETEMP_25C, len);
            break;
        case INA228_REG_VBUS:
            vbus_read_len = len;
            if (ina_16bit()) {
                put_be(buf, INA_VBUS_NTC >> 4, len); // 3.125mV steps
            } else {
                put_be(buf, (uint32_t)INA_VBUS_NTC << 4, len);
            }
            break;
        case INA228_REG_MANUFACTURER_ID:
            put_be(buf, 0x5449, len);
            break;
        case INA228_REG_DEVICE_ID:
            put_be(buf, ina_device_id, len);
            break;
        default:
            put_be(buf, 0, len);
            break;
    }
}

// ============================================================================
// Fake I2C driver: one transaction at a time, completing on the next 1ms step
// ============================================================================

static struct {
    bool pending;
    i2c_async_callback_t callback;
    void *user_data;
} fake_i2c;

static bool fake_i2c_start(i2c_async_callback_t callback, void *user_data) {
    if (fake_i2c.pending) {
        return false;
    }
    fake_i2c.pending = true;
    fake_i2c.callback = callback;
    fake_i2c.user_data = user_data;
    return true;
}

void i2c_async_init(i2c_inst_t *i2c, uint baudrate, uint sda_pin, uint scl_pin) {
    (void)i2c; (void)baudrate; (void)sda_pin; (void)scl_pin;
}

bool i2c_async_is_busy(i2c_inst_t *i2c) {
    (void)i2c;
    return fake_i2c.pending;
}

bool i2c_blocking_read_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buffer, size_t len, uint32_t timeout_us) {
    (void)i2c; (void)addr; (void)timeout_us;
    ina_read_reg(reg, buffer, len);
    return true;
}

bool i2c_blocking_write_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len, uint32_t timeout_us) {
    (void)i2c; (void)addr; (void)timeout_us;
    if (reg == INA228_REG_ADC_CONFIG && len == 2) {
        ina_write_adc_config((uint16_t)((data[0] << 8) | data[1]));
    }
    return true;
}

// The registers are sampled when the transaction starts; close enough at 1ms
bool i2c_async_read_regs_dual(i2c_inst_t *i2c, uint8_t addr, uint8_t reg1, size_t len1, uint8_t reg2, size_t len2, uint8_t *buffer, i2c_async_callback_t callback, void *user_data) {
    (void)i2c; (void)addr;
    if (!fake_i2c_start(callback, user_data)) return false;
    ina_read_reg(reg1, buffer, len1);
    ina_read_reg(reg2, buffer + len1, len2);
    return true;
}

bool i2c_async_read_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buffer, size_t len, i2c_async_callback_t callback, void *user_data) {
    (void)i2c; (void)addr;
    if (!fake_i2c_start(callback, user_data)) return false;
    ina_read_reg(reg, buffer, len);
    return true;
}

bool i2c_async_write_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len, i2c_async_callback_t callback, void *user_data) {
    (void)i2c; (void)addr;
    if (!fake_i2c_start(callback, user_data)) return false;
    if (reg == INA228_REG_ADC_CONFIG && len == 2) {
        ina_write_adc_config((uint16_t)((data[0] << 8) | data[1]));
    }
    return true;
}

// ============================================================================
// Fake repeating timer
// ============================================================================

static struct repeating_timer *poll_timer;
static uint64_t poll_period_us;
static uint64_t poll_next_us;

bool add_repeating_timer_us(int64_t delay_us, repeating_timer_callback_t callback, void *user_data, struct repeating_timer *out) {
    out->callback = callback;
    out->user_data = user_data;
    poll_timer = out;
    poll_period_us = (uint64_t)(delay_us < 0 ? -delay_us : delay_us);
    poll_next_us = now_us + poll_period_us;
    return true;
}

// ============================================================================
// Simulation
// ============================================================================

static ina228_t dev;

// Sample bookkeeping, as seen by the main loop
static int samples;
static uint64_t first_sample_us, last_sample_us, longest_sample_gap_us;
static int64_t charge_at_first, true_charge_at_first;
static int64_t charge_at_last, true_charge_at_last;

// Run for a while; the 160ms timer is held off (an I2C or timer stall) until
// stall_until_us
static void run(uint64_t duration_us, uint64_t stall_until_us) {
    uint64_t end = now_us + duration_us;
    while (now_us < end) {
        now_us += 1000;
        stored_millis = (millis_t)(now_us / 1000);
        stored_millis64 = (millis64_t)(now_us / 1000);
        true_charge_raw += (int64_t)true_current_raw * 1000;
        ina_advance(1000);

        if (fake_i2c.pending) {
            fake_i2c.pending = false;
            fake_i2c.callback(INA228_I2C, true, fake_i2c.user_data);
        }

        if (now_us >= poll_next_us) {
            poll_next_us += poll_period_us;
            if (now_us >= stall_until_us) {
                poll_timer->callback(poll_timer);
            }
        }

        if (now_us % 20000 == 0 && ina228_read_current_async(&dev)) {
            if (samples == 0) {
                first_sample_us = now_us;
                charge_at_first = model.charge_raw;
                true_charge_at_first = true_charge_raw;
            } else if (now_us - last_sample_us > longest_sample_gap_us) {
                longest_sample_gap_us = now_us - last_sample_us;
            }
            last_sample_us = now_us;
            charge_at_last = model.charge_raw;
            true_charge_at_last = true_charge_raw;
            samples++;
        }
    }
}

static void start_measuring(void) {
    samples = 0;
    longest_sample_gap_us = 0;
    ina.triggered_count = 0;
    log_len = 0;
    log_text[0] = '\0';
}

// The counted charge must match the true charge between the first and last
// samples, give or take the ends: each is timestamped when it was seen, up to
// a 20ms main loop tick earlier than the main loop picks it up
static void assert_charge_matches(void) {
    int64_t counted = charge_at_last - charge_at_first;
    int64_t truth = true_charge_at_last - true_charge_at_first;
    int64_t slack = (int64_t)(true_current_raw < 0 ? -true_current_raw : true_current_raw) * 2 * 21000;
    assert_true(counted > truth - slack && counted < truth + slack);
}

static int setup(void **state) {
    (void)state;
    memset(&model, 0, sizeof(model)); // contactors open, so temperature measurements run
    assert_true(ina228_init(&dev, 0x40, 0.001f, 100.0f));
    return 0;
}

// ============================================================================
// Tests
// ============================================================================

static void test_charge_through_temperature_measurements(void **state) {
    (void)state;

    true_current_raw = 400000; // 100A
    run(2000000, 0);
    start_measuring();
    run(300000000, 0); // 5 minutes, through the time_us_32 wrap

    // Conversions were paused for temperature measurements every ~53s, both
    // kinds of which were made
    assert_true(ina.triggered_count >= 5);
    assert_true(model.shunt_die_temperature_millis > 0);
    assert_true(model.shunt_ntc_millis > 0);
    assert_true(samples > 500);

    // Counting conversions at the nominal period would be 0.12% (36ms) out on
    // the clock alone, plus up to half a period at each temperature measurement
    assert_charge_matches();
    assert_int_equal(model.current_mA, 100000);

    // Nothing was lost before the first sample either: the count starts when
    // conversions do, at init, as the true charge does
    int64_t slack = (int64_t)true_current_raw * 21000;
    assert_true(charge_at_last > true_charge_at_last - slack && charge_at_last < true_charge_at_last + slack);
}

static void test_charge_in_both_directions(void **state) {
    (void)state;

    true_current_raw = -320000; // -80A
    start_measuring();
    run(120000000, 0);
    assert_charge_matches();
    assert_int_equal(model.current_mA, -80000);
    assert_true(raw_charge_to_mC(model.charge_raw - charge_at_first) < -9500000); // ~-9600C
}

static void test_charge_through_a_long_stall(void **state) {
    (void)state;

    // The timer is held off for 10s; the chip keeps converting but nothing
    // reads it. The sample after the stall covers the whole gap.
    true_current_raw = 200000; // 50A
    start_measuring();
    run(5000000, 0);
    run(20000000, now_us + 10000000);
    assert_charge_matches();
    assert_true(longest_sample_gap_us > 10000000);
    assert_non_null(strstr(log_text, "since the last current sample"));
}

// Fit a chip with the given DEVICE_ID and start the driver on it, on a board
// of the given revision (app/board_id.h codes)
static bool fit_chip(uint16_t device_id, uint16_t board_id) {
    ina_chip = device_id >> 4;
    ina_device_id = device_id;
    memset(&ina, 0, sizeof(ina));
    memset(&model, 0, sizeof(model));
    model.board_id = board_id;
    true_current_raw = 0;
    current_read_len = vbus_read_len = 0;
    return ina228_init(&dev, 0x40, 0.001f, 100.0f);
}

static void test_ina228_register_widths_and_temperatures(void **state) {
    (void)state;

    assert_true(fit_chip(0x2281, 10));
    assert_int_equal(dev.chip, INA_CHIP_INA228);
    assert_string_equal(ina228_chip_name(&dev), "INA228");

    true_current_raw = 4943; // 1235.75mA
    start_measuring();
    run(120000000, 0);
    assert_int_equal(current_read_len, 3);
    assert_int_equal(vbus_read_len, 3);
    assert_int_equal(model.current_mA, 1236);
    assert_int_equal(ina228_stats.last_vos_counts, INA_VOS_COUNTS);
    assert_float_equal(model.shunt_die_temperature, 25.0f, 0.001f);
    // 2.7V on a divider from 5.55V
    assert_float_equal(model.shunt_ntc_resistance_ohms, 9473.7f, 5.0f);
}

static void test_ina237_current_and_charge(void **state) {
    (void)state;

    assert_true(fit_chip(0x2370, 11));
    assert_int_equal(dev.chip, INA_CHIP_INA237);
    assert_true(dev.present);
    assert_string_equal(ina228_chip_name(&dev), "INA237");
    assert_non_null(strstr(log_text, "INA237: Detected"));

    true_current_raw = 400000; // 100A
    run(2000000, 0);
    start_measuring();
    run(300000000, 0);

    // Two-byte reads, scaled to the INA228's units, with no offset curve
    assert_int_equal(current_read_len, 2);
    assert_int_equal(ina228_stats.last_raw, 400000);
    assert_int_equal(ina228_stats.last_vos_counts, 0);
    assert_int_equal(model.current_mA, 100000);
    assert_true(ina.triggered_count >= 5);
    assert_true(samples > 500);
    assert_charge_matches();

    true_current_raw = -320000; // -80A
    run(2000000, 0);
    start_measuring();
    run(60000000, 0);
    assert_int_equal(model.current_mA, -80000);
    assert_charge_matches();
}

static void test_ina237_small_current_reads_in_4mA_steps(void **state) {
    (void)state;

    assert_true(fit_chip(0x2370, 11));
    true_current_raw = 4943; // 1235.75mA, between two 4mA steps
    run(5000000, 0);
    assert_int_equal(ina228_stats.last_raw, 4928);
    assert_int_equal(model.current_mA, 1232);

    true_current_raw = -4943; // rounds down, away from zero
    run(5000000, 0);
    assert_int_equal(ina228_stats.last_raw, -4944);
    assert_int_equal(model.current_mA, -1236);
}

static void test_ina237_temperatures(void **state) {
    (void)state;

    // Rev1.1: the NTC divider runs from 3.3V
    assert_true(fit_chip(0x2370, 11));
    run(120000000, 0);
    assert_int_equal(vbus_read_len, 2);
    assert_true(model.shunt_die_temperature_millis > 0);
    assert_float_equal(model.shunt_die_temperature, 25.0f, 0.001f);
    assert_true(model.shunt_ntc_millis > 0);
    assert_float_equal(ina228_stats.ntc_volts, 2.7f, 0.001f);
    assert_float_equal(model.shunt_ntc_resistance_ohms, 45000.0f, 5.0f);
}

// DEVICE_ID 0x238x (an INA238, or an INA237 that reports the same ID) is read
// exactly as 0x237x is
static void test_ina238(void **state) {
    (void)state;

    log_len = 0;
    assert_true(fit_chip(0x2381, 11));
    assert_int_equal(dev.chip, INA_CHIP_INA238);
    assert_string_equal(ina228_chip_name(&dev), "INA237/INA238");
    assert_non_null(strstr(log_text, "INA237/INA238: Detected"));

    true_current_raw = -320000; // -80A
    run(2000000, 0);
    start_measuring();
    run(120000000, 0);
    assert_int_equal(current_read_len, 2);
    assert_int_equal(vbus_read_len, 2);
    assert_int_equal(ina228_stats.last_raw, -320000);
    assert_int_equal(ina228_stats.last_vos_counts, 0);
    assert_int_equal(model.current_mA, -80000);
    assert_charge_matches();
    assert_float_equal(model.shunt_die_temperature, 25.0f, 0.001f);
    assert_float_equal(model.shunt_ntc_resistance_ohms, 45000.0f, 5.0f);
}

static void test_unknown_chip_is_rejected(void **state) {
    (void)state;

    log_len = 0;
    assert_false(fit_chip(0x2290, 11));
    assert_false(dev.present);
    assert_int_equal(dev.chip, 0);
    assert_string_equal(ina228_chip_name(&dev), "INA2xx");
    assert_non_null(strstr(log_text, "Invalid device ID: 0x2290"));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_charge_through_temperature_measurements),
        cmocka_unit_test(test_charge_in_both_directions),
        cmocka_unit_test(test_charge_through_a_long_stall),
        cmocka_unit_test(test_ina228_register_widths_and_temperatures),
        cmocka_unit_test(test_ina237_current_and_charge),
        cmocka_unit_test(test_ina237_small_current_reads_in_4mA_steps),
        cmocka_unit_test(test_ina237_temperatures),
        cmocka_unit_test(test_ina238),
        cmocka_unit_test(test_unknown_chip_is_rejected),
    };
    return cmocka_run_group_tests(tests, setup, NULL);
}
