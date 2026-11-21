// The soft and working cell voltage limits and the SoC, as written over the
// HMI: out-of-order limits and SoCs above 100% are refused, limits saved out
// of order fall back at boot, and the SoC estimators stay finite whatever the
// limits.

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
#include "app/firmware_id.h"
#include "app/estimators/ekf.h"
#include "app/estimators/estimators.h"
#include "drivers/comms/duart.h"
#include "protocols/hmi_serial/hmi_buf.h"
#include "protocols/hmi_serial/hmi_serial.h"
#include "sys/logging/logging.h"

#include "pico/unique_id.h"

// Everything logged since the test last cleared it
static char log_text[4096];
static size_t log_len;

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    va_list args;
    va_start(args, format);
    int n = vsnprintf(log_text + log_len, sizeof(log_text) - log_len, format, args);
    va_end(args);
    if (n > 0) log_len += (size_t)n < sizeof(log_text) - log_len ? (size_t)n : sizeof(log_text) - 1 - log_len;
}

size_t logging_read(uint8_t *dest, uint32_t len) {
    (void)dest;
    (void)len;
    return 0;
}

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;
uint32_t stored_timestep = 0;

const uint32_t bms_build_hash = 0x8b0c547u;
const uint32_t bms_build_time = 1792576800u;
const char bms_build_version[] = "8b0c547, built 2026-10-21 10:00 UTC";

void pico_get_unique_board_id(pico_unique_board_id_t *id_out) {
    memset(id_out, 0, sizeof(*id_out));
}

// The HMI link: one request in, the last packet sent kept
duart duart0, duart1;
static uint8_t rx_packet[256];
static size_t rx_packet_len;
static uint8_t tx_packet[256];
static size_t tx_packet_len;

bool init_duart(duart *u, uint baud_rate, uint tx_pin, uint rx_pin, bool deassert_tx_when_idle) {
    (void)u; (void)baud_rate; (void)tx_pin; (void)rx_pin; (void)deassert_tx_when_idle;
    return true;
}

size_t duart_read_packet(duart *u, uint8_t *buf, size_t buf_size) {
    (void)u;
    size_t len = rx_packet_len < buf_size ? rx_packet_len : buf_size;
    memcpy(buf, rx_packet, len);
    rx_packet_len = 0;
    return len;
}

bool duart_send_packet(duart *u, const uint8_t *payload, size_t payload_len) {
    (void)u;
    memcpy(tx_packet, payload, payload_len);
    tx_packet_len = payload_len;
    return true;
}

size_t duart_get_tx_free_space(duart *u) {
    (void)u;
    return 8192;
}

static int nvm_slow_saves;

bool nvm_save_persistent_slow(bms_model_t *model) {
    (void)model;
    nvm_slow_saves++;
    return true;
}

// model is model.c's

static int setup(void **state) {
    (void)state;
    memset(&model, 0, sizeof(model));
    nvm_slow_saves = 0;
    log_len = 0;
    log_text[0] = '\0';
    return 0;
}

// Write uint16 registers over the HMI, as (id, value) pairs
static void hmi_write_uint16(size_t count, const uint16_t *ids, const uint16_t *values) {
    size_t idx = 0;
    rx_packet[idx++] = HMI_MSG_WRITE_REGISTERS;
    rx_packet[idx++] = hmi_serial_device_address();
    for (size_t i = 0; i < count; i++) {
        idx += hmi_buf_append_uint16(&rx_packet[idx], ids[i]);
        rx_packet[idx++] = HMI_TYPE_UINT16;
        idx += hmi_buf_append_uint16(&rx_packet[idx], values[i]);
    }
    rx_packet_len = idx;
    tx_packet_len = 0;
    hmi_serial_tick(&model);
}

static void hmi_write_one(uint16_t id, uint16_t value) {
    hmi_write_uint16(1, &id, &value);
}

// The value the reply to the last write shows for a uint16 register
static uint16_t reply_value(uint16_t id) {
    assert_true(tx_packet_len >= 2);
    assert_int_equal(tx_packet[0], HMI_MSG_READ_REGISTERS_RESPONSE);
    size_t idx = 2;
    while (idx + 3 <= tx_packet_len) {
        uint16_t reg = hmi_buf_get_uint16(&tx_packet[idx]);
        uint8_t type = tx_packet[idx + 2];
        uint8_t size = type & 0x0F;
        if (reg == id) {
            assert_int_equal(type, HMI_TYPE_UINT16);
            return hmi_buf_get_uint16(&tx_packet[idx + 3]);
        }
        idx += 3 + size;
    }
    fail_msg("register 0x%04x not in the reply", id);
    return 0;
}

// The profile's (bluetesla, NMC): soft 3000-4200mV, working 3300-4050mV
static void test_profile_defaults_are_valid(void **state) {
    (void)state;
    assert_true(cell_voltage_limits_valid(&model));
    assert_int_equal(get_cell_voltage_soft_min_mV(&model), DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV);
    assert_int_equal(get_cell_voltage_soft_max_mV(&model), DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV);
    assert_int_equal(get_cell_voltage_working_min_mV(&model), DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV);
    assert_int_equal(get_cell_voltage_working_max_mV(&model), DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV);
}

static void test_working_min_at_or_above_max_refused(void **state) {
    (void)state;
    const uint16_t max = get_cell_voltage_working_max_mV(&model);
    const uint16_t values[] = { max, max + 50, max - 1 };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        hmi_write_one(HMI_REG_CELL_VOLTAGE_WORKING_MIN, values[i]);
        assert_int_equal(model.cell_voltage_working_min_mV, 0);
        assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MIN), DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV);
    }
    assert_int_equal(nvm_slow_saves, 0);
}

static void test_working_max_at_or_below_min_refused(void **state) {
    (void)state;
    const uint16_t min = get_cell_voltage_working_min_mV(&model);
    const uint16_t values[] = { min, min - 100, min + 1, 1000 };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        hmi_write_one(HMI_REG_CELL_VOLTAGE_WORKING_MAX, values[i]);
        assert_int_equal(model.cell_voltage_working_max_mV, 0);
        assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MAX), DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV);
    }
    assert_int_equal(nvm_slow_saves, 0);
}

// At least CELL_VOLTAGE_WORKING_MIN_GAP_mV between the working limits
static void test_working_gap(void **state) {
    (void)state;
    const uint16_t max = get_cell_voltage_working_max_mV(&model);

    hmi_write_one(HMI_REG_CELL_VOLTAGE_WORKING_MIN, max - CELL_VOLTAGE_WORKING_MIN_GAP_mV + 1);
    assert_int_equal(model.cell_voltage_working_min_mV, 0);
    assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MIN), DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV);
    assert_int_equal(nvm_slow_saves, 0);

    hmi_write_one(HMI_REG_CELL_VOLTAGE_WORKING_MIN, max - CELL_VOLTAGE_WORKING_MIN_GAP_mV);
    assert_int_equal(model.cell_voltage_working_min_mV, max - CELL_VOLTAGE_WORKING_MIN_GAP_mV);
    assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MIN), max - CELL_VOLTAGE_WORKING_MIN_GAP_mV);
    assert_int_equal(nvm_slow_saves, 1);

    // And from the other side, against the new min
    const uint16_t min = get_cell_voltage_working_min_mV(&model);
    hmi_write_one(HMI_REG_CELL_VOLTAGE_WORKING_MAX, min + CELL_VOLTAGE_WORKING_MIN_GAP_mV - 1);
    assert_int_equal(model.cell_voltage_working_max_mV, 0);
    assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MAX), max);
    assert_int_equal(nvm_slow_saves, 1);
}

// A soft min at or above the soft max, or above the working max (which it
// would push the working min past)
static void test_soft_min_out_of_order_refused(void **state) {
    (void)state;
    const uint16_t values[] = {
        DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV,        // = soft max
        DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV + 100,  // above it
        DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV,     // = working max
        DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV - 100, // working min pushed to within 75mV of the working max
    };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        hmi_write_one(HMI_REG_CELL_VOLTAGE_SOFT_MIN, values[i]);
        assert_int_equal(model.cell_voltage_soft_min_mV, 0);
        assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_SOFT_MIN), DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV);
    }
    assert_int_equal(nvm_slow_saves, 0);
}

static void test_soft_max_out_of_order_refused(void **state) {
    (void)state;
    const uint16_t values[] = {
        DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV,        // = soft min
        DEFAULT_CELL_VOLTAGE_SOFT_MIN_mV - 100,  // below it
        DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV,     // = working min
        DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV + 100, // working max pushed to within 75mV of the working min
    };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        hmi_write_one(HMI_REG_CELL_VOLTAGE_SOFT_MAX, values[i]);
        assert_int_equal(model.cell_voltage_soft_max_mV, 0);
        assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_SOFT_MAX), DEFAULT_CELL_VOLTAGE_SOFT_MAX_mV);
    }
    assert_int_equal(nvm_slow_saves, 0);
}

static void test_in_order_writes_accepted(void **state) {
    (void)state;
    const uint16_t ids[] = {
        HMI_REG_CELL_VOLTAGE_SOFT_MIN, HMI_REG_CELL_VOLTAGE_SOFT_MAX,
        HMI_REG_CELL_VOLTAGE_WORKING_MIN, HMI_REG_CELL_VOLTAGE_WORKING_MAX,
    };
    const uint16_t values[] = { 3100, 4150, 3400, 4000 };
    hmi_write_uint16(4, ids, values);
    for (size_t i = 0; i < 4; i++) {
        assert_int_equal(reply_value(ids[i]), values[i]);
    }
    assert_int_equal(model.cell_voltage_soft_min_mV, 3100);
    assert_int_equal(model.cell_voltage_soft_max_mV, 4150);
    assert_int_equal(model.cell_voltage_working_min_mV, 3400);
    assert_int_equal(model.cell_voltage_working_max_mV, 4000);
    assert_int_equal(nvm_slow_saves, 1);

    // Back to the profile's
    const uint16_t zeros[] = { 0, 0, 0, 0 };
    hmi_write_uint16(4, ids, zeros);
    assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MIN), DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV);
    assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MAX), DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV);
    assert_true(cell_voltage_limits_valid(&model));
    assert_int_equal(nvm_slow_saves, 2);
}

// Each write in a request is checked against the limits as they stand after
// the ones before it: raising both working limits past the old max needs the
// max first
static void test_writes_checked_in_order(void **state) {
    (void)state;
    const uint16_t ids_min_first[] = { HMI_REG_CELL_VOLTAGE_WORKING_MIN, HMI_REG_CELL_VOLTAGE_WORKING_MAX };
    const uint16_t values_min_first[] = { 4060, 4170 };
    hmi_write_uint16(2, ids_min_first, values_min_first);
    assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MIN), DEFAULT_CELL_VOLTAGE_WORKING_MIN_mV);
    assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MAX), 4170);

    setup(NULL);
    const uint16_t ids_max_first[] = { HMI_REG_CELL_VOLTAGE_WORKING_MAX, HMI_REG_CELL_VOLTAGE_WORKING_MIN };
    const uint16_t values_max_first[] = { 4170, 4060 };
    hmi_write_uint16(2, ids_max_first, values_max_first);
    assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MAX), 4170);
    assert_int_equal(reply_value(HMI_REG_CELL_VOLTAGE_WORKING_MIN), 4060);
}

static void test_soc_above_100_percent_refused(void **state) {
    (void)state;
    // Not started yet: a refused SoC doesn't start the filter
    hmi_write_one(HMI_REG_SOC, 10001);
    assert_false(model.ekf.initialized);
    hmi_write_one(HMI_REG_SOC, 65535);
    assert_false(model.ekf.initialized);

    hmi_write_one(HMI_REG_SOC, 10000);
    assert_true(model.ekf.initialized);
    const double full_used = model.ekf.x[0];

    hmi_write_one(HMI_REG_SOC, 20000);
    assert_true(model.ekf.x[0] == full_used);

    hmi_write_one(HMI_REG_SOC, 5000);
    assert_true(model.ekf.x[0] > full_used);
    assert_true(isfinite(model.ekf.x[0]));
}

// Boot: limits saved out of order fall back to the profile's

static void test_load_valid_limits_kept(void **state) {
    (void)state;
    model.cell_voltage_soft_min_mV = 3100;
    model.cell_voltage_soft_max_mV = 4150;
    model.cell_voltage_working_min_mV = 3400;
    model.cell_voltage_working_max_mV = 4000;
    model_check_cell_voltage_limits(&model);
    assert_int_equal(model.cell_voltage_soft_min_mV, 3100);
    assert_int_equal(model.cell_voltage_soft_max_mV, 4150);
    assert_int_equal(model.cell_voltage_working_min_mV, 3400);
    assert_int_equal(model.cell_voltage_working_max_mV, 4000);
    assert_int_equal(log_len, 0);
}

static void test_load_bad_working_pair_falls_back(void **state) {
    (void)state;
    model.cell_voltage_soft_min_mV = 3100;
    model.cell_voltage_soft_max_mV = 4150;
    model.cell_voltage_working_min_mV = 3900; // above the max
    model.cell_voltage_working_max_mV = 3800;
    model_check_cell_voltage_limits(&model);
    assert_int_equal(model.cell_voltage_soft_min_mV, 3100);
    assert_int_equal(model.cell_voltage_soft_max_mV, 4150);
    assert_int_equal(model.cell_voltage_working_min_mV, 0);
    assert_int_equal(model.cell_voltage_working_max_mV, 0);
    assert_true(cell_voltage_limits_valid(&model));
    assert_non_null(strstr(log_text, "using the profile's working limits"));
}

static void test_load_bad_soft_pair_falls_back(void **state) {
    (void)state;
    model.cell_voltage_soft_min_mV = 4100; // pushes the working min past the working max
    model.cell_voltage_soft_max_mV = 4150;
    model.cell_voltage_working_min_mV = 3400;
    model.cell_voltage_working_max_mV = 4000;
    model_check_cell_voltage_limits(&model);
    assert_int_equal(model.cell_voltage_soft_min_mV, 0);
    assert_int_equal(model.cell_voltage_soft_max_mV, 0);
    assert_int_equal(model.cell_voltage_working_min_mV, 3400);
    assert_int_equal(model.cell_voltage_working_max_mV, 4000);
    assert_true(cell_voltage_limits_valid(&model));
    assert_non_null(strstr(log_text, "using the profile's soft limits"));
}

static void test_load_both_pairs_bad_falls_back(void **state) {
    (void)state;
    model.cell_voltage_soft_min_mV = 4100;
    model.cell_voltage_soft_max_mV = 3100;
    model.cell_voltage_working_min_mV = 3900;
    model.cell_voltage_working_max_mV = 3900;
    model_check_cell_voltage_limits(&model);
    assert_int_equal(model.cell_voltage_soft_min_mV, 0);
    assert_int_equal(model.cell_voltage_soft_max_mV, 0);
    assert_int_equal(model.cell_voltage_working_min_mV, 0);
    assert_int_equal(model.cell_voltage_working_max_mV, 0);
    assert_true(cell_voltage_limits_valid(&model));
    assert_non_null(strstr(log_text, "using the profile's soft and working limits"));
}

// The estimators, should bad limits get past all that

static void check_estimators_finite(uint16_t working_min_mV, uint16_t working_max_mV) {
    setup(NULL);
    model.cell_voltage_working_min_mV = working_min_mV;
    model.cell_voltage_working_max_mV = working_max_mV;
    assert_false(cell_voltage_limits_valid(&model));

    // EKF: started from a voltage, then a few steps
    uint32_t soc = 0;
    for (int i = 0; i < 5; i++) {
        soc = ekf_tick(&model, 0.0f, -1000, 3700);
    }
    assert_true(model.ekf.initialized);
    assert_true(soc <= 10000);
    assert_true(isfinite(model.ekf.prev_soc_mul) && model.ekf.prev_soc_mul > 0.0f);
    // Unscaled: the working capacity is the whole capacity
    const uint32_t capacity_mC = get_actual_capacity_mC(&model);
    assert_true(model.working_capacity_mC + capacity_mC / 10000 >= capacity_mC
                && model.working_capacity_mC <= capacity_mC + capacity_mC / 10000);

    ekf_set_soc(&model, 5000);
    assert_true(isfinite(model.ekf.x[0]));
    assert_true(isfinite(model.charge_used_Ah));

    // Voltage based
    model.cell_voltage_total_mV = 3700 * NUM_CELLS;
    model.cell_voltage_min_mV = 3690;
    model.cell_voltage_max_mV = 3710;
    model.current_mA = 0;
    uint16_t v_soc = voltage_based_soc_estimate(&model);
    assert_true(v_soc > 0 && v_soc < 10000); // mid-table, unscaled
}

static void test_estimators_working_min_equals_max(void **state) {
    (void)state;
    check_estimators_finite(3700, 3700);
}

static void test_estimators_working_min_above_max(void **state) {
    (void)state;
    check_estimators_finite(3900, 3800);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_profile_defaults_are_valid, setup),
        cmocka_unit_test_setup(test_working_min_at_or_above_max_refused, setup),
        cmocka_unit_test_setup(test_working_max_at_or_below_min_refused, setup),
        cmocka_unit_test_setup(test_working_gap, setup),
        cmocka_unit_test_setup(test_soft_min_out_of_order_refused, setup),
        cmocka_unit_test_setup(test_soft_max_out_of_order_refused, setup),
        cmocka_unit_test_setup(test_in_order_writes_accepted, setup),
        cmocka_unit_test_setup(test_writes_checked_in_order, setup),
        cmocka_unit_test_setup(test_soc_above_100_percent_refused, setup),
        cmocka_unit_test_setup(test_load_valid_limits_kept, setup),
        cmocka_unit_test_setup(test_load_bad_working_pair_falls_back, setup),
        cmocka_unit_test_setup(test_load_bad_soft_pair_falls_back, setup),
        cmocka_unit_test_setup(test_load_both_pairs_bad_falls_back, setup),
        cmocka_unit_test_setup(test_estimators_working_min_equals_max, setup),
        cmocka_unit_test_setup(test_estimators_working_min_above_max, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
