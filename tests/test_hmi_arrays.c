#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "app/model.h"
#include "protocols/hmi_serial/hmi_serial.h"
#include "protocols/hmi_serial/hmi_arrays.h"

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;

#define DEVICE_ADDRESS 3
#define MAX_PAYLOAD 256

bms_model_t model;

// Stands in for the chemistry's table in ekf.c
static float factory_ocv[101];
const float *ekf_factory_ocv_table(void) {
    return factory_ocv;
}

static int setup(void **state) {
    (void) state;
    memset(&model, 0, sizeof(model));
    return 0;
}

// ============================================================================
// Helpers: decode the way the HMI would
// ============================================================================

static uint16_t get_uint16(const uint8_t *buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static uint32_t get_uint32(const uint8_t *buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

// Read count little-endian int16 elements
static void decode_int16(const uint8_t *buf, size_t count, int16_t *out) {
    for (size_t i = 0; i < count; i++) {
        out[i] = (int16_t)get_uint16(&buf[i * 2]);
    }
}


typedef struct {
    uint8_t type;
    uint8_t address;
    uint8_t array_id;
    uint8_t encoding;
    uint16_t start;
    uint16_t total;
    uint8_t count;
    uint32_t sample_millis;
    const uint8_t *elements;
} response_t;

static response_t parse_response(const uint8_t *buf, size_t len) {
    assert_true(len >= HMI_ARRAY_RESPONSE_HEADER_LEN);
    return (response_t){
        .type = buf[0],
        .address = buf[1],
        .array_id = buf[2],
        .encoding = buf[3],
        .start = get_uint16(&buf[4]),
        .total = get_uint16(&buf[6]),
        .count = buf[8],
        .sample_millis = get_uint32(&buf[9]),
        .elements = &buf[HMI_ARRAY_RESPONSE_HEADER_LEN],
    };
}

// Read a whole int16 array page by page, as the HMI would, with packets of at
// most capacity bytes. Returns the pages used.
static int read_int16_array(uint8_t array_id, int16_t *out, uint16_t expected_total, size_t capacity) {
    uint16_t start = 0;
    int pages = 0;
    while (start < expected_total) {
        uint8_t buf[MAX_PAYLOAD];
        size_t len = hmi_build_read_array_response(buf, capacity, DEVICE_ADDRESS, &model, array_id, start);
        assert_true(len <= capacity);

        response_t r = parse_response(buf, len);
        assert_int_equal(r.start, start);
        assert_int_equal(r.total, expected_total);
        assert_int_equal(r.encoding, HMI_TYPE_INT16);
        assert_true(r.count > 0);

        decode_int16(r.elements, r.count, &out[start]);
        assert_int_equal(HMI_ARRAY_RESPONSE_HEADER_LEN + 2 * r.count, len);

        start += r.count;
        pages++;
    }
    assert_int_equal(start, expected_total);
    return pages;
}


// ============================================================================
// Responses
// ============================================================================

static void test_cell_voltages_header(void **state) {
    (void) state;
    for (int i = 0; i < 120; i++) model.cell_voltages_mV[i] = 3300;
    model.cell_voltages_millis = 123456;

    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_CELL_VOLTAGES, 0);
    response_t r = parse_response(buf, len);

    assert_int_equal(r.type, HMI_MSG_READ_ARRAY_RESPONSE);
    assert_int_equal(r.address, DEVICE_ADDRESS);
    assert_int_equal(r.array_id, HMI_ARRAY_CELL_VOLTAGES);
    assert_int_equal(r.encoding, HMI_TYPE_INT16);
    assert_int_equal(r.start, 0);
    assert_int_equal(r.total, 120);
    assert_int_equal(r.count, 120);
    assert_int_equal(r.sample_millis, 123456);
    // 120 cells fit one packet: 13 + 240 = 253 bytes
    assert_int_equal(len, HMI_ARRAY_RESPONSE_HEADER_LEN + 2 * 120);
    assert_true(len <= MAX_PAYLOAD);

    int16_t out[120];
    decode_int16(r.elements, 120, out);
    assert_memory_equal(out, model.cell_voltages_mV, sizeof(out));
}

static void test_cell_voltages_full_range(void **state) {
    (void) state;
    // Every int16 value survives, including -1 for an absent channel
    const int16_t values[] = { -1, INT16_MIN, INT16_MAX, 0, 1, 16384, -16385, 3300 };
    for (int i = 0; i < 120; i++) model.cell_voltages_mV[i] = values[i % 8];
    model.cell_voltages_millis = 1;

    int16_t out[120];
    int pages = read_int16_array(HMI_ARRAY_CELL_VOLTAGES, out, 120, MAX_PAYLOAD);

    assert_int_equal(pages, 1);
    assert_memory_equal(out, model.cell_voltages_mV, sizeof(out));
}

static void test_cell_voltages_paged(void **state) {
    (void) state;
    // A smaller packet stands in for an array too big for one packet
    for (int i = 0; i < 120; i++) model.cell_voltages_mV[i] = (int16_t)(3000 + i);
    model.cell_voltages_mV[50] = -1;
    model.cell_voltages_millis = 1;

    int16_t out[120];
    int pages = read_int16_array(HMI_ARRAY_CELL_VOLTAGES, out, 120, HMI_ARRAY_RESPONSE_HEADER_LEN + 101);

    // 50 values (100 bytes) per page
    assert_int_equal(pages, 3);
    assert_memory_equal(out, model.cell_voltages_mV, sizeof(out));
}

static void test_cell_voltages_from_middle(void **state) {
    (void) state;
    for (int i = 0; i < 120; i++) model.cell_voltages_mV[i] = (int16_t)(3000 + i * 5);
    model.cell_voltages_millis = 1;

    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_CELL_VOLTAGES, 100);
    response_t r = parse_response(buf, len);

    assert_int_equal(r.start, 100);
    assert_int_equal(r.count, 20);
    int16_t out[20];
    decode_int16(r.elements, 20, out);
    assert_memory_equal(out, &model.cell_voltages_mV[100], sizeof(out));
}

static void test_cell_voltages_not_read_yet(void **state) {
    (void) state;
    model.cell_voltages_millis = 0;

    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_CELL_VOLTAGES, 0);
    response_t r = parse_response(buf, len);

    assert_int_equal(r.total, 120);
    assert_int_equal(r.count, 0);
    assert_int_equal(r.sample_millis, 0);
    assert_int_equal(len, HMI_ARRAY_RESPONSE_HEADER_LEN);
}

static void test_start_past_end(void **state) {
    (void) state;
    model.cell_voltages_millis = 1;

    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_CELL_VOLTAGES, 120);
    response_t r = parse_response(buf, len);

    assert_int_equal(r.start, 120);
    assert_int_equal(r.total, 120);
    assert_int_equal(r.count, 0);
    assert_int_equal(len, HMI_ARRAY_RESPONSE_HEADER_LEN);
}

static void test_unknown_array(void **state) {
    (void) state;
    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, 200, 0);
    response_t r = parse_response(buf, len);

    assert_int_equal(r.type, HMI_MSG_READ_ARRAY_RESPONSE);
    assert_int_equal(r.array_id, 200);
    assert_int_equal(r.encoding, HMI_ARRAY_ENCODING_NONE);
    assert_int_equal(r.total, 0);
    assert_int_equal(r.count, 0);
    assert_int_equal(len, HMI_ARRAY_RESPONSE_HEADER_LEN);
}

static void test_balancing_times_untimestamped(void **state) {
    (void) state;
    // Balancing times have no sample time, so are sent straight away. They
    // run up to INT16_MAX, beyond the 15 bits the old READ_BALANCING_TIMES carried.
    for (int i = 0; i < 120; i++) model.balancing_sm.balance_time_remaining[i] = (int16_t)(i * 3);
    model.balancing_sm.balance_time_remaining[10] = INT16_MAX;
    model.balancing_sm.balance_time_remaining[11] = 0;
    model.balancing_sm.balance_time_remaining[12] = 16384;
    model.balancing_sm.balance_time_remaining[13] = 20000;

    int16_t out[120];
    read_int16_array(HMI_ARRAY_BALANCING_TIMES, out, 120, MAX_PAYLOAD);
    assert_memory_equal(out, model.balancing_sm.balance_time_remaining, sizeof(out));
}

static void test_module_temps_floats(void **state) {
    (void) state;
    for (int i = 0; i < NUM_MODULE_TEMPS; i++) model.module_temperatures[i] = 20.5f + i;
    model.module_temperatures[0] = NAN;
    model.module_temperatures_millis = 42;

    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_MODULE_TEMPS, 0);
    response_t r = parse_response(buf, len);

    assert_int_equal(r.encoding, HMI_TYPE_FLOAT);
    assert_int_equal(r.total, NUM_MODULE_TEMPS);
    assert_int_equal(r.count, NUM_MODULE_TEMPS);
    assert_int_equal(r.sample_millis, 42);
    assert_int_equal(len, HMI_ARRAY_RESPONSE_HEADER_LEN + 4 * NUM_MODULE_TEMPS);

    for (int i = 0; i < NUM_MODULE_TEMPS; i++) {
        uint32_t u = get_uint32(&r.elements[i * 4]);
        float f;
        memcpy(&f, &u, sizeof(f));
        if (i == 0) {
            assert_true(isnan(f));
        } else {
            assert_true(f == model.module_temperatures[i]);
        }
    }
}

static void test_raw_temps_uint16_paged_by_capacity(void **state) {
    (void) state;
    const int total = sizeof(model.raw_temperatures) / sizeof(model.raw_temperatures[0]);
    for (int i = 0; i < total; i++) model.raw_temperatures[i] = (uint16_t)(0xF000 + i);

    // A small buffer forces several pages: room for 10 elements each
    const size_t capacity = HMI_ARRAY_RESPONSE_HEADER_LEN + 2 * 10 + 1;
    uint16_t start = 0;
    while (start < total) {
        uint8_t buf[MAX_PAYLOAD];
        size_t len = hmi_build_read_array_response(buf, capacity, DEVICE_ADDRESS, &model, HMI_ARRAY_RAW_TEMPS, start);
        assert_true(len <= capacity);
        response_t r = parse_response(buf, len);

        assert_int_equal(r.encoding, HMI_TYPE_UINT16);
        assert_int_equal(r.total, total);
        assert_int_equal(r.count, (total - start) < 10 ? (total - start) : 10);
        for (int i = 0; i < r.count; i++) {
            assert_int_equal(get_uint16(&r.elements[i * 2]), model.raw_temperatures[start + i]);
        }
        start += r.count;
    }
}

static void test_module_ids(void **state) {
    (void) state;
    uint8_t buf[MAX_PAYLOAD];

    // Nothing until the first read
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_MODULE_IDS, 0);
    response_t r = parse_response(buf, len);
    assert_int_equal(r.encoding, HMI_TYPE_UINT16);
    assert_int_equal(r.total, NUM_MODULES);
    assert_int_equal(r.count, 0);

    // One ID per module, with a failed read as BMB3Y_MODULE_ID_NONE
    for (int i = 0; i < NUM_MODULES; i++) model.bmb_diag.module_ids[i] = (uint16_t)(0x020 + i);
    model.bmb_diag.module_ids[NUM_MODULES - 1] = BMB3Y_MODULE_ID_NONE;
    model.bmb_diag.module_ids_millis = 1234;
    len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_MODULE_IDS, 0);
    r = parse_response(buf, len);
    assert_int_equal(r.count, NUM_MODULES);
    assert_int_equal(r.sample_millis, 1234);
    for (int i = 0; i < NUM_MODULES - 1; i++) {
        assert_int_equal(get_uint16(&r.elements[i * 2]), 0x020 + i);
    }
    assert_int_equal(get_uint16(&r.elements[(NUM_MODULES - 1) * 2]), 0xFFFF);
}

// ============================================================================
// Writes
// ============================================================================

static double get_double(const uint8_t *buf) {
    uint64_t u = (uint64_t)get_uint32(buf) | ((uint64_t)get_uint32(&buf[4]) << 32);
    double d;
    memcpy(&d, &u, sizeof(d));
    return d;
}

static void put_uint32(uint8_t *buf, uint32_t v) {
    for (int i = 0; i < 4; i++) buf[i] = (v >> (8 * i)) & 0xFF;
}

static void put_double(uint8_t *buf, double d) {
    uint64_t u;
    memcpy(&u, &d, sizeof(u));
    put_uint32(buf, (uint32_t)u);
    put_uint32(&buf[4], (uint32_t)(u >> 32));
}

static void put_float(uint8_t *buf, float f) {
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    put_uint32(buf, u);
}

// Build a WRITE_ARRAY message of doubles as the HMI would, returning its length
static size_t write_doubles_msg(uint8_t *buf, uint8_t array_id, uint16_t start, const double *values, uint8_t count) {
    buf[0] = HMI_MSG_WRITE_ARRAY;
    buf[1] = DEVICE_ADDRESS;
    buf[2] = array_id;
    buf[3] = HMI_TYPE_DOUBLE;
    buf[4] = start & 0xFF;
    buf[5] = start >> 8;
    buf[6] = count;
    for (int i = 0; i < count; i++) put_double(&buf[HMI_ARRAY_WRITE_HEADER_LEN + 8 * i], values[i]);
    return HMI_ARRAY_WRITE_HEADER_LEN + 8 * count;
}

static bool write_doubles(uint8_t array_id, uint16_t start, const double *values, uint8_t count) {
    uint8_t buf[MAX_PAYLOAD];
    size_t len = write_doubles_msg(buf, array_id, start, values, count);
    return hmi_apply_write_array(&model, buf, len);
}

// Read a whole double array page by page, returning the pages used
static int read_double_array(uint8_t array_id, double *out, uint16_t expected_total) {
    uint16_t start = 0;
    int pages = 0;
    while (start < expected_total) {
        uint8_t buf[MAX_PAYLOAD];
        size_t len = hmi_build_read_array_response(buf, MAX_PAYLOAD, DEVICE_ADDRESS, &model, array_id, start);
        response_t r = parse_response(buf, len);
        assert_int_equal(r.encoding, HMI_TYPE_DOUBLE);
        assert_int_equal(r.total, expected_total);
        assert_true(r.count > 0);
        for (int i = 0; i < r.count; i++) out[start + i] = get_double(&r.elements[8 * i]);
        start += r.count;
        pages++;
    }
    return pages;
}

static void init_ekf_tuning(void) {
    model.ekf.initialized = true;
    model.ekf.Q[0] = 1e-7;
    model.ekf.Q[1] = 1e-4;
    model.ekf.Q[2] = 1e-10;
    model.ekf.R = 1e-2;
    model.ekf.min_ocv_slope = 1.5;
    model.ekf.knee_ocv_slope = 5.0;
    model.ekf.R_ohm = 0.0003;
    model.ekf.offset_tau_s = 81.0;
}

static void test_ekf_tuning_before_init(void **state) {
    (void) state;
    // ekf_init would overwrite anything written now
    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_EKF_TUNING, 0);
    response_t r = parse_response(buf, len);
    assert_int_equal(r.encoding, HMI_TYPE_DOUBLE);
    assert_int_equal(r.total, HMI_EKF_TUNING_COUNT);
    assert_int_equal(r.count, 0);

    const double r_v = 0.5;
    assert_false(write_doubles(HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_R_V, &r_v, 1));
    assert_true(model.ekf.R == 0.0);
}

static void test_ekf_tuning_read(void **state) {
    (void) state;
    init_ekf_tuning();

    double out[HMI_EKF_TUNING_COUNT];
    assert_int_equal(read_double_array(HMI_ARRAY_EKF_TUNING, out, HMI_EKF_TUNING_COUNT), 1);
    assert_true(out[HMI_EKF_TUNING_Q_AH] == 1e-7);
    assert_true(out[HMI_EKF_TUNING_Q_OFFSET] == 1e-4);
    assert_true(out[HMI_EKF_TUNING_Q_BIAS] == 1e-10);
    assert_true(out[HMI_EKF_TUNING_R_V] == 1e-2);
    assert_true(out[HMI_EKF_TUNING_MIN_OCV_SLOPE] == 1.5);
    assert_true(out[HMI_EKF_TUNING_KNEE_OCV_SLOPE] == 5.0);
    assert_true(out[HMI_EKF_TUNING_R_OHM] == 0.0003);
    assert_true(out[HMI_EKF_TUNING_OFFSET_TAU_S] == 81.0);
}

static void test_ekf_tuning_write(void **state) {
    (void) state;
    init_ekf_tuning();

    // Just R_ohm and the time constant
    const double values[] = { 0.0013, 120.0 };
    assert_true(write_doubles(HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_R_OHM, values, 2));
    assert_true(model.ekf.R_ohm == 0.0013);
    assert_true(model.ekf.offset_tau_s == 120.0);
    // The rest are untouched
    assert_true(model.ekf.R == 1e-2);
    assert_true(model.ekf.Q[2] == 1e-10);

    // The response the BMS sends back holds the new values
    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_EKF_TUNING,
                                               HMI_EKF_TUNING_R_OHM);
    response_t r = parse_response(buf, len);
    assert_int_equal(r.count, 2);
    assert_true(get_double(&r.elements[0]) == 0.0013);
    assert_true(get_double(&r.elements[8]) == 120.0);
}

static void test_ekf_tuning_refuses_bad_values(void **state) {
    (void) state;
    init_ekf_tuning();

    // The filter divides by R and the time constant
    const double zero = 0.0, negative = -1e-6;
    assert_false(write_doubles(HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_R_V, &zero, 1));
    assert_false(write_doubles(HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_OFFSET_TAU_S, &zero, 1));
    assert_false(write_doubles(HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_Q_AH, &negative, 1));
    // But a zero process noise or resistance is fine
    assert_true(write_doubles(HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_Q_BIAS, &zero, 1));
    assert_true(model.ekf.Q[2] == 0.0);

    // One bad value refuses the whole write
    const double values[] = { 2e-7, 2e-4, NAN };
    assert_false(write_doubles(HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_Q_AH, values, 3));
    assert_true(model.ekf.Q[0] == 1e-7);
    assert_true(model.ekf.Q[1] == 1e-4);

    const double inf[] = { INFINITY };
    assert_false(write_doubles(HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_KNEE_OCV_SLOPE, inf, 1));
    assert_true(model.ekf.knee_ocv_slope == 5.0);
}

static void test_write_refuses_bad_messages(void **state) {
    (void) state;
    init_ekf_tuning();
    const double values[] = { 1.0, 2.0 };
    uint8_t buf[MAX_PAYLOAD];

    // Past the end of the array
    assert_false(write_doubles(HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_COUNT - 1, values, 2));
    assert_true(model.ekf.offset_tau_s == 81.0);

    // Shorter than its element count says
    size_t len = write_doubles_msg(buf, HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_MIN_OCV_SLOPE, values, 2);
    assert_false(hmi_apply_write_array(&model, buf, len - 1));
    assert_false(hmi_apply_write_array(&model, buf, HMI_ARRAY_WRITE_HEADER_LEN - 1));
    assert_true(model.ekf.min_ocv_slope == 1.5);

    // Not the array's encoding
    len = write_doubles_msg(buf, HMI_ARRAY_EKF_TUNING, HMI_EKF_TUNING_MIN_OCV_SLOPE, values, 2);
    buf[3] = HMI_TYPE_FLOAT;
    assert_false(hmi_apply_write_array(&model, buf, len));
    assert_true(model.ekf.min_ocv_slope == 1.5);

    // Unknown and read-only arrays
    assert_false(write_doubles(200, 0, values, 1));
    model.cell_voltages_mV[0] = 3300;
    model.cell_voltages_millis = 1;
    buf[2] = HMI_ARRAY_CELL_VOLTAGES;
    buf[3] = HMI_TYPE_INT16;
    buf[4] = buf[5] = 0;
    buf[6] = 1;
    buf[7] = 0;
    buf[8] = 0;
    assert_false(hmi_apply_write_array(&model, buf, HMI_ARRAY_WRITE_HEADER_LEN + 2));
    assert_int_equal(model.cell_voltages_mV[0], 3300);
}

static void test_ekf_state_read_only(void **state) {
    (void) state;
    // Nothing until the filter has started
    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_EKF_STATE, 0);
    response_t r = parse_response(buf, len);
    assert_int_equal(r.encoding, HMI_TYPE_DOUBLE);
    assert_int_equal(r.total, HMI_EKF_STATE_COUNT);
    assert_int_equal(r.count, 0);

    init_ekf_tuning();
    model.ekf.x[0] = 12.345;
    model.ekf.x[1] = -0.0021;
    model.ekf.x[2] = 0.0375;
    for (int i = 0; i < 3; i++)
        for (int k = 0; k < 3; k++) model.ekf.P[i][k] = (i == k) ? 1.0 + i : 0.01 * (i + k);

    double x[HMI_EKF_STATE_COUNT];
    read_double_array(HMI_ARRAY_EKF_STATE, x, HMI_EKF_STATE_COUNT);
    assert_true(x[HMI_EKF_STATE_AH_USED] == 12.345);
    assert_true(x[HMI_EKF_STATE_V_OFFSET] == -0.0021);
    assert_true(x[HMI_EKF_STATE_BIAS_A] == 0.0375);

    double p[9];
    read_double_array(HMI_ARRAY_EKF_COV, p, 9);
    assert_memory_equal(p, model.ekf.P, sizeof(p));

    // Not writable
    const double v = 1.0;
    assert_false(write_doubles(HMI_ARRAY_EKF_STATE, 0, &v, 1));
    assert_false(write_doubles(HMI_ARRAY_EKF_COV, 0, &v, 1));
    assert_true(model.ekf.x[0] == 12.345);
    assert_true(model.ekf.P[0][0] == 1.0);
}

// Read a whole float array page by page
static void read_float_array(uint8_t array_id, float *out, uint16_t expected_total) {
    uint16_t start = 0;
    while (start < expected_total) {
        uint8_t buf[MAX_PAYLOAD];
        size_t len = hmi_build_read_array_response(buf, MAX_PAYLOAD, DEVICE_ADDRESS, &model, array_id, start);
        response_t r = parse_response(buf, len);
        assert_int_equal(r.encoding, HMI_TYPE_FLOAT);
        assert_int_equal(r.total, expected_total);
        assert_true(r.count > 0);
        for (int i = 0; i < r.count; i++) {
            uint32_t u = get_uint32(&r.elements[4 * i]);
            memcpy(&out[start + i], &u, sizeof(float));
        }
        start += r.count;
    }
}

static void test_ocv_factory_read_only(void **state) {
    (void) state;
    for (int i = 0; i <= 100; i++) factory_ocv[i] = 2.9f + 0.0123f * i;

    // Always there; 101 floats (404 bytes) take two pages
    float out[101];
    read_float_array(HMI_ARRAY_OCV_FACTORY, out, 101);
    assert_memory_equal(out, factory_ocv, sizeof(out));

    uint8_t buf[MAX_PAYLOAD] = {
        HMI_MSG_WRITE_ARRAY, DEVICE_ADDRESS, HMI_ARRAY_OCV_FACTORY, HMI_TYPE_FLOAT, 0, 0, 1,
    };
    put_float(&buf[HMI_ARRAY_WRITE_HEADER_LEN], 3.0f);
    assert_false(hmi_apply_write_array(&model, buf, HMI_ARRAY_WRITE_HEADER_LEN + 4));
    assert_true(factory_ocv[0] == 2.9f);
}

#ifdef OCV_LEARN
static void test_ocv_learned_read_only(void **state) {
    (void) state;
    for (int i = 0; i <= 100; i++) model.ocv_learn.table[i] = 3.0f + 0.011f * i;

    // Nothing until learning has started and the learner has built its table
    uint8_t buf[MAX_PAYLOAD];
    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_OCV_LEARNED, 0);
    response_t r = parse_response(buf, len);
    assert_int_equal(r.total, 101);
    assert_int_equal(r.count, 0);

    model.ocv_learn.persistent.valid = true;
    len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_OCV_LEARNED, 0);
    assert_int_equal(parse_response(buf, len).count, 0);

    model.ocv_learn.factory = factory_ocv;
    float out[101];
    read_float_array(HMI_ARRAY_OCV_LEARNED, out, 101);
    assert_memory_equal(out, model.ocv_learn.table, sizeof(out));

    uint8_t msg[MAX_PAYLOAD] = {
        HMI_MSG_WRITE_ARRAY, DEVICE_ADDRESS, HMI_ARRAY_OCV_LEARNED, HMI_TYPE_FLOAT, 0, 0, 1,
    };
    put_float(&msg[HMI_ARRAY_WRITE_HEADER_LEN], 3.3f);
    assert_false(hmi_apply_write_array(&model, msg, HMI_ARRAY_WRITE_HEADER_LEN + 4));
    assert_true(model.ocv_learn.table[0] == 3.0f);
}

static void test_ocv_learn_not_started(void **state) {
    (void) state;
    // Until the learner starts, its first full charge would overwrite a write
    const uint8_t ids[] = { HMI_ARRAY_OCV_LEARN_CORR, HMI_ARRAY_OCV_LEARN_COV, HMI_ARRAY_OCV_LEARN_CYCLES };
    for (size_t i = 0; i < sizeof(ids); i++) {
        uint8_t buf[MAX_PAYLOAD];
        size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, ids[i], 0);
        response_t r = parse_response(buf, len);
        assert_true(r.total > 0);
        assert_int_equal(r.count, 0);
    }

    const double corr = 0.01;
    assert_false(write_doubles(HMI_ARRAY_OCV_LEARN_CORR, 0, &corr, 1));
    assert_true(model.ocv_learn.persistent.corr[0] == 0.0);
    assert_false(model.ocv_learn.dirty);
}

static void test_ocv_learn_corr_write(void **state) {
    (void) state;
    model.ocv_learn.persistent.valid = true;

    double corr[OCV_LEARN_BINS];
    for (int i = 0; i < OCV_LEARN_BINS; i++) corr[i] = 0.001 * i - 0.004;
    assert_true(write_doubles(HMI_ARRAY_OCV_LEARN_CORR, 0, corr, OCV_LEARN_BINS));
    assert_memory_equal(model.ocv_learn.persistent.corr, corr, sizeof(corr));
    // Rebuilt on the next tick, and saved to NVM
    assert_true(model.ocv_learn.edited);
    assert_true(model.ocv_learn.dirty);

    double out[OCV_LEARN_BINS];
    read_double_array(HMI_ARRAY_OCV_LEARN_CORR, out, OCV_LEARN_BINS);
    assert_memory_equal(out, corr, sizeof(out));
}

static void test_ocv_learn_cov_paged_write(void **state) {
    (void) state;
    model.ocv_learn.persistent.valid = true;
    const int total = OCV_LEARN_BINS * OCV_LEARN_BINS;

    double cov[OCV_LEARN_BINS * OCV_LEARN_BINS];
    for (int i = 0; i < OCV_LEARN_BINS; i++)
        for (int k = 0; k < OCV_LEARN_BINS; k++)
            cov[i * OCV_LEARN_BINS + k] = 3.6e-3 * exp(-fabs((double)(i - k)) / 1.5);

    // Write it the way the HMI would, as many per packet as fit
    const int per_page = (MAX_PAYLOAD - HMI_ARRAY_WRITE_HEADER_LEN) / 8;
    int pages = 0;
    for (int start = 0; start < total; start += per_page) {
        int n = total - start < per_page ? total - start : per_page;
        assert_true(write_doubles(HMI_ARRAY_OCV_LEARN_COV, (uint16_t)start, &cov[start], (uint8_t)n));
        pages++;
    }
    assert_int_equal(pages, 6);
    assert_memory_equal(model.ocv_learn.persistent.P, cov, sizeof(cov));

    double out[OCV_LEARN_BINS * OCV_LEARN_BINS];
    read_double_array(HMI_ARRAY_OCV_LEARN_COV, out, total);
    assert_memory_equal(out, cov, sizeof(out));

    // Variances must stay positive; covariances may be anything
    const double zero = 0.0, negative = -1e-4;
    assert_false(write_doubles(HMI_ARRAY_OCV_LEARN_COV, OCV_LEARN_BINS + 1, &zero, 1));
    assert_true(write_doubles(HMI_ARRAY_OCV_LEARN_COV, 1, &negative, 1));
    assert_true(model.ocv_learn.persistent.P[0][1] == -1e-4);
}

static void test_ocv_learn_cycles_write(void **state) {
    (void) state;
    model.ocv_learn.persistent.valid = true;

    uint8_t buf[MAX_PAYLOAD] = {
        HMI_MSG_WRITE_ARRAY, DEVICE_ADDRESS, HMI_ARRAY_OCV_LEARN_CYCLES, HMI_TYPE_FLOAT, 2, 0, 2,
    };
    put_float(&buf[HMI_ARRAY_WRITE_HEADER_LEN], 3.5f);
    put_float(&buf[HMI_ARRAY_WRITE_HEADER_LEN + 4], 0.25f);
    assert_true(hmi_apply_write_array(&model, buf, HMI_ARRAY_WRITE_HEADER_LEN + 8));
    assert_true(model.ocv_learn.persistent.cycles[2] == 3.5f);
    assert_true(model.ocv_learn.persistent.cycles[3] == 0.25f);

    size_t len = hmi_build_read_array_response(buf, sizeof(buf), DEVICE_ADDRESS, &model, HMI_ARRAY_OCV_LEARN_CYCLES, 0);
    response_t r = parse_response(buf, len);
    assert_int_equal(r.encoding, HMI_TYPE_FLOAT);
    assert_int_equal(r.count, OCV_LEARN_BINS);

    // A negative count is refused
    buf[0] = HMI_MSG_WRITE_ARRAY;
    buf[2] = HMI_ARRAY_OCV_LEARN_CYCLES;
    buf[3] = HMI_TYPE_FLOAT;
    buf[4] = 2;
    buf[5] = 0;
    buf[6] = 1;
    put_float(&buf[HMI_ARRAY_WRITE_HEADER_LEN], -1.0f);
    assert_false(hmi_apply_write_array(&model, buf, HMI_ARRAY_WRITE_HEADER_LEN + 4));
    assert_true(model.ocv_learn.persistent.cycles[2] == 3.5f);
}
#endif

int main(void) {
    const struct CMUnitTest tests[] = {

        // Responses
        cmocka_unit_test_setup(test_cell_voltages_header, setup),
        cmocka_unit_test_setup(test_cell_voltages_full_range, setup),
        cmocka_unit_test_setup(test_cell_voltages_paged, setup),
        cmocka_unit_test_setup(test_cell_voltages_from_middle, setup),
        cmocka_unit_test_setup(test_cell_voltages_not_read_yet, setup),
        cmocka_unit_test_setup(test_start_past_end, setup),
        cmocka_unit_test_setup(test_unknown_array, setup),
        cmocka_unit_test_setup(test_balancing_times_untimestamped, setup),
        cmocka_unit_test_setup(test_module_temps_floats, setup),
        cmocka_unit_test_setup(test_raw_temps_uint16_paged_by_capacity, setup),
        cmocka_unit_test_setup(test_module_ids, setup),

        // Writes
        cmocka_unit_test_setup(test_ekf_tuning_before_init, setup),
        cmocka_unit_test_setup(test_ekf_tuning_read, setup),
        cmocka_unit_test_setup(test_ekf_tuning_write, setup),
        cmocka_unit_test_setup(test_ekf_tuning_refuses_bad_values, setup),
        cmocka_unit_test_setup(test_write_refuses_bad_messages, setup),
        cmocka_unit_test_setup(test_ekf_state_read_only, setup),
        cmocka_unit_test_setup(test_ocv_factory_read_only, setup),
#ifdef OCV_LEARN
        cmocka_unit_test_setup(test_ocv_learned_read_only, setup),
        cmocka_unit_test_setup(test_ocv_learn_not_started, setup),
        cmocka_unit_test_setup(test_ocv_learn_corr_write, setup),
        cmocka_unit_test_setup(test_ocv_learn_cov_paged_write, setup),
        cmocka_unit_test_setup(test_ocv_learn_cycles_write, setup),
#endif
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
