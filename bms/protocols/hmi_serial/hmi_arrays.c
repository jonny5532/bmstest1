#include "hmi_arrays.h"
#include "hmi_serial.h"
#include "app/model.h"

#include <math.h>
#include <string.h>

typedef struct {
    const void *values;
    // For arrays that aren't contiguous in the model: the address of an element
    void *(*element)(bms_model_t *model, uint16_t index);
    uint16_t total;
    uint8_t encoding;
    millis_t sample_millis; // when the values were read, 0 = no reading yet
    bool timestamped;       // whether sample_millis is tracked for this array
    bool unavailable;       // no values yet, so nothing to read or write

    bool writable;
    bool (*check)(uint16_t index, double value); // a value may be written there (NULL = any)
    void (*written)(bms_model_t *model);         // after a write is applied
} hmi_array_t;

static void *ekf_tuning_element(bms_model_t *model, uint16_t index) {
    ekf_t *ekf = &model->ekf;
    switch (index) {
        case HMI_EKF_TUNING_Q_AH:           return &ekf->Q[0];
        case HMI_EKF_TUNING_Q_OFFSET:       return &ekf->Q[1];
        case HMI_EKF_TUNING_Q_BIAS:         return &ekf->Q[2];
        case HMI_EKF_TUNING_R_V:            return &ekf->R;
        case HMI_EKF_TUNING_MIN_OCV_SLOPE:  return &ekf->min_ocv_slope;
        case HMI_EKF_TUNING_KNEE_OCV_SLOPE: return &ekf->knee_ocv_slope;
        case HMI_EKF_TUNING_R_OHM:          return &ekf->R_ohm;
        case HMI_EKF_TUNING_OFFSET_TAU_S:   return &ekf->offset_tau_s;
    }
    return NULL;
}

static bool ekf_tuning_check(uint16_t index, double value) {
    // The filter divides by these
    if (index == HMI_EKF_TUNING_R_V || index == HMI_EKF_TUNING_OFFSET_TAU_S) return value > 0.0;
    return value >= 0.0;
}

#ifdef OCV_LEARN
static bool ocv_learn_cov_check(uint16_t index, double value) {
    // Variances on the diagonal must stay positive
    if (index % (OCV_LEARN_BINS + 1) == 0) return value > 0.0;
    return true;
}

static bool ocv_learn_cycles_check(uint16_t index, double value) {
    (void)index;
    return value >= 0.0;
}

static void ocv_learn_written(bms_model_t *model) {
    ocv_learn_mark_edited(&model->ocv_learn);
}
#endif

// Takes the model non-const for hmi_apply_write_array; reads cast to it
static bool hmi_get_array(bms_model_t *model, uint8_t array_id, hmi_array_t *array) {
    switch (array_id) {
        case HMI_ARRAY_CELL_VOLTAGES:
            *array = (hmi_array_t){
                .values = model->cell_voltages_mV,
                .total = sizeof(model->cell_voltages_mV) / sizeof(model->cell_voltages_mV[0]),
                .encoding = HMI_TYPE_INT16,
                .sample_millis = model->cell_voltages_millis,
                .timestamped = true,
            };
            return true;
        case HMI_ARRAY_BALANCING_TIMES:
            *array = (hmi_array_t){
                .values = model->balancing_sm.balance_time_remaining,
                .total = sizeof(model->balancing_sm.balance_time_remaining) / sizeof(model->balancing_sm.balance_time_remaining[0]),
                .encoding = HMI_TYPE_INT16,
            };
            return true;
        case HMI_ARRAY_MODULE_TEMPS:
            *array = (hmi_array_t){
                .values = model->module_temperatures,
                .total = sizeof(model->module_temperatures) / sizeof(model->module_temperatures[0]),
                .encoding = HMI_TYPE_FLOAT,
                .sample_millis = model->module_temperatures_millis,
                .timestamped = true,
            };
            return true;
        case HMI_ARRAY_RAW_TEMPS:
            *array = (hmi_array_t){
                .values = model->raw_temperatures,
                .total = sizeof(model->raw_temperatures) / sizeof(model->raw_temperatures[0]),
                .encoding = HMI_TYPE_UINT16,
            };
            return true;
        case HMI_ARRAY_MODULE_IDS:
            *array = (hmi_array_t){
                .values = model->bmb_diag.module_ids,
                .total = NUM_MODULE_VOLTAGES,
                .encoding = HMI_TYPE_UINT16,
                .sample_millis = model->bmb_diag.module_ids_millis,
                .timestamped = true,
            };
            return true;
        case HMI_ARRAY_EKF_TUNING:
            *array = (hmi_array_t){
                .element = ekf_tuning_element,
                .total = HMI_EKF_TUNING_COUNT,
                .encoding = HMI_TYPE_DOUBLE,
                .unavailable = !model->ekf.initialized, // ekf_init would overwrite a write
                .writable = true,
                .check = ekf_tuning_check,
            };
            return true;
        case HMI_ARRAY_EKF_STATE:
            *array = (hmi_array_t){
                .values = model->ekf.x,
                .total = HMI_EKF_STATE_COUNT,
                .encoding = HMI_TYPE_DOUBLE,
                .unavailable = !model->ekf.initialized,
            };
            return true;
        case HMI_ARRAY_EKF_COV:
            *array = (hmi_array_t){
                .values = model->ekf.P,
                .total = HMI_EKF_STATE_COUNT * HMI_EKF_STATE_COUNT,
                .encoding = HMI_TYPE_DOUBLE,
                .unavailable = !model->ekf.initialized,
            };
            return true;
        case HMI_ARRAY_OCV_FACTORY:
            *array = (hmi_array_t){
                .values = ekf_factory_ocv_table(),
                .total = 101,
                .encoding = HMI_TYPE_FLOAT,
            };
            return true;
#ifdef OCV_LEARN
        // Before the learner starts, its first full charge would overwrite a write
        case HMI_ARRAY_OCV_LEARN_CORR:
            *array = (hmi_array_t){
                .values = model->ocv_learn.persistent.corr,
                .total = OCV_LEARN_BINS,
                .encoding = HMI_TYPE_DOUBLE,
                .unavailable = !model->ocv_learn.persistent.valid,
                .writable = true,
                .written = ocv_learn_written,
            };
            return true;
        case HMI_ARRAY_OCV_LEARN_COV:
            *array = (hmi_array_t){
                .values = model->ocv_learn.persistent.P,
                .total = OCV_LEARN_BINS * OCV_LEARN_BINS,
                .encoding = HMI_TYPE_DOUBLE,
                .unavailable = !model->ocv_learn.persistent.valid,
                .writable = true,
                .check = ocv_learn_cov_check,
                .written = ocv_learn_written,
            };
            return true;
        case HMI_ARRAY_OCV_LEARNED:
            *array = (hmi_array_t){
                .values = model->ocv_learn.table,
                .total = sizeof(model->ocv_learn.table) / sizeof(model->ocv_learn.table[0]),
                .encoding = HMI_TYPE_FLOAT,
                // Built on the learner's first tick once learning has started, and rebuilt
                // a tick after an HMI write, so may lag a write by a second
                .unavailable = !model->ocv_learn.persistent.valid || !model->ocv_learn.factory,
            };
            return true;
        case HMI_ARRAY_OCV_LEARN_CYCLES:
            *array = (hmi_array_t){
                .values = model->ocv_learn.persistent.cycles,
                .total = OCV_LEARN_BINS,
                .encoding = HMI_TYPE_FLOAT,
                .unavailable = !model->ocv_learn.persistent.valid,
                .writable = true,
                .check = ocv_learn_cycles_check,
                .written = ocv_learn_written,
            };
            return true;
#endif
    }
    return false;
}

// Address of an element in the model
static void *hmi_array_element(bms_model_t *model, const hmi_array_t *array, uint16_t index) {
    if (array->element) return array->element(model, index);
    return (uint8_t *)array->values + (size_t)index * (array->encoding & 0x0F);
}

static inline void put_uint16(uint8_t *buf, uint16_t value) {
    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
}

static inline void put_uint32(uint8_t *buf, uint32_t value) {
    buf[0] = value & 0xFF;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = (value >> 16) & 0xFF;
    buf[3] = (value >> 24) & 0xFF;
}

static inline void put_uint64(uint8_t *buf, uint64_t value) {
    put_uint32(buf, (uint32_t)value);
    put_uint32(&buf[4], (uint32_t)(value >> 32));
}

static inline uint16_t get_uint16(const uint8_t *buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static inline uint32_t get_uint32(const uint8_t *buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static inline uint64_t get_uint64(const uint8_t *buf) {
    return (uint64_t)get_uint32(buf) | ((uint64_t)get_uint32(&buf[4]) << 32);
}

size_t hmi_encode_int16_delta_array(uint8_t *buf, size_t capacity, const int16_t *values,
                                    size_t count, size_t *encoded) {
    // Absolute values are sent as two bytes, big-endian, with the MSB set (so
    // only the low 15 bits of the value are kept). Deltas from the previous
    // value in -64 to +63 are sent as one byte with the MSB clear. The
    // previous value starts at zero.

    int16_t last_value = 0;
    size_t idx = 0;
    size_t i;
    for (i = 0; i < count; i++) {
        int16_t value = values[i];
        int32_t delta = (int32_t)value - last_value;
        if (delta >= -64 && delta <= 63) {
            if (idx + 1 > capacity) break;
            buf[idx++] = delta & 0x7F;
        } else {
            if (idx + 2 > capacity) break;
            buf[idx++] = ((value >> 8) & 0xFF) | 0x80;
            buf[idx++] = value & 0xFF;
        }
        last_value = value;
    }
    if (encoded) *encoded = i;
    return idx;
}

static size_t hmi_encode_fixed_array(uint8_t *buf, size_t capacity, bms_model_t *model,
                                     const hmi_array_t *array, uint16_t start, size_t count, size_t *encoded) {
    size_t size = array->encoding & 0x0F;
    size_t n = capacity / size;
    if (n > count) n = count;

    for (size_t i = 0; i < n; i++) {
        uint8_t *out = &buf[i * size];
        const void *value = hmi_array_element(model, array, (uint16_t)(start + i));
        switch (array->encoding) {
            case HMI_TYPE_DOUBLE: {
                uint64_t u;
                memcpy(&u, value, sizeof(u));
                put_uint64(out, u);
                break;
            }
            case HMI_TYPE_FLOAT: {
                uint32_t u;
                memcpy(&u, value, sizeof(u));
                put_uint32(out, u);
                break;
            }
            case HMI_TYPE_INT16:
                put_uint16(out, (uint16_t)*(const int16_t *)value);
                break;
            case HMI_TYPE_UINT16:
                put_uint16(out, *(const uint16_t *)value);
                break;
        }
    }
    *encoded = n;
    return n * size;
}

size_t hmi_build_read_array_response(uint8_t *buf, size_t capacity, uint8_t device_address,
                                     const bms_model_t *model, uint8_t array_id, uint16_t start) {
    // Only read through, despite hmi_get_array
    bms_model_t *m = (bms_model_t *)model;
    hmi_array_t array;
    if (!hmi_get_array(m, array_id, &array)) {
        array = (hmi_array_t){ .total = 0, .encoding = HMI_ARRAY_ENCODING_NONE };
    }

    size_t idx = 0;
    buf[idx++] = HMI_MSG_READ_ARRAY_RESPONSE;
    buf[idx++] = device_address;
    buf[idx++] = array_id;
    buf[idx++] = array.encoding;
    put_uint16(&buf[idx], start);
    idx += 2;
    put_uint16(&buf[idx], array.total);
    idx += 2;
    size_t count_idx = idx++;
    put_uint32(&buf[idx], array.sample_millis);
    idx += 4;

    // Nothing to send past the end, or before the first reading
    size_t count = 0;
    if (start < array.total && !array.unavailable && !(array.timestamped && array.sample_millis == 0)) {
        size_t remaining = array.total - start;
        // The count is one byte
        if (remaining > 255) remaining = 255;

        idx += hmi_encode_fixed_array(&buf[idx], capacity - idx, m, &array, start, remaining, &count);
    }
    buf[count_idx] = (uint8_t)count;

    return idx;
}

// Decode one element of the given encoding, as a double (exact for all of them)
static double decode_element(const uint8_t *in, uint8_t encoding) {
    switch (encoding) {
        case HMI_TYPE_DOUBLE: {
            uint64_t u = get_uint64(in);
            double d;
            memcpy(&d, &u, sizeof(d));
            return d;
        }
        case HMI_TYPE_FLOAT: {
            uint32_t u = get_uint32(in);
            float f;
            memcpy(&f, &u, sizeof(f));
            return f;
        }
        case HMI_TYPE_INT16:
            return (int16_t)get_uint16(in);
        case HMI_TYPE_UINT16:
            return get_uint16(in);
    }
    return NAN;
}

static void store_element(void *out, uint8_t encoding, double value) {
    switch (encoding) {
        case HMI_TYPE_DOUBLE: *(double *)out = value; break;
        case HMI_TYPE_FLOAT:  *(float *)out = (float)value; break;
        case HMI_TYPE_INT16:  *(int16_t *)out = (int16_t)value; break;
        case HMI_TYPE_UINT16: *(uint16_t *)out = (uint16_t)value; break;
    }
}

bool hmi_apply_write_array(bms_model_t *model, const uint8_t *msg, size_t len) {
    if (len < HMI_ARRAY_WRITE_HEADER_LEN) return false;
    uint8_t array_id = msg[2];
    uint8_t encoding = msg[3];
    uint16_t start = get_uint16(&msg[4]);
    uint8_t count = msg[6];
    const uint8_t *elements = &msg[HMI_ARRAY_WRITE_HEADER_LEN];

    hmi_array_t array;
    if (!hmi_get_array(model, array_id, &array) || !array.writable || array.unavailable) return false;
    if (encoding != array.encoding) return false;
    size_t size = encoding & 0x0F;
    if ((size_t)start + count > array.total) return false;
    if (len < HMI_ARRAY_WRITE_HEADER_LEN + count * size) return false;
    if (count == 0) return true;

    // Check every value before changing any
    bool is_float = encoding == HMI_TYPE_DOUBLE || encoding == HMI_TYPE_FLOAT;
    for (size_t i = 0; i < count; i++) {
        double value = decode_element(&elements[i * size], encoding);
        if (is_float && !isfinite(value)) return false;
        if (array.check && !array.check((uint16_t)(start + i), value)) return false;
    }

    for (size_t i = 0; i < count; i++) {
        store_element(hmi_array_element(model, &array, (uint16_t)(start + i)), encoding,
                      decode_element(&elements[i * size], encoding));
    }
    if (array.written) array.written(model);
    return true;
}
