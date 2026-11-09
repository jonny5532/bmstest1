#pragma once

// Little-endian field packing for HMI serial messages

#include <stdint.h>

static inline uint8_t hmi_buf_append_uint64(uint8_t *buf, uint64_t value) {
    uint8_t idx = 0;
    buf[idx++] = (value >> 0) & 0xFF;
    buf[idx++] = (value >> 8) & 0xFF;
    buf[idx++] = (value >> 16) & 0xFF;
    buf[idx++] = (value >> 24) & 0xFF;
    buf[idx++] = (value >> 32) & 0xFF;
    buf[idx++] = (value >> 40) & 0xFF;
    buf[idx++] = (value >> 48) & 0xFF;
    buf[idx++] = (value >> 56) & 0xFF;
    return idx;
}

static inline uint8_t hmi_buf_append_uint32(uint8_t *buf, uint32_t value) {
    uint8_t idx = 0;
    buf[idx++] = (value >> 0) & 0xFF;
    buf[idx++] = (value >> 8) & 0xFF;
    buf[idx++] = (value >> 16) & 0xFF;
    buf[idx++] = (value >> 24) & 0xFF;
    return idx;
}

static inline uint8_t hmi_buf_append_uint16(uint8_t *buf, uint16_t value) {
    uint8_t idx = 0;
    buf[idx++] = (value >> 0) & 0xFF;
    buf[idx++] = (value >> 8) & 0xFF;
    return idx;
}

static inline uint8_t hmi_buf_append_float(uint8_t *buf, float value) {
    uint8_t idx = 0;
    union {
        float f;
        uint32_t u;
    } u;
    u.f = value;
    idx += hmi_buf_append_uint32(buf, u.u);
    return idx;
}

static inline uint16_t hmi_buf_get_uint16(const uint8_t *buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static inline uint32_t hmi_buf_get_uint32(const uint8_t *buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static inline float hmi_buf_get_float(const uint8_t *buf) {
    union {
        float f;
        uint32_t u;
    } u;
    u.u = hmi_buf_get_uint32(buf);
    return u.f;
}

static inline uint64_t hmi_buf_get_uint64(const uint8_t *buf) {
    return (uint64_t)buf[0] | ((uint64_t)buf[1] << 8) | ((uint64_t)buf[2] << 16) | ((uint64_t)buf[3] << 24) |
           ((uint64_t)buf[4] << 32) | ((uint64_t)buf[5] << 40) | ((uint64_t)buf[6] << 48) | ((uint64_t)buf[7] << 56);
}
