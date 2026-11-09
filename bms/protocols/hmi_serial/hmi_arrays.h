#pragma once

// Paged reads and writes of the model's arrays for the HMI (HMI_MSG_READ_ARRAY,
// HMI_MSG_WRITE_ARRAY). Kept apart from hmi_serial.c, and free of the hardware,
// so the host tests can build the responses.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct bms_model bms_model_t;

// Array IDs
#define HMI_ARRAY_CELL_VOLTAGES   1 // int16 (mV)
#define HMI_ARRAY_BALANCING_TIMES 2 // int16 (balancing periods remaining)
#define HMI_ARRAY_MODULE_TEMPS    3 // float (C)
#define HMI_ARRAY_RAW_TEMPS       4 // uint16, raw BMB temperature readings

// Writable. The EKF tuning starts from the profile at each boot, so a write
// lasts until the next reboot; the OCV learner is saved to NVM. Neither has
// any elements until it has started: the EKF at its first voltage reading,
// the learner at its first settled full charge, or HMI_REG_OCV_LEARN_S_FULL.
#define HMI_ARRAY_EKF_TUNING      5 // double, HMI_EKF_TUNING_* (see profiles/chemistry/*.h)
#define HMI_ARRAY_OCV_LEARN_CORR  6 // double (V), learned OCV correction per bin, from full down
#define HMI_ARRAY_OCV_LEARN_COV   7 // double (V^2), covariance of the corrections, row by row
#define HMI_ARRAY_OCV_LEARN_CYCLES 8 // float, decaying count of full charge cycles with data per bin

// Read-only, with no elements until the EKF has started
#define HMI_ARRAY_EKF_STATE       9 // double, HMI_EKF_STATE_*
#define HMI_ARRAY_EKF_COV        10 // double, covariance of the state, row by row (3x3)

// Read-only OCV tables, 101 points from 0% to 100% SOC in 1% steps
#define HMI_ARRAY_OCV_FACTORY    11 // float (V/cell), the chemistry's factory curve
#define HMI_ARRAY_OCV_LEARNED    12 // float (V/cell), factory + learned correction; no elements
                                    // until learning starts. Only used by the EKF with
                                    // OCV_LEARN_TABLE (NMC); LFP learns it for capacity only.

// Read-only, one element per BMB, in chain order; no elements until the first
// read (the sample time is that of the last read, re-read every minute)
#define HMI_ARRAY_MODULE_IDS     13 // uint16, the BMB's 11-bit module ID code, or 0xFFFF
                                    // (BMB3Y_MODULE_ID_NONE) if its last read failed the
                                    // CRC or the ID's own check bits

// Elements of HMI_ARRAY_EKF_TUNING
#define HMI_EKF_TUNING_Q_AH           0 // EKF_Q_AH, >= 0
#define HMI_EKF_TUNING_Q_OFFSET       1 // EKF_Q_OFFSET, >= 0
#define HMI_EKF_TUNING_Q_BIAS         2 // EKF_Q_BIAS, >= 0
#define HMI_EKF_TUNING_R_V            3 // EKF_R_V, > 0
#define HMI_EKF_TUNING_MIN_OCV_SLOPE  4 // EKF_MIN_OCV_SLOPE, >= 0
#define HMI_EKF_TUNING_KNEE_OCV_SLOPE 5 // EKF_KNEE_OCV_SLOPE, >= 0
#define HMI_EKF_TUNING_R_OHM          6 // EKF_R_OHM, >= 0
#define HMI_EKF_TUNING_OFFSET_TAU_S   7 // EKF_OFFSET_TAU_S, > 0
#define HMI_EKF_TUNING_COUNT          8

// Elements of HMI_ARRAY_EKF_STATE (see ekf_t)
#define HMI_EKF_STATE_AH_USED  0 // Ah used from full
#define HMI_EKF_STATE_V_OFFSET 1 // V the OCV curve and resistance don't explain
#define HMI_EKF_STATE_BIAS_A   2 // current sensor bias (A)
#define HMI_EKF_STATE_COUNT    3

// The encoding byte of a response is the elements' HMI_TYPE_*, each element in
// that type's usual little-endian form. The top bit (0x80) is reserved for a
// packed form of the type, should one be needed (see hmi_serial.h, 2.9).

// Encoding of an unknown array, which has no elements
#define HMI_ARRAY_ENCODING_NONE 0x00

// Bytes before the elements in a READ_ARRAY_RESPONSE
#define HMI_ARRAY_RESPONSE_HEADER_LEN 13

// Bytes before the elements in a WRITE_ARRAY
#define HMI_ARRAY_WRITE_HEADER_LEN 7

// Build a READ_ARRAY_RESPONSE in buf holding as many elements from start
// onwards as fit in capacity bytes. Returns the response length.
size_t hmi_build_read_array_response(uint8_t *buf, size_t capacity, uint8_t device_address,
                                     const bms_model_t *model, uint8_t array_id, uint16_t start);

// Apply a WRITE_ARRAY message (msg[0] is the message type) to the model. The
// write is all or nothing: it is refused, changing nothing, if the array isn't
// writable or has no elements yet, the encoding isn't the array's, it runs past
// the end of the array or the message, or any value is out of range (non-finite
// floats always are). Returns whether it was applied.
bool hmi_apply_write_array(bms_model_t *model, const uint8_t *msg, size_t len);
