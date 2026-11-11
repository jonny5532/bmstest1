#pragma once

#include <stdbool.h>
#include <stdint.h>

// Each BMB has two NTC thermistor channels, A and B, in words 0 and 2 of the
// three words returned by 0x0D. The middle word is a voltage (seemingly the
// thermistor divider supply).
#define BMB3Y_THERMISTOR_CHANNELS 2
#define BMB3Y_TEMPS3_WORDS 3
#define BMB3Y_NTC_WORD(channel) ((channel) * 2)

// The thermistor curve covers -40 to 115C (raw 439 to 23291). A value outside
// that is probably a fault.
#define BMB3Y_NTC_RAW_MIN 439
#define BMB3Y_NTC_RAW_MAX 23291

// BMB3Y NTC thermistor: raw code -> tenths of a degree C.
//
// The raw code is the little-endian 16-bit word from a 0x0D read: word 0
// (bytes 0,1) is thermistor A, word 2 (bytes 4,5) is thermistor B. 0xFFFF
// means open/absent, and reads 300.0 C.
static inline int16_t bmb3y_ntc_raw_to_dC(uint16_t raw) {
    static const uint16_t ntc_raw[32] = {
          439,   572,   737,   942,  1193,  1498,  1863,  2295,  2800,  3385,  4051,
         4801,  5633,  6542,  7521,  8560,  9646, 10765, 11901, 13038, 14163, 15261,
        16320, 17332, 18289, 19186, 20023, 20795, 21507, 22158, 22752, 23291,
    };

    if(raw < 439) {
        return (int16_t)(-600 + (200u * raw) / 439u);
    }
    if(raw >= 23291) {
        return (int16_t)(1150 + (1850u * (raw - 23291u)) / 42244u);
    }

    // 31 segments, and raw < 23291 keeps i <= 30. A fixed 5-step binary search
    // for ntc_raw[i] <= raw < ntc_raw[i+1], which unrolls to conditional moves.
    unsigned i = 0;
    for(unsigned step = 16; step; step >>= 1) {
        if(raw >= ntc_raw[i + step]) {
            i += step;
        }
    }

    unsigned x0 = ntc_raw[i];
    return (int16_t)(-400 + 50 * (int)i +
                     (int)((50u * (raw - x0)) / (ntc_raw[i + 1] - x0)));
}

// Whether a thermistor's raw code is on the curve, rather than open or shorted
static inline bool bmb3y_ntc_raw_valid(uint16_t raw) {
    return raw >= BMB3Y_NTC_RAW_MIN && raw <= BMB3Y_NTC_RAW_MAX;
}

// BMB die temperature: raw (0x0E word 1) -> tenths of a degree C. Affine and
// Kelvin-referenced, raw * 2.1778 - 273.1C.
static inline int16_t bmb3y_die_raw_to_dC(uint16_t raw) {
    return (int16_t)(((int32_t)raw * 21778) / 10000 - 2731);
}

// The index counts only the thermistors that are actually present, in chain
// order with A before B.
static inline int bmb3y_thermistor_index(const uint8_t *presence, int module, int channel) {
    if(((presence[module] >> channel) & 1) == 0) {
        return -1;
    }
    int index = __builtin_popcount(presence[module] & ((1u << channel) - 1));
    for(int m = 0; m < module; m++) {
        index += __builtin_popcount(presence[m] & 0x3);
    }
    return index;
}
