#include <stdint.h>

#include "RP2350.h"

// Saturating add for int32_t
static inline int32_t sadd_i32(int32_t a, int32_t b) {
    return __qadd(a, b);
}

// Saturating subtract for int32_t
static inline int32_t ssub_i32(int32_t a, int32_t b) {
    return __qsub(a, b);
}


// Saturating add for int16_t
static inline int16_t sadd_i16(int16_t a, int16_t b) {
    return __qadd16(a, b);
}

// Saturating add for uint16_t
static inline uint16_t sadd_u16(uint16_t a, uint16_t b) {
    return (uint16_t)__uqadd16(a, b);
}