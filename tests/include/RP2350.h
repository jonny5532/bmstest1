#include <stdint.h>

static inline int32_t __qadd(int32_t a, int32_t b){
    int64_t result = (int64_t)a + (int64_t)b;
    if (result > INT32_MAX) return INT32_MAX;
    if (result < INT32_MIN) return INT32_MIN;
    return (int32_t)result;
}
static inline int32_t __qsub(int32_t a, int32_t b) {
    int64_t result = (int64_t)a - (int64_t)b;
    if (result > INT32_MAX) return INT32_MAX;
    if (result < INT32_MIN) return INT32_MIN;
    return (int32_t)result;
}
static inline int32_t __qadd16(int16_t a, int16_t b) {
    int32_t result = (int32_t)a + (int32_t)b;
    if (result > INT16_MAX) return INT16_MAX;
    if (result < INT16_MIN) return INT16_MIN;
    return (int32_t)result;
}
static inline uint32_t __uqadd16(uint16_t a, uint16_t b) {
    uint32_t result = (uint32_t)a + (uint32_t)b;
    if (result > UINT16_MAX) return UINT16_MAX;
    return (uint32_t)result;
}
static inline uint32_t __uqsub16(uint16_t a, uint16_t b) {
    int32_t result = (int32_t)a - (int32_t)b;
    if (result < 0) return 0;
    return (uint32_t)result;
}
