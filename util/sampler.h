#pragma once

#include "../hw/time.h"

#include <stdint.h>

typedef struct {
    int32_t accumulator;
    // How much to prescale the accumulated value by (right shift)
    uint16_t sample_count;
    //uint16_t sample_max;
    int32_t cur_min_value;
    int32_t cur_max_value;

    int32_t value;
    int32_t min_value;
    int32_t max_value;
    millis_t timestamp;
} sampler_t;

void sampler_add(sampler_t* sampler, int32_t sample, uint16_t max_samples, uint16_t divide_shift);
