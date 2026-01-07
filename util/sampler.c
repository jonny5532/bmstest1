#include "sampler.h"

#include "../lib/math.h"

void sampler_add(sampler_t* sampler, int32_t sample, uint16_t max_samples, uint16_t divide_shift) {
    sampler->accumulator = sadd_i32(sampler->accumulator, sample >> divide_shift);
    sampler->sample_count += 1;
    if (sample < sampler->cur_min_value || sampler->sample_count == 1) {
        sampler->cur_min_value = sample;
    }
    if (sample > sampler->cur_max_value || sampler->sample_count == 1) {
        sampler->cur_max_value = sample;
    }

    if(sampler->sample_count >= max_samples) {
        // finalize
        sampler->value = sampler->accumulator;
        sampler->min_value = sampler->cur_min_value;
        sampler->max_value = sampler->cur_max_value;
        
        sampler->timestamp = millis();
        
        // reset for next
        sampler->cur_min_value = 0;
        sampler->cur_max_value = 0;
        sampler->accumulator = 0;
        sampler->sample_count = 0;
    }
}

