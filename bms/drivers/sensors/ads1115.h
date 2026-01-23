#pragma once

#include "sys/time/time.h"
#include "lib/sampler.h"

#include "hardware/i2c.h"
#include <stdint.h>
#include <stdbool.h>

#define ADS1115_OVERSAMPLING 8

#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01
#define ADS1115_REG_LO_THRESH  0x02
#define ADS1115_REG_HI_THRESH  0x03

#define ADS1115_CONFIG_OS_SINGLE    (1 << 15)
#define ADS1115_CONFIG_MUX_DIFF_0_1 (0x0 << 12)
#define ADS1115_CONFIG_MUX_DIFF_0_3 (0x1 << 12)
#define ADS1115_CONFIG_MUX_DIFF_1_3 (0x2 << 12)
#define ADS1115_CONFIG_MUX_DIFF_2_3 (0x3 << 12)
#define ADS1115_CONFIG_MUX_SINGLE_0 (0x4 << 12)
#define ADS1115_CONFIG_MUX_SINGLE_1 (0x5 << 12)
#define ADS1115_CONFIG_MUX_SINGLE_2 (0x6 << 12)
#define ADS1115_CONFIG_MUX_SINGLE_3 (0x7 << 12)
#define ADS1115_CONFIG_PGA_6_144V   (0x0 << 9)
#define ADS1115_CONFIG_PGA_4_096V   (0x1 << 9)
#define ADS1115_CONFIG_PGA_2_048V   (0x2 << 9)
#define ADS1115_CONFIG_PGA_1_024V   (0x3 << 9)
#define ADS1115_CONFIG_PGA_0_512V   (0x4 << 9)
#define ADS1115_CONFIG_PGA_0_256V   (0x5 << 9)
#define ADS1115_CONFIG_MODE_SINGLE  (1 << 8)
#define ADS1115_CONFIG_DR_128SPS    (0x4 << 5)
#define ADS1115_CONFIG_DR_250SPS    (0x5 << 5)
#define ADS1115_CONFIG_COMP_QUE_NONE (0x3)

extern sampler_t samples[5];

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    int current_channel;
    bool busy;

    int32_t cal_accumulator[4];
    uint16_t cal_samples_left[4];
    
    // Async state
    enum {
        ADS1115_STATE_IDLE,
        ADS1115_STATE_WRITE_CONFIG,
        ADS1115_STATE_WAIT_CONVERSION,
        ADS1115_STATE_READ_CONVERSION_REG_PTR,
        ADS1115_STATE_READ_CONVERSION_DATA
    } state;

    uint8_t async_buf[3];
    int async_idx;
    int async_len;
} ads1115_t;

bool ads1115_init(ads1115_t *dev, uint8_t addr);
void ads1115_start_sampling(ads1115_t *dev);
void ads1115_irq_handler(ads1115_t *dev);
int16_t ads1115_get_sample_range(int channel);
millis_t ads1115_get_sample_millis(int channel);
void ads1115_start_calibration(ads1115_t *dev, uint16_t num_samples);
bool ads1115_calibration_finished(ads1115_t *dev);
int32_t ads1115_get_calibration(ads1115_t *dev, int channel);

static inline int64_t div_round_closest(const int64_t n, const int64_t d)
{
  return ((n < 0) == (d < 0)) ? ((n + d/2)/d) : ((n - d/2)/d);
}

static inline int32_t ads1115_scaled_sample(int channel, int32_t multiplier) {//full_scale_mv) {
    // Add biases to stop the integer division from flooring the result
    int32_t ret = (int32_t)div_round_closest(
        (int64_t)samples[channel].value * div_round_closest(multiplier, ADS1115_OVERSAMPLING),
        4096
    );

    // int64_t numerator = ((int64_t)samples[channel].value * (multiplier/ADS1115_OVERSAMPLING + (ADS1115_OVERSAMPLING/2)));
    // int32_t ret = (numerator + (4096/2)) / 4096;

    // int32_t ret = (int32_t)(
    //     (samples[channel].value * (multiplier/ADS1115_OVERSAMPLING)) / 4096 // / (32768 * ADS1115_OVERSAMPLING)
    //     //(int64_t)samples[channel].value * full_scale_mv / (32768 * ADS1115_OVERSAMPLING)
    // );
    // if(channel==0) {
    //     printf("ADS1115 scaled sample ch0: raw=%d mul=%d out=%d\n",
    //         samples[0].value,
    //         multiplier,
    //         ret
    //     );
    // }
    return ret;
}
static inline int32_t ads1115_scaled_sample_range(int channel, int32_t full_scale_mv) {
    return (int32_t)(
        (int64_t)(samples[channel].max_value - samples[channel].min_value) * full_scale_mv / 32768
    );
}
