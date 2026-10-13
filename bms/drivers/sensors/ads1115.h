#pragma once

#include "sys/time/time.h"
#include "lib/sampler.h"
#include "config/limits.h"

#include "hardware/i2c.h"
#include <stdint.h>
#include <stdbool.h>

// How many scan cycles the sampler accumulates before publishing its averaged
// value and min/max range. Freshness timestamps and the filtered values that
// the application consumes update every cycle (~85ms), independent of this.
#define ADS1115_OVERSAMPLING 4

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

/* Logical channels:

ADC A (0x48), all board revs:
0: AIN0-AIN1 (battery voltage)
1: AIN2-AIN3 (output/FC voltage)
2: AIN1-AIN3 (voltage across negative path)
3: AIN0-AIN3 (battery positive to output negative)
4: AIN0 single (battery pos, isolation)
5: AIN1 single (battery neg, isolation)

ADC B (0x49), rev1 boards:
6: AIN2-AIN3 (bat pos - link pos = across Link Positive contactor)
7: AIN0-AIN3 (link neg - link pos = negated link voltage)
8: AIN1-AIN3 (out pos - link pos = drop across F4 fuse/jumper)
9: AIN1 single (out pos, diagnostics)
*/

#ifdef HAS_ADS1115_SECONDARY
#define ADS1115_CHANNEL_COUNT 10
#else
#define ADS1115_CHANNEL_COUNT 6
#endif

extern sampler_t samples[ADS1115_CHANNEL_COUNT];
extern float filtered_samples[ADS1115_CHANNEL_COUNT];
extern float sample_deviations[ADS1115_CHANNEL_COUNT];

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    bool present;
} ads1115_t;

// Device handles owned by the driver
extern ads1115_t ads1115_dev;   // ADC A (0x48)
#ifdef HAS_ADS1115_SECONDARY
extern ads1115_t ads1115_dev_b; // ADC B (0x49)
#endif

bool ads1115_init(ads1115_t *dev, uint8_t addr);
int16_t ads1115_get_sample_range(int channel);
millis_t ads1115_get_sample_millis(int channel);
void ads1115_start_calibration(uint16_t num_samples);
bool ads1115_calibration_finished(void);
int32_t ads1115_get_calibration(int channel);

static inline int64_t div_round_closest(const int64_t n, const int64_t d)
{
  return ((n < 0) == (d < 0)) ? ((n + d/2)/d) : ((n - d/2)/d);
}

static inline int32_t ads1115_scaled_sample(int channel, int32_t multiplier) {
    // Add biases to stop the integer division from flooring the result
    int32_t ret = (int32_t)div_round_closest(
        (int64_t)samples[channel].value * div_round_closest(multiplier, ADS1115_OVERSAMPLING),
        4096
    );
    return ret;
}
static inline int32_t ads1115_scaled_sample_range(int channel, int32_t full_scale_mv) {
    return (int32_t)(
        (int64_t)(samples[channel].max_value - samples[channel].min_value) * full_scale_mv / 32768
    );
}

static inline float ads1115_float_sample(int channel, float multiplier) {
    return filtered_samples[channel] * multiplier;
}

static inline float ads1115_float_deviation(int channel, float multiplier) {
    return sample_deviations[channel] * multiplier;
}
