#pragma once

#include "../chip/time.h"

#include "hardware/i2c.h"
#include <stdint.h>
#include <stdbool.h>

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
#define ADS1115_CONFIG_COMP_QUE_NONE (0x3)

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    int current_channel;
    bool busy;
    
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
int16_t ads1115_get_sample(int channel);
millis_t ads1115_get_sample_millis(int channel);
