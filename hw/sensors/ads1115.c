#include "ads1115.h"
#include "../allocation.h"
#include "../pins.h"
#include "../model.h"

#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Global pointer for ISR context
static ads1115_t *ads_irq_ctx = NULL;

static void ads1115_i2c_write_async(ads1115_t *dev, uint8_t reg, uint16_t value);
static void ads1115_i2c_read_async(ads1115_t *dev, uint8_t reg);
static int64_t ads1115_conversion_timer_callback(alarm_id_t id, void *user_data);
static bool ads1115_periodic_timer_callback(struct repeating_timer *t);

static void ads1115_internal_irq_handler(void) {
    if (ads_irq_ctx) {
        ads1115_irq_handler(ads_irq_ctx);
    }
}

int16_t ads1115_samples[5];
millis_t ads1115_sample_millis[5];

bool ads1115_init(ads1115_t *dev, uint8_t addr) {
    dev->i2c = ADS1115_I2C; // Based on pins 18, 19
    dev->addr = addr;
    dev->busy = false;
    dev->state = ADS1115_STATE_IDLE;
    dev->current_channel = 0;

    // Initialize I2C if it hasn't been initialized yet
    // Note: i2c_init is idempotent if called with same frequency? 
    // Actually it resets the hardware, so be careful if other devices are on the same bus.
    // But here we assume we own the bus or it's the first init.
    i2c_init(dev->i2c, 400 * 1000);
    gpio_set_function(PIN_ADS1115_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_ADS1115_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_ADS1115_I2C_SDA);
    gpio_pull_up(PIN_ADS1115_I2C_SCL);

    // Test read to check if chip is present
    uint8_t reg = ADS1115_REG_CONFIG;
    uint8_t test_buf[2];
    if (i2c_write_blocking(dev->i2c, dev->addr, &reg, 1, true) != 1) {
        return false;
    }
    if (i2c_read_blocking(dev->i2c, dev->addr, test_buf, 2, false) != 2) {
        return false;
    }

    // Set target address for subsequent manual HW access if needed
    i2c_get_hw(dev->i2c)->tar = dev->addr;

    // Set IRQ mask to only things we care about
    i2c_get_hw(dev->i2c)->intr_mask = I2C_IC_INTR_MASK_M_TX_ABRT_BITS;

    ads_irq_ctx = dev;
    uint irq_num = (dev->i2c == i2c0) ? I2C0_IRQ : I2C1_IRQ;
    irq_set_exclusive_handler(irq_num, ads1115_internal_irq_handler);
    irq_set_enabled(irq_num, true);

    // Set interrupt thresholds
    i2c_get_hw(dev->i2c)->rx_tl = 0; // Interrupt when 1 byte in RX FIFO
    i2c_get_hw(dev->i2c)->tx_tl = 0; // Interrupt when TX FIFO is empty

    // Start periodic sampling every 100ms
    static struct repeating_timer timer;
    add_repeating_timer_ms(100, ads1115_periodic_timer_callback, dev, &timer);

    return true;
}

static void ads1115_start_conversion(ads1115_t *dev, int channel) {
    uint16_t config = ADS1115_CONFIG_OS_SINGLE | 
                      //ADS1115_CONFIG_PGA_4_096V |
                      //ADS1115_CONFIG_PGA_2_048V | 
                      ADS1115_CONFIG_MODE_SINGLE | 
                      ADS1115_CONFIG_DR_128SPS | 
                      ADS1115_CONFIG_COMP_QUE_NONE;
    
    switch (channel) {
        case 0: 
            // ADC0 - ADC1 (battery voltage)
            config |= ADS1115_CONFIG_PGA_1_024V | ADS1115_CONFIG_MUX_DIFF_0_1; 
            break;
        case 1: 
            // ADC2 - ADC3 (output terminal voltage)
            config |= ADS1115_CONFIG_PGA_1_024V | ADS1115_CONFIG_MUX_DIFF_2_3; 
            break;
        case 2: 
            // ADC1 - ADC3 (voltage across negative contactor)
            config |= ADS1115_CONFIG_PGA_1_024V | ADS1115_CONFIG_MUX_DIFF_1_3; 
            break;
        case 3: 
            // ADC0 single-ended (Bat+)
            config |= ADS1115_CONFIG_PGA_4_096V | ADS1115_CONFIG_MUX_SINGLE_0;
            break;
        case 4:
            // ADC2 single-ended (Output+)
            config |= ADS1115_CONFIG_PGA_4_096V | ADS1115_CONFIG_MUX_SINGLE_2;
            break;
    }

    dev->state = ADS1115_STATE_WRITE_CONFIG;
    ads1115_i2c_write_async(dev, ADS1115_REG_CONFIG, config);
}

void ads1115_start_sampling(ads1115_t *dev) {
    if (dev->busy) return;
    dev->busy = true;
    dev->current_channel = 0;
    ads1115_start_conversion(dev, 0);
}

static void ads1115_i2c_write_async(ads1115_t *dev, uint8_t reg, uint16_t value) {
    i2c_hw_t *hw = i2c_get_hw(dev->i2c);
    dev->async_buf[0] = reg;
    dev->async_buf[1] = (value >> 8) & 0xFF;
    dev->async_buf[2] = value & 0xFF;
    dev->async_idx = 0;
    dev->async_len = 3;

    // Enable TX empty interrupt to start sending
    hw->intr_mask |= I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;
}

static void ads1115_i2c_read_async(ads1115_t *dev, uint8_t reg) {
    i2c_hw_t *hw = i2c_get_hw(dev->i2c);
    
    // First write the register pointer (with restart)
    // We do this synchronously for simplicity of the state machine, 
    // or we could make it part of the async flow.
    // Given the requirement for "interrupt for both reads and writes", 
    // let's make the pointer write async too if we want to be strict.
    // But usually, writing 1 byte is fast.
    
    dev->async_buf[0] = reg;
    dev->async_idx = 0;
    dev->async_len = 1;
    dev->state = ADS1115_STATE_READ_CONVERSION_REG_PTR;
    
    hw->intr_mask |= I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;
}

static void store_battery_voltage(int16_t raw) {
    // TODO - check conversion factor
    model.battery_voltage_mV = (raw * 2048) / 32768; // in mV
    model.battery_voltage_millis = millis();
}

static void store_output_voltage(int16_t raw) {
    // TODO - check conversion factor
    model.output_voltage_mV = (raw * 2048) / 32768; // in mV
    model.output_voltage_millis = millis();
}

void ads1115_irq_handler(ads1115_t *dev) {
    i2c_hw_t *hw = i2c_get_hw(dev->i2c);
    uint32_t intr_stat = hw->intr_stat;

    if (intr_stat & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS) {
        if (dev->async_idx < dev->async_len) {
            bool stop = (dev->async_idx == dev->async_len - 1) && (dev->state != ADS1115_STATE_READ_CONVERSION_REG_PTR);
            // If we are writing the pointer for a read, we don't want a STOP, we want a RESTART next.
            hw->data_cmd = (stop << 9) | dev->async_buf[dev->async_idx++];
        } else {
            // Done writing
            hw->intr_mask &= ~I2C_IC_INTR_MASK_M_TX_EMPTY_BITS;
            
            if (dev->state == ADS1115_STATE_WRITE_CONFIG) {
                dev->state = ADS1115_STATE_WAIT_CONVERSION;
                // Wait for conversion to complete (128 SPS = 7.8ms, so 10ms is safe)
                add_alarm_in_ms(10, ads1115_conversion_timer_callback, dev, true);
            } else if (dev->state == ADS1115_STATE_READ_CONVERSION_REG_PTR) {
                // Now start the read part
                dev->state = ADS1115_STATE_READ_CONVERSION_DATA;
                dev->async_idx = 0;
                dev->async_len = 2;
                
                // Queue 2 read commands
                for (int i = 0; i < 2; i++) {
                    bool restart = (i == 0); // Restart on first byte to switch to read
                    bool stop = (i == 1);
                    hw->data_cmd = (restart << 10) | (stop << 9) | (1 << 8); // Restart, Stop, Read
                }
                hw->intr_mask |= I2C_IC_INTR_MASK_M_RX_FULL_BITS;
            }
        }
    }

    if (intr_stat & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        while (hw->rxflr > 0) {
            uint8_t val = (uint8_t)(hw->data_cmd & 0xFF);
            if (dev->async_idx < dev->async_len) {
                dev->async_buf[dev->async_idx++] = val;
            }
        }

        if (dev->async_idx >= dev->async_len) {
            hw->intr_mask &= ~I2C_IC_INTR_MASK_M_RX_FULL_BITS;
            
            if (dev->state == ADS1115_STATE_READ_CONVERSION_DATA) {
                ads1115_samples[dev->current_channel] = (dev->async_buf[0] << 8) | dev->async_buf[1];
                ads1115_sample_millis[dev->current_channel] = millis();
                
                dev->current_channel++;
                if (dev->current_channel < 5) {
                    ads1115_start_conversion(dev, dev->current_channel);
                } else {
                    dev->busy = false;
                    dev->state = ADS1115_STATE_IDLE;
                }
            }
        }
    }

    if (intr_stat & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        hw->clr_tx_abrt;
        dev->busy = false;
        dev->state = ADS1115_STATE_IDLE;
    }
}

static int64_t ads1115_conversion_timer_callback(alarm_id_t id, void *user_data) {
    (void)id;
    ads1115_t *dev = (ads1115_t *)user_data;
    ads1115_i2c_read_async(dev, ADS1115_REG_CONVERSION);
    return 0;
}

static bool ads1115_periodic_timer_callback(struct repeating_timer *t) {
    ads1115_t *dev = (ads1115_t *)t->user_data;
    ads1115_start_sampling(dev);
    return true;
}

int16_t ads1115_get_sample(int channel) {
    return ads1115_samples[channel];
}

millis_t ads1115_get_sample_millis(int channel) {
    return ads1115_sample_millis[channel];
}