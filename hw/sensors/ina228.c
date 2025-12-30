#include "ina228.h"

#include "../allocation.h"
#include "../pins.h"

#include "hardware/irq.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include <math.h>

// Helper to write a 16-bit register
static bool write_reg16(ina228_t *dev, uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;
    return i2c_write_blocking_until(
        dev->i2c,
        dev->addr,
        buf,
        3, 
        false,
        make_timeout_time_us(INA228_I2C_TIMEOUT_US)
    ) == 3;
}

// Helper to read a 16-bit register
static uint16_t read_reg16(ina228_t *dev, uint8_t reg) {
    uint8_t buf[2];
    // TODO - handle errors
    i2c_write_blocking_until(dev->i2c, dev->addr, &reg, 1, true, make_timeout_time_us(INA228_I2C_TIMEOUT_US));
    i2c_read_blocking_until(dev->i2c, dev->addr, buf, 2, false, make_timeout_time_us(INA228_I2C_TIMEOUT_US));
    return (buf[0] << 8) | buf[1];
}

// Helper to read a 24-bit register
static uint32_t read_reg24(ina228_t *dev, uint8_t reg) {
    uint8_t buf[3];
    // TODO - handle errors
    i2c_write_blocking_until(dev->i2c, dev->addr, &reg, 1, true, make_timeout_time_us(INA228_I2C_TIMEOUT_US));
    i2c_read_blocking_until(dev->i2c, dev->addr, buf, 3, false, make_timeout_time_us(INA228_I2C_TIMEOUT_US));
    return (buf[0] << 16) | (buf[1] << 8) | buf[2];
}

// Global pointer for ISR context (limitation: one INA228 per I2C instance/system)
static ina228_t *irq_ctx = NULL;

static void ina228_start_async_read(ina228_t *dev, uint8_t reg, uint8_t len);

static void ina228_irq_handler(void) {
    ina228_t *dev = irq_ctx;
    if (!dev) return;

    i2c_inst_t *i2c = dev->i2c;
    i2c_hw_t *hw = i2c_get_hw(i2c);

    // Clear interrupt
    uint32_t intr_stat = hw->intr_stat;
    if (intr_stat & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {
        // Read from FIFO
        while (hw->rxflr > 0) {
            uint32_t val = hw->data_cmd;
            if (dev->async_bytes_received < dev->async_bytes_expected) {
                dev->async_buf[dev->async_bytes_received++] = (uint8_t)(val & 0xFF);
            }
        }

        // Check if transaction complete
        if (dev->async_bytes_received >= dev->async_bytes_expected) {
            // Disable RX interrupt
            hw->intr_mask &= ~I2C_IC_INTR_MASK_M_RX_FULL_BITS;

            // Process data
            if (dev->async_reg == INA228_REG_CURRENT && dev->async_bytes_expected == 3) {
                uint32_t raw = (dev->async_buf[0] << 16) | (dev->async_buf[1] << 8) | dev->async_buf[2];
                int32_t signed_raw = (int32_t)raw;
                if (raw & 0x800000) signed_raw |= 0xFF000000;
                dev->current_a = (float)(signed_raw >> 4) * dev->current_lsb;
            } else if (dev->async_reg == INA228_REG_CHARGE && dev->async_bytes_expected == 5) {
                int64_t raw = ((int64_t)dev->async_buf[0] << 32) |
                              ((int64_t)dev->async_buf[1] << 24) |
                              ((int64_t)dev->async_buf[2] << 16) |
                              ((int64_t)dev->async_buf[3] << 8) |
                              (int64_t)dev->async_buf[4];
                
                // 40-bit sign extension
                if (raw & 0x8000000000LL) raw |= 0xFFFFFF0000000000LL;
                
                // Charge (C) = Charge_Register * Current_LSB * 16
                dev->charge_c = (double)raw * (double)dev->current_lsb * 16.0;
            }

            dev->async_busy = false;
        }
    }
    
    // Clear other interrupts if any (TX_ABRT etc)
    if (intr_stat & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        hw->clr_tx_abrt;
        dev->async_busy = false; // Abort
    }
}

static void ina228_start_async_read(ina228_t *dev, uint8_t reg, uint8_t len) {
    dev->async_reg = reg;
    dev->async_bytes_expected = len;
    dev->async_bytes_received = 0;
    dev->async_busy = true;

    i2c_inst_t *i2c = dev->i2c;
    i2c_hw_t *hw = i2c_get_hw(i2c);

    // Enable RX interrupt
    hw->intr_mask |= I2C_IC_INTR_MASK_M_RX_FULL_BITS;

    // Write Register Address (Write mode, Start)
    // Bit 8: CMD (0=Write, 1=Read)
    // Bit 9: STOP
    // Bit 10: RESTART
    
    // 1. Send Register Address
    bool restart = true; // Always restart if we are chaining or just starting
    hw->data_cmd = (bool_to_bit(restart) << 10) | (0 << 9) | (0 << 8) | reg;

    // 2. Send Read Commands
    for (int i = 0; i < len; i++) {
        bool is_last = (i == len - 1);
        // Restart on first read byte to switch direction
        bool do_restart = (i == 0); 
        bool do_stop = is_last;
        
        hw->data_cmd = (bool_to_bit(do_restart) << 10) | (bool_to_bit(do_stop) << 9) | (1 << 8);
    }
}

bool ina228_init(ina228_t *dev, uint8_t addr, float shunt_resistor_ohms, float max_current_a) {
    // init i2c
    i2c_init(INA228_I2C, 100 * 1000);
    gpio_set_function(PIN_INA228_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_INA228_I2C_SCL, GPIO_FUNC_I2C);

    dev->i2c = INA228_I2C;
    dev->addr = addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;
    dev->sampling_enabled = false;
    dev->async_busy = false;

    // Calculate Current_LSB
    // Current_LSB = Max_Current / 2^19
    // Example: 10A / 524288 = 19uA. 
    // Let's pick a nice round number close to that if possible, or just use the calculated one.
    dev->current_lsb = max_current_a / 524288.0f;

    // Calculate SHUNT_CAL
    // SHUNT_CAL = 13107.2 * 10^6 * Current_LSB * R_SHUNT
    // The constant 13107.2e6 is for the INA228 specifically.
    float shunt_cal_val = 13107.2e6f * dev->current_lsb * dev->shunt_resistor_ohms;
    
    // Write SHUNT_CAL
    if(!write_reg16(dev, INA228_REG_SHUNT_CAL, (uint16_t)shunt_cal_val)) {
        // Didn't acknowledge, probably not connected
        return false;
    }
    
    // Configure ADC
    ina228_configure(dev);

    // Setup IRQ
    irq_ctx = dev;
    uint irq_num = (dev->i2c == i2c0) ? I2C0_IRQ : I2C1_IRQ;
    irq_set_exclusive_handler(irq_num, ina228_irq_handler);
    irq_set_enabled(irq_num, true);
    
    // Set interrupt threshold for RX FIFO (1 byte)
    i2c_get_hw(dev->i2c)->rx_tl = 0; 

    return true;
}

void ina228_configure(ina228_t *dev) {
    // Reset
    write_reg16(dev, INA228_REG_CONFIG, 0x8000);
    sleep_ms(10);

    // CONFIG register (0x00)
    // Default is fine for now, or set ADCRANGE if needed.
    // Bit 4: ADCRANGE (0 = +/- 163.84mV, 1 = +/- 40.96mV)
    // Let's assume default range (0) for higher current capability.
    write_reg16(dev, INA228_REG_CONFIG, 0x0000);

    // ADC_CONFIG register (0x01)
    // MODE (bits 15-12): 1111 = Continuous Bus Voltage, Shunt Voltage, and Temperature
    // VBUSCT (bits 11-9): Conversion time for VBUS
    // VSHCT (bits 8-6): Conversion time for VSHUNT
    // VTCT (bits 5-3): Conversion time for Temp
    // AVG (bits 2-0): Averaging count
    
    // Set to continuous mode, default conversion times (1052us), 16 averages
    // MODE=0xF, VBUSCT=4 (1052us), VSHCT=4 (1052us), VTCT=4 (1052us), AVG=2 (16) -> 0xFB49?
    // Let's use defaults but ensure continuous mode.
    // 0xFB68 -> Continuous all, 1052us, 64 averages.
    write_reg16(dev, INA228_REG_ADC_CONFIG, 0xFB68); 
}

float ina228_read_current(ina228_t *dev) {
    uint32_t raw = read_reg24(dev, INA228_REG_CURRENT);
    // Convert 24-bit signed to 32-bit signed
    int32_t signed_raw = (int32_t)raw;
    if (raw & 0x800000) {
        signed_raw |= 0xFF000000;
    }
    
    // Current = (raw >> 4) * Current_LSB
    // The bottom 4 bits are reserved/zero in some modes, but for INA228 the register is 24-bit 
    // and the value is 20-bit aligned to MSB?
    // Datasheet: "The value of the Current register is calculated by multiplying the decimal value by the Current_LSB."
    // The register is 24 bits. The 20-bit result is left-aligned?
    // Actually, INA228 datasheet says: "24-bit, 2's complement".
    // "The 4 LSBs are 0." -> So we can just use the full 24-bit value and divide by 16, or just multiply by LSB if LSB accounts for it.
    // Usually: Real Current = (Register Value / 16) * Current_LSB
    
    dev->current_a = (float)(signed_raw >> 4) * dev->current_lsb;

    return dev->current_a;
}

double ina228_read_charge(ina228_t *dev) {
    uint8_t buf[5];
    uint8_t reg = INA228_REG_CHARGE;
    
    i2c_write_blocking_until(dev->i2c, dev->addr, &reg, 1, true, make_timeout_time_us(INA228_I2C_TIMEOUT_US));
    i2c_read_blocking_until(dev->i2c, dev->addr, buf, 5, false, make_timeout_time_us(INA228_I2C_TIMEOUT_US));
    
    int64_t raw = ((int64_t)buf[0] << 32) |
                  ((int64_t)buf[1] << 24) |
                  ((int64_t)buf[2] << 16) |
                  ((int64_t)buf[3] << 8) |
                  (int64_t)buf[4];
    
    if (raw & 0x8000000000LL) raw |= 0xFFFFFF0000000000LL;
    
    dev->charge_c = (double)raw * (double)dev->current_lsb * 16.0;
    return dev->charge_c;
}

// Timer callback for periodic sampling
static bool ina228_timer_callback(struct repeating_timer *t) {
    ina228_t *dev = (ina228_t *)t->user_data;
    
    if (!dev->sampling_enabled) return false; // Stop timer

    if (!dev->async_busy) {
        ina228_start_async_read(dev, INA228_REG_CHARGE, 5);
    }
    
    return true; // Keep repeating
}

bool ina228_start_periodic_sampling(ina228_t *dev, int32_t interval_ms) {
    if (dev->sampling_enabled) return true;
    
    dev->sampling_enabled = true;
    return add_repeating_timer_ms(interval_ms, ina228_timer_callback, dev, &dev->timer);
}

void ina228_stop_periodic_sampling(ina228_t *dev) {
    if (dev->sampling_enabled) {
        cancel_repeating_timer(&dev->timer);
        dev->sampling_enabled = false;
    }
}
