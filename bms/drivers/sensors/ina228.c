#include "ina228.h"

#include "config/allocations.h"
#include "config/pins.h"
#include "app/model.h"

#include "hardware/irq.h"
#include "hardware/i2c.h"

#include <stdio.h>
#include <math.h>

// INA228 CONFIG register bits
#define INA228_CONFIG_RST          (1 << 15)
#define INA228_CONFIG_RSTACC       (1 << 14)
#define INA228_CONFIG_CONVDLY(x)   (((x) & 0xFF) << 6)
#define INA228_CONFIG_TEMPCOMP     (1 << 5)
#define INA228_CONFIG_ADCRANGE     (1 << 4)  // 0 = ±163.84 mV, 1 = ±40.96 mV

// ADC_CONFIG register bits
#define INA228_ADC_MODE_CONT_SHUNT  0xa
#define INA228_ADC_MODE_CONT_ALL   0xf


#define INA228_ADC_VBUSCT(x)       (((x) & 0x7) << 9)
#define INA228_ADC_VSHCT(x)        (((x) & 0x7) << 6)
#define INA228_ADC_VTCT(x)         (((x) & 0x7) << 3)
#define INA228_ADC_AVG(x)          ((x) & 0x7)

// Conversion time settings (samples)
#define INA228_CT_50US    0
#define INA228_CT_84US    1
#define INA228_CT_150US   2
#define INA228_CT_280US   3
#define INA228_CT_540US   4
#define INA228_CT_1052US  5
#define INA228_CT_2074US  6
#define INA228_CT_4120US  7

// Averaging settings
#define INA228_AVG_1      0
#define INA228_AVG_4      1
#define INA228_AVG_16     2
#define INA228_AVG_64     3
#define INA228_AVG_128    4
#define INA228_AVG_256    5
#define INA228_AVG_512    6
#define INA228_AVG_1024   7

// Global variables for storing current measurements
static int32_t ina228_current_raw = 0;
static millis_t ina228_current_millis = 0;
static int64_t ina228_charge_raw = 0;
static millis_t ina228_charge_millis = 0;

// Helper function to write a 16-bit register
static bool ina228_write_reg16(ina228_t *dev, uint8_t reg, uint16_t value) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (value >> 8) & 0xFF;
    buf[2] = value & 0xFF;
    
    int result = i2c_write_timeout_us(dev->i2c, dev->addr, buf, 3, false, INA228_I2C_TIMEOUT_US);
    return result == 3;
}

// Helper function to read a 20-bit register (3 bytes)
static bool ina228_read_reg20(ina228_t *dev, uint8_t reg, int32_t *value) {
    uint8_t buf[3];
    
    // Write register address
    int result = i2c_write_timeout_us(dev->i2c, dev->addr, &reg, 1, true, INA228_I2C_TIMEOUT_US);
    if (result != 1) {
        return false;
    }
    
    // Read 3 bytes
    result = i2c_read_timeout_us(dev->i2c, dev->addr, buf, 3, false, INA228_I2C_TIMEOUT_US);
    if (result != 3) {
        return false;
    }
    
    // Combine bytes - INA228 uses 20-bit signed values in MSB first format
    int32_t raw = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    
    // Sign extend from 20 bits to 32 bits
    if (raw & 0x80000) {
        raw |= 0xFFF00000;
    }
    
    *value = raw;
    return true;
}

// Helper function to read a 40-bit register (5 bytes)
static bool ina228_read_reg40(ina228_t *dev, uint8_t reg, int64_t *value) {
    uint8_t buf[5];
    
    // Write register address
    int result = i2c_write_timeout_us(dev->i2c, dev->addr, &reg, 1, true, INA228_I2C_TIMEOUT_US);
    if (result != 1) {
        return false;
    }
    
    // Read 5 bytes
    result = i2c_read_timeout_us(dev->i2c, dev->addr, buf, 5, false, INA228_I2C_TIMEOUT_US);
    if (result != 5) {
        return false;
    }
    
    // Combine bytes
    int64_t raw = ((int64_t)buf[0] << 32) | ((int64_t)buf[1] << 24) | 
                  ((int64_t)buf[2] << 16) | ((int64_t)buf[3] << 8) | (int64_t)buf[4];
    
    *value = raw;
    return true;
}

// Helper function to read a 16-bit register
static bool ina228_read_reg16(ina228_t *dev, uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    
    // Write register address
    int result = i2c_write_timeout_us(dev->i2c, dev->addr, &reg, 1, true, INA228_I2C_TIMEOUT_US);
    if (result != 1) {
        return false;
    }
    
    // Read 2 bytes
    result = i2c_read_timeout_us(dev->i2c, dev->addr, buf, 2, false, INA228_I2C_TIMEOUT_US);
    if (result != 2) {
        return false;
    }
    
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

bool ina228_init(ina228_t *dev, uint8_t i2c_addr, float shunt_resistor_ohms, float max_current_a) {
    dev->i2c = INA228_I2C;
    dev->addr = i2c_addr;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;
    dev->async_busy = false;
    
    // Initialize I2C if not already done
    i2c_init(dev->i2c, 400 * 1000);  // 400 kHz
    gpio_set_function(PIN_INA228_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_INA228_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(PIN_INA228_I2C_SDA);
    gpio_pull_up(PIN_INA228_I2C_SCL);
    
    // Read and verify manufacturer ID (should be 0x5449 = "TI")
    uint16_t mfg_id;
    if (!ina228_read_reg16(dev, INA228_REG_MANUFACTURER_ID, &mfg_id)) {
        printf("INA228: Failed to read manufacturer ID\n");
        return false;
    }
    
    if (mfg_id != 0x5449) {
        printf("INA228: Invalid manufacturer ID: 0x%04X (expected 0x5449)\n", mfg_id);
        return false;
    }
    
    // Read device ID (should be 0x228)
    uint16_t dev_id;
    if (!ina228_read_reg16(dev, INA228_REG_DEVICE_ID, &dev_id)) {
        printf("INA228: Failed to read device ID\n");
        return false;
    }
    
    uint16_t expected_id = (0x228 << 4);  // Device ID is in upper 12 bits
    if ((dev_id & 0xFFF0) != expected_id) {
        printf("INA228: Invalid device ID: 0x%04X (expected 0x%04X)\n", dev_id, expected_id);
        return false;
    }
    
    printf("INA228: Detected (MFG=0x%04X, DEV=0x%04X)\n", mfg_id, dev_id);
    
    // Perform a software reset
    if (!ina228_write_reg16(dev, INA228_REG_CONFIG, INA228_CONFIG_RST)) {
        printf("INA228: Reset failed\n");
        return false;
    }
    
    // Wait for reset to complete
    sleep_ms(2);
    
    // Configure the device
    ina228_configure(dev);
    
    return true;
}

void ina228_configure(ina228_t *dev) {
    // Calculate SHUNT_CAL value
    // Current_LSB = Max Current / 2^19
    // For 100A max: Current_LSB ≈ 0.0001907 A = 190.7 µA
    // SHUNT_CAL = 13107.2 × 10^6 × Current_LSB × R_shunt
    
    // Use a reasonable current LSB
    // For max_current_a = 100A, we want good resolution
    // Current_LSB = 100 / 524288 = 0.000190735 A/LSB
    
    // Calculate current_lsb (in A/LSB) to use full 20-bit range efficiently
    dev->current_lsb = 100.0f / 524288.0f;  // For 100A max current
    dev->current_lsb = 0.001f;
    
    // SHUNT_CAL = 13107.2 × 10^6 × Current_LSB × R_shunt
    float shunt_cal_float = 13107.2e6f * dev->current_lsb * dev->shunt_resistor_ohms;
    uint16_t shunt_cal = (uint16_t)shunt_cal_float;
    // 332 to read mA, *4 due to adcrange, /4 because we want in 0.25mA units instead
    shunt_cal = 332*4/4;
    
    printf("INA228: Current LSB = %.6f A/LSB\n", dev->current_lsb);
    printf("INA228: SHUNT_CAL = %u (0x%04X)\n", shunt_cal, shunt_cal);
    
    // Write SHUNT_CAL register
    if (!ina228_write_reg16(dev, INA228_REG_SHUNT_CAL, shunt_cal)) {
        printf("INA228: Failed to write SHUNT_CAL\n");
        return;
    }
    
    // Configure ADC: continuous mode, all measurements
    // Use 540µs conversion time for balance between speed and noise
    // Use 4x averaging for good noise rejection
    uint16_t adc_config = (INA228_ADC_MODE_CONT_SHUNT << 12) |
                          INA228_ADC_VBUSCT(INA228_CT_2074US) |
                          INA228_ADC_VSHCT(INA228_CT_2074US) |
                          INA228_ADC_VTCT(INA228_CT_2074US) |
                          INA228_ADC_AVG(INA228_AVG_256);
    
    if (!ina228_write_reg16(dev, INA228_REG_ADC_CONFIG, adc_config)) {
        printf("INA228: Failed to write ADC_CONFIG\n");
        return;
    }
    
    // Configure main CONFIG register
    uint16_t config = 0x0010; 
    
    if (!ina228_write_reg16(dev, INA228_REG_CONFIG, config)) {
        printf("INA228: Failed to write CONFIG\n");
        return;
    }

    // Configure Diagnostic flags
    // uint16_t diag_alrt = 0x0001; // No alerts for now
    // if (!ina228_write_reg16(dev, INA228_REG_DIAG_ALRT, diag_alrt)) {
    //     printf("INA228: Failed to write DIAG_ALRT\n");
    //     return;
    // }
    
    printf("INA228: Configuration complete\n");
}

static int32_t div_round_closest(const int32_t n, const int32_t d)
{
  return ((n < 0) == (d < 0)) ? ((n + d/2)/d) : ((n - d/2)/d);
}

//#define SAMPLING_PERIOD_SMOOTHING 2048
uint32_t last_sample_us = 0;
//uint32_t average_sampling_period_us = 530000; //530944; // Initial estimate based on INA228 datasheet
float average_sampling_period_us = 530307.0f; // Initial estimate based on INA228 datasheet

// Read current from the INA228 (blocking)
bool ina228_read_current_blocking(ina228_t *dev) {
    int32_t current_raw;
    int32_t current_corrected;
    
    uint16_t diag_alert;
    if (!ina228_read_reg16(dev, INA228_REG_DIAG_ALRT, &diag_alert)) {
        printf("INA228: Failed to read DIAG_ALRT\n");
        return false;
    }

    if (!ina228_read_reg20(dev, INA228_REG_CURRENT, &current_raw)) {
        printf("INA228: Failed to read CURRENT\n");
        return false;
    }
    
    current_corrected = current_raw - model.current_offset;
    // Positive current means charging.
    model.current_mA = div_round_closest(current_corrected, 4);

    // Was a new conversion
    if(diag_alert & 0x0002) {
        model.current_millis = millis();

        //printf("raw current: %d\n", current_raw);

        uint32_t now_us = time_us_32();
        uint32_t elapsed_us = now_us - last_sample_us;

        // TODO: do we care about this?
        if(last_sample_us != 0) {
            average_sampling_period_us = (average_sampling_period_us * 0.99999f) + ((float)elapsed_us * 0.00001f);
        }

        // average_sampling_period_us = (average_sampling_period_us * (SAMPLING_PERIOD_SMOOTHING - 1) + elapsed_us) / SAMPLING_PERIOD_SMOOTHING;
        last_sample_us = now_us;
        //printf("avg: %.2f us\n", average_sampling_period_us);

        // Is a new conversion, update charge
        model.charge_raw += (int64_t)current_corrected;
        model.charge_millis = model.current_millis;

        // charge_raw is in units equivalent to 0.132736mC (0.25mA LSB, 530.944ms per sample)

        // We sample at the same rate as the INA228 conversions, which has a
        // clock accurate to 1% - we would do better to use the crystal instead,
        // but the jitter would probably outweigh the accuracy improvement.

        dev->null_accumulator += current_raw;
        dev->null_counter++;
    }
    
    return true;
}

// Read charge accumulator from the INA228 (blocking)
bool ina228_read_charge(ina228_t *dev) {
    int64_t charge_raw;
    
    if (!ina228_read_reg40(dev, INA228_REG_CHARGE, &charge_raw)) {
        return false;
    }
    
    // Store in global variables
    ina228_charge_raw = charge_raw;
    ina228_charge_millis = millis();
    
    // Update model
    model.charge_raw = charge_raw;
    model.charge_millis = ina228_charge_millis;
    
    return true;
}

// Read shunt voltage (blocking)
bool ina228_read_shunt_voltage(ina228_t *dev, float *voltage_mv) {
    int32_t vshunt_raw;
    
    if (!ina228_read_reg20(dev, INA228_REG_VSHUNT, &vshunt_raw)) {
        return false;
    }
    
    // VSHUNT LSB = 312.5 nV for ±163.84 mV range
    *voltage_mv = (float)vshunt_raw * 312.5e-6f;  // Convert to mV
    
    return true;
}

// Read bus voltage (blocking)
bool ina228_read_bus_voltage(ina228_t *dev, float *voltage_mv) {
    int32_t vbus_raw;
    
    if (!ina228_read_reg20(dev, INA228_REG_VBUS, &vbus_raw)) {
        return false;
    }
    
    // VBUS LSB = 195.3125 µV
    *voltage_mv = (float)vbus_raw * 0.1953125f;  // Convert to mV
    
    return true;
}

// Getter functions
int32_t ina228_get_current_raw() {
    return ina228_current_raw;
}

millis_t ina228_get_current_millis() {
    return ina228_current_millis;
}

int64_t ina228_get_charge_raw() {
    return ina228_charge_raw;
}

millis_t ina228_get_charge_millis() {
    return ina228_charge_millis;
}
