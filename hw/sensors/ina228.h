#ifndef HW_INA228_H
#define HW_INA228_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"

// INA228 Register Map
#define INA228_REG_CONFIG          0x00
#define INA228_REG_ADC_CONFIG      0x01
#define INA228_REG_SHUNT_CAL       0x02
#define INA228_REG_SHUNT_TEMP      0x03
#define INA228_REG_VSHUNT          0x04
#define INA228_REG_VBUS            0x05
#define INA228_REG_DIETEMP         0x06
#define INA228_REG_CURRENT         0x07
#define INA228_REG_POWER           0x08
#define INA228_REG_ENERGY          0x09
#define INA228_REG_CHARGE          0x0A
#define INA228_REG_DIAG_ALRT       0x0B
#define INA228_REG_SOVL            0x0C
#define INA228_REG_SUVL            0x0D
#define INA228_REG_BOVL            0x0E
#define INA228_REG_BUVL            0x0F
#define INA228_REG_TEMP_LIMIT      0x10
#define INA228_REG_PWR_LIMIT       0x11
#define INA228_REG_MANUFACTURER_ID 0x3E
#define INA228_REG_DEVICE_ID       0x3F

#define INA228_I2C_TIMEOUT_US     10000

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    float current_lsb;
    float shunt_resistor_ohms;
    
    // Latest readings
    volatile float current_a;
    volatile float power_w;
    volatile float temp_c;
    volatile double charge_c;
    
    // Internal state for periodic sampling
    struct repeating_timer timer;
    bool sampling_enabled;

    // Async state
    volatile bool async_busy;
    uint8_t async_reg;
    uint8_t async_buf[5];
    uint8_t async_bytes_expected;
    volatile uint8_t async_bytes_received;
} ina228_t;

/**
 * @brief Initialize the INA228 driver structure
 * 
 * @param dev Pointer to ina228_t structure
 * @param i2c I2C instance (i2c0 or i2c1)
 * @param addr I2C address (default 0x40)
 * @param shunt_resistor_ohms Value of the shunt resistor in Ohms
 * @param max_current_a Expected maximum current for calibration
 */
bool ina228_init(ina228_t *dev, uint8_t addr, float shunt_resistor_ohms, float max_current_a);

/**
 * @brief Configure the INA228 ADC settings
 * 
 * @param dev Pointer to ina228_t structure
 */
void ina228_configure(ina228_t *dev);

/**
 * @brief Read the current register and update the struct
 * 
 * @param dev Pointer to ina228_t structure
 * @return float Current in Amps
 */
float ina228_read_current(ina228_t *dev);

/**
 * @brief Read the charge register and update the struct
 * 
 * @param dev Pointer to ina228_t structure
 * @return double Charge in Coulombs
 */
double ina228_read_charge(ina228_t *dev);

/**
 * @brief Start periodic sampling using a repeating timer (interrupt driven)
 * 
 * @param dev Pointer to ina228_t structure
 * @param interval_ms Sampling interval in milliseconds
 * @return bool true if timer started successfully
 */
bool ina228_start_periodic_sampling(ina228_t *dev, int32_t interval_ms);

/**
 * @brief Stop periodic sampling
 * 
 * @param dev Pointer to ina228_t structure
 */
void ina228_stop_periodic_sampling(ina228_t *dev);

#endif // HW_INA228_H
