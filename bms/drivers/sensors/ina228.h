#pragma once

#include "../../sys/time/time.h"

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

// External shunt NTC divider (Shunt_5V -> 10k pullup -> NTC -> GND_Shunt),
// sensed via the INA228 VBUS pin. The supply is an unregulated isolated DC-DC,
// so absolute accuracy is limited; the computed resistance is exposed over the
// HMI to allow later calibration of these constants.
#define INA228_NTC_PULLUP_OHMS   10000.0f
#define INA228_NTC_R25_OHMS      10000.0f
#define INA228_NTC_BETA          3950.0f
#define INA228_NTC_SUPPLY_V      5.55f

// How many accepted current samples (~531ms each) between temperature
// measurements. Each trigger alternates between the die temp and the NTC.
#define INA228_TEMP_MEASURE_INTERVAL_SAMPLES 100

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    bool present;
    float current_lsb;
    float shunt_resistor_ohms;

    int32_t null_accumulator;
    uint32_t null_counter;
    //int32_t null_offset;
    
    // Async state
    volatile bool async_busy;
    uint8_t async_reg;
    uint8_t async_buf[5];
    uint8_t async_bytes_expected;
    volatile uint8_t async_bytes_received;
} ina228_t;

bool ina228_init(ina228_t *dev, uint8_t i2c_addr, float shunt_resistor_ohms, float max_current_a);
void ina228_configure(ina228_t *dev);

// Blocking read functions
bool ina228_read_current_blocking(ina228_t *dev);
bool ina228_read_charge(ina228_t *dev);
bool ina228_read_shunt_voltage(ina228_t *dev, float *voltage_mv);
bool ina228_read_bus_voltage(ina228_t *dev, float *voltage_mv);

/**
 * @brief Read the latest current measurement sampled by the background timer.
 * 
 * @param dev Pointer to INA228 device handle
 * @return true if data was available and updated
 */
bool ina228_read_current_async(ina228_t *dev);

// Getter functions for last read values
int32_t ina228_get_current_raw();
millis_t ina228_get_current_millis();
int64_t ina228_get_charge_raw();
millis_t ina228_get_charge_millis();

static inline int64_t raw_charge_to_mC(int64_t charge_raw) {
    // Each LSB of charge_raw represents 0.132736 mC
    // (0.25mA * 530.944ms)
    return (charge_raw * 132736) / 1000000;
}

static inline float raw_charge_to_Ah(int32_t charge_raw) {
    // Each LSB of charge_raw represents 0.132736 mC
    // (0.25mA * 530.944ms)
    return charge_raw * (0.132736f / 3600000.0f);
}