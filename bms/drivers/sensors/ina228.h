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

// Log a gap between current samples longer than this (normally ~531ms, or
// up to ~1.2s around a temperature measurement)
#define INA228_SAMPLE_GAP_LOG_US 5000000

// Conversion from shunt input offset voltage (uV) to raw current counts.
// One raw count is 0.25mA, which across the 1mOhm shunt is 0.25uV, so
// counts = uV / 0.25. Adjust if a different shunt resistance is used.
#define INA228_VOS_UV_TO_COUNTS 4.0f

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

// Sampling diagnostics (for the CLI)
typedef struct {
    uint32_t samples;          // accepted current samples
    uint32_t max_gap_us;       // longest interval between current samples
    uint32_t i2c_errors;       // failed I2C transactions
    uint32_t die_measurements; // completed die temperature measurements
    uint32_t ntc_measurements; // completed NTC (VBUS) measurements
    int32_t last_raw;          // last CURRENT register value (0.25mA counts)
    int32_t last_vos_counts;   // offset voltage correction applied to it
    float ntc_volts;           // last NTC divider voltage
} ina228_stats_t;
extern ina228_stats_t ina228_stats;

bool ina228_init(ina228_t *dev, uint8_t i2c_addr, float shunt_resistor_ohms, float max_current_a);
void ina228_configure(ina228_t *dev);

/**
 * @brief Read the latest current measurement sampled by the background timer.
 * 
 * @param dev Pointer to INA228 device handle
 * @return true if data was available and updated
 */
bool ina228_read_current_async(ina228_t *dev);

// model.charge_raw is in current LSBs (0.25mA) x microseconds, i.e. 0.25nC
#define INA228_CHARGE_RAW_PER_MC 4000000

static inline int64_t raw_charge_to_mC(int64_t charge_raw) {
    return charge_raw / INA228_CHARGE_RAW_PER_MC;
}

static inline float raw_charge_to_Ah(int64_t charge_raw) {
    return (float)charge_raw * (1.0f / (INA228_CHARGE_RAW_PER_MC * 3600000.0f));
}
