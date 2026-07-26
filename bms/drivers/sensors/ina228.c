#include "ina228.h"

#include "config/allocations.h"
#include "config/limits.h"
#include "config/pins.h"
#include "app/model.h"
#include "drivers/chip/i2c.h"
#include "sys/logging/logging.h"
#include "lib/aema.h"

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
#define INA228_ADC_MODE_TRIG_BUS   0x1
#define INA228_ADC_MODE_TRIG_TEMP  0x4
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

// Normal operation: continuous shunt conversions, 2074us x 256 avg ~= 531ms/sample
#define INA228_ADC_CONFIG_CONTINUOUS ((INA228_ADC_MODE_CONT_SHUNT << 12) | \
                                      INA228_ADC_VBUSCT(INA228_CT_2074US) | \
                                      INA228_ADC_VSHCT(INA228_CT_2074US) | \
                                      INA228_ADC_VTCT(INA228_CT_2074US) | \
                                      INA228_ADC_AVG(INA228_AVG_256))

// Triggered single-shot temperature/bus conversions: 2074us x 64 avg ~= 133ms.
// Both channels are inherently quiet, so 64x averaging is already essentially
// noise-free while keeping the pause in current sampling short.
#define INA228_ADC_CONFIG_TRIG_TEMP ((INA228_ADC_MODE_TRIG_TEMP << 12) | \
                                     INA228_ADC_VBUSCT(INA228_CT_2074US) | \
                                     INA228_ADC_VSHCT(INA228_CT_2074US) | \
                                     INA228_ADC_VTCT(INA228_CT_2074US) | \
                                     INA228_ADC_AVG(INA228_AVG_64))

#define INA228_ADC_CONFIG_TRIG_BUS  ((INA228_ADC_MODE_TRIG_BUS << 12) | \
                                     INA228_ADC_VBUSCT(INA228_CT_2074US) | \
                                     INA228_ADC_VSHCT(INA228_CT_2074US) | \
                                     INA228_ADC_VTCT(INA228_CT_2074US) | \
                                     INA228_ADC_AVG(INA228_AVG_64))

// A triggered conversion takes 2074us x 64 = 132.7ms; wait at least this long
// after writing the trigger config before reading the result register (we
// don't rely on the CNVRF flag, which can be stale from continuous mode).
#define INA228_TEMP_CONVERSION_WAIT_US 140000

// Global variables for storing current measurements
// static int32_t ina228_current_raw = 0;
// static millis_t ina228_current_millis = 0;
// static int64_t ina228_charge_raw = 0;
// static millis_t ina228_charge_millis = 0;

static void ina228_start_async_timer(ina228_t *dev);

// Helper function to write a 16-bit register
static bool ina228_write_reg16(ina228_t *dev, uint8_t reg, uint16_t value) {
    uint8_t buf[2];
    buf[0] = (value >> 8) & 0xFF;
    buf[1] = value & 0xFF;
    
    return i2c_blocking_write_reg(dev->i2c, dev->addr, reg, buf, 2, INA228_I2C_TIMEOUT_US);
}

// Helper function to read a 20-bit register (3 bytes)
// static bool ina228_read_reg20(ina228_t *dev, uint8_t reg, int32_t *value) {
//     uint8_t buf[3];
    
//     if (!i2c_blocking_read_reg(dev->i2c, dev->addr, reg, buf, 3, INA228_I2C_TIMEOUT_US)) {
//         return false;
//     }
    
//     // Combine bytes - INA228 uses 20-bit signed values in MSB first format
//     int32_t raw = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    
//     // Sign extend from 20 bits to 32 bits
//     if (raw & 0x80000) {
//         raw |= 0xFFF00000;
//     }
    
//     *value = raw;
//     return true;
// }

// Helper function to read a 40-bit register (5 bytes)
// static bool ina228_read_reg40(ina228_t *dev, uint8_t reg, int64_t *value) {
//     uint8_t buf[5];
    
//     if (!i2c_blocking_read_reg(dev->i2c, dev->addr, reg, buf, 5, INA228_I2C_TIMEOUT_US)) {
//         return false;
//     }
    
//     // Combine bytes
//     int64_t raw = ((int64_t)buf[0] << 32) | ((int64_t)buf[1] << 24) | 
//                   ((int64_t)buf[2] << 16) | ((int64_t)buf[3] << 8) | (int64_t)buf[4];
    
//     *value = raw;
//     return true;
// }

// Helper function to read a 16-bit register
static bool ina228_read_reg16(ina228_t *dev, uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    
    if (!i2c_blocking_read_reg(dev->i2c, dev->addr, reg, buf, 2, INA228_I2C_TIMEOUT_US)) {
        return false;
    }
    
    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

bool ina228_init(ina228_t *dev, uint8_t i2c_addr, float shunt_resistor_ohms, float max_current_a) {
    dev->i2c = INA228_I2C;
    dev->addr = i2c_addr;
    dev->present = false;
    dev->shunt_resistor_ohms = shunt_resistor_ohms;
    dev->async_busy = false;
    
    // Initialize I2C using new async-capable driver
    i2c_async_init(dev->i2c, 400 * 1000, PIN_INA228_I2C_SDA, PIN_INA228_I2C_SCL);
    
    // Read and verify manufacturer ID (should be 0x5449 = "TI")
    uint16_t mfg_id;
    if (!ina228_read_reg16(dev, INA228_REG_MANUFACTURER_ID, &mfg_id)) {
        error_printf("INA228: Failed to read manufacturer ID\n");
        return false;
    }
    
    if (mfg_id != 0x5449) {
        error_printf("INA228: Invalid manufacturer ID: 0x%04X (expected 0x5449)\n", mfg_id);
        return false;
    }
    
    // Read device ID (should be 0x228)
    uint16_t dev_id;
    if (!ina228_read_reg16(dev, INA228_REG_DEVICE_ID, &dev_id)) {
        error_printf("INA228: Failed to read device ID\n");
        return false;
    }
    
    uint16_t expected_id = (0x228 << 4);  // Device ID is in upper 12 bits
    if ((dev_id & 0xFFF0) != expected_id) {
        error_printf("INA228: Invalid device ID: 0x%04X (expected 0x%04X)\n", dev_id, expected_id);
        return false;
    }
    
    info_printf("INA228: Detected (MFG=0x%04X, DEV=0x%04X)\n", mfg_id, dev_id);
    
    // Perform a software reset
    if (!ina228_write_reg16(dev, INA228_REG_CONFIG, INA228_CONFIG_RST)) {
        error_printf("INA228: Reset failed\n");
        return false;
    }
    
    // Wait for reset to complete
    sleep_ms(2);
    
    // Configure the device
    ina228_configure(dev);

    ina228_start_async_timer(dev);

    dev->present = true;
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
    shunt_cal = INA228_SHUNT_CAL*4/4;
    
    debug_printf("INA228: Current LSB = %.6f A/LSB\n", dev->current_lsb);
    debug_printf("INA228: SHUNT_CAL = %u (0x%04X)\n", shunt_cal, shunt_cal);
    
    // Write SHUNT_CAL register
    if (!ina228_write_reg16(dev, INA228_REG_SHUNT_CAL, shunt_cal)) {
        error_printf("INA228: Failed to write SHUNT_CAL\n");
        return;
    }
    
    // Configure ADC: continuous shunt conversions with long averaging
    uint16_t adc_config = INA228_ADC_CONFIG_CONTINUOUS;
    
    if (!ina228_write_reg16(dev, INA228_REG_ADC_CONFIG, adc_config)) {
        error_printf("INA228: Failed to write ADC_CONFIG\n");
        return;
    }
    
    // Configure main CONFIG register
    uint16_t config = 0x0010; 
    
    if (!ina228_write_reg16(dev, INA228_REG_CONFIG, config)) {
        error_printf("INA228: Failed to write CONFIG\n");
        return;
    }

    // Configure Diagnostic flags
    // uint16_t diag_alrt = 0x0001; // No alerts for now
    // if (!ina228_write_reg16(dev, INA228_REG_DIAG_ALRT, diag_alrt)) {
    //     printf("INA228: Failed to write DIAG_ALRT\n");
    //     return;
    // }
    
    info_printf("INA228: Configuration complete\n");
}

static int32_t div_round_closest(const int32_t n, const int32_t d)
{
  return ((n < 0) == (d < 0)) ? ((n + d/2)/d) : ((n - d/2)/d);
}

//#define SAMPLING_PERIOD_SMOOTHING 2048
uint32_t last_sample_us = 0;
//uint32_t average_sampling_period_us = 530000; //530944; // Initial estimate based on INA228 datasheet
float average_sampling_period_us = 530307.0f; // Initial estimate based on INA228 datasheet

static uint8_t async_read_buf[5];
static volatile int32_t latest_async_current_raw = 0;
static volatile bool async_new_reading_available = false;

// Temperature measurement state machine. Every
// INA228_TEMP_MEASURE_INTERVAL_SAMPLES accepted current samples, the main tick
// requests a triggered single-shot conversion (alternating between the die
// temperature and the NTC via VBUS). All I2C transactions are started from the
// 160ms timer callback, so they never race with each other.
typedef enum {
    MEAS_STATE_CURRENT = 0,       // Continuous shunt mode, polling current
    MEAS_STATE_TRIGGER_PENDING,   // Need to write the triggered ADC config
    MEAS_STATE_CONVERTING,        // Triggered conversion in progress
    MEAS_STATE_RESTORE_PENDING,   // Result read; restore continuous config
} meas_state_t;

static volatile meas_state_t meas_state = MEAS_STATE_CURRENT;
static bool meas_is_ntc = false;               // Which channel the current trigger targets
static volatile bool temp_result_is_ntc = false; // Which channel the pending result is from
static volatile uint32_t trigger_written_us = 0;
static uint32_t temp_interval_counter = 0;

static uint8_t temp_read_buf[3];
static volatile int32_t latest_temp_raw = 0;
static volatile bool temp_reading_available = false;

static void on_read_complete(i2c_inst_t *i2c, bool success, void *user_data) {
    (void)i2c;
    (void)user_data;

    if (success) {
        uint16_t diag = ((uint16_t)async_read_buf[0] << 8) | async_read_buf[1];
        if (diag & 0x0002) { // Conversion Ready
            int32_t raw = ((int32_t)async_read_buf[2] << 12) | 
                          ((int32_t)async_read_buf[3] << 4) | 
                          ((int32_t)async_read_buf[4] >> 4);
            
            // Sign extend from 20 bits
            if (raw & 0x80000) {
                raw |= 0xFFF00000;
            }
            
            latest_async_current_raw = raw;
            async_new_reading_available = true;
        }
    }
}

static void on_config_write_complete(i2c_inst_t *i2c, bool success, void *user_data) {
    (void)i2c;
    (void)user_data;

    if (!success) {
        return; // Stay in the current state; the timer will retry
    }

    if (meas_state == MEAS_STATE_TRIGGER_PENDING) {
        trigger_written_us = time_us_32();
        meas_state = MEAS_STATE_CONVERTING;
    } else if (meas_state == MEAS_STATE_RESTORE_PENDING) {
        meas_state = MEAS_STATE_CURRENT;
    }
}

static void on_temp_read_complete(i2c_inst_t *i2c, bool success, void *user_data) {
    (void)i2c;
    (void)user_data;

    if (!success) {
        return; // Stay in MEAS_STATE_CONVERTING; the timer will retry
    }

    if (meas_is_ntc) {
        // VBUS is a 20-bit value in the upper bits of 3 bytes
        int32_t raw = ((int32_t)temp_read_buf[0] << 12) |
                      ((int32_t)temp_read_buf[1] << 4) |
                      ((int32_t)temp_read_buf[2] >> 4);
        if (raw & 0x80000) {
            raw |= 0xFFF00000;
        }
        latest_temp_raw = raw;
    } else {
        // DIETEMP is a signed 16-bit value
        latest_temp_raw = (int16_t)(((uint16_t)temp_read_buf[0] << 8) | temp_read_buf[1]);
    }
    temp_result_is_ntc = meas_is_ntc;
    temp_reading_available = true;
    meas_state = MEAS_STATE_RESTORE_PENDING;
}

static bool start_async_config_write(ina228_t *dev, uint16_t value) {
    static uint8_t cfg_buf[2];
    cfg_buf[0] = (value >> 8) & 0xFF;
    cfg_buf[1] = value & 0xFF;
    return i2c_async_write_reg(dev->i2c, dev->addr, INA228_REG_ADC_CONFIG,
                               cfg_buf, 2, on_config_write_complete, dev);
}

static bool timer_callback(struct repeating_timer *t) {
    ina228_t *dev = (ina228_t *)t->user_data;
    
    // Only start if not already busy
    if (i2c_async_is_busy(dev->i2c)) {
        return true;
    }

    switch (meas_state) {
        case MEAS_STATE_CURRENT:
            i2c_async_read_regs_dual(dev->i2c, dev->addr, 
                                     INA228_REG_DIAG_ALRT, 2, 
                                     INA228_REG_CURRENT, 3, 
                                     async_read_buf, on_read_complete, dev);
            break;
        case MEAS_STATE_TRIGGER_PENDING:
            start_async_config_write(dev, meas_is_ntc ?
                INA228_ADC_CONFIG_TRIG_BUS : INA228_ADC_CONFIG_TRIG_TEMP);
            break;
        case MEAS_STATE_CONVERTING:
            // The triggered conversion completes after a fixed time; read the
            // result once it has definitely finished.
            if ((time_us_32() - trigger_written_us) >= INA228_TEMP_CONVERSION_WAIT_US) {
                if (meas_is_ntc) {
                    i2c_async_read_reg(dev->i2c, dev->addr, INA228_REG_VBUS,
                                       temp_read_buf, 3, on_temp_read_complete, dev);
                } else {
                    i2c_async_read_reg(dev->i2c, dev->addr, INA228_REG_DIETEMP,
                                       temp_read_buf, 2, on_temp_read_complete, dev);
                }
            }
            break;
        case MEAS_STATE_RESTORE_PENDING:
            start_async_config_write(dev, INA228_ADC_CONFIG_CONTINUOUS);
            break;
    }
    
    return true;
}

static void ina228_start_async_timer(ina228_t *dev) {
    static struct repeating_timer timer;
    // 160ms = 160,000us. Negative value for start-to-start timing.
    add_repeating_timer_us(-160000, timer_callback, dev, &timer);
}

// Convert and store a completed temperature measurement into the model
static void store_temp_result(bool is_ntc, int32_t raw) {
    if (is_ntc) {
        // NTC divider voltage via the VBUS input (195.3125 uV/LSB)
        float v = (float)raw * 195.3125e-6f;
        // Clamp to avoid division by zero / log of a non-positive number
        // (e.g. with the NTC disconnected the node floats up to the supply)
        float v_max = INA228_NTC_SUPPLY_V - 0.001f;
        if (v < 0.001f) v = 0.001f;
        if (v > v_max) v = v_max;

        float r_ntc = INA228_NTC_PULLUP_OHMS * v / (INA228_NTC_SUPPLY_V - v);
        // Beta equation: 1/T = 1/T25 + ln(R/R25)/B
        float temp_c = 1.0f / (1.0f / 298.15f
                               + logf(r_ntc / INA228_NTC_R25_OHMS) / INA228_NTC_BETA)
                       - 273.15f;

        model.shunt_ntc_resistance_ohms = r_ntc;
        model.shunt_ntc_temperature = temp_c;
        model.shunt_ntc_millis = millis();
    } else {
        // DIETEMP: 7.8125 m degrees C per LSB
        model.shunt_die_temperature = (float)raw * 0.0078125f;
        model.shunt_die_temperature_millis = millis();
    }
}

// Apparent input offset voltage vs die temperature, fitted to measured
// zero-current data (band centers of a logged temperature sweep, calibrated
// zero at 33.7C, temperature axis shifted -1C from the logging MCU sensor to
// reference the INA228 die temp). The observed drift (~-3uV/C at the hot end,
// ~-24mA at 48C through the 1mOhm shunt) is ~400x larger than the INA228
// datasheet die-offset curve (Figure 6-4), so the die offset itself cannot be
// the dominant mechanism (likely thermal EMFs at the shunt sense junctions),
// but it tracks die temperature repeatably so we correct against it. The curve
// is anchored to ~0 at the 32.7C (die) calibration point so existing stored
// zero-current offsets remain valid.
typedef struct {
    float temp_c;
    float vos_uv;
} vos_point_t;

static const vos_point_t vos_table[] = {
    {  19.0f,   0.5f },
    {  29.0f,   0.5f },
    {  32.7f,   0.0f },
    {  34.0f,  -1.5f },
    {  39.0f,  -5.5f },
    {  44.0f, -14.5f },
    {  47.0f, -24.0f },
    // Extrapolated beyond the measured 19-48C range at ~-3uV/C (unverified)
    {  60.0f, -63.0f },
};
#define VOS_TABLE_LEN (sizeof(vos_table) / sizeof(vos_table[0]))

// Returns the expected input offset voltage at the given die temperature,
// expressed in raw current counts (0.25mA each), using linear interpolation
// clamped at the table ends.
static float ina228_vos_counts(float die_temp_c) {
    float vos_uv;
    if (die_temp_c <= vos_table[0].temp_c) {
        vos_uv = vos_table[0].vos_uv;
    } else if (die_temp_c >= vos_table[VOS_TABLE_LEN - 1].temp_c) {
        vos_uv = vos_table[VOS_TABLE_LEN - 1].vos_uv;
    } else {
        vos_uv = vos_table[0].vos_uv;
        for (size_t i = 1; i < VOS_TABLE_LEN; i++) {
            if (die_temp_c <= vos_table[i].temp_c) {
                float frac = (die_temp_c - vos_table[i - 1].temp_c)
                             / (vos_table[i].temp_c - vos_table[i - 1].temp_c);
                vos_uv = vos_table[i - 1].vos_uv
                         + frac * (vos_table[i].vos_uv - vos_table[i - 1].vos_uv);
                break;
            }
        }
    }
    return vos_uv * INA228_VOS_UV_TO_COUNTS;
}

// Only interrupt current sampling for a temperature measurement when the
// contactor state machine is settled, so precharge/weld-test/calibration logic
// (which relies on fresh current readings) never sees the sampling pause.
static bool contactors_settled(void) {
    switch (model.contactor_sm.state) {
        case CONTACTORS_STATE_OPEN:
        case CONTACTORS_STATE_CLOSED:
        case CONTACTORS_STATE_PRECHARGE_FAILED:
        case CONTACTORS_STATE_TESTING_FAILED:
            return true;
        default:
            return false;
    }
}

bool ina228_read_current_async(ina228_t *dev) {
    // Consume any completed temperature measurement
    if (temp_reading_available) {
        bool is_ntc = temp_result_is_ntc;
        int32_t temp_raw = latest_temp_raw;
        temp_reading_available = false;
        store_temp_result(is_ntc, temp_raw);
    }

    if (!async_new_reading_available) {
        return false;
    }

    int32_t current_raw = latest_async_current_raw;
    async_new_reading_available = false;

    // Correct for the temperature-dependent apparent input offset voltage.
    // Until the first die temperature reading arrives, assume 25C (where the
    // fitted curve is near zero). The fractional part of the correction is
    // carried between samples (error diffusion) so the applied correction
    // averages to the exact value, which is what matters for charge
    // integration. Applied to the raw value before the null-calibration
    // accumulator, so the stored zero-current offset stays
    // temperature-independent.
    static float vos_residual = 0.0f;
    float die_temp_c = (model.shunt_die_temperature_millis > 0)
                       ? model.shunt_die_temperature : 25.0f;
    float vos_total = ina228_vos_counts(die_temp_c) + vos_residual;
    int32_t vos_whole = (int32_t)lroundf(vos_total);
    vos_residual = vos_total - (float)vos_whole;

    int32_t raw_compensated = current_raw - vos_whole;

    int32_t current_corrected = raw_compensated - model.current_offset;
    model.current_mA = div_round_closest(current_corrected, 4);
    model.current_millis = millis();

    aema_update(
        &model.current_filtered_mA, 
        &model.current_deviation, 
        model.current_mA, 
        0.005f, // slow alpha
        0.5f,   // fast alpha
        8.0f,   // slow threshold
        20.0f   // fast threshold
    );
    
    // Update charge. Normally each accepted sample represents one ~531ms
    // conversion period, but a temperature measurement pauses current
    // conversions for roughly one extra period, so scale this sample's charge
    // contribution by the number of elapsed periods to keep coulomb counting
    // honest through the gap.
    uint32_t now_us = time_us_32();
    int32_t periods = 1;
    if (last_sample_us != 0) {
        uint32_t elapsed_us = now_us - last_sample_us;
        periods = (int32_t)((elapsed_us + 265472) / 530944); // round to nearest
        if (periods < 1) periods = 1;
        if (periods > 3) periods = 3;
    }
    last_sample_us = now_us;
    model.charge_raw += (int64_t)current_corrected * periods;
    model.charge_millis = model.current_millis;

    dev->null_accumulator += raw_compensated;
    dev->null_counter++;

    // Periodically measure the die temperature / shunt NTC, interleaved
    temp_interval_counter++;
    if (temp_interval_counter >= INA228_TEMP_MEASURE_INTERVAL_SAMPLES
            && meas_state == MEAS_STATE_CURRENT
            && contactors_settled()) {
        temp_interval_counter = 0;
        meas_is_ntc = !meas_is_ntc;
        meas_state = MEAS_STATE_TRIGGER_PENDING;
    }

    return true;
}
// Read current from the INA228 (blocking)
// bool ina228_read_current_blocking(ina228_t *dev) {
//     int32_t current_raw;
//     int32_t current_corrected;
    
//     uint16_t diag_alert;
//     if (!ina228_read_reg16(dev, INA228_REG_DIAG_ALRT, &diag_alert)) {
//         printf("INA228: Failed to read DIAG_ALRT\n");
//         return false;
//     }

//     if (!ina228_read_reg20(dev, INA228_REG_CURRENT, &current_raw)) {
//         printf("INA228: Failed to read CURRENT\n");
//         return false;
//     }
    
//     current_corrected = current_raw - model.current_offset;
//     // Positive current means charging.
//     model.current_mA = div_round_closest(current_corrected, 4);

//     // Was a new conversion
//     if(diag_alert & 0x0002) {
//         model.current_millis = millis();

//         //printf("raw current: %d\n", current_raw);

//         uint32_t now_us = time_us_32();
//         uint32_t elapsed_us = now_us - last_sample_us;

//         // TODO: do we care about this?
//         if(last_sample_us != 0) {
//             average_sampling_period_us = (average_sampling_period_us * 0.99999f) + ((float)elapsed_us * 0.00001f);
//         }

//         // average_sampling_period_us = (average_sampling_period_us * (SAMPLING_PERIOD_SMOOTHING - 1) + elapsed_us) / SAMPLING_PERIOD_SMOOTHING;
//         last_sample_us = now_us;
//         //printf("avg: %.2f us\n", average_sampling_period_us);

//         // Is a new conversion, update charge
//         model.charge_raw += (int64_t)current_corrected;
//         model.charge_millis = model.current_millis;

//         // charge_raw is in units equivalent to 0.132736mC (0.25mA LSB, 530.944ms per sample)

//         // We sample at the same rate as the INA228 conversions, which has a
//         // clock accurate to 1% - we would do better to use the crystal instead,
//         // but the jitter would probably outweigh the accuracy improvement.

//         dev->null_accumulator += current_raw;
//         dev->null_counter++;
//     }
    
//     return true;
// }

// Read charge accumulator from the INA228 (blocking)
// bool ina228_read_charge(ina228_t *dev) {
//     int64_t charge_raw;
    
//     if (!ina228_read_reg40(dev, INA228_REG_CHARGE, &charge_raw)) {
//         return false;
//     }
    
//     // Store in global variables
//     ina228_charge_raw = charge_raw;
//     ina228_charge_millis = millis();
    
//     // Update model
//     model.charge_raw = charge_raw;
//     model.charge_millis = ina228_charge_millis;
    
//     return true;
// }

// Read shunt voltage (blocking)
// bool ina228_read_shunt_voltage(ina228_t *dev, float *voltage_mv) {
//     int32_t vshunt_raw;
    
//     if (!ina228_read_reg20(dev, INA228_REG_VSHUNT, &vshunt_raw)) {
//         return false;
//     }
    
//     // VSHUNT LSB = 312.5 nV for ±163.84 mV range
//     *voltage_mv = (float)vshunt_raw * 312.5e-6f;  // Convert to mV
    
//     return true;
// }

// Read bus voltage (blocking)
// bool ina228_read_bus_voltage(ina228_t *dev, float *voltage_mv) {
//     int32_t vbus_raw;
    
//     if (!ina228_read_reg20(dev, INA228_REG_VBUS, &vbus_raw)) {
//         return false;
//     }
    
//     // VBUS LSB = 195.3125 µV
//     *voltage_mv = (float)vbus_raw * 0.1953125f;  // Convert to mV
    
//     return true;
// }

// Getter functions
// int32_t ina228_get_current_raw() {
//     return ina228_current_raw;
// }

// millis_t ina228_get_current_millis() {
//     return ina228_current_millis;
// }

// int64_t ina228_get_charge_raw() {
//     return ina228_charge_raw;
// }

// millis_t ina228_get_charge_millis() {
//     return ina228_charge_millis;
// }
