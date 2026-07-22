#include "ads1115.h"
#include "config/allocations.h"
#include "config/pins.h"
#include "app/model.h"
#include "drivers/chip/i2c.h"
#include "sys/logging/logging.h"
#include "lib/aema.h"

#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>

// With floating inputs, higher sample rates lead to larger readings - eg, 350mV
// at 128SPS, but 1000mV at 250SPS, it is not clear why.

// How fast to sample (data rate setting)
static const int ADS1115_SAMPLE_RATE_SETTING = ADS1115_CONFIG_DR_128SPS;
// How long each conversion takes
static const int ADS1115_CONVERSION_TIME_MS = 8;
// How long extra to wait to be safe before reading conversion result
static const int ADS1115_CONVERSION_TIME_EXTRA_MS = 1;

// The scan runs in rounds. In each round, one channel from each present device
// is converted concurrently (the chips convert independently; only the I2C
// transactions are serialized). ADC A has the most channels, so it sets the
// number of rounds and the cycle time is the same with or without ADC B.
#define ADS1115_SCAN_ROUNDS 6
// Max devices converting concurrently in one round
#define ADS1115_MAX_PER_ROUND 2

// The scan is free-running: each cycle starts the next as soon as it
// completes. A full cycle takes ~85ms in practice (6 rounds of ~9ms
// conversion wait plus I2C transaction overhead), so that is the effective
// sample period per channel. This timer only acts as a watchdog to restart
// the scan if it was abandoned after an I2C error.
static const int ADS1115_WATCHDOG_PERIOD_MS = 100;
// I2C timeout for blocking calls
static const uint32_t ADS1115_I2C_TIMEOUT_US = 10000;

// Sample timestamps update every cycle (~85ms); the oversampling accumulator
// only affects the (currently unused) averaged value and min/max range

static void ads1115_i2c_callback(i2c_inst_t *i2c, bool success, void *user_data);
static int64_t ads1115_conversion_timer_callback(alarm_id_t id, void *user_data);
static bool ads1115_periodic_timer_callback(struct repeating_timer *t);
static void ads1115_start_sampling(void);

sampler_t samples[ADS1115_CHANNEL_COUNT] = {0};
float filtered_samples[ADS1115_CHANNEL_COUNT] = { [0 ... (ADS1115_CHANNEL_COUNT-1)] = NAN };
float sample_deviations[ADS1115_CHANNEL_COUNT] = { [0 ... (ADS1115_CHANNEL_COUNT-1)] = NAN };

ads1115_t ads1115_dev = {0};   // ADC A (0x48)
#ifdef HAS_ADS1115_SECONDARY
ads1115_t ads1115_dev_b = {0}; // ADC B (0x49)
#endif

typedef struct {
    ads1115_t *dev;
    uint16_t config; // mux | pga bits
} ads1115_channel_cfg_t;

// Logical channel table (see ads1115.h for the meaning of each channel)
static const ads1115_channel_cfg_t channel_table[ADS1115_CHANNEL_COUNT] = {
    // ADC A
    [0] = { &ads1115_dev, ADS1115_CONFIG_MUX_DIFF_0_1 | ADS1115_CONFIG_PGA_1_024V }, // battery voltage
    [1] = { &ads1115_dev, ADS1115_CONFIG_MUX_DIFF_2_3 | ADS1115_CONFIG_PGA_1_024V }, // output voltage
    [2] = { &ads1115_dev, ADS1115_CONFIG_MUX_DIFF_1_3 | ADS1115_CONFIG_PGA_1_024V }, // across negative path
    [3] = { &ads1115_dev, ADS1115_CONFIG_MUX_DIFF_0_3 | ADS1115_CONFIG_PGA_1_024V }, // battery pos to output neg
    [4] = { &ads1115_dev, ADS1115_CONFIG_MUX_SINGLE_0 | ADS1115_CONFIG_PGA_4_096V }, // battery pos (isolation)
    [5] = { &ads1115_dev, ADS1115_CONFIG_MUX_SINGLE_1 | ADS1115_CONFIG_PGA_4_096V }, // battery neg (isolation)
#ifdef HAS_ADS1115_SECONDARY
    // ADC B
    [6] = { &ads1115_dev_b, ADS1115_CONFIG_MUX_DIFF_2_3 | ADS1115_CONFIG_PGA_1_024V }, // across pos (Link Positive) contactor
    [7] = { &ads1115_dev_b, ADS1115_CONFIG_MUX_DIFF_0_3 | ADS1115_CONFIG_PGA_1_024V }, // negated link voltage
    [8] = { &ads1115_dev_b, ADS1115_CONFIG_MUX_DIFF_1_3 | ADS1115_CONFIG_PGA_1_024V }, // fuse/jumper drop
    [9] = { &ads1115_dev_b, ADS1115_CONFIG_MUX_SINGLE_1 | ADS1115_CONFIG_PGA_4_096V }, // out pos (diagnostics)
#endif
};

// The channel each device converts in a given round, or -1 if none.
static int round_channel(const ads1115_t *dev, int round) {
    for (int ch = 0; ch < ADS1115_CHANNEL_COUNT; ch++) {
        if (channel_table[ch].dev != dev) continue;
        if (round == 0) return ch;
        round--;
    }
    return -1;
}

// Scan state (one scan services all devices)
static struct {
    enum {
        SCAN_STATE_IDLE,
        SCAN_STATE_WRITING_CONFIG,
        SCAN_STATE_WAIT_CONVERSION,
        SCAN_STATE_READING_CONVERSION,
    } state;
    bool busy;
    int round;
    // Channels being converted this round (one per present device)
    int round_channels[ADS1115_MAX_PER_ROUND];
    int round_count;
    int round_idx; // which round entry the current I2C transaction is for
    uint8_t async_buf[2];
} scan = { .state = SCAN_STATE_IDLE };

static int32_t cal_accumulator[ADS1115_CHANNEL_COUNT];
static uint32_t cal_samples_left[ADS1115_CHANNEL_COUNT];

bool ads1115_init(ads1115_t *dev, uint8_t addr) {
    static bool bus_initialized = false;
    static bool timer_started = false;

    dev->i2c = ADS1115_I2C; // Based on pins 22, 23
    dev->addr = addr;
    dev->present = false;

    if (!bus_initialized) {
        // Initialize I2C using new async-capable driver
        i2c_async_init(dev->i2c, 400 * 1000, PIN_ADS1115_I2C_SDA, PIN_ADS1115_I2C_SCL);
        bus_initialized = true;
    }

    // Test read to check if chip is present (blocking is fine for init)
    uint8_t test_buf[2];
    if (!i2c_blocking_read_reg(dev->i2c, dev->addr, ADS1115_REG_CONFIG, test_buf, 2, ADS1115_I2C_TIMEOUT_US)) {
        return false;
    }
    dev->present = true;

    // Start the scan watchdog once any device is present; its first fire
    // kicks off the free-running scan
    if (!timer_started) {
        static struct repeating_timer timer;
        add_repeating_timer_ms(ADS1115_WATCHDOG_PERIOD_MS, ads1115_periodic_timer_callback, NULL, &timer);
        timer_started = true;
    }

    return true;
}

static void ads1115_write_config(int channel) {
    const ads1115_channel_cfg_t *cfg = &channel_table[channel];
    uint16_t config = ADS1115_CONFIG_OS_SINGLE |
                      ADS1115_CONFIG_MODE_SINGLE |
                      ADS1115_SAMPLE_RATE_SETTING |
                      ADS1115_CONFIG_COMP_QUE_NONE |
                      cfg->config;

    uint8_t buf[2];
    buf[0] = (config >> 8) & 0xFF;
    buf[1] = config & 0xFF;

    i2c_async_write_reg(cfg->dev->i2c, cfg->dev->addr, ADS1115_REG_CONFIG, buf, 2, ads1115_i2c_callback, NULL);
}

static void ads1115_read_conversion(int channel) {
    const ads1115_channel_cfg_t *cfg = &channel_table[channel];
    i2c_async_read_reg(cfg->dev->i2c, cfg->dev->addr, ADS1115_REG_CONVERSION, scan.async_buf, 2, ads1115_i2c_callback, NULL);
}

static bool ads1115_setup_round(void) {
    // Collect this round's channel for each present device. Returns false if
    // no device has a channel this round.
    scan.round_count = 0;

    int ch_a = ads1115_dev.present ? round_channel(&ads1115_dev, scan.round) : -1;
    if (ch_a >= 0) {
        scan.round_channels[scan.round_count++] = ch_a;
    }
#ifdef HAS_ADS1115_SECONDARY
    int ch_b = ads1115_dev_b.present ? round_channel(&ads1115_dev_b, scan.round) : -1;
    if (ch_b >= 0) {
        scan.round_channels[scan.round_count++] = ch_b;
    }
#endif

    return scan.round_count > 0;
}

static void ads1115_start_round(void) {
    while (scan.round < ADS1115_SCAN_ROUNDS) {
        if (ads1115_setup_round()) {
            scan.round_idx = 0;
            scan.state = SCAN_STATE_WRITING_CONFIG;
            ads1115_write_config(scan.round_channels[0]);
            return;
        }
        scan.round++;
    }

    // Cycle complete
    scan.busy = false;
    scan.state = SCAN_STATE_IDLE;

    // Free-running: start the next cycle straight away. Guarded by a device
    // being present so this can't recurse endlessly with nothing to scan.
    if (ads1115_dev.present
#ifdef HAS_ADS1115_SECONDARY
        || ads1115_dev_b.present
#endif
    ) {
        ads1115_start_sampling();
    }
}

static void ads1115_start_sampling(void) {
    if (scan.busy) return;
    scan.busy = true;
    scan.round = 0;
    ads1115_start_round();
}

static void ads1115_store_sample(int channel, int16_t sample) {
    sampler_add(&samples[channel], (int32_t)sample, ADS1115_OVERSAMPLING, 0);

    // Publish freshness per sample rather than per accumulator rollover: the
    // values the application consumes (filtered_samples) update every cycle,
    // so the staleness timestamp should too
    samples[channel].timestamp = millis();

    aema_update(
        &filtered_samples[channel],
        &sample_deviations[channel],
        (float)sample,
        0.001f, 0.5f, 15.0f, 150.0f
    );

    if (cal_samples_left[channel] > 0) {
        cal_accumulator[channel] += sample;
        cal_samples_left[channel]--;
        if (channel == 0) {
            debug_printf("Cal remaining: %lu\n", cal_samples_left[0]);
        }
    }
}

static void ads1115_i2c_callback(i2c_inst_t *i2c, bool success, void *user_data) {
    (void)i2c;
    (void)user_data;

    if (!success) {
        // Abandon this cycle; the watchdog timer will start a fresh one
        scan.busy = false;
        scan.state = SCAN_STATE_IDLE;
        return;
    }

    if (scan.state == SCAN_STATE_WRITING_CONFIG) {
        scan.round_idx++;
        if (scan.round_idx < scan.round_count) {
            // Kick off the next device's conversion for this round
            ads1115_write_config(scan.round_channels[scan.round_idx]);
        } else {
            // All conversions started, wait for them to complete
            scan.state = SCAN_STATE_WAIT_CONVERSION;
            add_alarm_in_ms(
                ADS1115_CONVERSION_TIME_MS + ADS1115_CONVERSION_TIME_EXTRA_MS,
                ads1115_conversion_timer_callback,
                NULL,
                true
            );
        }
    } else if (scan.state == SCAN_STATE_READING_CONVERSION) {
        const int16_t sample = (int16_t)((scan.async_buf[0] << 8) | scan.async_buf[1]);
        ads1115_store_sample(scan.round_channels[scan.round_idx], sample);

        scan.round_idx++;
        if (scan.round_idx < scan.round_count) {
            ads1115_read_conversion(scan.round_channels[scan.round_idx]);
        } else {
            scan.round++;
            ads1115_start_round();
        }
    }
}

static int64_t ads1115_conversion_timer_callback(alarm_id_t id, void *user_data) {
    (void)id;
    (void)user_data;
    scan.state = SCAN_STATE_READING_CONVERSION;
    scan.round_idx = 0;
    ads1115_read_conversion(scan.round_channels[0]);
    return 0;
}

static bool ads1115_periodic_timer_callback(struct repeating_timer *t) {
    (void)t;
    ads1115_start_sampling();
    return true;
}

void ads1115_start_calibration(uint16_t num_samples) {
    for (int ch = 0; ch < ADS1115_CHANNEL_COUNT; ch++) {
        cal_accumulator[ch] = 0;
        // Only arm channels whose device is actually present, otherwise
        // calibration would never finish
        if (channel_table[ch].dev->present) {
            debug_printf("Starting calibration for channel %d, num_samples=%d\n", ch, num_samples);
            cal_samples_left[ch] = num_samples;
        } else {
            cal_samples_left[ch] = 0;
        }
    }
}

bool ads1115_calibration_finished(void) {
    for (int ch = 0; ch < ADS1115_CHANNEL_COUNT; ch++) {
        if (cal_samples_left[ch] > 0) {
            return false;
        }
    }
    return true;
}

int32_t ads1115_get_calibration(int channel) {
    return cal_accumulator[channel];
}


int16_t ads1115_get_sample_range(int channel) {
    return (int16_t)(samples[channel].max_value - samples[channel].min_value);
}

millis_t ads1115_get_sample_millis(int channel) {
    return samples[channel].timestamp;
}
