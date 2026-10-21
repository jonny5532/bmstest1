#include "internal_adc.h"
#include "sys/time/time.h"
#include "config/pins.h"
#include "lib/sampler.h"
#include "sys/logging/logging.h"

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "pico/time.h"

#if PICO_RP2350A == 1
// A variant
#error wrong one
#define ADC_CHANNEL_GPIO_OFFSET 26
#define ADC_CHANNEL_TEMP_SENSOR 4
#elif PICO_RP2350A == 0
// B variant
#define ADC_CHANNEL_GPIO_OFFSET 40
#define ADC_CHANNEL_TEMP_SENSOR 8
#endif

#define OVERSAMPLING 256

#define ADC_NUM_CHANNELS 5

// Total conversions per second across all channels. Per-channel finalize
// period is (ADC_NUM_CHANNELS * OVERSAMPLING) / rate = 500ms, which must stay
// comfortably under SUPPLY_VOLTAGE_STALE_THRESHOLD_MS (2000ms).
#define ADC_TARGET_SAMPLE_RATE 2560

// Ring buffer sizing. channel_config_set_ring takes the size in bytes as a
// power of two, and the buffer must be aligned to its size.
#define ADC_RING_BITS 12 // 4096 bytes = 2048 16-bit samples = 800ms at 2560 S/s
#define ADC_RING_BYTES (1u << ADC_RING_BITS)
#define ADC_RING_SAMPLES (ADC_RING_BYTES / sizeof(uint16_t))

// If this long passes between drains, the ring may have wrapped over unread
// samples and channel attribution can no longer be trusted; force a resync.
// (Normal drain interval is one 20ms tick.)
#define ADC_MAX_DRAIN_GAP_MS ((ADC_RING_SAMPLES * 1000 / ADC_TARGET_SAMPLE_RATE) / 2)

static uint16_t adc_ring[ADC_RING_SAMPLES] __attribute__((aligned(ADC_RING_BYTES)));

static sampler_t samples[9] = {0};
static int dma_channel;
static dma_channel_hw_t *dma_chan_hw;
static dma_channel_config dma_config;
static uint32_t last_read_addr;
static millis64_t last_drain_millis64;
static int sample_index = 0;

static float strap_pull_up, strap_pull_down, strap_open;

// One-off averaged reading of the board revision strap, with the given pull
// (+1 up, -1 down, 0 none). Only valid before the round robin is started.
static float read_strap(int pull) {
    if (pull > 0) {
        gpio_pull_up(PIN_BOARD_REV);
    } else if (pull < 0) {
        gpio_pull_down(PIN_BOARD_REV);
    } else {
        gpio_disable_pulls(PIN_BOARD_REV);
    }
    sleep_us(500);

    adc_select_input(PIN_BOARD_REV - ADC_CHANNEL_GPIO_OFFSET);
    (void)adc_read();
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc_read();
    }
    return sum / 16.0f;
}

void internal_adc_board_strap(float *pull_up, float *pull_down, float *open) {
    *pull_up = strap_pull_up;
    *pull_down = strap_pull_down;
    *open = strap_open;
}

static void internal_adc_start() {
    sample_index = 0;

    // Clear any FIFO overflow flag and stale samples
    hw_set_bits(&adc_hw->fcs, ADC_FCS_OVER_BITS);
    adc_fifo_drain();

    // (Re)arm the DMA ring from the start of the buffer
    dma_channel_configure(
        dma_channel,
        &dma_config,
        adc_ring,          // write to the ring buffer
        &adc_hw->fifo,     // read from the ADC FIFO
        dma_encode_endless_transfer_count(),
        true               // start now (idles until DREQ)
    );
    last_read_addr = dma_chan_hw->write_addr;
    // Live timer rather than the tick-cached millis64(), because init runs
    // before the first update_millis() (when the cached value is still 0)
    last_drain_millis64 = time_us_64() / 1000;

    // Round robin always starts from channel 0 so the sample order is known
    adc_select_input(0);
    adc_run(true);
}

void init_internal_adc() {
    adc_init();

    // Board revision strap. adc_gpio_init leaves the digital input disabled,
    // so the internal pulls can be used on a floating pin (RP2350-E9).
    adc_gpio_init(PIN_BOARD_REV);
    strap_pull_up = read_strap(1);
    strap_pull_down = read_strap(-1);
    strap_open = read_strap(0);

    adc_gpio_init(PIN_3V3_SENSE);
    adc_gpio_init(PIN_5V_SENSE);
    adc_gpio_init(PIN_12V_SENSE);
    adc_gpio_init(PIN_CONTACTOR_SENSE);

    adc_set_temp_sensor_enabled(true);

    adc_set_round_robin(0x10F); // First 4 channels + temp sensor

    // clk_adc is 48MHz by SDK default, but derive the divider from the actual
    // clock so the target sample rate holds even if clocks are reconfigured.
    adc_set_clkdiv((float)clock_get_hz(clk_adc) / ADC_TARGET_SAMPLE_RATE);

    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ asserted when at least 1 sample is present
        false,   // No error bit
        false    // No shift (keep full 12-bit samples)
    );

    dma_channel = dma_claim_unused_channel(false);
    if (dma_channel < 0) {
        panic("No free dma channels");
    }
    dma_chan_hw = dma_channel_hw_addr(dma_channel);

    dma_config = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_config, false);
    channel_config_set_write_increment(&dma_config, true);
    channel_config_set_dreq(&dma_config, DREQ_ADC);
    // Endless ring on the write side; wraps within the aligned buffer
    channel_config_set_ring(&dma_config, true, ADC_RING_BITS);

    internal_adc_start();
}

// Drain new samples from the DMA ring. Called once per main loop tick, so
// unlike the old FIFO-interrupt approach, samples keep flowing even while
// interrupts are disabled (e.g. during flash writes) - the DMA channel needs
// no CPU. Timestamps are applied by the sampler at finalize time using the
// tick-cached millis(), same resolution as before.
void internal_adc_check() {
    millis64_t now = millis64();

    bool overflowed = adc_hw->fcs & ADC_FCS_OVER_BITS;
    bool stalled = (now - last_drain_millis64) > ADC_MAX_DRAIN_GAP_MS;

    if (overflowed || stalled) {
        // FIFO overflow means DMA stopped draining (shouldn't happen); a long
        // gap between drains means the ring may have wrapped over unread
        // samples. Either way channel attribution is lost - restart cleanly.
        adc_run(false);
        dma_channel_abort(dma_channel);
        debug_printf("ADC resync (%s)\n", overflowed ? "fifo overflow" : "drain gap");
        internal_adc_start();
        return;
    }

    uint32_t write_addr = dma_chan_hw->write_addr;
    while (last_read_addr != write_addr) {
        uint16_t sample = *(volatile uint16_t *)last_read_addr;

        // Advance, wrapping within the (aligned) ring buffer
        last_read_addr =
            (last_read_addr & ~(ADC_RING_BYTES - 1))
            | ((last_read_addr + sizeof(uint16_t)) & (ADC_RING_BYTES - 1));

        sampler_add(&samples[sample_index], (int32_t)sample, OVERSAMPLING, 0);

        if (++sample_index >= ADC_NUM_CHANNELS) {
            sample_index = 0;
        }
    }
    last_drain_millis64 = now;
}

int32_t get_temperature_c_times10() {
    // based on pico-examples/adc/onboard_temperature/onboard_temperature.c
    const float conversionFactor = ( 3.3f / (1 << 12) ) / OVERSAMPLING;
    float adc = (float)samples[INTERNAL_ADC_TEMP_INDEX].value * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;
    return (int16_t)(tempC*10);
}

int32_t internal_adc_read(uint8_t channel) {
    return samples[channel].value;
}

// TODO - figure out why the ADCs are all off a bit

int32_t internal_adc_read_3v3_mv() {
    return (samples[INTERNAL_ADC_3V3_INDEX].value * 18) / 2878;
}

int32_t internal_adc_read_5v_mv() {
    return (samples[INTERNAL_ADC_5V_INDEX].value * 5) / 468;
}

int32_t internal_adc_read_12v_mv() {
    return (samples[INTERNAL_ADC_12V_INDEX].value * 40) / 1179;
}

int32_t internal_adc_read_contactor_mv() {
    return (samples[INTERNAL_ADC_CONTACTOR_INDEX].value * 40) / 1179;
}

millis_t internal_adc_read_3v3_millis() {
    return samples[INTERNAL_ADC_3V3_INDEX].timestamp;
}

millis_t internal_adc_read_5v_millis() {
    return samples[INTERNAL_ADC_5V_INDEX].timestamp;
}

millis_t internal_adc_read_12v_millis() {
    return samples[INTERNAL_ADC_12V_INDEX].timestamp;
}

millis_t internal_adc_read_contactor_millis() {
    return samples[INTERNAL_ADC_CONTACTOR_INDEX].timestamp;
}
