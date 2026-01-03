#include "internal_adc.h"
#include "../chip/time.h"
#include "../pins.h"
#include "../util/sampler.h"

#include "hardware/adc.h"

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

//#define SMOOTHING 64 // should be power of 2

// uint16_t adc_samples_raw[8];
// uint32_t adc_samples_smooth_accum[8];
// uint16_t adc_samples_smoothed[8];
// uint32_t adc_samples_variance_accum[8];
// uint16_t adc_samples_variance[8];
// millis_t adc_samples_millis[8];

static sampler_t samples[9] = {0};

void __isr __not_in_flash_func(adc_irq_handler)() {
    int i = 0;
    for(int i=0; i < 8 && !adc_fifo_is_empty(); i++) {
        uint32_t sample = adc_fifo_get();
        // TODO - check for error bit?
        // adc_samples_raw[i] = sample;
        // adc_samples_smooth_accum[i] = (adc_samples_smooth_accum[i] * (SMOOTHING - 1) + sample * 256) / SMOOTHING;
        // adc_samples_smoothed[i] = adc_samples_smooth_accum[i] / 256;
        // int32_t diff = (int32_t)sample - (int32_t)adc_samples_smoothed[i];
        // adc_samples_variance_accum[i] = (adc_samples_variance_accum[i] * (SMOOTHING - 1) + diff * diff * 256) / SMOOTHING;
        // adc_samples_variance[i] = adc_samples_variance_accum[i] / 256;
        // // Record when we got this sample
        // adc_samples_millis[i] = millis();

        // Feed to sampler
        sampler_add(&samples[i], (int32_t)sample, OVERSAMPLING, 0);

    }

    // Always reset to first channel so we don't get out of sync
    adc_select_input(0);
}

int32_t get_temperature_c_times10() {
//    return samples[ADC_SAMPLE_INDEX_TEMP_SENSOR].value * 10;

    // based on pico-examples/adc/onboard_temperature/onboard_temperature.c
    const float conversionFactor = ( 3.3f / (1 << 12) ) / OVERSAMPLING;
    //float adc = (float)adc_samples_smoothed[ADC_SAMPLE_INDEX_TEMP_SENSOR] * conversionFactor;
    float adc = (float)samples[INTERNAL_ADC_TEMP_INDEX].value * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;
    return (int16_t)(tempC*10);
}

void init_internal_adc() {
    adc_init();

    adc_gpio_init(PIN_3V3_SENSE);
    adc_gpio_init(PIN_5V_SENSE);
    adc_gpio_init(PIN_12V_SENSE);
    adc_gpio_init(PIN_CONTACTOR_SENSE);

    adc_set_temp_sensor_enabled(true);
    
    // adc_set_round_robin(
    //     // (1<<(PIN_3V3_SENSE - ADC_CHANNEL_GPIO_OFFSET)) |
    //     // (1<<(PIN_5V_SENSE - ADC_CHANNEL_GPIO_OFFSET)) |
    //     // (1<<(PIN_12V_SENSE - ADC_CHANNEL_GPIO_OFFSET)) |
    //     // (1<<(PIN_CONTACTOR_SENSE - ADC_CHANNEL_GPIO_OFFSET)) |
    //     (1<<ADC_CHANNEL_TEMP_SENSOR)
    // );
    adc_set_round_robin(0x10F); // First 4 channels + temp sensor

    adc_set_clkdiv(150000.0f); // Cycles per sample (if too low (fast), the ISR can't keep up)

    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        false,   // Disable DMA data request (DREQ)
        5,       // IRQ threshold at 5 samples
        false,    // Include error bit
        false    // No shift
    );

    // setup irq
    adc_irq_set_enabled(true);

    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq_handler);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    adc_select_input(0);//PIN_12V_SENSE - ADC_CHANNEL_GPIO_OFFSET);
    adc_fifo_drain();
    adc_run(true);
}

int32_t internal_adc_read(uint8_t channel) {
    return samples[channel].value;
}

int32_t internal_adc_read_3v3_mv() {
    return samples[INTERNAL_ADC_3V3_INDEX].value / (int32_t)(
        1.0 / ((1.0/256.0)*(3300.0/4096.0)*(1/0.5))
    );
}

int32_t internal_adc_read_5v_mv() {
    return samples[INTERNAL_ADC_5V_INDEX].value / (int32_t)(
        1.0 / ((1.0/256.0)*(3300.0/4096.0)*(1.0/0.294118))
    );
}

int32_t internal_adc_read_12v_mv() {
    return samples[INTERNAL_ADC_12V_INDEX].value / (int32_t)(
        1.0 / ((1.0/256.0)*(3300.0/4096.0)*(1.0/0.083326))
    );
}

int32_t internal_adc_read_contactor_mv() {
    return samples[INTERNAL_ADC_CONTACTOR_INDEX].value / (int32_t)(
        1.0 / ((1.0/256.0)*(3300.0/4096.0)*(1.0/0.083326))
    );
}
