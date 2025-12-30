#include "../chip/time.h"
#include "../pins.h"
#include "../util/sampler.h"

#include "hardware/adc.h"

// this is 40 on rp2350b
#define ADC_CHANNEL_GPIO_OFFSET 26
#define ADC_CHANNEL_TEMP_SENSOR 4

#define ADC_SAMPLE_INDEX_12V_SENSE 0
#define ADC_SAMPLE_INDEX_TEMP_SENSOR 1

#define TEMP_OVERSAMPLING 256

#define SMOOTHING 64 // should be power of 2

uint16_t adc_samples_raw[8];
uint32_t adc_samples_smooth_accum[8];
uint16_t adc_samples_smoothed[8];
uint32_t adc_samples_variance_accum[8];
uint16_t adc_samples_variance[8];
millis_t adc_samples_millis[8];

sampler_t samples[8] = {0};

void __isr __not_in_flash_func(adc_irq_handler)() {
    int i = 0;
    for(int i=0; i < 8 && !adc_fifo_is_empty(); i++) {
        uint32_t sample = adc_fifo_get();
        // TODO - check for error bit?
        adc_samples_raw[i] = sample;
        adc_samples_smooth_accum[i] = (adc_samples_smooth_accum[i] * (SMOOTHING - 1) + sample * 256) / SMOOTHING;
        adc_samples_smoothed[i] = adc_samples_smooth_accum[i] / 256;
        int32_t diff = (int32_t)sample - (int32_t)adc_samples_smoothed[i];
        adc_samples_variance_accum[i] = (adc_samples_variance_accum[i] * (SMOOTHING - 1) + diff * diff * 256) / SMOOTHING;
        adc_samples_variance[i] = adc_samples_variance_accum[i] / 256;
        // Record when we got this sample
        adc_samples_millis[i] = millis();

        // Feed to sampler
        sampler_add(&samples[i], (int32_t)sample, TEMP_OVERSAMPLING, 0);

    }
}

int32_t get_temperature_c_times10() {
//    return samples[ADC_SAMPLE_INDEX_TEMP_SENSOR].value * 10;

    // based on pico-examples/adc/onboard_temperature/onboard_temperature.c
    const float conversionFactor = ( 3.3f / (1 << 12) ) / TEMP_OVERSAMPLING;
    //float adc = (float)adc_samples_smoothed[ADC_SAMPLE_INDEX_TEMP_SENSOR] * conversionFactor;
    float adc = (float)samples[ADC_SAMPLE_INDEX_TEMP_SENSOR].value * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;
    return (int16_t)(tempC*10);
}

void init_adc() {
    adc_init();

    adc_gpio_init(PIN_12V_SENSE);

    adc_set_temp_sensor_enabled(true);
    
    adc_set_round_robin(
        (1<<(PIN_12V_SENSE - ADC_CHANNEL_GPIO_OFFSET)) |
        (1<<ADC_CHANNEL_TEMP_SENSOR)
    );

    adc_set_clkdiv(150000.0f); // Cycles per sample (if too low (fast), the ISR can't keep up)

    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        false,   // Disable DMA data request (DREQ)
        2,       // IRQ threshold at 2 samples
        true,    // Include error bit
        false    // No shift
    );

    // setup irq
    adc_irq_set_enabled(true);

    irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_irq_handler);
    irq_set_enabled(ADC_IRQ_FIFO, true);

    adc_select_input(PIN_12V_SENSE - ADC_CHANNEL_GPIO_OFFSET);
    adc_run(true);
}