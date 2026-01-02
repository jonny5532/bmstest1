#include <stdint.h>

#define INTERNAL_ADC_3V3_INDEX 0
#define INTERNAL_ADC_5V_INDEX 1
#define INTERNAL_ADC_12V_INDEX 2
#define INTERNAL_ADC_CONTACTOR_INDEX 3
#define INTERNAL_ADC_TEMP_INDEX 4

// extern uint16_t adc_samples_raw[8];
// extern uint16_t adc_samples_smoothed[8];

void init_adc();
int32_t get_temperature_c_times10();
int32_t internal_adc_read_3v3_mv();
int32_t internal_adc_read_5v_mv();
int32_t internal_adc_read_12v_mv();
int32_t internal_adc_read_contactor_mv();
