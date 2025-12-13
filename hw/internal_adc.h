#include <stdint.h>

extern uint16_t adc_samples_raw[8];
extern uint16_t adc_samples_smoothed[8];

void init_adc();
int32_t get_temperature_c_times10();
