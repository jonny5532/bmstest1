#include <stdint.h>

typedef struct bms_model bms_model_t;

uint32_t kalman_update(int32_t charge_mC, int32_t current_mA, int32_t voltage_mV);
uint16_t voltage_based_soc_estimate(bms_model_t *model);
uint16_t basic_count_soc_estimate(bms_model_t *model);
uint16_t fancy_count_soc_estimate(bms_model_t *model);

float nmc_ocv_to_soc(float ocv);
