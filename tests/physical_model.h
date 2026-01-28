#include <stdint.h>

typedef struct {
    float capacity_Ah;
    float soc;
    float internal_resistance_Ohm;
} battery_model_t;

typedef struct bms_model bms_model_t;

float soc_to_ocv(float soc);
void battery_model_tick(battery_model_t *bat, bms_model_t *model, float current_A, uint32_t dt_ms);