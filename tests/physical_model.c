#include "physical_model.h"

#include "app/model.h"
#include "config/limits.h"

#include <stdint.h>



static const float test_nmc_ocv_curve[] = {
    2.50005757, 3.10031943, 3.24625788, 3.33618626, 3.40073441,
    3.44897447, 3.46893947, 3.47395086, 3.47864389, 3.48333693,
    3.48834082, 3.49409536, 3.5013001 , 3.50967053, 3.51931585,
    3.52977034, 3.53957733, 3.54936385, 3.55979467, 3.56845999,
    3.57621643, 3.5856238 , 3.5952045 , 3.60126826, 3.60541126,
    3.60924125, 3.61263967, 3.61573646, 3.61867025, 3.6216477 ,
    3.62429166, 3.62713001, 3.62979388, 3.63270688, 3.6354027 ,
    3.6382091 , 3.64104817, 3.64373467, 3.64666554, 3.6496491 ,
    3.65261197, 3.65568686, 3.6591093 , 3.66244314, 3.666044  ,
    3.6696043 , 3.67341564, 3.67755864, 3.68202541, 3.6866434 ,
    3.69144414, 3.69704364, 3.70266863, 3.70969914, 3.71740244,
    3.72737144, 3.73953482, 3.75054572, 3.75889634, 3.76654077,
    3.77382326, 3.78142929, 3.78922413, 3.79728866, 3.80564429,
    3.8144326 , 3.82301974, 3.83208227, 3.8411448 , 3.85059132,
    3.86039399, 3.86997469, 3.88014603, 3.8899815 , 3.90001143,
    3.91023944, 3.92053556, 3.93082809, 3.94124702, 3.95179866,
    3.96235322, 3.973037  , 3.98377727, 3.99456048, 4.00572681,
    4.01705073, 4.02805948, 4.03954935, 4.05100517, 4.06243052,
    4.07434177, 4.0861864 , 4.09816332, 4.11036443, 4.12256861,
    4.13502932, 4.14774885, 4.16033888, 4.17348003, 4.18652358,
    4.20008564
};

// Implement the extern function required by current_limits.c
float nmc_ocv_to_soc(float ocv) {
    if (ocv <= test_nmc_ocv_curve[0]) return 0.0f;
    if (ocv >= test_nmc_ocv_curve[100]) return 1.0f;
    for (int i = 0; i < 100; i++) {
        if (ocv < test_nmc_ocv_curve[i + 1]) {
            float frac = (ocv - test_nmc_ocv_curve[i]) / (test_nmc_ocv_curve[i + 1] - test_nmc_ocv_curve[i]);
            return (i + frac) / 100.0f;
        }
    }
    return 1.0f;
}

float soc_to_ocv(float soc) {
    if (soc <= 0.0f) return test_nmc_ocv_curve[0];
    if (soc >= 1.0f) return test_nmc_ocv_curve[100];
    float index = soc * 100.0f;
    int i = (int)index;
    float frac = index - (float)i;
    return test_nmc_ocv_curve[i] * (1.0f - frac) + test_nmc_ocv_curve[i+1] * frac;
}

// Update the physical model and copy to the BMS model
void battery_model_tick(battery_model_t *bat, bms_model_t *model, float current_A, uint32_t dt_ms) {
    stored_millis += dt_ms;
    stored_millis64 += dt_ms;
    stored_timestep++;

    float dt_s = dt_ms / 1000.0f;
    // current_A: positive is charging
    bat->soc += (current_A * dt_s) / (bat->capacity_Ah * 3600.0f);
    if (bat->soc < 0) bat->soc = 0;
    if (bat->soc > 1.0) bat->soc = 1.0;

    float ocv = soc_to_ocv(bat->soc);
    // V = OCV + I*R
    float v_cell = ocv + current_A * bat->internal_resistance_Ohm;

    model->current_mA = (int32_t)(current_A * 1000.0f);
    model->current_millis = stored_millis;

    model->cell_voltage_min_mV = (int16_t)(v_cell * 1000.0f);
    model->cell_voltage_max_mV = (int16_t)(v_cell * 1000.0f);
    model->cell_voltage_millis = stored_millis;

    model->battery_voltage_mV = model->cell_voltage_min_mV * NUM_CELLS;
    model->cell_voltage_total_mV = model->battery_voltage_mV;
    model->battery_voltage_millis = stored_millis;

    model->module_temperatures_millis = stored_millis;

    printf("Battery: SoC %.2f, Cell Voltage %.2f V, Current %.2f A\n",
           bat->soc, v_cell, current_A);
}
