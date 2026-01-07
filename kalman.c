#include <stddef.h>
#include <stdint.h>

float LFP_SOC_VOLTAGES[102] = {
    2.540500f, 2.862982f, 2.971290f, 3.042848f, 3.097827f, 3.141087f, 3.173979f, 3.196349f, 3.205820f, 3.206718f, 3.210001f, 3.212566f, 3.214501f, 3.218610f, 3.224501f, 3.230588f, 3.235502f, 3.239824f, 3.243502f, 3.250502f, 3.256502f, 3.257502f, 3.264502f, 3.265502f, 3.269502f, 3.272499f, 3.276003f, 3.277307f, 3.281235f, 3.283003f, 3.287003f, 3.287003f, 3.287054f, 3.290503f, 3.291003f, 3.291003f, 3.291873f, 3.294504f, 3.295533f, 3.296613f, 3.297192f, 3.297504f, 3.297504f, 3.299432f, 3.300504f, 3.300798f, 3.301088f, 3.301505f, 3.301505f, 3.301505f, 3.301505f, 3.301505f, 3.301505f, 3.302230f, 3.305658f, 3.308005f, 3.308469f, 3.308527f, 3.309006f, 3.310709f, 3.312006f, 3.312506f, 3.312506f, 3.319028f, 3.319807f, 3.323097f, 3.323507f, 3.326507f, 3.328427f, 3.329507f, 3.330587f, 3.333667f, 3.334007f, 3.335348f, 3.337507f, 3.338008f, 3.338008f, 3.338008f, 3.338008f, 3.338008f, 3.339508f, 3.341465f, 3.342276f, 3.343316f, 3.344204f, 3.345008f, 3.345009f, 3.345009f, 3.345266f, 3.345509f, 3.345509f, 3.345509f, 3.345509f, 3.346009f, 3.348509f, 3.349010f, 3.349010f, 3.352010f, 3.352510f, 3.356329f, 3.366452f,
    3.55f
};

float capacity = 160.0f * 3600.0f;
float internal_resistance = 0.001f; // ohms

float process_noise = 1e-11f;
float measurement_noise = 0.1f;
float covariance = 0.1f;

float soc_estimate = 0.5f; // initial SOC estimate (0.0 to 1.0)

static float soc_to_ocv(float soc, float *jacobian) {
    if(soc < 0.0f) soc = 0.0f;
    else if(soc > 1.0f) soc = 1.0f;

    // if(soc <= 0.0f) return LFP_SOC_VOLTAGES[0];
    // if(soc >= 1.0f) return LFP_SOC_VOLTAGES[100];

    float soc_percent = soc * 100.0f;
    
    int lower_index = (int)soc_percent;
    int upper_index = lower_index + 1;
    
    float lower_voltage = LFP_SOC_VOLTAGES[lower_index];
    float upper_voltage = LFP_SOC_VOLTAGES[upper_index];

    float voltage = lower_voltage + (upper_voltage - lower_voltage) * (soc_percent - lower_index);

    if(jacobian != NULL) {
        *jacobian = (upper_voltage - lower_voltage) / (upper_index - lower_index) / 100.0f;
    }

    return voltage;
}


uint32_t kalman_update(int32_t charge_mC, int32_t current_mA, int32_t voltage_mV) {
    float charge_C = charge_mC / 1000.0f;
    //float dt_s = dt_ms / 1000.0f;
    float current_A = current_mA / 1000.0f;
    
    float voltage_V = (voltage_mV / 106) / 1000.0f;

    // Predict

    //(current_A * dt_s)
    float soc_prediction = soc_estimate - (charge_C / capacity);
    float covariance_prediction = covariance + process_noise;

    // Update

    float jacobian = 0;
    float estimated_ocv = soc_to_ocv(soc_prediction, &jacobian);
    float estimated_voltage = estimated_ocv - (current_A * internal_resistance);

    float innovation = voltage_V - estimated_voltage;

    float innovation_covariance = jacobian * covariance_prediction * jacobian + measurement_noise;
    float kalman_gain = covariance_prediction * jacobian / innovation_covariance;

    soc_estimate = soc_prediction + kalman_gain * innovation;
    if(soc_estimate < 0.0f) soc_estimate = 0.0f;
    else if(soc_estimate > 1.0f) soc_estimate = 1.0f;

    covariance = (1.0f - kalman_gain * jacobian) * covariance_prediction;

    return (uint32_t)(soc_estimate * 10000.0f);
}
