#include "../hw/chip/time.h"
#include "../limits.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

float LFP_SOC_VOLTAGES[102] = {
    2.540500f, 2.862982f, 2.971290f, 3.042848f, 3.097827f, 3.141087f, 3.173979f, 
    3.196349f, 3.205820f, 3.206718f, 3.210001f, 3.212566f, 3.214501f, 3.218610f, 
    3.224501f, 3.230588f, 3.235502f, 3.239824f, 3.243502f, 3.250502f, 3.256502f, 
    3.257502f, 3.264502f, 3.265502f, 3.269502f, 3.272499f, 3.276003f, 3.277307f, 
    3.281235f, 3.283003f, 3.287003f, 3.287003f, 3.287054f, 3.290503f, 3.291003f, 
    3.291003f, 3.291873f, 3.294504f, 3.295533f, 3.296613f, 3.297192f, 3.297504f, 
    3.297504f, 3.299432f, 3.300504f, 3.300798f, 3.301088f, 3.301505f, 3.301505f, 
    3.301505f, 3.301505f, 3.301505f, 3.301505f, 3.302230f, 3.305658f, 3.308005f, 
    3.308469f, 3.308527f, 3.309006f, 3.310709f, 3.312006f, 3.312506f, 3.312506f, 
    3.319028f, 3.319807f, 3.323097f, 3.323507f, 3.326507f, 3.328427f, 3.329507f, 
    3.330587f, 3.333667f, 3.334007f, 3.335348f, 3.337507f, 3.338008f, 3.338008f, 
    3.338008f, 3.338008f, 3.338008f, 3.339508f, 3.341465f, 3.342276f, 3.343316f, 
    3.344204f, 3.345008f, 3.345009f, 3.345009f, 3.345266f, 3.345509f, 3.345509f, 
    3.345509f, 3.345509f, 3.346009f, 3.348509f, 3.349010f, 3.349010f, 3.352010f, 
    3.352510f, 3.356329f, 3.366452f,
    3.55f
};

float NMC_SOC_VOLTAGES[201] = {
    2.5f,        2.9439919f,  3.10031943f, 3.18455733f, 3.24625788f, 3.29535356f,
    3.33618626f, 3.37079542f, 3.40073441f, 3.42678965f, 3.44897447f, 3.46359372f,
    3.46893947f, 3.47152328f, 3.47395086f, 3.4763782f,  3.47864389f, 3.48090959f,
    3.48333693f, 3.48562202f, 3.48834082f, 3.49122151f, 3.49409536f, 3.49757814f,
    3.5013001f,  3.50534606f, 3.50967053f, 3.51449319f, 3.51931585f, 3.5243203f,
    3.52977034f, 3.53463745f, 3.53957733f, 3.54439999f, 3.54936385f, 3.55436882f,
    3.55979467f, 3.56433782f, 3.56845999f, 3.57220298f, 3.57621643f, 3.58071533f,
    3.5856238f,  3.59052238f, 3.5952045f,  3.59872222f, 3.60126826f, 3.60333976f,
    3.60541126f, 3.60732088f, 3.60924125f, 3.61102152f, 3.61263967f, 3.61436572f,
    3.61573646f, 3.61717105f, 3.61867025f, 3.62017092f, 3.6216477f,  3.62299705f,
    3.62429166f, 3.62574816f, 3.62713001f, 3.62866092f, 3.62979388f, 3.63141203f,
    3.63270688f, 3.6340148f,  3.6354027f,  3.63691449f, 3.6382091f,  3.63958235f,
    3.64104817f, 3.64236107f, 3.64373467f, 3.64500594f, 3.64666554f, 3.64816901f,
    3.6496491f,  3.65117917f, 3.65261197f, 3.65427121f, 3.65568686f, 3.65741052f,
    3.6591093f,  3.66070342f, 3.66244314f, 3.66419097f, 3.666044f,   3.66768656f,
    3.6696043f,  3.67154622f, 3.67341564f, 3.67553486f, 3.67755864f, 3.67963767f,
    3.68202541f, 3.6843307f,  3.6866434f,  3.68918562f, 3.69144414f, 3.69416295f,
    3.69704364f, 3.69986653f, 3.70266863f, 3.70617114f, 3.70969914f, 3.71329856f,
    3.71740244f, 3.72206317f, 3.72737144f, 3.73336554f, 3.73953482f, 3.74550295f,
    3.75054572f, 3.75488292f, 3.75889634f, 3.76298046f, 3.76654077f, 3.77012764f,
    3.77382326f, 3.77754545f, 3.78142929f, 3.78537236f, 3.78922413f, 3.79323217f,
    3.79728866f, 3.80140437f, 3.80564429f, 3.80977689f, 3.8144326f,  3.81845127f,
    3.82301974f, 3.82755113f, 3.83208227f, 3.83645177f, 3.8411448f,  3.8459071f,
    3.85059132f, 3.85541398f, 3.86039399f, 3.86509585f, 3.86997469f, 3.87502815f,
    3.88014603f, 3.88500094f, 3.8899815f,  3.89505919f, 3.90001143f, 3.90496347f,
    3.91023944f, 3.91526341f, 3.92053556f, 3.92568185f, 3.93082809f, 3.93597447f,
    3.94124702f, 3.94659063f, 3.95179866f, 3.9569127f,  3.96235322f, 3.96769643f,
    3.973037f,   3.97843944f, 3.98377727f, 3.9892174f,  3.99456048f, 4.00015737f,
    4.00572681f, 4.01122904f, 4.01705073f, 4.02248834f, 4.02805948f, 4.03378627f,
    4.03954935f, 4.04521143f, 4.05100517f, 4.05670357f, 4.06243052f, 4.06822426f,
    4.07434177f, 4.07997339f, 4.0861864f,  4.09220793f, 4.09816332f, 4.10428035f,
    4.11036443f, 4.11635325f, 4.12256861f, 4.12884839f, 4.13502932f, 4.14130776f,
    4.14774885f, 4.15409207f, 4.16033888f, 4.16690981f, 4.17348003f, 4.17995355f,
    4.18652358f, 4.19332106f, 4.2f,
};

float nmc_ocv_to_soc(float ocv) {
    if(ocv <= NMC_SOC_VOLTAGES[0]) return 0.0f;
    if(ocv >= NMC_SOC_VOLTAGES[200]) return 1.0f;

    for(size_t i = 0; i < 200; i++) {
        if(ocv < NMC_SOC_VOLTAGES[i + 1]) {
            float lower_voltage = NMC_SOC_VOLTAGES[i];
            float upper_voltage = NMC_SOC_VOLTAGES[i + 1];
            float soc = (float)i / 200.0f;
            float next_soc = (float)(i + 1) / 200.0f;

            return soc + (next_soc - soc) * (ocv - lower_voltage) / (upper_voltage - lower_voltage);
        }
    }

    return 1.0f;
}




//float capacity = 160.0f * 3600.0f;
float capacity = 2.0f * 3600.0f; // 2Ah in Coulombs
float internal_resistance = 0.001f; // ohms

float process_noise = 1e-11f;
float measurement_noise = 0.1f;
//float covariance = 0.1f;
float covariance = 0.1f;

float discharge_estimate = 0.0f; // 0 = full charge
//float soc_estimate = 0.5f; // initial SOC estimate (0.0 to 1.0)
bool initialized = false;

static float lfp_soc_to_ocv(float soc, float *jacobian) {
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
        *jacobian = (upper_voltage - lower_voltage) / (upper_index - lower_index);// / 100.0f;
    }

    return voltage;
}

static float nmc_soc_to_ocv(float soc, float *jacobian) {
    if(soc < 0.0f) soc = 0.0f;
    else if(soc > 1.0f) soc = 1.0f;

    float soc_percent = soc * 200.0f;
    
    int lower_index = (int)soc_percent;
    int upper_index = lower_index + 1;

    if(upper_index > 200) {
        upper_index = 200;
        lower_index = 199;
    }
    
    float lower_voltage = NMC_SOC_VOLTAGES[lower_index];
    float upper_voltage = NMC_SOC_VOLTAGES[upper_index];

    float voltage = lower_voltage + (upper_voltage - lower_voltage) * (soc_percent - lower_index);

    if(jacobian != NULL) {
        *jacobian = (upper_voltage - lower_voltage) / (upper_index - lower_index) / 200.0f;
    }

    return voltage;
}


uint32_t kalman_update(int32_t charge_mC, int32_t current_mA, int32_t voltage_mV) {
    float charge_C = charge_mC / 1000.0f;
    //float dt_s = dt_ms / 1000.0f;
    float current_A = current_mA / 1000.0f;
    
    float voltage_V = voltage_mV / 1000.0f;

    //printf("Voltage is %.3f V\n", voltage_V);
    if(!initialized && voltage_V > 0.0f && timestep() > 200) {
        // Initialize SOC estimate based on OCV
        float soc_estimate = nmc_ocv_to_soc(voltage_V);
        discharge_estimate = (1.0f - soc_estimate) * capacity;
        initialized = true;
        //printf("Kalman filter initialized: OCV=%.3f V, initial SOC=%.5f\n", voltage_V, soc_estimate);
    }


    // Predict

    //(current_A * dt_s)
    //float soc_prediction = soc_estimate + (charge_C / capacity);
    float discharge_prediction = discharge_estimate - charge_C;
    float covariance_prediction = covariance + process_noise;
    float soc_prediction = 1.0f - (discharge_prediction / capacity);

    // Update

    float jacobian = 0;
    float estimated_ocv = nmc_soc_to_ocv(soc_prediction, &jacobian);
    float estimated_voltage = estimated_ocv + (current_A * internal_resistance);

    float innovation = voltage_V - estimated_voltage;

    float innovation_covariance = jacobian * covariance_prediction * jacobian + measurement_noise;
    float kalman_gain = covariance_prediction * jacobian / innovation_covariance;

    discharge_estimate = discharge_prediction + kalman_gain * innovation;
    //soc_estimate = soc_prediction + kalman_gain * innovation;
    // printf("Kalman update: soc_prediction=%.5f, jacobian=%.5f, estimated_ocv=%.3f V, estimated_voltage=%.3f V, innovation=%.3f V, kalman_gain=%.5f, soc_estimate=%.5f\n",
    //     soc_prediction,
    //     jacobian,
    //     estimated_ocv,
    //     estimated_voltage,
    //     innovation,
    //     kalman_gain,
    //     soc_estimate
    // );
    if(soc_estimate < 0.0f) soc_estimate = 0.0f;
    else if(soc_estimate > 1.0f) soc_estimate = 1.0f;

    covariance = (1.0f - kalman_gain * jacobian) * covariance_prediction;

    return (uint32_t)(soc_estimate * 10000.0f);
}
