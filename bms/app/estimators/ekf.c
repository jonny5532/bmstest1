#include "ekf.h"
#include "../../sys/time/time.h"
#include "../model.h"

#include <stdbool.h>
#include <stdint.h>

static const float lfp_ocv_curve[101] = {
    2.50827118, 2.81277545, 2.92562818, 2.99923622, 3.05435954,
    3.09888556, 3.13558147, 3.16660405, 3.19173064, 3.20460002,
    3.20940259, 3.2123331 , 3.21494468, 3.2175122 , 3.22077868,
    3.22580475, 3.2311278 , 3.23607688, 3.2404376 , 3.24487523,
    3.2493354 , 3.25361751, 3.25750209, 3.26123834, 3.26433285,
    3.26753942, 3.27055384, 3.27341014, 3.27591078, 3.2786343 ,
    3.28124953, 3.28378858, 3.28656461, 3.28908066, 3.29175419,
    3.2943855 , 3.29723899, 3.29939813, 3.3008971 , 3.30190962,
    3.30260896, 3.30325985, 3.30354474, 3.30404527, 3.30438519,
    3.30469298, 3.30486427, 3.30525901, 3.30550225, 3.30592406,
    3.30622191, 3.30638122, 3.30682048, 3.30708543, 3.30730898,
    3.30762559, 3.30778387, 3.30814082, 3.3084771 , 3.30887748,
    3.30925707, 3.30965514, 3.31002083, 3.31064899, 3.31115627,
    3.31185438, 3.31254113, 3.31361151, 3.31468565, 3.3163257 ,
    3.31852673, 3.32137013, 3.32573451, 3.33095533, 3.3358285 ,
    3.33946827, 3.34166208, 3.34289795, 3.34364631, 3.34419716,
    3.34474578, 3.34496701, 3.34553225, 3.34586871, 3.34647481,
    3.34681898, 3.34731842, 3.34788944, 3.34849277, 3.34941575,
    3.35018322, 3.35131328, 3.35301375, 3.35503708, 3.35820093,
    3.36319514, 3.37059453, 3.38333911, 3.40687433, 3.45216287,
    3.54337311
};

static const float lfp_ocv_curve_diff[100] = {
    3.04504279e+01, 1.12852723e+01, 7.36080383e+00, 5.51233281e+00,
    4.45260149e+00, 3.66959150e+00, 3.10225816e+00, 2.51265842e+00,
    1.28693842e+00, 4.80256891e-01, 2.93050462e-01, 2.61157934e-01,
    2.56751913e-01, 3.26648183e-01, 5.02606972e-01, 5.32305790e-01,
    4.94907965e-01, 4.36072002e-01, 4.43762908e-01, 4.46017035e-01,
    4.28211109e-01, 3.88457429e-01, 3.73624734e-01, 3.09451285e-01,
    3.20657021e-01, 3.01442391e-01, 2.85629494e-01, 2.50064631e-01,
    2.72351310e-01, 2.61522934e-01, 2.53905605e-01, 2.77602187e-01,
    2.51605857e-01, 2.67352178e-01, 2.63131214e-01, 2.85348878e-01,
    2.15914181e-01, 1.49897034e-01, 1.01252482e-01, 6.99333444e-02,
    6.50894896e-02, 2.84886292e-02, 5.00528107e-02, 3.39919125e-02,
    3.07798386e-02, 1.71284905e-02, 3.94742400e-02, 2.43234253e-02,
    4.21812775e-02, 2.97851429e-02, 1.59312431e-02, 4.39257577e-02,
    2.64946995e-02, 2.23556717e-02, 3.16603521e-02, 1.58282885e-02,
    3.56952678e-02, 3.36276390e-02, 4.00384190e-02, 3.79584737e-02,
    3.98073290e-02, 3.65687399e-02, 6.28162463e-02, 5.07282226e-02,
    6.98110058e-02, 6.86744258e-02, 1.07038369e-01, 1.07413640e-01,
    1.64005338e-01, 2.20103399e-01, 2.84339974e-01, 4.36437416e-01,
    5.22081945e-01, 4.87316958e-01, 3.63976796e-01, 2.19380955e-01,
    1.23587336e-01, 7.48360482e-02, 5.50851299e-02, 5.48622327e-02,
    2.21231080e-02, 5.65240771e-02, 3.36453669e-02, 6.06097020e-02,
    3.44172818e-02, 4.99443193e-02, 5.71021557e-02, 6.03331964e-02,
    9.22976916e-02, 7.67467000e-02, 1.13005804e-01, 1.70047729e-01,
    2.02332608e-01, 3.16384891e-01, 4.99421087e-01, 7.39938801e-01,
    1.27445802e+00, 2.35352176e+00, 4.52885401e+00, 9.12102423e+00
};

static float nmc_ocv_curve[101] = {
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

static float nmc_ocv_curve_diff[100] = {
    60.02618656, 14.59384462,  8.99283794,  6.45481512,  4.82400582,
    1.99650039,  0.50113891,  0.46930313,  0.46930313,  0.50038961,
    0.57545365,  0.72047384,  0.83704336,  0.96453191,  1.04544938,
    0.98069849,  0.97865236,  1.04308176,  0.86653185,  0.77564423,
    0.94073653,  0.95807081,  0.60637599,  0.4143    ,  0.3829984 ,
    0.33984184,  0.30967921,  0.29337938,  0.29774512,  0.26439556,
    0.28383479,  0.26638761,  0.29129982,  0.26958231,  0.28064009,
    0.28390649,  0.26865009,  0.29308647,  0.29835633,  0.29628725,
    0.30748844,  0.3422442 ,  0.33338428,  0.36008567,  0.35603046,
    0.38113396,  0.4143    ,  0.44667724,  0.4617986 ,  0.48007407,
    0.55994991,  0.56249923,  0.7030505 ,  0.77033047,  0.99689964,
    1.21633849,  1.10109018,  0.83506111,  0.76444303,  0.72824955,
    0.76060295,  0.77948431,  0.80645224,  0.83556307,  0.87883157,
    0.85871403,  0.90625286,  0.90625286,  0.94465207,  0.98026644,
    0.95807081,  1.01713332,  0.98354717,  1.00299322,  1.02280092,
    1.02961159,  1.02925304,  1.04189365,  1.05516395,  1.05545602,
    1.06837823,  1.07402669,  1.07832087,  1.11663342,  1.13239178,
    1.10087505,  1.14898682,  1.14558201,  1.14253523,  1.19112451,
    1.18446342,  1.1976915 ,  1.22011146,  1.22041754,  1.24607086,
    1.27195335,  1.2590027 ,  1.3141156 ,  1.30435473,  1.35620603
};


// --- Helper: OCV Curve ---
// Returns Open Circuit Voltage for a given SOC
static float soc_to_ocv(float soc) {
    // Clamp SOC for safety
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;

    // Simple linear interpolation on the OCV curve
    float index = soc * 100.0f;
    int idx_lower = (int)index;
    if (idx_lower >= 100) return nmc_ocv_curve[100];
    int idx_upper = idx_lower + 1;
    float frac = index - (float)idx_lower;
    return nmc_ocv_curve[idx_lower] * (1.0f - frac) + nmc_ocv_curve[idx_upper] * frac;
}

// --- Helper: OCV Derivative ---
// Returns d(OCV)/d(SOC)
static float soc_to_ocv_derivative(float soc) {
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;

    float index = soc * 100.0f;
    int idx_lower = (int)index;
    if (idx_lower >= 100) return nmc_ocv_curve_diff[99];
    return nmc_ocv_curve_diff[idx_lower];
}

void ekf_init(EKF *ekf, float initial_soc, float initial_capacity) {
    // Initial States
    ekf->x[0] = (1.0f - initial_soc) * initial_capacity; // Ah_used
    ekf->x[1] = 0.0f;               // V_c1 starts relaxed
    ekf->x[2] = initial_capacity;   // Initial guess
    
    // Initial Covariance P (Identity * scalar)
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) ekf->P[i][j] = 0.0f;
    }
    ekf->P[0][0] = 0.001f; // Uncertainty in Ah_used
    ekf->P[1][1] = 0.01f;  // Uncertainty in V_c1
    ekf->P[2][2] = 1.0f;   // High uncertainty in Capacity

    // Process Noise Q
    ekf->Q[0] = 1e-5f;     // Trust current integration highly
    ekf->Q[1] = 1e-2f;     // V_c1 can vary
    ekf->Q[2] = 1e-7f;     // Capacity changes very slowly
    
    // Measurement Noise R
    ekf->R = 0.02f;        // Voltage sensor variance (e.g. 0.14V std dev)

    // Model Parameters (Example Cell)
    ekf->R0 = 0.02f;
    ekf->R1 = 0.03f;
    ekf->C1 = 2000.0f;
}

void ekf_step(EKF *ekf, float charge_Ah, float current_amps, float voltage_measured) {
    // -----------------------------------------
    // 1. PREDICTION STEP
    // -----------------------------------------
    float dt = 1.0f;
    
    // Positive current/charge = Charging

    // --- State Prediction ---
    // x[0] = Ah_used + I * dt / 3600
    ekf->x[0] = ekf->x[0] - charge_Ah;
    //(current_amps * ekf->dt) / 3600.0f;
    
    // x[1] = V_c1 * exp + I * R1 * (1 - exp)
    float exp_val = expf(-dt / (ekf->R1 * ekf->C1));
    ekf->x[1] = ekf->x[1] * exp_val - current_amps * ekf->R1 * (1.0f - exp_val);
    //printf("Predicted V_c1: %2.3f V | exp: %2.8f | I*R1*(1-exp): %2.8f V\n",
    //       ekf->x[1], exp_val, -current_amps * ekf->R1 * (1.0f - exp_val));    

    // x[2] Capacity stays constant in prediction
    // ekf->x[2] = ekf->x[2]; 

    // --- Covariance Prediction ---
    // F is Jacobian of process model.
    // Since Ah_used doesn't depend on Cap, F is diagonal-ish.
    // F = [1, 0, 0]
    //     [0, exp, 0]
    //     [0, 0, 1]
    
    // P_pred = F * P * F^T + Q
    // We update P elements manually taking advantage of F's sparsity.
    
    // Row 0 (Ah_used) -> multiplied by 1
    // Row 1 (V_c1)    -> multiplied by exp_val
    // Row 2 (Cap)     -> multiplied by 1
    
    // Update diagonals (simplified for sparse F)
    // Note: A full matrix multiply is safer if F becomes complex, 
    // but for this specific physics model, this manual update is efficient.
    
    // P[1][1] scales by exp^2
    ekf->P[1][1] = ekf->P[1][1] * exp_val * exp_val;
    
    // Cross terms involving row 1 or col 1 scale by exp
    ekf->P[0][1] *= exp_val;
    ekf->P[1][0] *= exp_val;
    ekf->P[1][2] *= exp_val;
    ekf->P[2][1] *= exp_val;

    // Add Process Noise Q
    ekf->P[0][0] += ekf->Q[0];
    ekf->P[1][1] += ekf->Q[1];
    ekf->P[2][2] += ekf->Q[2];

    // -----------------------------------------
    // 2. UPDATE STEP
    // -----------------------------------------

    float ah_used = ekf->x[0];
    float v_c1    = ekf->x[1];
    float cap     = ekf->x[2];

    // Calculate dynamic SOC: 1 - (Ah_used / Capacity)
    float soc_est = 1.0f - (ah_used / cap);
    
    // Predict Voltage
    float ocv = soc_to_ocv(soc_est);
    float v_pred = ocv - v_c1 + (current_amps * ekf->R0);
    //printf("v_pred: %2.3f V | OCV: %2.3f V | V_c1: %2.3f V | I*R0: %2.3f V | SOC_est: %2.2f %%\n",
    //       v_pred, ocv, v_c1, -current_amps * ekf->R0, soc_est * 100.0f);
    
    // Calculate Residual (y)
    float y = voltage_measured - v_pred;

    // --- Calculate Jacobian H ---
    // H = [dH/dAh, dH/dVc1, dH/dCap]
    float d_ocv = soc_to_ocv_derivative(soc_est);
    
    // dV/dAh = dOCV/dSOC * dSOC/dAh = dOCV * (-1/Cap)
    float h0 = d_ocv * (-1.0f / cap);
    
    // dV/dVc1 = -1
    float h1 = -1.0f;
    
    // dV/dCap = dOCV/dSOC * dSOC/dCap = dOCV * (Ah / Cap^2)
    float h2 = d_ocv * (ah_used / (cap * cap));
    
    float H[3] = {h0, h1, h2};

    // --- Calculate Kalman Gain K ---
    // S = H * P * H^T + R (Scalar, since measurement is 1D)
    
    // H * P (1x3 vector temp)
    float HP[3];
    HP[0] = H[0]*ekf->P[0][0] + H[1]*ekf->P[1][0] + H[2]*ekf->P[2][0];
    HP[1] = H[0]*ekf->P[0][1] + H[1]*ekf->P[1][1] + H[2]*ekf->P[2][1];
    HP[2] = H[0]*ekf->P[0][2] + H[1]*ekf->P[1][2] + H[2]*ekf->P[2][2];

    // S = (HP) * H^T + R
    float S = (HP[0]*H[0] + HP[1]*H[1] + HP[2]*H[2]) + ekf->R;
    
    // K = P * H^T * (1/S) -> (3x1 vector)
    float K[3];
    // PH_T corresponds to the column vector resulting from P * H^T
    // Since P is symmetric, P * H^T is the same as (H * P)^T which is HP computed above
    // EXCEPT we need to be careful with indices. Let's do it explicitly:
    // K[i] = (sum(P[i][j] * H[j])) / S
    for(int i=0; i<3; i++) {
        float sum = 0.0f;
        for(int j=0; j<3; j++) {
            sum += ekf->P[i][j] * H[j];
        }
        K[i] = sum / S;
    }

    // --- Update State Vector ---
    // x = x + K * y
    for(int i=0; i<3; i++) {
        ekf->x[i] += K[i] * y;
    }

    // Constraints
    // Capacity cannot be <= 0
    if(ekf->x[2] < 0.1f) ekf->x[2] = 0.1f; 
    // Ah_used cannot be negative (cannot be "more than full")
    if(ekf->x[0] < 0.0f) ekf->x[0] = 0.0f;

    // --- Update Covariance P ---
    // P = (I - K * H) * P
    // We use a temporary matrix to store (I - KH)
    float I_KH[3][3];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            float identity = (i == j) ? 1.0f : 0.0f;
            I_KH[i][j] = identity - (K[i] * H[j]);
        }
    }

    // Now P_new = I_KH * P_old
    // We need a temp buffer for P to avoid overwriting while reading
    float P_new[3][3];
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            P_new[i][j] = 0.0f;
            for(int k=0; k<3; k++) {
                P_new[i][j] += I_KH[i][k] * ekf->P[k][j];
            }
        }
    }

    // Copy back
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            ekf->P[i][j] = P_new[i][j];
        }
    }
}

float ekf_get_soc(EKF *ekf) {
    float ah = ekf->x[0];
    float cap = ekf->x[2];
    if (cap < 0.0001f) return 0.0f; // Prevent div by zero
    return 1.0f - (ah / cap);
}

static bool initialized = false;
static EKF ekf_instance;

uint32_t ekf_tick(int32_t charge_mC, int32_t current_mA, int32_t voltage_mV) {
    float charge_Ah = (float)charge_mC / 3600000.0f; // Convert mC to Ah
    float current_amps = (float)current_mA / 1000.0f;      // Convert mA to A
    float voltage_volts = (float)voltage_mV / 1000.0f;     // Convert mV to V

    if (!initialized && voltage_mV > 0.0f && timestep() > 200) {
        float initial_soc = 1.0f;
        for(int i=0; i<10; i++) {
            initial_soc += (voltage_volts - soc_to_ocv(initial_soc)); // Simple convergence
        }

        float initial_capacity_ah = model.capacity_mC / 3600000; // in Ah
        ekf_init(&ekf_instance, initial_soc, initial_capacity_ah);

        initialized = true;
    } else if(!initialized) {
        return 0; // Not initialized yet

    }

    // if(timestep() > 300) {
    //     current_amps -= 1.0f;
    //     charge_Ah -= 1.0f / 3600.0f;
    // }


    ekf_step(&ekf_instance, charge_Ah, current_amps, voltage_volts);

    float soc = ekf_get_soc(&ekf_instance);
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;

    return (uint32_t)(soc * 10000.0f); // Return SOC in 0.01% units
}
