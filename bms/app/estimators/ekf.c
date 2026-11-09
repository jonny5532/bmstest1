#include "ekf.h"
#include "ocv_learn.h"
#include "config/limits.h"
#include "sys/time/time.h"
#include "app/model.h"
#include "sys/logging/logging.h"

#include <stdbool.h>
#include <stdint.h>

// LFP_A (kalman/tables/LFP_A_soc_ocv.csv), midpoint of the upper and lower branches.
// The previous curve is lfp1 below.
static const float lfp_ocv_curve[101] = {
    2.916f,     3.0034f,    3.0734f,    3.1256f,    3.1644f,
    3.1924f,    3.2058f,    3.2089f,    3.2106f,    3.2119f,
    3.2136f,    3.2158f,    3.2196f,    3.2256f,    3.2324f,
    3.2385f,    3.244f,     3.2496f,    3.2551f,    3.26f,
    3.264f,     3.2677f,    3.2715f,    3.2753f,    3.2786f,
    3.2814f,    3.284f,     3.2866f,    3.2888f,    3.2905f,
    3.2922f,    3.2945f,    3.296f,     3.2966f,    3.297f,
    3.2972f,    3.2976f,    3.2978f,    3.298f,     3.2982f,
    3.2985f,    3.2986f,    3.2989f,    3.2992f,    3.2995f,
    3.2998f,    3.3f,       3.3004f,    3.3007f,    3.301f,
    3.3014f,    3.3018f,    3.3023f,    3.3028f,    3.3034f,
    3.3042f,    3.3052f,    3.3066f,    3.3084f,    3.3104f,
    3.3128f,    3.3183f,    3.3265f,    3.3324f,    3.3342f,
    3.3348f,    3.335f,     3.3352f,    3.3353f,    3.3354f,
    3.3354f,    3.3354f,    3.3355f,    3.3356f,    3.3356f,
    3.3357f,    3.3358f,    3.3358f,    3.3359f,    3.336f,
    3.336f,     3.3362f,    3.3362f,    3.3362f,    3.3364f,
    3.3364f,    3.3366f,    3.3366f,    3.3368f,    3.3369f,
    3.337f,     3.3372f,    3.3374f,    3.3375f,    3.3377f,
    3.338f,     3.3384f,    3.3387f,    3.3404f,    3.3642f,
    3.5252f
};

//     2.50827118, 2.81277545, 2.92562818, 2.99923622, 3.05435954,
//     3.09888556, 3.13558147, 3.16660405, 3.19173064, 3.20460002,
//     3.20940259, 3.2123331 , 3.21494468, 3.2175122 , 3.22077868,
//     3.22580475, 3.2311278 , 3.23607688, 3.2404376 , 3.24487523,
//     3.2493354 , 3.25361751, 3.25750209, 3.26123834, 3.26433285,
//     3.26753942, 3.27055384, 3.27341014, 3.27591078, 3.2786343 ,
//     3.28124953, 3.28378858, 3.28656461, 3.28908066, 3.29175419,
//     3.2943855 , 3.29723899, 3.29939813, 3.3008971 , 3.30190962,
//     3.30260896, 3.30325985, 3.30354474, 3.30404527, 3.30438519,
//     3.30469298, 3.30486427, 3.30525901, 3.30550225, 3.30592406,
//     3.30622191, 3.30638122, 3.30682048, 3.30708543, 3.30730898,
//     3.30762559, 3.30778387, 3.30814082, 3.3084771 , 3.30887748,
//     3.30925707, 3.30965514, 3.31002083, 3.31064899, 3.31115627,
//     3.31185438, 3.31254113, 3.31361151, 3.31468565, 3.3163257 ,
//     3.31852673, 3.32137013, 3.32573451, 3.33095533, 3.3358285 ,
//     3.33946827, 3.34166208, 3.34289795, 3.34364631, 3.34419716,
//     3.34474578, 3.34496701, 3.34553225, 3.34586871, 3.34647481,
//     3.34681898, 3.34731842, 3.34788944, 3.34849277, 3.34941575,
//     3.35018322, 3.35131328, 3.35301375, 3.35503708, 3.35820093,
//     3.36319514, 3.37059453, 3.38333911, 3.40687433, 3.45216287,
//     3.54337311
// };


// use nmc5? looks most normal, widest range
// nmc1 { 2.9f, 2.9551f, 2.9992f, 3.0358f, 3.0669f, 3.0938f, 3.1189f, 3.142f, 3.1640999f, 3.1824999f, 3.1952f, 3.2103f, 3.2255f, 3.2419f, 3.2600999f, 3.2804f, 3.3025f, 3.3261f, 3.3501f, 3.3733f, 3.3954f, 3.4163f, 3.4347f, 3.4512f, 3.4640999f, 3.4726f, 3.4798999f, 3.4870999f, 3.4930999f, 3.5016999f, 3.5132f, 3.5269f, 3.5402999f, 3.5539f, 3.5676999f, 3.5799999f, 3.5908999f, 3.6017f, 3.6127999f, 3.6236f, 3.6338999f, 3.6434f, 3.6526f, 3.6615f, 3.67f, 3.6786f, 3.6872f, 3.696f, 3.7051f, 3.7144f, 3.724f, 3.7337999f, 3.7437999f, 3.7537f, 3.7636f, 3.7735f, 3.7832f, 3.7927999f, 3.8022f, 3.8113f, 3.8204f, 3.8295f, 3.8390999f, 3.8499999f, 3.8626f, 3.8763f, 3.8889999f, 3.8994999f, 3.9084f, 3.9165f, 3.9244f, 3.9324f, 3.9409f, 3.9498999f, 3.9595f, 3.9698f, 3.9807f, 3.9921999f, 4.0040998f, 4.0160999f, 4.0278f, 4.0388999f, 4.0490999f, 4.0583f, 4.0664f, 4.0735f, 4.0794f, 4.0844998f, 4.089f, 4.0931f, 4.0971999f, 4.1012f, 4.1055999f, 4.1103f, 4.1156998f, 4.1217999f, 4.1290998f, 4.138f, 4.1493f, 4.1643f, 4.1866999f }
// nmc3 { 2.9f, 2.9749999f, 3.0239999f, 3.0639999f, 3.099f, 3.125f, 3.15f, 3.178f, 3.203f, 3.23f, 3.256f, 3.283f, 3.309f, 3.3329999f, 3.355f, 3.3759999f, 3.395f, 3.4119999f, 3.4289999f, 3.443f, 3.454f, 3.461f, 3.4679999f, 3.4749999f, 3.482f, 3.49f, 3.499f, 3.51f, 3.523f, 3.536f, 3.549f, 3.562f, 3.5739999f, 3.584f, 3.595f, 3.605f, 3.615f, 3.625f, 3.635f, 3.644f, 3.654f, 3.663f, 3.671f, 3.6789999f, 3.688f, 3.696f, 3.7049999f, 3.7149999f, 3.724f, 3.734f, 3.744f, 3.753f, 3.763f, 3.773f, 3.782f, 3.792f, 3.801f, 3.8099999f, 3.818f, 3.8269999f, 3.836f, 3.846f, 3.858f, 3.8729999f, 3.888f, 3.898f, 3.9059999f, 3.913f, 3.921f, 3.928f, 3.937f, 3.9449999f, 3.954f, 3.964f, 3.9749999f, 3.986f, 3.9979999f, 4.01f, 4.0209999f, 4.033f, 4.043f, 4.053f, 4.0609999f, 4.0689998f, 4.0749998f, 4.0799999f, 4.085f, 4.089f, 4.0929999f, 4.096f, 4.0999999f, 4.104f, 4.109f, 4.1139998f, 4.1199999f, 4.1269999f, 4.135f, 4.145f, 4.158f, 4.1739998f, 4.196f }
// nmc4 { 2.9f, 3.0415f, 3.14f, 3.2147f, 3.2749f, 3.3218f, 3.3525f, 3.3683f, 3.3787f, 3.3877f, 3.3961f, 3.4040999f, 3.4124f, 3.4223f, 3.4347f, 3.4482999f, 3.4619f, 3.4756999f, 3.4881f, 3.4991f, 3.5088999f, 3.5186f, 3.5281f, 3.5374f, 3.5462f, 3.5546f, 3.5627f, 3.5706999f, 3.5781f, 3.585f, 3.5913999f, 3.5975f, 3.6033f, 3.6089f, 3.6143f, 3.6198f, 3.6252999f, 3.6307f, 3.6362f, 3.6417999f, 3.6475f, 3.6535f, 3.6598f, 3.6664f, 3.6733999f, 3.6809f, 3.6888f, 3.6973f, 3.7065f, 3.7162f, 3.7267f, 3.7384f, 3.7516f, 3.7667f, 3.7827f, 3.7981f, 3.8116f, 3.8234f, 3.8341f, 3.8439f, 3.8532f, 3.8620999f, 3.8706f, 3.8787999f, 3.8866f, 3.8938f, 3.9008f, 3.9075999f, 3.9144f, 3.9214f, 3.9287f, 3.9365f, 3.9451f, 3.9544f, 3.9645f, 3.9754f, 3.9869f, 3.9986999f, 4.0106f, 4.0221f, 4.0328999f, 4.0426998f, 4.0514f, 4.0587f, 4.0647998f, 4.0695f, 4.073f, 4.0757999f, 4.0781999f, 4.0805f, 4.083f, 4.086f, 4.0895f, 4.0938f, 4.0991f, 4.1057f, 4.1141f, 4.1248999f, 4.1394f, 4.1594f, 4.1877999f };
// nmc5 { 2.9f, 3.0443f, 3.1399f, 3.2162f, 3.2762f, 3.3229f, 3.3527f, 3.3664999f, 3.3755f, 3.3834f, 3.3908f, 3.3982999f, 3.4063f, 3.4164f, 3.4296f, 3.4435f, 3.4572999f, 3.4716f, 3.4847f, 3.4958999f, 3.5062f, 3.5165f, 3.5265999f, 3.5360999f, 3.5448f, 3.553f, 3.5606999f, 3.5682f, 3.5754f, 3.582f, 3.588f, 3.5934999f, 3.5987f, 3.6036999f, 3.6085999f, 3.6133f, 3.6180999f, 3.6228f, 3.6275f, 3.6322999f, 3.6372f, 3.6422f, 3.6475f, 3.6529f, 3.6585f, 3.6645f, 3.6707f, 3.6775f, 3.6847999f, 3.6926f, 3.7012f, 3.7107999f, 3.7218f, 3.7346f, 3.7492f, 3.7646999f, 3.7794f, 3.7925f, 3.8043f, 3.8152f, 3.8255f, 3.8355f, 3.845f, 3.8543f, 3.8634f, 3.8722f, 3.8806f, 3.8889f, 3.897f, 3.9049f, 3.9128f, 3.9207f, 3.9288f, 3.9370999f, 3.9458f, 3.9549f, 3.9644f, 3.9744f, 3.9849f, 3.9956999f, 4.0068f, 4.0179f, 4.0288f, 4.0394f, 4.0495f, 4.0587f, 4.0668998f, 4.0739999f, 4.0798998f, 4.0844998f, 4.0883999f, 4.092f, 4.0956998f, 4.0998998f, 4.1048f, 4.1108999f, 4.1188f, 4.1290998f, 4.143f, 4.1617999f, 4.1880999f }
// nmc2 { 2.9f, 3.048f, 3.1429999f, 3.206f, 3.2479999f, 3.277f, 3.301f, 3.323f, 3.3429999f, 3.365f, 3.385f, 3.408f, 3.4219999f, 3.43f, 3.437f, 3.444f, 3.451f, 3.459f, 3.4679999f, 3.479f, 3.4909999f, 3.5039999f, 3.515f, 3.527f, 3.54f, 3.552f, 3.562f, 3.573f, 3.585f, 3.595f, 3.6029999f, 3.611f, 3.619f, 3.6259999f, 3.6329999f, 3.6389999f, 3.6459999f, 3.652f, 3.6589999f, 3.6659999f, 3.673f, 3.68f, 3.687f, 3.6949999f, 3.704f, 3.7119999f, 3.721f, 3.7309999f, 3.74f, 3.75f, 3.76f, 3.77f, 3.78f, 3.79f, 3.8f, 3.8099999f, 3.8199999f, 3.8299999f, 3.8399999f, 3.849f, 3.8599999f, 3.874f, 3.888f, 3.898f, 3.905f, 3.9119999f, 3.92f, 3.927f, 3.934f, 3.9419999f, 3.95f, 3.959f, 3.9679999f, 3.9779999f, 3.989f, 4.0f, 4.012f, 4.024f, 4.0349998f, 4.046f, 4.056f, 4.065f, 4.073f, 4.0799999f, 4.086f, 4.091f, 4.0949998f, 4.099f, 4.103f, 4.1069999f, 4.111f, 4.1149998f, 4.119f, 4.125f, 4.13f, 4.137f, 4.1459999f, 4.155f, 4.1669998f, 4.181f, 4.196 };
// nmc6 { 2.9f, 2.9621999f, 3.0113f, 3.0511999f, 3.0848f, 3.1145f, 3.1414f, 3.1671f, 3.1867f, 3.2089f, 3.2318f, 3.256f, 3.2811f, 3.3062f, 3.3304999f, 3.3534999f, 3.3747f, 3.3949f, 3.4131999f, 3.4301f, 3.4449f, 3.4582f, 3.4665f, 3.4737f, 3.4805f, 3.4876f, 3.4939f, 3.5018f, 3.5128f, 3.5260999f, 3.5399f, 3.5534999f, 3.5676f, 3.5799f, 3.5906999f, 3.6012f, 3.612f, 3.6224999f, 3.6321f, 3.6414f, 3.65f, 3.6591f, 3.6677f, 3.6761f, 3.6845f, 3.6931f, 3.7018f, 3.7109f, 3.7202f, 3.7297f, 3.7392f, 3.7488f, 3.7585f, 3.7681f, 3.7776f, 3.7867f, 3.7959f, 3.8048f, 3.8134f, 3.8218999f, 3.8304999f, 3.8392999f, 3.8491f, 3.8604f, 3.8741f, 3.8884f, 3.9001f, 3.9094f, 3.9177999f, 3.9256999f, 3.9338f, 3.9421f, 3.9511f, 3.9605999f, 3.9707f, 3.9809999f, 3.9928f, 4.0043998f, 4.0160999f, 4.0275f, 4.0384f, 4.0485f, 4.0576f, 4.0655999f, 4.0725f, 4.0784998f, 4.0836f, 4.0879f, 4.0918f, 4.0956998f, 4.0991998f, 4.103f, 4.1071f, 4.1117f, 4.1169f, 4.1231999f, 4.1307f, 4.1399f, 4.1513f, 4.1662998f, 4.188f, }

// use lfp2? has widest range.
// lfp1 { 2.7f, 2.9449f, 3.0336f, 3.1003f, 3.1505f, 3.1898f, 3.2140999f, 3.22f, 3.2214999f, 3.223f, 3.2253f, 3.2283f, 3.2323999f, 3.2381999f, 3.2444f, 3.2513f, 3.2579999f, 3.2644999f, 3.2702999f, 3.2751999f, 3.28f, 3.2844999f, 3.2888999f, 3.2934f, 3.2965f, 3.2986f, 3.3004999f, 3.3024f, 3.3034999f, 3.3041f, 3.3044f, 3.3046f, 3.3048f, 3.3048999f, 3.3051f, 3.3053f, 3.3053999f, 3.3055f, 3.3055999f, 3.3057f, 3.3059f, 3.3060999f, 3.3062999f, 3.3066f, 3.3067999f, 3.3071f, 3.3074f, 3.3076999f, 3.3079f, 3.3081999f, 3.3087f, 3.3092f, 3.3097f, 3.3102f, 3.3106999f, 3.3113999f, 3.3123f, 3.3153999f, 3.3213f, 3.3297999f, 3.3373f, 3.339f, 3.3392999f, 3.3397f, 3.3399999f, 3.3404f, 3.3408f, 3.3412f, 3.3415999f, 3.3419f, 3.3420999f, 3.3420999f, 3.3422f, 3.3422f, 3.3422999f, 3.3424f, 3.3424f, 3.3425f, 3.3426f, 3.3426f, 3.3427f, 3.3427999f, 3.3427999f, 3.3429f, 3.3429999f, 3.3429999f, 3.3431f, 3.3432f, 3.3432f, 3.3433f, 3.3433f, 3.3434f, 3.3434999f, 3.3434999f, 3.3436999f, 3.345f, 3.3454f, 3.3455f, 3.3543999f, 3.4626999f, 3.571f };
// lfp2 { 2.7f, 2.9354999f, 3.0335f, 3.0976f, 3.1454f, 3.1759f, 3.1912999f, 3.1982f, 3.201f, 3.2024f, 3.2028f, 3.204f, 3.2063999f, 3.2126999f, 3.2188f, 3.2244999f, 3.2300999f, 3.2355f, 3.2409f, 3.2460999f, 3.2509999f, 3.2543f, 3.2576f, 3.2605f, 3.2637f, 3.2672999f, 3.2709f, 3.2744999f, 3.2781999f, 3.282f, 3.2852f, 3.286f, 3.2869f, 3.2878f, 3.2887f, 3.2894f, 3.2894f, 3.2894f, 3.2894f, 3.2894f, 3.2894f, 3.2894f, 3.2894f, 3.2894f, 3.2895f, 3.2895f, 3.2895f, 3.2895f, 3.2895f, 3.2895f, 3.2895999f, 3.2904f, 3.2911999f, 3.2921f, 3.2929f, 3.2937f, 3.2948f, 3.2962999f, 3.2988f, 3.3032999f, 3.3127f, 3.3229f, 3.3274f, 3.3278f, 3.3283f, 3.3287f, 3.3288f, 3.3288f, 3.3289f, 3.3289f, 3.329f, 3.3290999f, 3.3292f, 3.3292999f, 3.3292999f, 3.3294f, 3.3294f, 3.3295f, 3.3295f, 3.3295f, 3.3295f, 3.3296f, 3.3296f, 3.3296f, 3.3297f, 3.3297f, 3.3297999f, 3.3299f, 3.3299f, 3.3299999f, 3.3301f, 3.3303f, 3.3306f, 3.331f, 3.3315f, 3.332f, 3.3327f, 3.3338f, 3.336f, 3.3548999f, 3.571f, }
// lfp3 { 2.9f, 2.9995f, 3.0682f, 3.1194f, 3.1554999f, 3.1810999f, 3.1926999f, 3.1966f, 3.1988f, 3.2000999f, 3.2012f, 3.2025f, 3.2056f, 3.2119f, 3.2188f, 3.2242999f, 3.2288f, 3.2337f, 3.239f, 3.2437999f, 3.2478f, 3.2509999f, 3.2537f, 3.2565f, 3.2593999f, 3.2629f, 3.2663f, 3.2693999f, 3.2722f, 3.275f, 3.2778f, 3.2823f, 3.2851f, 3.2860999f, 3.2865f, 3.2869f, 3.2871f, 3.2873f, 3.2876f, 3.2878f, 3.2879f, 3.288f, 3.2883f, 3.2885f, 3.2887f, 3.2888999f, 3.2890999f, 3.2893f, 3.2895999f, 3.2899f, 3.2902f, 3.2904999f, 3.2909f, 3.2914f, 3.2918999f, 3.2925999f, 3.2936f, 3.2946999f, 3.2962999f, 3.2973f, 3.2981999f, 3.3032f, 3.3127f, 3.3206999f, 3.3239999f, 3.3253f, 3.3257999f, 3.3262f, 3.3262999f, 3.3264f, 3.3264999f, 3.3266f, 3.3267f, 3.3268f, 3.3269f, 3.3271f, 3.3271999f, 3.3273f, 3.3275f, 3.3276999f, 3.3278f, 3.328f, 3.3281f, 3.3282f, 3.3283999f, 3.3285999f, 3.3288f, 3.3289f, 3.3292f, 3.3295f, 3.3297f, 3.3299999f, 3.3304f, 3.3306999f, 3.3311f, 3.3317f, 3.3324f, 3.3331f, 3.3354f, 3.3575f, 3.5251999f, }

static const float nmc_ocv_curve[101] = {
    2.9f,       3.0115f,    3.0835f,    3.135f,     3.1735f,
    3.201f,     3.2255f,    3.2505f,    3.273f,     3.2975f,
    3.3205f,    3.3455f,    3.3655f,    3.3815f,    3.396f,
    3.41f,      3.423f,     3.4355f,    3.4485f,    3.461f,
    3.4725f,    3.4825f,    3.4915f,    3.501f,     3.511f,
    3.521f,     3.5305f,    3.5415f,    3.554f,     3.5655f,
    3.576f,     3.5865f,    3.5965f,    3.605f,     3.614f,
    3.622f,     3.6305f,    3.6385f,    3.647f,     3.655f,
    3.6635f,    3.6715f,    3.679f,     3.687f,     3.696f,
    3.704f,     3.713f,     3.723f,     3.732f,     3.742f,
    3.752f,     3.7615f,    3.7715f,    3.7815f,    3.791f,
    3.801f,     3.8105f,    3.82f,      3.829f,     3.838f,
    3.848f,     3.86f,      3.873f,     3.8855f,    3.8965f,
    3.905f,     3.913f,     3.92f,      3.9275f,    3.935f,
    3.9435f,    3.952f,     3.961f,     3.971f,     3.982f,
    3.993f,     4.005f,     4.017f,     4.028f,     4.0395f,
    4.0495f,    4.059f,     4.067f,     4.0745f,    4.0805f,
    4.0855f,    4.09f,      4.094f,     4.098f,     4.1015f,
    4.1055f,    4.1095f,    4.114f,     4.1195f,    4.125f,
    4.132f,     4.1405f,    4.15f,      4.1625f,    4.1775f,
    4.196f
};

    // 2.50005757, 3.10031943, 3.24625788, 3.33618626, 3.40073441,
    // 3.44897447, 3.46893947, 3.47395086, 3.47864389, 3.48333693,
    // 3.48834082, 3.49409536, 3.5013001 , 3.50967053, 3.51931585,
    // 3.52977034, 3.53957733, 3.54936385, 3.55979467, 3.56845999,
    // 3.57621643, 3.5856238 , 3.5952045 , 3.60126826, 3.60541126,
    // 3.60924125, 3.61263967, 3.61573646, 3.61867025, 3.6216477 ,
    // 3.62429166, 3.62713001, 3.62979388, 3.63270688, 3.6354027 ,
    // 3.6382091 , 3.64104817, 3.64373467, 3.64666554, 3.6496491 ,
    // 3.65261197, 3.65568686, 3.6591093 , 3.66244314, 3.666044  ,
    // 3.6696043 , 3.67341564, 3.67755864, 3.68202541, 3.6866434 ,
    // 3.69144414, 3.69704364, 3.70266863, 3.70969914, 3.71740244,
    // 3.72737144, 3.73953482, 3.75054572, 3.75889634, 3.76654077,
    // 3.77382326, 3.78142929, 3.78922413, 3.79728866, 3.80564429,
    // 3.8144326 , 3.82301974, 3.83208227, 3.8411448 , 3.85059132,
    // 3.86039399, 3.86997469, 3.88014603, 3.8899815 , 3.90001143,
    // 3.91023944, 3.92053556, 3.93082809, 3.94124702, 3.95179866,
    // 3.96235322, 3.973037  , 3.98377727, 3.99456048, 4.00572681,
    // 4.01705073, 4.02805948, 4.03954935, 4.05100517, 4.06243052,
    // 4.07434177, 4.0861864 , 4.09816332, 4.11036443, 4.12256861,
    // 4.13502932, 4.14774885, 4.16033888, 4.17348003, 4.18652358,
    // 4.20008564

// The filter tuning and cell model come from the pack profile (profiles/chemistry/*.h,
// which a profile can override):
//
// EKF_P0_* is the initial covariance, EKF_Q_* the process noise per 1s step,
// EKF_R_V the voltage measurement noise. EKF_P0_AH_VOLTAGE replaces EKF_P0_AH
// when the starting SOC comes from a voltage lookup rather than NVM.
//
// EKF_R_OHM: the cell's resistance. Voltages are the mean cell voltage and currents
// the pack current, so this is per cell in series. Where the offset below is quick
// enough to follow polarisation (LFP), the instant resistance does best; otherwise
// it wants the resistance a sustained current sees.
//
// EKF_OFFSET_TAU_S: the voltage offset state x[1] takes up what the OCV curve and
// resistance don't explain (hysteresis, curve mismatch, polarisation), so it
// doesn't all land on the SOC. It decays back to zero with this
// time constant, and moves by EKF_Q_OFFSET per step. It used to be the voltage
// across an RC pair, but replaying 1s data showed the RC physics made no difference
// to SOC accuracy; only the slack mattered, and its tuning shouldn't be tied to
// the cells' polarisation.
//
// OCV_DIFF_HALF_WIDTH: the Jacobian uses the OCV slope averaged over a window of
// 2 * OCV_DIFF_HALF_WIDTH percent around the current SoC, rather than the exact
// local slope. The LFP curve is flat to within a few mV between ~65% and ~97%,
// so with the exact slope the filter can sit in that region with a large
// residual (e.g. at float, 80mV above the plateau) and still see no reason to
// move. The wider window lets it see the knee coming. A half width of 1 gives
// the exact segment slope.
//
// EKF_MIN_OCV_SLOPE: the voltage update only runs where the OCV curve is steep
// enough for the voltage to say something about SOC. On the LFP plateaus a few
// mV of hysteresis or curve mismatch is worth tens of percent of SOC, and since
// that error is slow rather than white noise, the filter would follow it, and
// then misread the resulting SOC drift as a current bias.
//
// EKF_KNEE_OCV_SLOPE: but if the estimate has drifted onto a plateau while the
// battery is really at a knee, the slope at the estimate is small and the gate
// above would keep the filter from ever seeing it. So the update also runs when
// the measured voltage (corrected for the cell model) sits on a part of the curve
// at least this steep. This uses the unsmoothed slope, because the LFP curve is
// flat right up to the start of the top knee.
//
// OCV_LEARN learns a per-pack correction to the OCV table (ocv_learn.c), and
// OCV_LEARN_TABLE has the filter use it, with the SOC pinned at full charge.
#if CHEMISTRY == LFP
    #define ocv_factory lfp_ocv_curve
#elif CHEMISTRY == NMC
    #define ocv_factory nmc_ocv_curve
#else
    #error "Unsupported chemistry"
#endif

#ifdef OCV_LEARN_TABLE
    #define ocv_curve ocv_table
#else
    #define ocv_curve ocv_factory
#endif

// Largest current sensor bias the filter may learn
#define EKF_BIAS_LIMIT_A 0.5f

#define EKF_N 3

#ifdef OCV_LEARN_TABLE
// The table in use: the factory one until the learner has built its own
static const float *ocv_table = ocv_factory;
#endif

float table_soc(const float *table, float ocv) {
    // Simple linear search (could be optimized with binary search)
    if (ocv <= table[0]) return 0.0f;
    if (ocv >= table[100]) return 1.0f;

    for (int i = 0; i < 100; i++) {
        if (ocv < table[i + 1]) {
            float frac = (ocv - table[i]) / (table[i + 1] - table[i]);
            return (i + frac) / 100.0f;
        }
    }
    return 1.0f; // Should not reach here
}

const float *ekf_factory_ocv_table(void) {
    return ocv_factory;
}

float ocv_to_soc(float ocv) {
    return table_soc(ocv_curve, ocv);
}

double table_ocv(const float *table, double soc) {
    // Clamp SOC for safety
    if (soc < 0.0) soc = 0.0;
    if (soc > 1.0) soc = 1.0;

    // Simple linear interpolation on the OCV curve
    double index = soc * 100.0;
    int idx_lower = (int)index;
    if (idx_lower >= 100) return table[100];
    int idx_upper = idx_lower + 1;
    double frac = index - (double)idx_lower;
    return table[idx_lower] * (1.0 - frac) + table[idx_upper] * frac;
}

// --- Helper: OCV Curve ---
// Returns Open Circuit Voltage for a given SOC
static double soc_to_ocv(double soc) {
    return table_ocv(ocv_curve, soc);
}

// --- Helper: OCV Derivative ---
// Returns d(OCV)/d(SOC)
static double soc_to_ocv_derivative(double soc) {
    if (soc < 0.0) soc = 0.0;
    if (soc > 1.0) soc = 1.0;

    int idx = (int)(soc * 100.0);
    if (idx > 99) idx = 99;

    int lo = idx - OCV_DIFF_HALF_WIDTH + 1;
    int hi = idx + OCV_DIFF_HALF_WIDTH;
    if (lo < 0) lo = 0;
    if (hi > 100) hi = 100;
    return (ocv_curve[hi] - ocv_curve[lo]) * 100.0 / (double)(hi - lo);
}

// Returns d(OCV)/d(SOC) of the curve segment containing soc, without smoothing
static double ocv_segment_slope(double soc) {
    if (soc < 0.0) soc = 0.0;
    if (soc > 1.0) soc = 1.0;

    int idx = (int)(soc * 100.0);
    if (idx > 99) idx = 99;
    return (ocv_curve[idx + 1] - ocv_curve[idx]) * 100.0;
}

void ekf_init(ekf_t *ekf, double initial_soc, double initial_capacity, double initial_bias_A) {
    // Initial States
    ekf->x[0] = (1.0 - initial_soc) * initial_capacity; // Ah_used
    ekf->x[1] = 0.0;               // No voltage offset
    ekf->x[2] = initial_bias_A;     // Current sensor bias, from NVM if we have it
    ekf->cap = initial_capacity;

    // Initial Covariance P
    for(int i=0; i<EKF_N; i++) {
        for(int j=0; j<EKF_N; j++) ekf->P[i][j] = 0.0;
    }
    ekf->P[0][0] = EKF_P0_AH;   // Uncertainty in Ah_used
    ekf->P[1][1] = EKF_P0_OFFSET; // Uncertainty in the voltage offset
    ekf->P[2][2] = EKF_P0_BIAS; // Uncertainty in current bias

    // Process Noise Q (per 1s step)
    ekf->Q[0] = EKF_Q_AH;
    ekf->Q[1] = EKF_Q_OFFSET;
    ekf->Q[2] = EKF_Q_BIAS;

    // Measurement Noise R. This is dominated by OCV model error (hysteresis,
    // and the curve not quite matching the cells), not by the ADC.
    ekf->R = EKF_R_V;
    ekf->min_ocv_slope = EKF_MIN_OCV_SLOPE;
    ekf->knee_ocv_slope = EKF_KNEE_OCV_SLOPE;

    // Cell model
    ekf->R_ohm = EKF_R_OHM;
    ekf->offset_tau_s = EKF_OFFSET_TAU_S;

    // Reset scaling cache
    ekf->prev_min_mV = 0;
    ekf->prev_max_mV = 0;

    ekf->initialized = true;
}

void ekf_step(ekf_t *ekf, double charge_Ah, double current_amps, double voltage_measured) {
    // -----------------------------------------
    // 1. PREDICTION STEP
    // -----------------------------------------
    double dt = 1.0;

    // Positive current/charge = Charging. The measured values include the
    // sensor bias x[2], so the true current is current_amps - bias.
    double bias = ekf->x[2];
    double current_true = current_amps - bias;

    // --- State Prediction ---
    // x[0] = Ah_used - true charge = Ah_used - charge_Ah + bias * dt / 3600
    ekf->x[0] = ekf->x[0] - charge_Ah + bias * dt / 3600.0;

    // x[1] = V_off decays towards zero
    double decay = exp(-dt / ekf->offset_tau_s);
    ekf->x[1] *= decay;

    // x[2] bias stays constant in prediction

    // --- Covariance Prediction ---
    // F is the Jacobian of the process model:
    // F = [1, 0,     dt/3600]
    //     [0, decay, 0      ]
    //     [0, 0,     1      ]
    double F[EKF_N][EKF_N] = {
        {1.0, 0.0,   dt / 3600.0},
        {0.0, decay, 0.0},
        {0.0, 0.0,   1.0},
    };

    // P_pred = F * P * F^T + Q
    double FP[EKF_N][EKF_N];
    for(int i=0; i<EKF_N; i++) {
        for(int j=0; j<EKF_N; j++) {
            FP[i][j] = 0.0;
            for(int k=0; k<EKF_N; k++) FP[i][j] += F[i][k] * ekf->P[k][j];
        }
    }
    for(int i=0; i<EKF_N; i++) {
        for(int j=0; j<EKF_N; j++) {
            double sum = 0.0;
            for(int k=0; k<EKF_N; k++) sum += FP[i][k] * F[j][k];
            ekf->P[i][j] = sum;
        }
        ekf->P[i][i] += ekf->Q[i];
    }

    // -----------------------------------------
    // 2. UPDATE STEP
    // -----------------------------------------

    double ah_used = ekf->x[0];
    double v_off   = ekf->x[1];
    double cap     = ekf->cap;

    // Calculate dynamic SOC: 1 - (Ah_used / Capacity)
    double soc_est = 1.0 - (ah_used / cap);

    // Predict Voltage
    double ocv = soc_to_ocv(soc_est);
    double v_pred = ocv + current_true * ekf->R_ohm + v_off;

    // Calculate Residual (y)
    double y = voltage_measured - v_pred;

    // --- Calculate Jacobian H ---
    // H = [dH/dAh, dH/dVoff, dH/dBias]
    double d_ocv = soc_to_ocv_derivative(soc_est);

    // On the plateaus, rely on (bias corrected) coulomb counting alone
    if (fabs(d_ocv) < ekf->min_ocv_slope) {
        double ocv_measured = voltage_measured - v_off - current_true * ekf->R_ohm;
        if (fabs(ocv_segment_slope(ocv_to_soc(ocv_measured))) < ekf->knee_ocv_slope) return;
    }

    double H[EKF_N] = {
        d_ocv * (-1.0 / cap),             // dV/dAh = dOCV/dSOC * dSOC/dAh
        1.0,                              // dV/dVoff
        -ekf->R_ohm,                      // dV/dBias, via the I*R term
    };

    // --- Calculate Kalman Gain K ---
    // PH = P * H^T, S = H * P * H^T + R (scalar, since measurement is 1D)
    double PH[EKF_N];
    double S = ekf->R;
    for(int i=0; i<EKF_N; i++) {
        PH[i] = 0.0;
        for(int j=0; j<EKF_N; j++) PH[i] += ekf->P[i][j] * H[j];
        S += H[i] * PH[i];
    }

    double K[EKF_N];
    for(int i=0; i<EKF_N; i++) K[i] = PH[i] / S;

    // --- Update State Vector ---
    // x = x + K * y
    for(int i=0; i<EKF_N; i++) {
        ekf->x[i] += K[i] * y;
    }

    // Constraints
    // Ah_used cannot be negative (cannot be "more than full")
    if(ekf->x[0] < 0.0) ekf->x[0] = 0.0;
    // Keep the bias within a plausible range for the shunt
    if(ekf->x[2] > EKF_BIAS_LIMIT_A) ekf->x[2] = EKF_BIAS_LIMIT_A;
    if(ekf->x[2] < -EKF_BIAS_LIMIT_A) ekf->x[2] = -EKF_BIAS_LIMIT_A;

    // --- Update Covariance P ---
    // P = (I - K * H) * P = P - K * (H * P), and H * P = PH^T since P is symmetric
    for(int i=0; i<EKF_N; i++) {
        for(int j=0; j<EKF_N; j++) {
            ekf->P[i][j] -= K[i] * PH[j];
        }
    }
    // Keep P symmetric against rounding
    for(int i=0; i<EKF_N; i++) {
        for(int j=i+1; j<EKF_N; j++) {
            double avg = 0.5 * (ekf->P[i][j] + ekf->P[j][i]);
            ekf->P[i][j] = avg;
            ekf->P[j][i] = avg;
        }
    }
}

#ifdef OCV_LEARN_TABLE
// Pin the SOC with the given uncertainty, forgetting its correlation with the other states
static void ekf_pin_soc(ekf_t *ekf, double soc, double var_Ah) {
    ekf->x[0] = (1.0 - soc) * ekf->cap;
    for (int i = 0; i < EKF_N; i++) {
        ekf->P[0][i] = 0.0;
        ekf->P[i][0] = 0.0;
    }
    ekf->P[0][0] = var_Ah;
}
#endif

double ekf_get_soc(ekf_t *ekf) {
    double ah = ekf->x[0];
    double cap = ekf->cap;
    if (cap < 0.0001) return 0.0; // Prevent div by zero
    return 1.0 - (ah / cap);
}

// Sets *from_voltage when the result is the voltage lookup rather than the stored SOC
static float estimate_initial_soc(const bms_model_t *model, float voltage_volts, float capacity_ah,
                                  bool *from_voltage) {
    float voltage_soc = ocv_to_soc(voltage_volts);
    *from_voltage = true;
    if (capacity_ah <= 0.0f) return voltage_soc;
    if (model->charge_used_Ah >= 0.0f && model->charge_used_Ah <= capacity_ah) {
        float stored_soc = 1.0f - (model->charge_used_Ah / capacity_ah);
        // Accept stored value only if within 10% of voltage estimate.
        // Tightened from 20% to reduce the window where a stale persistent value
        // overrides a fresh OCV reading after a reboot.
        // No != 0 guard needed: zero Ah used is valid at 100% SoC, and the tolerance
        // already rejects an uninitialized zero (stored=100%, voltage≈50% → 0.5 diff).
        if (fabsf(stored_soc - voltage_soc) < 0.1f) {
            *from_voltage = false;
            return stored_soc;
        }
    }
    return voltage_soc;
}

// Learned current bias from NVM. A corrupt or out-of-range value is ignored,
// and the filter re-learns from zero at the next full charge.
static float ekf_stored_bias(const bms_model_t *model) {
    float bias = model->current_bias_A;
    if (!(bias >= -EKF_BIAS_LIMIT_A && bias <= EKF_BIAS_LIMIT_A)) return 0.0f;
    return bias;
}

double ekf_bias_A(const bms_model_t *model) {
    return model->ekf.initialized ? model->ekf.x[2] : ekf_stored_bias(model);
}

static void ekf_init_from_model(bms_model_t *model, float voltage_volts) {
    if (model->ekf.initialized || voltage_volts <= 0.0f) return;
    float capacity_ah = (float)get_actual_capacity_mC(model) / 3600000.0f;
    bool from_voltage;
    float initial_soc = estimate_initial_soc(model, voltage_volts, capacity_ah, &from_voltage);
    info_printf("EKF init: SOC %.2f%% from %s (voltage %.3fV, stored %.4fAh, bias %.1fmA)\n",
                initial_soc * 100.0f, from_voltage ? "voltage" : "NVM", voltage_volts,
                model->charge_used_Ah, model->current_bias_A * 1000.0f);
    ekf_init(&model->ekf, initial_soc, capacity_ah, ekf_stored_bias(model));
    if (from_voltage) model->ekf.P[0][0] = EKF_P0_AH_VOLTAGE;
}

static void ekf_update_limits(bms_model_t *model) {
    uint16_t min_mV = get_cell_voltage_working_min_mV(model);
    uint16_t max_mV = get_cell_voltage_working_max_mV(model);

    // Skip update if limits haven't changed
    if (min_mV == model->ekf.prev_min_mV && max_mV == model->ekf.prev_max_mV) {
        return;
    }

    // Cache the scaling factors. On the factory table, so learning doesn't move 0% and
    // 100%; a full charge is pinned at the charge voltage's factory SOC, so reads 100%.
    model->ekf.prev_soc_min = table_soc(ocv_factory, (float)min_mV / 1000.0f);
    float soc_max = table_soc(ocv_factory, (float)max_mV / 1000.0f);
    model->ekf.prev_soc_mul = 1.0f / (soc_max - model->ekf.prev_soc_min);
    model->ekf.prev_min_mV = min_mV;
    model->ekf.prev_max_mV = max_mV;

    info_printf("EKF Scaling: %d-%d mV (SOC: %2.2f-%2.2f %%, Mul: %2.3f)\n",
                min_mV, max_mV, model->ekf.prev_soc_min * 100.0f, soc_max * 100.0f, model->ekf.prev_soc_mul);

    // working_capacity_mC is the usable subset of the actual capacity given the current
    // working voltage range. Computed here because it shares the same min/max limit
    // inputs as the SOC scaling factors above.
    //
    // It's an overestimate, knowingly tolerated. This treats the working min as the mean
    // cell's resting voltage, but discharge stops when the weakest cell reaches it under
    // load, while the mean cell still rests higher (by I * R plus the min-to-mean spread,
    // which grows at the bottom knee). On an aged 96S NMC pack at 10A that was ~60mV, ~3%
    // of SOC, ~5Ah; the steep bottom of an LFP curve makes it negligible. Correcting it
    // would need the spread learned against SOC and a typical discharge current. Don't
    // "fix" this against the learned capacity: that one is the mean cell's at rest, which
    // is what the filter tracks, and doesn't have this problem.
    model->working_capacity_mC = (soc_max - model->ekf.prev_soc_min) * (float)get_actual_capacity_mC(model);
}

uint32_t ekf_tick(bms_model_t *model, float charge_Ah, int32_t current_mA, int32_t voltage_mV) {
    float current_amps = (float)current_mA * 0.001f;
    float voltage_volts = (float)voltage_mV * 0.001f;

#ifdef OCV_LEARN
    // Where the inverter stops charging, per cell: the working max plus the inverter limit offset
    float charge_V = (get_cell_voltage_working_max_mV(model) * NUM_CELLS * 0.001f
                      + model->pack_voltage_limit_upper_offset_dV * 0.1f) / NUM_CELLS;
    if (ocv_learn_tick(&model->ocv_learn, ocv_factory, (double)get_actual_capacity_mC(model) / 3600000.0,
                       charge_V, charge_Ah, current_amps, voltage_volts)) {
#ifdef OCV_LEARN_TABLE
        ocv_table = model->ocv_learn.table;
#endif
    }
#endif

    ekf_init_from_model(model, voltage_volts);

    if (!model->ekf.initialized) {
        return 0xFFFFFFFF;
    }

    ekf_step(&model->ekf, charge_Ah, current_amps, voltage_volts);

#ifdef OCV_LEARN_TABLE
    // While floating, the charger holds the voltage above OCV with almost no current,
    // which the filter would read as fuller than full. Full is where the learner says.
    // This also corrects a wrong starting count at the first full charge.
    if (ocv_learn_at_full(&model->ocv_learn)) {
        ekf_pin_soc(&model->ekf, model->ocv_learn.persistent.s_full, 1.0); // (1Ah)^2
    }
#endif

    float soc = ekf_get_soc(&model->ekf);

    ekf_update_limits(model);

    // Apply scaling
    soc = (soc - model->ekf.prev_soc_min) * model->ekf.prev_soc_mul;

    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;

    model->charge_used_Ah = model->ekf.x[0];
    model->current_bias_A = model->ekf.x[2];

    // Round rather than truncate, so a SoC a hair under 1.0 still reads 100.00%
    return (uint32_t)(soc * 10000.0f + 0.5f); // Return SOC in 0.01% units
}

void ekf_set_soc(bms_model_t *model, uint16_t soc) {
    // soc is in 0.01% units (0-10000)
    float soc_rel = (float)soc / 10000.0f;

    ekf_update_limits(model);

    // Convert relative SOC back to absolute SOC
    float soc_abs = (soc_rel / model->ekf.prev_soc_mul) + model->ekf.prev_soc_min;

    if (!model->ekf.initialized) {
        float initial_capacity_ah = (float)get_actual_capacity_mC(model) / 3600000.0f;
        ekf_init(&model->ekf, soc_abs, initial_capacity_ah, ekf_stored_bias(model));
    } else {
        model->ekf.x[0] = (1.0f - soc_abs) * model->ekf.cap;
    }
    
    model->charge_used_Ah = model->ekf.x[0];
    
    info_printf("EKF SOC Manual Set: %u (rel=%2.2f%%, abs=%2.2f%%, Ah Used: %f)\n", 
                soc, soc_rel * 100.0f, soc_abs * 100.0f, model->ekf.x[0]);
}

void ekf_set_capacity(bms_model_t *model, uint32_t capacity_mC) {
    model->actual_capacity_mC = capacity_mC;
    model->ekf.prev_min_mV = 0; // so the next tick recomputes working_capacity_mC
    if (!model->ekf.initialized) return;

    // The Ah used since full was measured, so keep it, and the SOC moves to suit the new capacity
    model->ekf.cap = (double)get_actual_capacity_mC(model) / 3600000.0;
    info_printf("EKF capacity set: %.1fAh%s, SOC %.2f%%\n", model->ekf.cap,
                capacity_mC > 0 ? "" : " (the profile's)", ekf_get_soc(&model->ekf) * 100.0);
}

void ekf_reset_bias(bms_model_t *model) {
    model->current_bias_A = 0.0f;
    if (!model->ekf.initialized) return;

    model->ekf.x[2] = 0.0f;
    for (int i = 0; i < EKF_N; i++) {
        model->ekf.P[i][2] = 0.0f;
        model->ekf.P[2][i] = 0.0f;
    }
    model->ekf.P[2][2] = EKF_P0_BIAS;
}

void ekf_print_state(ekf_t *ekf) {
    float soc = ekf_get_soc(ekf);
    debug_printf("EKF State: Ah_used=%.4f Ah, V_off=%.4f V, Capacity=%.2f Ah, Bias=%.1f mA, SOC=%.2f%%\n",
           ekf->x[0], ekf->x[1], ekf->cap, ekf->x[2] * 1000.0f, soc * 100.0f);
}