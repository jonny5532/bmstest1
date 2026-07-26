#include "ekf.h"
#include "config/limits.h"
#include "sys/time/time.h"
#include "app/model.h"
#include "sys/logging/logging.h"

#include <stdbool.h>
#include <stdint.h>

static const float lfp_ocv_curve[101] = {
    2.7f,       2.9449f,    3.0336f,    3.1003f,    3.1505f,
    3.1898f,    3.2140999f, 3.22f,      3.2214999f, 3.223f,
    3.2253f,    3.2283f,    3.2323999f, 3.2381999f, 3.2444f,
    3.2513f,    3.2579999f, 3.2644999f, 3.2702999f, 3.2751999f,
    3.28f,      3.2844999f, 3.2888999f, 3.2934f,    3.2965f,
    3.2986f,    3.3004999f, 3.3024f,    3.3034999f, 3.3041f,
    3.3044f,    3.3046f,    3.3048f,    3.3048999f, 3.3051f,
    3.3053f,    3.3053999f, 3.3055f,    3.3055999f, 3.3057f,
    3.3059f,    3.3060999f, 3.3062999f, 3.3066f,    3.3067999f,
    3.3071f,    3.3074f,    3.3076999f, 3.3079f,    3.3081999f,
    3.3087f,    3.3092f,    3.3097f,    3.3102f,    3.3106999f,
    3.3113999f, 3.3123f,    3.3153999f, 3.3213f,    3.3297999f,
    3.3373f,    3.339f,     3.3392999f, 3.3397f,    3.3399999f,
    3.3404f,    3.3408f,    3.3412f,    3.3415999f, 3.3419f,
    3.3420999f, 3.3420999f, 3.3422f,    3.3422f,    3.3422999f,
    3.3424f,    3.3424f,    3.3425f,    3.3426f,    3.3426f,
    3.3427f,    3.3427999f, 3.3427999f, 3.3429f,    3.3429999f,
    3.3429999f, 3.3431f,    3.3432f,    3.3432f,    3.3433f,
    3.3433f,    3.3434f,    3.3434999f, 3.3434999f, 3.3436999f,
    3.345f,     3.3454f,    3.3455f,    3.3543999f, 3.4626999f,
    3.571f
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

static float nmc_ocv_curve[101] = {
    2.9f,       3.0443f,    3.1399f,    3.2162f,    3.2762f,
    3.3229f,    3.3527f,    3.3664999f, 3.3755f,    3.3834f,
    3.3908f,    3.3982999f, 3.4063f,    3.4164f,    3.4296f,
    3.4435f,    3.4572999f, 3.4716f,    3.4847f,    3.4958999f,
    3.5062f,    3.5165f,    3.5265999f, 3.5360999f, 3.5448f,
    3.553f,     3.5606999f, 3.5682f,    3.5754f,    3.582f,
    3.588f,     3.5934999f, 3.5987f,    3.6036999f, 3.6085999f,
    3.6133f,    3.6180999f, 3.6228f,    3.6275f,    3.6322999f,
    3.6372f,    3.6422f,    3.6475f,    3.6529f,    3.6585f,
    3.6645f,    3.6707f,    3.6775f,    3.6847999f, 3.6926f,
    3.7012f,    3.7107999f, 3.7218f,    3.7346f,    3.7492f,
    3.7646999f, 3.7794f,    3.7925f,    3.8043f,    3.8152f,
    3.8255f,    3.8355f,    3.845f,     3.8543f,    3.8634f,
    3.8722f,    3.8806f,    3.8889f,    3.897f,     3.9049f,
    3.9128f,    3.9207f,    3.9288f,    3.9370999f, 3.9458f,
    3.9549f,    3.9644f,    3.9744f,    3.9849f,    3.9956999f,
    4.0068f,    4.0179f,    4.0288f,    4.0394f,    4.0495f,
    4.0587f,    4.0668998f, 4.0739999f, 4.0798998f, 4.0844998f,
    4.0883999f, 4.092f,     4.0956998f, 4.0998998f, 4.1048f,
    4.1108999f, 4.1188f,    4.1290998f, 4.143f,     4.1617999f,
    4.1880999f

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
};

static float nmc_ocv_curve_diff[100] = {
    14.43   ,  9.56   ,  7.63   ,  6.     ,  4.67   ,  2.98   ,
    1.37999,  0.90001,  0.79   ,  0.74   ,  0.74999,  0.80001,
    1.01   ,  1.32   ,  1.39   ,  1.37999,  1.43001,  1.31   ,
    1.11999,  1.03001,  1.03   ,  1.00999,  0.95   ,  0.87001,
    0.82   ,  0.76999,  0.75001,  0.72   ,  0.66   ,  0.6    ,
    0.54999,  0.52001,  0.49999,  0.49   ,  0.47001,  0.47999,
    0.47001,  0.47   ,  0.47999,  0.49001,  0.5    ,  0.53   ,
    0.54   ,  0.56   ,  0.6    ,  0.62   ,  0.68   ,  0.72999,
    0.78001,  0.86   ,  0.95999,  1.10001,  1.28   ,  1.46   ,
    1.54999,  1.47001,  1.31   ,  1.18   ,  1.09   ,  1.03   ,
    1.     ,  0.95   ,  0.93   ,  0.91   ,  0.88   ,  0.84   ,
    0.83   ,  0.81   ,  0.79   ,  0.79   ,  0.79   ,  0.81   ,
    0.82999,  0.87001,  0.91   ,  0.95   ,  1.     ,  1.05   ,
    1.07999,  1.11001,  1.11   ,  1.09   ,  1.06   ,  1.01   ,
    0.92   ,  0.81998,  0.71001,  0.58999,  0.46   ,  0.39001,
    0.36001,  0.36998,  0.42   ,  0.49002,  0.60999,  0.79001,
    1.02998,  1.39002,  1.87999,  2.63
    
    // 60.02618656, 14.59384462,  8.99283794,  6.45481512,  4.82400582,
    // 1.99650039,  0.50113891,  0.46930313,  0.46930313,  0.50038961,
    // 0.57545365,  0.72047384,  0.83704336,  0.96453191,  1.04544938,
    // 0.98069849,  0.97865236,  1.04308176,  0.86653185,  0.77564423,
    // 0.94073653,  0.95807081,  0.60637599,  0.4143    ,  0.3829984 ,
    // 0.33984184,  0.30967921,  0.29337938,  0.29774512,  0.26439556,
    // 0.28383479,  0.26638761,  0.29129982,  0.26958231,  0.28064009,
    // 0.28390649,  0.26865009,  0.29308647,  0.29835633,  0.29628725,
    // 0.30748844,  0.3422442 ,  0.33338428,  0.36008567,  0.35603046,
    // 0.38113396,  0.4143    ,  0.44667724,  0.4617986 ,  0.48007407,
    // 0.55994991,  0.56249923,  0.7030505 ,  0.77033047,  0.99689964,
    // 1.21633849,  1.10109018,  0.83506111,  0.76444303,  0.72824955,
    // 0.76060295,  0.77948431,  0.80645224,  0.83556307,  0.87883157,
    // 0.85871403,  0.90625286,  0.90625286,  0.94465207,  0.98026644,
    // 0.95807081,  1.01713332,  0.98354717,  1.00299322,  1.02280092,
    // 1.02961159,  1.02925304,  1.04189365,  1.05516395,  1.05545602,
    // 1.06837823,  1.07402669,  1.07832087,  1.11663342,  1.13239178,
    // 1.10087505,  1.14898682,  1.14558201,  1.14253523,  1.19112451,
    // 1.18446342,  1.1976915 ,  1.22011146,  1.22041754,  1.24607086,
    // 1.27195335,  1.2590027 ,  1.3141156 ,  1.30435473,  1.35620603
};

#if CHEMISTRY == LFP
    #define ocv_curve lfp_ocv_curve
    #define ocv_curve_diff lfp_ocv_curve_diff
#elif CHEMISTRY == NMC
    #define ocv_curve nmc_ocv_curve
    #define ocv_curve_diff nmc_ocv_curve_diff
#else
    #error "Unsupported chemistry"
#endif

float ocv_to_soc(float ocv) {
    // Simple linear search (could be optimized with binary search)
    if (ocv <= ocv_curve[0]) return 0.0f;
    if (ocv >= ocv_curve[100]) return 1.0f;

    for (int i = 0; i < 100; i++) {
        if (ocv < ocv_curve[i + 1]) {
            float frac = (ocv - ocv_curve[i]) / (ocv_curve[i + 1] - ocv_curve[i]);
            return (i + frac) / 100.0f;
        }
    }
    return 1.0f; // Should not reach here
}

// --- Helper: OCV Curve ---
// Returns Open Circuit Voltage for a given SOC
static float soc_to_ocv(float soc) {
    // Clamp SOC for safety
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;

    // Simple linear interpolation on the OCV curve
    float index = soc * 100.0f;
    int idx_lower = (int)index;
    if (idx_lower >= 100) return ocv_curve[100];
    int idx_upper = idx_lower + 1;
    float frac = index - (float)idx_lower;
    return ocv_curve[idx_lower] * (1.0f - frac) + ocv_curve[idx_upper] * frac;
}

// --- Helper: OCV Derivative ---
// Returns d(OCV)/d(SOC)
static float soc_to_ocv_derivative(float soc) {
    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;

    float index = soc * 100.0f;
    int idx_lower = (int)index;
    if (idx_lower >= 100) return ocv_curve_diff[99];
    return ocv_curve_diff[idx_lower];
}

void ekf_init(ekf_t *ekf, float initial_soc, float initial_capacity) {
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
    ekf->Q[0] = 1e-3f;     // Trust current integration highly
    ekf->Q[1] = 1e-2f;     // V_c1 can vary
    ekf->Q[2] = 1e-7f;     // Capacity changes very slowly
    
    // Measurement Noise R
    ekf->R = 0.01f;        // Voltage sensor variance (e.g. 0.14V std dev)

    // Fitted RC model parameters
    ekf->R0 = 0.00076f;
    ekf->R1 = 0.00054f;
    ekf->C1 = 150000.0f;

    // Reset scaling cache
    ekf->prev_min = 0.0f;
    ekf->prev_max = 0.0f;

    ekf->initialized = true;
}

void ekf_step(ekf_t *ekf, float charge_Ah, float current_amps, float voltage_measured) {
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
    // Ah_used cannot exceed capacity (cannot be "less than empty") - without
    // this a sustained fault (e.g. stuck current reading) lets the raw state
    // run away and distorts the capacity Jacobian even though the reported
    // SoC is clamped downstream.
    if(ekf->x[0] > ekf->x[2]) ekf->x[0] = ekf->x[2];

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

float ekf_get_soc(ekf_t *ekf) {
    float ah = ekf->x[0];
    float cap = ekf->x[2];
    if (cap < 0.0001f) return 0.0f; // Prevent div by zero
    return 1.0f - (ah / cap);
}

static float estimate_initial_soc(const bms_model_t *model, float voltage_volts, float capacity_ah) {
    float voltage_soc = ocv_to_soc(voltage_volts);
    if (capacity_ah <= 0.0f) return voltage_soc;
    if (model->charge_used_Ah >= 0.0f && model->charge_used_Ah <= capacity_ah) {
        float stored_soc = 1.0f - (model->charge_used_Ah / capacity_ah);
        // Accept stored value only if within 10% of voltage estimate.
        // Tightened from 20% to reduce the window where a stale persistent value
        // overrides a fresh OCV reading after a reboot.
        // No != 0 guard needed: zero Ah used is valid at 100% SoC, and the tolerance
        // already rejects an uninitialized zero (stored=100%, voltage≈50% → 0.5 diff).
        if (fabsf(stored_soc - voltage_soc) < 0.1f)
            return stored_soc;
    }
    return voltage_soc;
}

static void ekf_init_from_model(bms_model_t *model, float voltage_volts) {
    if (model->ekf.initialized || voltage_volts <= 0.0f) return;
    float capacity_ah = (float)model->nameplate_capacity_mC / 3600000.0f;
    float initial_soc = estimate_initial_soc(model, voltage_volts, capacity_ah);
    info_printf("EKF init: SOC %.2f%% (voltage %.3fV, stored %.4fAh)\n",
                initial_soc * 100.0f, voltage_volts, model->charge_used_Ah);
    ekf_init(&model->ekf, initial_soc, capacity_ah);
}

static void ekf_update_limits(bms_model_t *model) {
    uint16_t min_mV = get_cell_voltage_working_min_mV(model);
    uint16_t max_mV = get_cell_voltage_working_max_mV(model);

    // Skip update if limits haven't changed
    if ((float)min_mV == model->ekf.prev_min && (float)max_mV == model->ekf.prev_max) {
        return;
    }

    // Cache the scaling factors
    model->ekf.prev_soc_min = ocv_to_soc((float)min_mV / 1000.0f);
    float soc_max = ocv_to_soc((float)max_mV / 1000.0f);
    model->ekf.prev_soc_mul = 1.0f / (soc_max - model->ekf.prev_soc_min);
    model->ekf.prev_min = (float)min_mV;
    model->ekf.prev_max = (float)max_mV;

    info_printf("EKF Scaling: %d-%d mV (SOC: %2.2f-%2.2f %%, Mul: %2.3f)\n",
                min_mV, max_mV, model->ekf.prev_soc_min * 100.0f, soc_max * 100.0f, model->ekf.prev_soc_mul);

    // working_capacity_mC is the usable subset of nameplate capacity given the current
    // working voltage range. Computed here because it shares the same min/max limit
    // inputs as the SOC scaling factors above.
    model->working_capacity_mC = (soc_max - model->ekf.prev_soc_min) * (float)model->nameplate_capacity_mC;
}

uint32_t ekf_tick(bms_model_t *model, float charge_Ah, int32_t current_mA, int32_t voltage_mV) {
    float current_amps = (float)current_mA * 0.001f;
    float voltage_volts = (float)voltage_mV * 0.001f;

    ekf_init_from_model(model, voltage_volts);

    if (!model->ekf.initialized) {
        return 0xFFFFFFFF;
    }

    ekf_step(&model->ekf, charge_Ah, current_amps, voltage_volts);

    float soc = ekf_get_soc(&model->ekf);

    ekf_update_limits(model);

    // Apply scaling
    soc = (soc - model->ekf.prev_soc_min) * model->ekf.prev_soc_mul;

    if (soc < 0.0f) soc = 0.0f;
    if (soc > 1.0f) soc = 1.0f;

    model->charge_used_Ah = model->ekf.x[0];

    return (uint32_t)(soc * 10000.0f); // Return SOC in 0.01% units
}

void ekf_set_soc(bms_model_t *model, uint16_t soc) {
    // soc is in 0.01% units (0-10000)
    float soc_rel = (float)soc / 10000.0f;

    ekf_update_limits(model);

    // Convert relative SOC back to absolute SOC
    float soc_abs = (soc_rel / model->ekf.prev_soc_mul) + model->ekf.prev_soc_min;

    if (!model->ekf.initialized) {
        float initial_capacity_ah = (float)model->nameplate_capacity_mC / 3600000.0f;
        ekf_init(&model->ekf, soc_abs, initial_capacity_ah);
    } else {
        model->ekf.x[0] = (1.0f - soc_abs) * model->ekf.x[2];
    }
    
    model->charge_used_Ah = model->ekf.x[0];
    
    info_printf("EKF SOC Manual Set: %u (rel=%2.2f%%, abs=%2.2f%%, Ah Used: %f)\n", 
                soc, soc_rel * 100.0f, soc_abs * 100.0f, model->ekf.x[0]);
}

void ekf_print_state(ekf_t *ekf) {
    float soc = ekf_get_soc(ekf);
    debug_printf("EKF State: Ah_used=%.4f Ah, V_c1=%.4f V, Capacity=%.2f Ah, SOC=%.2f%%\n",
           ekf->x[0], ekf->x[1], ekf->x[2], soc * 100.0f);
}