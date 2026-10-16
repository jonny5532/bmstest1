#include "board_id.h"

#include "config/limits.h"

// Upper edge of each Rev1.x window, in 12-bit counts: midway between the
// nominal readings of neighbouring R76 values with R75 = 10k
// (0, 372, 738, 1383, 2048, 2891, 3377, 3723).
static const uint16_t REV1_WINDOW_TOP[] = {185, 554, 1060, 1715, 2469, 3133, 3549, 4095};

uint16_t board_id_classify(float pull_up_counts, float pull_down_counts, float counts) {
    if (pull_up_counts - pull_down_counts > BOARD_ID_FLOATING_SPAN_COUNTS) {
        return BOARD_ID_REV0;
    }
    if (!(counts >= 0.0f) || counts > 4095.5f) {
        return BOARD_ID_UNKNOWN;
    }
    for (uint16_t minor = 0; minor < sizeof(REV1_WINDOW_TOP) / sizeof(REV1_WINDOW_TOP[0]); minor++) {
        if (counts <= REV1_WINDOW_TOP[minor] + 0.5f) {
            return 10 + minor;
        }
    }
    return BOARD_ID_UNKNOWN;
}

bool board_id_matches_build(uint16_t code) {
#if BMS_BOARD == BMS_BOARD_REV1
    return code >= 10 && code <= 19;
#else
    return code == BOARD_ID_REV0;
#endif
}
