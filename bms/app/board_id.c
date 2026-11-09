#include "board_id.h"

#include "config/limits.h"

// Upper edge of the Rev1.0-1.6 windows, in 12-bit counts: midway between the
// nominal readings of neighbouring R76 values with R75 = 10k
// (0, 372, 738, 1383, 2048, 2891, 3377, 3723). Anything higher is Rev1.7.
static const uint16_t REV1_WINDOW_TOP[] = {185, 554, 1060, 1715, 2469, 3133, 3549};
#define REV1_WINDOWS (sizeof(REV1_WINDOW_TOP) / sizeof(REV1_WINDOW_TOP[0]))

uint16_t board_id_classify(uint16_t pull_up_counts, uint16_t pull_down_counts, uint16_t counts) {
    if (pull_up_counts > pull_down_counts + BOARD_ID_FLOATING_SPAN_COUNTS) {
        return BOARD_ID_REV0;
    }
    uint16_t minor = 0;
    while (minor < REV1_WINDOWS && counts > REV1_WINDOW_TOP[minor]) {
        minor++;
    }
    return 10 + minor;
}

bool board_id_matches_build(uint16_t code) {
#if IS_BMS_BOARD(REV1)
    return code != BOARD_ID_REV0;
#else
    return code == BOARD_ID_REV0;
#endif
}
