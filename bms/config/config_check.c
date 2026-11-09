// We build this file with -pedantic-errors so that we can catch a pack profile
// or board that changes a setting without #undef-ing it first.
#include BMS_PROFILE_HEADER
#include BMS_BOARD_HEADER

// ISO C forbids an empty translation unit
typedef int config_check_t;
