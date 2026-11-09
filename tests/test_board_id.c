#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>

#include "app/board_id.h"
#include "config/limits.h"

#define R_TOP 10000.0f
#define R_PULL 50000.0f // RP2350 internal pull, roughly

static uint16_t divider(float r_top, float r_bottom) {
    return (uint16_t)(4095.0f * r_bottom / (r_bottom + r_top) + 0.5f);
}

static float parallel(float a, float b) {
    return a * b / (a + b);
}

// Readings for R75 = 10k and the given R76, with the internal pull-up, the
// pull-down and no pulls, scaled by (1 + err) on R76 to model tolerance.
static uint16_t classify_strap(float r_bottom, float err) {
    float rb = r_bottom * (1.0f + err);
    uint16_t up = divider(parallel(R_TOP, R_PULL), rb);
    uint16_t down = divider(R_TOP, parallel(rb, R_PULL));
    uint16_t open = divider(R_TOP, rb);
    return board_id_classify(up, down, open);
}

static void test_rev0_floating(void **state) {
    (void)state;
    // Floating: the pulls drag the pin rail to rail; the unpulled reading is noise
    assert_int_equal(board_id_classify(4095, 0, 1234), BOARD_ID_REV0);
    assert_int_equal(board_id_classify(3900, 150, 4095), BOARD_ID_REV0);
}

static void test_rev1_0_grounded(void **state) {
    (void)state;
    assert_int_equal(board_id_classify(0, 0, 0), 10);
    assert_int_equal(board_id_classify(3, 2, 3), 10); // the desk board
}

static void test_every_strap_value_with_tolerance(void **state) {
    (void)state;
    const float r76[] = {1000, 2200, 5100, 10000, 24000, 47000, 100000};
    for (int i = 0; i < 7; i++) {
        for (float err = -0.02f; err <= 0.021f; err += 0.01f) {
            assert_int_equal(classify_strap(r76[i], err), 11 + i);
        }
    }
}

static void test_pulls_dont_look_floating_on_a_strap(void **state) {
    (void)state;
    // The biggest pull swing is on the largest R76: still far below the threshold
    float rb = 100000.0f;
    uint16_t up = divider(parallel(R_TOP, R_PULL), rb);
    uint16_t down = divider(R_TOP, parallel(rb, R_PULL));
    assert_true(up - down < BOARD_ID_FLOATING_SPAN_COUNTS / 2);
}

static void test_full_scale_is_rev1_7(void **state) {
    (void)state;
    assert_int_equal(board_id_classify(4095, 4000, 4095), 17);
}

static void test_matches_build(void **state) {
    (void)state;
#if IS_BMS_BOARD(REV1)
    assert_true(board_id_matches_build(10));
    assert_true(board_id_matches_build(17));
    assert_false(board_id_matches_build(BOARD_ID_REV0));
#else
    assert_true(board_id_matches_build(BOARD_ID_REV0));
    assert_false(board_id_matches_build(10));
    assert_false(board_id_matches_build(13));
#endif
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_rev0_floating),
        cmocka_unit_test(test_rev1_0_grounded),
        cmocka_unit_test(test_every_strap_value_with_tolerance),
        cmocka_unit_test(test_pulls_dont_look_floating_on_a_strap),
        cmocka_unit_test(test_full_scale_is_rev1_7),
        cmocka_unit_test(test_matches_build),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
