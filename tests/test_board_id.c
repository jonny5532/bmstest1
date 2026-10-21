#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "app/board_id.h"
#include "config/limits.h"

#define R_TOP 10000.0f
#define R_PULL 50000.0f // RP2350 internal pull, roughly

static float divider(float r_top, float r_bottom) {
    return 4095.0f * r_bottom / (r_bottom + r_top);
}

static float parallel(float a, float b) {
    return a * b / (a + b);
}

// Readings for R75 = 10k and the given R76, with the internal pull-up, the
// pull-down and no pulls, scaled by (1 + err) on R76 to model tolerance.
static uint16_t classify_strap(float r_bottom, float err) {
    float rb = r_bottom * (1.0f + err);
    float up = divider(parallel(R_TOP, R_PULL), rb);
    float down = divider(R_TOP, parallel(rb, R_PULL));
    float open = divider(R_TOP, rb);
    return board_id_classify(up, down, open);
}

static void test_rev0_floating(void **state) {
    (void)state;
    // Floating: the pulls drag the pin rail to rail; the unpulled reading is noise
    assert_int_equal(board_id_classify(4095.0f, 0.0f, 1234.0f), BOARD_ID_REV0);
    assert_int_equal(board_id_classify(3900.0f, 150.0f, 4095.0f), BOARD_ID_REV0);
}

static void test_rev1_0_grounded(void **state) {
    (void)state;
    assert_int_equal(board_id_classify(0.0f, 0.0f, 0.0f), 10);
    assert_int_equal(board_id_classify(3.0f, 0.0f, 1.5f), 10);
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
    float up = divider(parallel(R_TOP, R_PULL), rb);
    float down = divider(R_TOP, parallel(rb, R_PULL));
    assert_true(up - down < BOARD_ID_FLOATING_SPAN_COUNTS / 2.0f);
}

static void test_bad_reading_is_unknown(void **state) {
    (void)state;
    assert_int_equal(board_id_classify(10.0f, 0.0f, NAN), BOARD_ID_UNKNOWN);
    assert_int_equal(board_id_classify(10.0f, 0.0f, -5.0f), BOARD_ID_UNKNOWN);
    assert_int_equal(board_id_classify(10.0f, 0.0f, 5000.0f), BOARD_ID_UNKNOWN);
}

static void test_matches_build(void **state) {
    (void)state;
#if BMS_BOARD == BMS_BOARD_REV1
    assert_true(board_id_matches_build(10));
    assert_true(board_id_matches_build(17));
    assert_false(board_id_matches_build(BOARD_ID_REV0));
#else
    assert_true(board_id_matches_build(BOARD_ID_REV0));
    assert_false(board_id_matches_build(10));
    assert_false(board_id_matches_build(13));
#endif
    assert_false(board_id_matches_build(BOARD_ID_UNKNOWN));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_rev0_floating),
        cmocka_unit_test(test_rev1_0_grounded),
        cmocka_unit_test(test_every_strap_value_with_tolerance),
        cmocka_unit_test(test_pulls_dont_look_floating_on_a_strap),
        cmocka_unit_test(test_bad_reading_is_unknown),
        cmocka_unit_test(test_matches_build),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
