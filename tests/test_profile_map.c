// A pack profile's cell map, as the BMS sends it to the supervisor, passes the
// supervisor's consistency checks (shared/cell_map.h). Built once per built-in
// profile (test_profile_map_<profile>), so a profile with a mask that doesn't
// match its cell count or module count fails here rather than leaving the
// contactors unable to close.

#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdint.h>

#include TEST_PROFILE_HEADER
#include "shared/cell_map.h"

static void test_profile_map_is_consistent(void **state) {
    (void) state;
    cell_map_t map = {
        .chemistry = CHEMISTRY,
        .num_cells = NUM_CELLS,
        .num_modules = NUM_MODULE_VOLTAGES,
        .presence = { CELL_PRESENCE_WORDS },
    };
    uint8_t errors = cell_map_check(&map);
    if(errors) {
        print_error("Profile %s: cell_map_check() = 0x%02x\n", TEST_PROFILE_NAME, errors);
    }
    assert_int_equal(errors, 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_profile_map_is_consistent),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
