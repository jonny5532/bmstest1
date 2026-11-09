#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "app/model.h"
#include "protocols/hmi_serial/hmi_serial.h"
#include "protocols/hmi_serial/hmi_registers.h"

#include "pico/unique_id.h"

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;

void pico_get_unique_board_id(pico_unique_board_id_t *id_out) {
    memset(id_out, 0, sizeof(*id_out));
}

bms_model_t model;

static int setup(void **state) {
    (void) state;
    memset(&model, 0, sizeof(model));
    return 0;
}

// Encode one register, check its id and type, and return the float value the
// HMI would decode
static float read_float_register(uint16_t reg_id) {
    uint8_t buf[16];
    uint8_t len = hmi_append_register_value(buf, reg_id, &model);
    assert_int_equal(len, 2 + 1 + 4);
    assert_int_equal((uint16_t)buf[0] | ((uint16_t)buf[1] << 8), reg_id);
    assert_int_equal(buf[2], HMI_TYPE_FLOAT);
    union {
        uint32_t u;
        float f;
    } u;
    u.u = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24);
    return u.f;
}

static void test_temperature_min_max_are_exact_floats(void **state) {
    (void) state;
    // Fractional and below-zero values, which once were truncated to whole
    // degrees (and below zero, wrapped) on the way out
    const float values[] = { -5.3f, -0.4f, 0.0f, 12.75f, 41.9f, -40.0f, 85.0f };
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        model.temperature_min = values[i];
        model.temperature_max = values[i] + 3.25f;
        assert_true(read_float_register(HMI_REG_TEMPERATURE_MIN) == model.temperature_min);
        assert_true(read_float_register(HMI_REG_TEMPERATURE_MAX) == model.temperature_max);
    }
}

static void test_temperature_min_max_available_after_first_reading(void **state) {
    (void) state;
    assert_false(hmi_register_is_available(HMI_REG_TEMPERATURE_MIN, &model));
    assert_false(hmi_register_is_available(HMI_REG_TEMPERATURE_MAX, &model));
    model.temperature_millis = 1;
    assert_true(hmi_register_is_available(HMI_REG_TEMPERATURE_MIN, &model));
    assert_true(hmi_register_is_available(HMI_REG_TEMPERATURE_MAX, &model));
}

static void test_module_temp_registers_are_floats(void **state) {
    (void) state;
    model.module_temperatures[0] = -12.6f;
    model.module_temperatures[7] = 33.3f;
    assert_true(read_float_register(HMI_REG_MODULE_TEMPS_START) == -12.6f);
    assert_true(read_float_register(HMI_REG_MODULE_TEMPS_START + 7) == 33.3f);
}

static void test_mcu_temperature_register(void **state) {
    (void) state;
    assert_false(hmi_register_is_available(HMI_REG_MCU_TEMPERATURE, &model));
    model.mcu_temperature = 47.3f;
    model.mcu_temperature_millis = 1;
    assert_true(hmi_register_is_available(HMI_REG_MCU_TEMPERATURE, &model));
    assert_true(read_float_register(HMI_REG_MCU_TEMPERATURE) == 47.3f);
}

static void test_unknown_register_writes_nothing(void **state) {
    (void) state;
    uint8_t buf[16];
    assert_int_equal(hmi_append_register_value(buf, 0xFFFF, &model), 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_temperature_min_max_are_exact_floats, setup),
        cmocka_unit_test_setup(test_temperature_min_max_available_after_first_reading, setup),
        cmocka_unit_test_setup(test_module_temp_registers_are_floats, setup),
        cmocka_unit_test_setup(test_mcu_temperature_register, setup),
        cmocka_unit_test_setup(test_unknown_register_writes_nothing, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
