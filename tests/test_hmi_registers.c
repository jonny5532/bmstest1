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
#include "app/firmware_id.h"

#include "pico/unique_id.h"

// Mock globals
millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;

// Stand in for the generated firmware ID
const uint32_t bms_build_hash = 0x8b0c547u | BMS_BUILD_HASH_DIRTY;
const uint32_t bms_build_time = 1792576800u;
const char bms_build_version[] = "8b0c547-dirty, built 2026-10-21 10:00 UTC";

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


static void test_mcu_temperature_register(void **state) {
    (void) state;
    assert_false(hmi_register_is_available(HMI_REG_MCU_TEMPERATURE, &model));
    model.mcu_temperature = 47.3f;
    model.mcu_temperature_millis = 1;
    assert_true(hmi_register_is_available(HMI_REG_MCU_TEMPERATURE, &model));
    assert_true(read_float_register(HMI_REG_MCU_TEMPERATURE) == 47.3f);
}

static uint32_t read_uint32_register(uint16_t reg_id) {
    uint8_t buf[16];
    uint8_t len = hmi_append_register_value(buf, reg_id, &model);
    assert_int_equal(len, 2 + 1 + 4);
    assert_int_equal((uint16_t)buf[0] | ((uint16_t)buf[1] << 8), reg_id);
    assert_int_equal(buf[2], HMI_TYPE_UINT32);
    return (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16) | ((uint32_t)buf[6] << 24);
}

static void test_build_id_registers(void **state) {
    (void) state;
    // Constants, so available from boot
    assert_true(hmi_register_is_available(HMI_REG_BUILD_HASH, &model));
    assert_true(hmi_register_is_available(HMI_REG_BUILD_TIME, &model));
    assert_int_equal(read_uint32_register(HMI_REG_BUILD_HASH), 0x8b0c547u | BMS_BUILD_HASH_DIRTY);
    assert_int_equal(read_uint32_register(HMI_REG_BUILD_TIME), 1792576800u);
}

static void test_lifetime_registers(void **state) {
    (void) state;
    // Loaded from NVM at boot, so available from the start
    assert_true(hmi_register_is_available(HMI_REG_LIFETIME_CHARGED_MAH, &model));
    model.lifetime_charged_mAh = 0xFFFFFFF0u;
    model.lifetime_discharged_mAh = 1234567u;
    model.lifetime_charged_Wh = 89012u;
    model.lifetime_discharged_Wh = 3456u;
    assert_int_equal(read_uint32_register(HMI_REG_LIFETIME_CHARGED_MAH), 0xFFFFFFF0u);
    assert_int_equal(read_uint32_register(HMI_REG_LIFETIME_DISCHARGED_MAH), 1234567u);
    assert_int_equal(read_uint32_register(HMI_REG_LIFETIME_CHARGED_WH), 89012u);
    assert_int_equal(read_uint32_register(HMI_REG_LIFETIME_DISCHARGED_WH), 3456u);
}

static void test_unknown_register_writes_nothing(void **state) {
    (void) state;
    uint8_t buf[16];
    assert_int_equal(hmi_append_register_value(buf, 0xFFFF, &model), 0);
}

// ============================================================================
// The register map (protocol version 7) and LIST_REGISTERS
// ============================================================================

static void test_protocol_version(void **state) {
    (void) state;
    // Fixed IDs, the same as before the renumber, so any HMI can find them
    assert_int_equal(HMI_REG_SERIAL, 1);
    assert_int_equal(HMI_REG_PROTOCOL_VERSION, 71);
    uint8_t buf[16];
    assert_int_equal(hmi_append_register_value(buf, HMI_REG_PROTOCOL_VERSION, &model), 5);
    assert_int_equal(buf[2], HMI_TYPE_UINT16);
    assert_int_equal((uint16_t)buf[3] | ((uint16_t)buf[4] << 8), 7);
}

static void test_register_table_sorted_and_unique(void **state) {
    (void) state;
    assert_true(hmi_register_count() > 0);
    for (size_t i = 1; i < hmi_register_count(); i++) {
        assert_true(hmi_register_info(i - 1)->id < hmi_register_info(i)->id);
    }
    assert_null(hmi_register_info(hmi_register_count()));
}

// Put the model where every register is encoded (the OCV learner started)
static void make_all_registers_readable(void) {
#ifdef OCV_LEARN
    model.ocv_learn.persistent.valid = true;
#endif
}

static void test_register_table_matches_encoder(void **state) {
    (void) state;
    make_all_registers_readable();
    // Every listed register encodes with its listed type, other than the
    // write-only commands, which encode nothing
    for (size_t i = 0; i < hmi_register_count(); i++) {
        const hmi_register_info_t *r = hmi_register_info(i);
        uint8_t buf[16];
        uint8_t len = hmi_append_register_value(buf, r->id, &model);
        if ((r->id >> 8) == 0x30) {
            assert_int_equal(len, 0);
            assert_true(r->flags & HMI_REG_FLAG_WRITABLE);
            continue;
        }
        assert_int_equal(len, 2 + 1 + (r->type & 0x0F));
        assert_int_equal((uint16_t)buf[0] | ((uint16_t)buf[1] << 8), r->id);
        assert_int_equal(buf[2], r->type);
    }
    // And nothing outside the table encodes
    for (uint32_t id = 0; id <= 0xFFFF; id++) {
        uint8_t buf[16];
        if (hmi_append_register_value(buf, (uint16_t)id, &model) > 0) {
            assert_non_null(hmi_find_register((uint16_t)id));
        }
    }
}

static void test_no_old_register_ids_reused(void **state) {
    (void) state;
    // Before version 7 registers were 1-84 and 0x100-0x238. Only SERIAL and
    // PROTOCOL_VERSION keep old IDs, so an older HMI never reads another
    // register's value at one of them.
    for (size_t i = 0; i < hmi_register_count(); i++) {
        uint16_t id = hmi_register_info(i)->id;
        if (id == HMI_REG_SERIAL || id == HMI_REG_PROTOCOL_VERSION) continue;
        assert_false(id <= 84);
        assert_false(id >= 0x100 && id <= 0x238);
    }
}

static void test_settings_group_is_the_saved_registers(void **state) {
    (void) state;
    for (size_t i = 0; i < hmi_register_count(); i++) {
        const hmi_register_info_t *r = hmi_register_info(i);
        bool settings = (r->id >> 8) == 0x20;
        bool saved = r->flags & HMI_REG_FLAG_SAVED;
        if (settings) {
            assert_true(r->flags & HMI_REG_FLAG_WRITABLE);
            assert_true(saved);
        } else if (saved) {
            // Saved by the learner itself, so kept with it
            assert_int_equal(r->id, HMI_REG_OCV_LEARN_S_FULL);
        }
        // Saved implies writable
        if (saved) assert_true(r->flags & HMI_REG_FLAG_WRITABLE);
    }
}

static void test_list_registers_pages_cover_the_table(void **state) {
    (void) state;
    const uint8_t address = 3;
    uint16_t start = 0;
    int pages = 0;
    while (start < hmi_register_count()) {
        // A small capacity, to force several pages
        uint8_t buf[HMI_LIST_REGISTERS_HEADER_LEN + 10 * HMI_LIST_REGISTERS_ENTRY_LEN + 3];
        size_t len = hmi_build_list_registers_response(buf, sizeof(buf), address, start);
        assert_int_equal(buf[0], HMI_MSG_LIST_REGISTERS_RESPONSE);
        assert_int_equal(buf[1], address);
        assert_int_equal((uint16_t)buf[2] | ((uint16_t)buf[3] << 8), start);
        assert_int_equal((uint16_t)buf[4] | ((uint16_t)buf[5] << 8), hmi_register_count());
        uint8_t count = buf[6];
        assert_true(count > 0 && count <= 10);
        assert_int_equal(len, HMI_LIST_REGISTERS_HEADER_LEN + count * HMI_LIST_REGISTERS_ENTRY_LEN);
        for (uint8_t e = 0; e < count; e++) {
            const uint8_t *entry = &buf[HMI_LIST_REGISTERS_HEADER_LEN + e * HMI_LIST_REGISTERS_ENTRY_LEN];
            const hmi_register_info_t *r = hmi_register_info(start + e);
            assert_int_equal((uint16_t)entry[0] | ((uint16_t)entry[1] << 8), r->id);
            assert_int_equal(entry[2], r->type);
            assert_int_equal(entry[3], r->flags);
        }
        start += count;
        pages++;
    }
    assert_true(pages > 1);

    // Past the end: an empty page, still with the total
    uint8_t buf[256];
    size_t len = hmi_build_list_registers_response(buf, sizeof(buf), address, (uint16_t)hmi_register_count());
    assert_int_equal(len, HMI_LIST_REGISTERS_HEADER_LEN);
    assert_int_equal(buf[6], 0);
    // A full packet holds as many entries as fit
    len = hmi_build_list_registers_response(buf, sizeof(buf), address, 0);
    size_t fit = (sizeof(buf) - HMI_LIST_REGISTERS_HEADER_LEN) / HMI_LIST_REGISTERS_ENTRY_LEN;
    assert_int_equal(buf[6], hmi_register_count() < fit ? hmi_register_count() : fit);
}

static void test_old_cell_register_ranges_gone(void **state) {
    (void) state;
    uint8_t buf[16];
    assert_int_equal(hmi_append_register_value(buf, 0x100, &model), 0);
    assert_int_equal(hmi_append_register_value(buf, 0x200, &model), 0);
    assert_int_equal(hmi_append_register_value(buf, 0x208, &model), 0);
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_temperature_min_max_are_exact_floats, setup),
        cmocka_unit_test_setup(test_temperature_min_max_available_after_first_reading, setup),
        cmocka_unit_test_setup(test_mcu_temperature_register, setup),
        cmocka_unit_test_setup(test_build_id_registers, setup),
        cmocka_unit_test_setup(test_lifetime_registers, setup),
        cmocka_unit_test_setup(test_unknown_register_writes_nothing, setup),
        cmocka_unit_test_setup(test_protocol_version, setup),
        cmocka_unit_test_setup(test_register_table_sorted_and_unique, setup),
        cmocka_unit_test_setup(test_register_table_matches_encoder, setup),
        cmocka_unit_test_setup(test_no_old_register_ids_reused, setup),
        cmocka_unit_test_setup(test_settings_group_is_the_saved_registers, setup),
        cmocka_unit_test_setup(test_list_registers_pages_cover_the_table, setup),
        cmocka_unit_test_setup(test_old_cell_register_ranges_gone, setup),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
