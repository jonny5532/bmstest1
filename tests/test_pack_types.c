#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app/battery/pack_types.h"
#include "app/model.h"
#include "config/limits.h"
#include "sys/events/events.h"
#include "drivers/bmb3y/bmb3y.h"
#include "drivers/bmb3y/crc.h"
#include "sys/logging/logging.h"
#include "sys/time/time.h"

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
}

millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;

int bmb3y_count_live_channels(void) { return -1; }
int bmb3y_count_cell_modules(void) { return -1; }

static const uint16_t VECTORS[][2] = {
    // raw, module ID
    { 0xF804, 0x004 },
    { 0x8805, 0x005 },
    { 0xA006, 0x006 },
    { 0x201A, 0x01A },
    { 0x8023, 0x023 },
    { 0x7860, 0x060 },
};

static bool word_valid(uint16_t raw) {
    return (raw >> BMB3Y_MODULE_ID_CHECK_SHIFT) == module_id_check_bits(raw & BMB3Y_MODULE_ID_MASK);
}

static void test_check_bits(void **state) {
    (void)state;
    for(size_t i=0; i<sizeof(VECTORS)/sizeof(VECTORS[0]); i++) {
        uint16_t raw = VECTORS[i][0];
        assert_int_equal(raw & BMB3Y_MODULE_ID_MASK, VECTORS[i][1]);
        assert_true(word_valid(raw));
        // Any one-bit error fails the check
        for(int bit=0; bit<16; bit++) {
            assert_false(word_valid(raw ^ (uint16_t)(1 << bit)));
        }
    }
}

static pack_context_t context(int num_cells, uint8_t chemistry, int present) {
    return (pack_context_t){
        .num_cells = num_cells, .chemistry = chemistry, .present = present,
        .live_channels = -1, .cell_modules = -1, .mean_cell_mV = -1,
    };
}

static void classify(const uint16_t *ids, int count, pack_context_t ctx, pack_classification_t *c) {
    pack_types_classify_ids(ids, count, &ctx, c);
}

static void test_sample_readout(void **state) {
    (void)state;
    const uint16_t ids[] = { 0x004, 0x005, 0x005, 0x005, 0x006 };
    pack_classification_t c;
    classify(ids, 5, context(96, NMC, 5), &c);
    assert_int_equal(c.result, PACK_RESULT_CLASSIFIED);
    assert_int_equal(c.id_count, 3);
    assert_int_equal(c.candidates[0]->series, 96);
    assert_int_equal(c.candidates[0]->parallel, 46);
    assert_int_equal(c.candidates[0]->range, PACK_RANGE_LR);
    assert_int_equal(c.candidates[0]->temp_sensors, 16);
}

static void test_same_id_on_every_bmb(void **state) {
    (void)state;
    const uint16_t ids[] = { 0x004, 0x004, 0x004, 0x004, 0x004, 0x004, 0x004, 0x004 };
    pack_classification_t c;
    classify(ids, 8, context(96, NMC, 8), &c);
    assert_int_equal(c.result, PACK_RESULT_CLASSIFIED);
    assert_int_equal(c.id_count, 1);
    assert_int_equal(c.candidates[0]->temp_sensors, 16);
}

static void test_ambiguous_sensor_variants(void **state) {
    (void)state;
    const uint16_t ids[] = { 0x005, 0x006 };
    pack_classification_t c;
    classify(ids, 2, context(96, NMC, 2), &c);
    assert_int_equal(c.result, PACK_RESULT_AMBIGUOUS);
    assert_int_equal(c.candidate_count, 2);
    for(int i=0; i<2; i++) {
        assert_int_equal(c.candidates[i]->series, 96);
        assert_int_equal(c.candidates[i]->parallel, 46);
    }
    assert_int_not_equal(c.candidates[0]->temp_sensors, c.candidates[1]->temp_sensors);
}

static void test_shared_id_split_by_profile(void **state) {
    (void)state;
    // 0x03D is listed for both 96s31p SR NMC and 108s1p SR LFP
    const uint16_t ids[] = { 0x03D, 0x03D, 0x03D, 0x03D, 0x03D, 0x03D, 0x03D, 0x03D };
    pack_classification_t c;

    classify(ids, 8, context(108, LFP, 8), &c);
    assert_int_equal(c.result, PACK_RESULT_CLASSIFIED);
    assert_int_equal(c.candidates[0]->series, 108);
    assert_int_equal(c.candidates[0]->chemistry, LFP);
    assert_int_equal(c.candidates[0]->devices, 8);

    classify(ids, 8, context(96, NMC, 8), &c);
    assert_int_equal(c.result, PACK_RESULT_AMBIGUOUS);
    assert_int_equal(c.candidate_count, 2);
    assert_int_equal(c.candidates[0]->parallel, 31);
    assert_int_equal(c.candidates[1]->parallel, 31);
}

static void test_device_count_narrows(void **state) {
    (void)state;
    // 108s1p lists 8 devices, so a single BMB answering doesn't fit it
    const uint16_t ids[] = { 0x03D };
    pack_classification_t c;
    classify(ids, 1, context(108, LFP, 1), &c);
    assert_int_equal(c.result, PACK_RESULT_CONFLICT);
}

static void test_conflict(void **state) {
    (void)state;
    pack_classification_t c;

    // LFP module IDs on an NMC profile
    const uint16_t lfp[] = { 0x023 };
    classify(lfp, 1, context(96, NMC, 8), &c);
    assert_int_equal(c.result, PACK_RESULT_CONFLICT);

    // IDs from two different topologies
    const uint16_t mixed[] = { 0x004, 0x009 };
    classify(mixed, 2, context(96, NMC, 2), &c);
    assert_int_equal(c.result, PACK_RESULT_CONFLICT);
}

static void test_unknown(void **state) {
    (void)state;
    const uint16_t ids[] = { 0x123, 0x004 };
    pack_classification_t c;
    classify(ids, 2, context(96, NMC, 2), &c);
    assert_int_equal(c.result, PACK_RESULT_UNKNOWN);
    assert_int_equal(c.unknown_count, 1);
    assert_int_equal(c.unknown_ids[0], 0x123);
}

static void test_no_ids(void **state) {
    (void)state;
    const pack_type_t *out[PACK_TYPE_MAX_CANDIDATES];
    assert_int_equal(pack_types_match(NULL, 0, out, PACK_TYPE_MAX_CANDIDATES), 0);

    pack_classification_t c;
    classify(NULL, 0, context(96, NMC, 0), &c);
    assert_int_equal(c.result, PACK_RESULT_NO_ID);
}

static void test_cross_checks(void **state) {
    (void)state;
    const uint16_t ids[] = { 0x023, 0x023, 0x023, 0x023, 0x023, 0x023, 0x023, 0x023 };
    pack_context_t ctx = context(108, LFP, 8);
    pack_classification_t c;

    ctx.live_channels = 108;
    ctx.cell_modules = 8;
    ctx.mean_cell_mV = 3300;
    classify(ids, 8, ctx, &c);
    assert_int_equal(c.result, PACK_RESULT_CLASSIFIED);
    assert_true(c.cells_ok);
    assert_true(c.devices_ok);
    assert_false(c.voltage_suggests_nmc);

    ctx.live_channels = 96;
    ctx.cell_modules = 7;
    ctx.mean_cell_mV = 3900;
    classify(ids, 8, ctx, &c);
    assert_false(c.cells_ok);
    assert_false(c.devices_ok);
    assert_true(c.voltage_suggests_nmc);
}

static void test_table_rules(void **state) {
    (void)state;
    int rows = 0, custom_rows = 0;
    for(const pack_type_t *t; (t = pack_type_at(rows)) != NULL; rows++) {
        assert_int_equal(pack_type_index(t), rows);
        if(t->custom) {
            // From the profile: last, and only when it lists its IDs
            custom_rows++;
            assert_int_equal(t->series, NUM_CELLS);
            assert_int_equal(t->chemistry, CHEMISTRY);
            assert_int_equal(t->devices, NUM_MODULE_VOLTAGES);
        } else {
            // Chemistry is inferred from the parallel count
            assert_int_equal(custom_rows, 0);
            assert_int_equal(t->chemistry == LFP, t->parallel == 1);
        }
        assert_int_equal(t->module_id_count == 0, t->module_ids == NULL);
        for(int i=0; i<t->module_id_count; i++) {
            assert_true(t->module_ids[i] < 0x800);
            assert_true(pack_types_id_known(t->module_ids[i]));
        }
    }
#ifdef CUSTOM_PACK_MODULE_IDS
    assert_int_equal(custom_rows, 1);
#else
    assert_int_equal(custom_rows, 0);
#endif
    assert_int_equal(rows, 34 + custom_rows);
}

// A model whose first 'bmbs' BMBs all reported the same valid raw ID word
static void model_with_ids(bms_model_t *m, int bmbs, uint16_t raw) {
    extern bms_event_slot_t bms_event_slots[ERR_HIGHEST];
    for(int i=0; i<ERR_HIGHEST; i++) {
        bms_event_slots[i] = (bms_event_slot_t){0};
    }
    *m = (bms_model_t){ .pack_type_index = -1, .module_ids_millis = 1 };
    m->module_ids_present_mask = (uint8_t)((1 << bmbs) - 1);
    m->module_ids_valid_mask = m->module_ids_present_mask;
    for(int i=0; i<bmbs; i++) {
        m->module_ids_raw[i] = raw;
    }
}

static void test_conflict_is_fatal(void **state) {
    (void)state;
    static bms_model_t m;
#if BMS_PROFILE == BMS_LFP1
    // 0x004 is a 96s46p NMC ID: the wrong pack for a 108s LFP profile
    model_with_ids(&m, 8, 0xF804);
#else
    // 0x023 is a 108s1p LFP ID. On the default 96s NMC profile that's a
    // conflict; on the Desk profile (its custom pack lists 0x023, with 1 BMB)
    // eight BMBs are.
    model_with_ids(&m, 8, 0x8023);
#endif
    pack_types_classify(&m);
    assert_int_equal(get_event_level(ERR_BMB_MODULE_ID_CONFLICT), LEVEL_FATAL);
    assert_int_equal(get_event_level(ERR_BMB_MODULE_ID_CHECK_FAILED), LEVEL_NONE);
    assert_int_equal(m.pack_type_index, -1);
}

static void test_classified_index(void **state) {
    (void)state;
    static bms_model_t m;
#if BMS_PROFILE == BMS_DESK
    // The desk's one BMB, from a 108s1p LFP pack, measuring its 15 cells
    model_with_ids(&m, 1, 0x8023);
#elif BMS_PROFILE == BMS_LFP1
    // ck1: eight BMBs reporting 0x023
    model_with_ids(&m, 8, 0x8023);
#else
    model_with_ids(&m, 8, 0xF804);
#endif
    pack_types_classify(&m);
    assert_int_equal(get_event_level(ERR_BMB_MODULE_ID_CONFLICT), LEVEL_NONE);
    assert_int_equal(get_event_level(ERR_BMB_MODULE_ID_UNKNOWN), LEVEL_NONE);
    const pack_type_t *t = pack_type_at(m.pack_type_index);
    assert_non_null(t);
#if BMS_PROFILE == BMS_DESK
    assert_true(t->custom);
    assert_int_equal(t->series, 15);
    assert_int_equal(t->chemistry, NMC);
    assert_int_equal(t->devices, 1);
#elif BMS_PROFILE == BMS_LFP1
    assert_false(t->custom);
    assert_int_equal(t->series, 108);
    assert_int_equal(t->parallel, 1);
    assert_int_equal(t->chemistry, LFP);
    assert_int_equal(t->devices, 8);
#else
    assert_false(t->custom);
    assert_int_equal(t->series, 96);
    assert_int_equal(t->parallel, 46);
    assert_int_equal(t->temp_sensors, 16);
#endif
}

static void test_partial_read_would_conflict(void **state) {
    (void)state;
    // Why the IDs are only read after a full cell read: a chain still waking
    // up answers with fewer BMBs, and where the pack type gives a BMB count
    // (ck1's 108s1p LFP: 8) the good IDs then fit no row
    pack_classification_t c;
    const uint16_t ids[] = { 0x023, 0x023, 0x023, 0x023, 0x023 };
    classify(ids, 5, context(108, LFP, 5), &c);
    assert_int_equal(c.result, PACK_RESULT_CONFLICT);
    classify(ids, 5, context(108, LFP, 8), &c);
    assert_int_equal(c.result, PACK_RESULT_CLASSIFIED);
}

static void test_read_waits_for_full_cell_read(void **state) {
    (void)state;
    // No full cell read yet: not even a CLI request reads
    assert_false(pack_types_module_id_read_due(false, false, -1));
    assert_false(pack_types_module_id_read_due(false, true, -1));
    // After one: read once, then only on request
    assert_true(pack_types_module_id_read_due(false, false, 8));
    assert_false(pack_types_module_id_read_due(true, false, 8));
    assert_true(pack_types_module_id_read_due(true, true, 8));
    // A full read is full even if BMBs failed their CRCs on it
    assert_true(pack_types_module_id_read_due(false, false, 0));
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test(test_check_bits),
        cmocka_unit_test(test_sample_readout),
        cmocka_unit_test(test_same_id_on_every_bmb),
        cmocka_unit_test(test_ambiguous_sensor_variants),
        cmocka_unit_test(test_shared_id_split_by_profile),
        cmocka_unit_test(test_device_count_narrows),
        cmocka_unit_test(test_conflict),
        cmocka_unit_test(test_unknown),
        cmocka_unit_test(test_no_ids),
        cmocka_unit_test(test_cross_checks),
        cmocka_unit_test(test_table_rules),
        cmocka_unit_test(test_conflict_is_fatal),
        cmocka_unit_test(test_classified_index),
        cmocka_unit_test(test_partial_read_would_conflict),
        cmocka_unit_test(test_read_waits_for_full_cell_read),
    };
    return cmocka_run_group_tests(tests, NULL, NULL);
}
