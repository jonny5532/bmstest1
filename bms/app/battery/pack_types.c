#include "pack_types.h"

#include "app/model.h"
#include "config/limits.h"
#include "drivers/bmb3y/bmb3y.h"
#include "shared/cell_map.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"

#include <stddef.h>
#include <stdio.h>

// Transcribed from the four type reference tables - verify before relying on
// it. One row per reference entry: rows with the same topology are matched as
// one group. "Not listed" is 0 and never rejects a match. Disabled rows have no
// IDs, so can't be matched, but are kept as known topologies.
//
// Chemistry isn't in the reference tables. It's inferred: 1p is LFP (large
// prismatic cells), 31p-46p is NMC (small cylindrical cells), and 6p-10p is
// taken as NMC with larger cells (the least certain).

#define R_UNC PACK_RANGE_UNKNOWN, true
#define R_SR PACK_RANGE_SR, false
#define R_MR PACK_RANGE_MR, false
#define R_LR PACK_RANGE_LR, false
#define ID_COUNT(name) (uint8_t)(sizeof(name) / sizeof(name[0]))
#define IDS(name) name, ID_COUNT(name), false
#define DISABLED NULL, 0, false

static const uint16_t IDS_104S6P[] = { 0x020 };
static const uint16_t IDS_96S10P[] = { 0x025 };
static const uint16_t IDS_92S9P_D6[] = { 0x02B };
static const uint16_t IDS_92S9P_A[] = { 0x02E, 0x031, 0x032 };
static const uint16_t IDS_92S9P_B[] = { 0x031, 0x032 };
static const uint16_t IDS_92S9P_C[] = { 0x031, 0x032, 0x034 };
static const uint16_t IDS_92S9P_D[] = { 0x038 };
static const uint16_t IDS_92S9P_T16[] = { 0x057 };
static const uint16_t IDS_96S46P_LR_T16_A[] = { 0x004 };
static const uint16_t IDS_96S46P_LR_T16_B[] = { 0x005, 0x006 };
static const uint16_t IDS_96S31P_SR_T16[] = { 0x00D, 0x03D, 0x05D, 0x06D, 0x075, 0x07D };
static const uint16_t IDS_96S37P_MR_T13[] = { 0x007, 0x385, 0x585, 0x685, 0x705, 0x785 };
static const uint16_t IDS_96S37P_MR_T8[] = { 0x009 };
static const uint16_t IDS_96S46P_LR_T8_A[] = { 0x005, 0x006, 0x008, 0x00F, 0x013, 0x016, 0x017, 0x019, 0x022, 0x044, 0x045, 0x049 };
static const uint16_t IDS_96S32P[] = { 0x00B };
static const uint16_t IDS_96S31P_SR_T8[] = { 0x00A, 0x00D, 0x00E, 0x012, 0x014, 0x015, 0x018, 0x03D, 0x05D, 0x06D, 0x075, 0x07D };
static const uint16_t IDS_96S35P[] = { 0x042 };
static const uint16_t IDS_96S46P_LR_T8_B[] = { 0x019, 0x040, 0x041, 0x058 };
static const uint16_t IDS_96S46P_LR_T8_C[] = { 0x019, 0x044, 0x049, 0x04B, 0x05C };
static const uint16_t IDS_96S40P_SR_A[] = { 0x050, 0x05B };
static const uint16_t IDS_100S46P_LR[] = { 0x052, 0x053 };
static const uint16_t IDS_96S40P_SR_B[] = { 0x05B };
static const uint16_t IDS_106S1P_SR[] = { 0x01A };
static const uint16_t IDS_108S1P_SR_A[] = { 0x023 };
static const uint16_t IDS_105S1P_SR_A[] = { 0x02C };
static const uint16_t IDS_105S1P_SR_B[] = { 0x030 };
static const uint16_t IDS_105S1P_SR_C[] = { 0x033 };
static const uint16_t IDS_108S1P_SR_B[] = { 0x03D };
static const uint16_t IDS_108S1P_SR_C[] = { 0x048 };
static const uint16_t IDS_108S1P_SR_D[] = { 0x048, 0x060 };
static const uint16_t IDS_108S1P_SR_E[] = { 0x060 };
#ifdef CUSTOM_PACK_MODULE_IDS
static const uint16_t IDS_CUSTOM_PACK[] = CUSTOM_PACK_MODULE_IDS;
#endif

static const pack_type_t PACK_TYPES[] = {
    // series, parallel, chemistry, range, modules, devices, temps, IDs
    { 104,  6, NMC, R_UNC, 0, 8,  8, IDS(IDS_104S6P) },
    {  96, 10, NMC, R_UNC, 0, 8,  8, IDS(IDS_96S10P) },
    {  92,  9, NMC, R_UNC, 0, 6,  8, IDS(IDS_92S9P_D6) },
    {  92,  9, NMC, R_UNC, 0, 8,  8, IDS(IDS_92S9P_A) },
    {  92,  9, NMC, R_UNC, 0, 8,  8, IDS(IDS_92S9P_B) },
    {  92,  9, NMC, R_UNC, 0, 8,  8, IDS(IDS_92S9P_C) },
    {  92,  9, NMC, R_UNC, 0, 8,  8, IDS(IDS_92S9P_D) },
    {  92,  9, NMC, R_UNC, 0, 8, 16, IDS(IDS_92S9P_T16) },
    {  96, 31, NMC, R_SR,  0, 0, 16, DISABLED },
    {  96, 46, NMC, R_LR,  0, 0, 16, IDS(IDS_96S46P_LR_T16_A) },
    {  96, 46, NMC, R_LR,  0, 0, 16, IDS(IDS_96S46P_LR_T16_B) },
    {  96, 31, NMC, R_SR,  0, 0, 16, IDS(IDS_96S31P_SR_T16) },
    {  96, 43, NMC, R_UNC, 0, 0, 16, DISABLED },
    {  96, 37, NMC, R_MR,  0, 0, 13, IDS(IDS_96S37P_MR_T13) },
    {  96, 37, NMC, R_MR,  0, 0,  8, IDS(IDS_96S37P_MR_T8) },
    {  96, 46, NMC, R_LR,  0, 0,  8, IDS(IDS_96S46P_LR_T8_A) },
    {  96, 32, NMC, R_UNC, 0, 0,  8, IDS(IDS_96S32P) },
    {  96, 31, NMC, R_SR,  0, 0,  8, IDS(IDS_96S31P_SR_T8) },
    {  96, 35, NMC, R_UNC, 0, 0,  8, IDS(IDS_96S35P) },
    {  96, 46, NMC, R_LR,  0, 0,  8, IDS(IDS_96S46P_LR_T8_B) },
    {  96, 46, NMC, R_LR,  0, 0,  8, IDS(IDS_96S46P_LR_T8_C) },
    {  96, 40, NMC, R_SR,  0, 0,  8, IDS(IDS_96S40P_SR_A) },
    { 100, 46, NMC, R_LR,  0, 0,  8, IDS(IDS_100S46P_LR) },
    {  96, 40, NMC, R_SR,  0, 0,  8, IDS(IDS_96S40P_SR_B) },
    { 106,  1, LFP, R_SR,  4, 8,  6, DISABLED },
    { 106,  1, LFP, R_SR,  4, 8, 12, IDS(IDS_106S1P_SR) },
    { 108,  1, LFP, R_SR,  4, 8, 12, IDS(IDS_108S1P_SR_A) },
    { 105,  1, LFP, R_SR,  1, 7,  5, IDS(IDS_105S1P_SR_A) },
    { 105,  1, LFP, R_SR,  1, 7,  5, IDS(IDS_105S1P_SR_B) },
    { 105,  1, LFP, R_SR,  1, 7,  5, IDS(IDS_105S1P_SR_C) },
    { 108,  1, LFP, R_SR,  4, 8, 12, IDS(IDS_108S1P_SR_B) },
    { 108,  1, LFP, R_SR,  4, 8, 12, IDS(IDS_108S1P_SR_C) },
    { 108,  1, LFP, R_SR,  4, 8, 12, IDS(IDS_108S1P_SR_D) },
    { 108,  1, LFP, R_SR,  4, 8, 12, IDS(IDS_108S1P_SR_E) },
#ifdef CUSTOM_PACK_MODULE_IDS
    // This profile's own pack (config/limits.h)
    { NUM_CELLS, 0, CHEMISTRY, PACK_RANGE_UNKNOWN, false, 0, NUM_MODULE_VOLTAGES, 0,
      IDS_CUSTOM_PACK, ID_COUNT(IDS_CUSTOM_PACK), true },
#endif
};

#define PACK_TYPE_COUNT ((int)(sizeof(PACK_TYPES) / sizeof(PACK_TYPES[0])))

// Above this mean cell voltage the pack is almost certainly NMC. Below it,
// LFP and a low NMC pack overlap.
#define NMC_HINT_MEAN_CELL_mV 3600

int pack_type_index(const pack_type_t *type) {
    return type ? (int)(type - PACK_TYPES) : -1;
}

const pack_type_t *pack_type_at(int index) {
    return (index >= 0 && index < PACK_TYPE_COUNT) ? &PACK_TYPES[index] : NULL;
}

static bool same_topology(const pack_type_t *a, const pack_type_t *b) {
    return a->series == b->series && a->parallel == b->parallel &&
        a->chemistry == b->chemistry && a->range == b->range &&
        a->modules == b->modules && a->devices == b->devices &&
        a->temp_sensors == b->temp_sensors;
}

static bool row_has_id(const pack_type_t *type, uint16_t module_id) {
    for(int i=0; i<type->module_id_count; i++) {
        if(type->module_ids[i] == module_id) {
            return true;
        }
    }
    return false;
}

// Whether any row of the group starting at row 'first' lists the ID
static bool group_has_id(int first, uint16_t module_id) {
    for(int i=first; i<PACK_TYPE_COUNT; i++) {
        if(same_topology(&PACK_TYPES[first], &PACK_TYPES[i]) && row_has_id(&PACK_TYPES[i], module_id)) {
            return true;
        }
    }
    return false;
}

static bool is_group_first_row(int index) {
    for(int i=0; i<index; i++) {
        if(same_topology(&PACK_TYPES[i], &PACK_TYPES[index])) {
            return false;
        }
    }
    return true;
}

int pack_types_match(const uint16_t *ids, int count, const pack_type_t **out, int max_out) {
    if(count <= 0) {
        // No IDs would match every group
        return 0;
    }

    int found = 0;
    for(int g=0; g<PACK_TYPE_COUNT; g++) {
        if(!is_group_first_row(g)) {
            continue;
        }
        bool covers = true;
        for(int i=0; i<count && covers; i++) {
            covers = group_has_id(g, ids[i] & BMB3Y_MODULE_ID_MASK);
        }
        if(covers) {
            if(found < max_out) {
                out[found] = &PACK_TYPES[g];
            }
            found++;
        }
    }
    return found;
}

bool pack_types_id_known(uint16_t module_id) {
    module_id &= BMB3Y_MODULE_ID_MASK;
    for(int i=0; i<PACK_TYPE_COUNT; i++) {
        if(row_has_id(&PACK_TYPES[i], module_id)) {
            return true;
        }
    }
    return false;
}

const char *pack_range_name(pack_range_t range) {
    switch(range) {
        case PACK_RANGE_SR: return "SR";
        case PACK_RANGE_MR: return "MR";
        case PACK_RANGE_LR: return "LR";
        default: return "range?";
    }
}

const char *pack_chemistry_name(uint8_t chemistry) {
    switch(chemistry) {
        case NMC: return "NMC";
        case LFP: return "LFP";
        default: return "?";
    }
}

void pack_types_classify_ids(const uint16_t *ids, int count, const pack_context_t *context,
                             pack_classification_t *result) {
    *result = (pack_classification_t){ .result = PACK_RESULT_NO_ID, .cells_ok = true, .devices_ok = true };

    // Distinct IDs, and the ones no row lists
    for(int i=0; i<count; i++) {
        uint16_t id = ids[i] & BMB3Y_MODULE_ID_MASK;
        bool seen = false;
        for(int j=0; j<result->id_count; j++) {
            seen = seen || result->ids[j] == id;
        }
        if(seen || result->id_count >= 8) {
            continue;
        }
        result->ids[result->id_count++] = id;
        if(!pack_types_id_known(id)) {
            result->unknown_ids[result->unknown_count++] = id;
        }
    }

    uint8_t chemistry = context->chemistry;
    if(result->id_count > 0) {
        const pack_type_t *matched[PACK_TYPE_COUNT];
        int matched_count = pack_types_match(result->ids, result->id_count, matched, PACK_TYPE_COUNT);

        // Narrow to the profile and the chain that answered
        for(int i=0; i<matched_count; i++) {
            const pack_type_t *type = matched[i];
            if(type->chemistry != context->chemistry || type->series != context->num_cells) {
                continue;
            }
            if(type->devices != 0 && type->devices != context->present) {
                continue;
            }
            if(result->candidate_count < PACK_TYPE_MAX_CANDIDATES) {
                result->candidates[result->candidate_count++] = type;
            }
        }

        if(result->candidate_count == 1) {
            result->result = PACK_RESULT_CLASSIFIED;
        } else if(result->candidate_count > 1) {
            result->result = PACK_RESULT_AMBIGUOUS;
        } else if(result->unknown_count > 0) {
            result->result = PACK_RESULT_UNKNOWN;
        } else {
            result->result = PACK_RESULT_CONFLICT;
        }
    }

    if(result->result == PACK_RESULT_CLASSIFIED) {
        const pack_type_t *type = result->candidates[0];
        chemistry = type->chemistry;
        if(context->live_channels >= 0) {
            result->cells_ok = type->series == context->live_channels;
        }
        if(type->devices != 0 && context->cell_modules >= 0) {
            result->devices_ok = type->devices == context->cell_modules;
        }
    }

    result->voltage_suggests_nmc = chemistry == LFP && context->mean_cell_mV > NMC_HINT_MEAN_CELL_mV;
}

bool pack_types_module_id_read_due(bool ids_read, bool requested, int cell_modules) {
    return cell_modules >= 0 && (!ids_read || requested);
}

bool pack_types_evaluate(const bms_model_t *model, pack_classification_t *result) {
    if(model->module_ids_millis == 0) {
        *result = (pack_classification_t){ .result = PACK_RESULT_NOT_READ, .cells_ok = true, .devices_ok = true };
        return false;
    }

    uint16_t ids[CELL_MAP_MODULES];
    int count = 0;
    for(int module=0; module<CELL_MAP_MODULES; module++) {
        if(model->module_ids_valid_mask & (1 << module)) {
            ids[count++] = model->module_ids_raw[module] & BMB3Y_MODULE_ID_MASK;
        }
    }

    pack_context_t context = {
        .num_cells = NUM_CELLS,
        .chemistry = CHEMISTRY,
        .present = __builtin_popcount(model->module_ids_present_mask),
        .live_channels = bmb3y_count_live_channels(),
        .cell_modules = bmb3y_count_cell_modules(),
        .mean_cell_mV = model->cell_voltage_millis > 0 ? (int)(model->cell_voltage_total_mV / NUM_CELLS) : -1,
    };
    pack_types_classify_ids(ids, count, &context, result);
    return true;
}

// e.g. "96s46p LR NMC, 16 temp sensors" or "15s custom NMC, 1 devices"
static void format_topology(const pack_type_t *type, char *buf, size_t size) {
    int n;
    if(type->custom) {
        n = snprintf(buf, size, "%us custom %s", type->series, pack_chemistry_name(type->chemistry));
    } else {
        n = snprintf(buf, size, "%us%up %s %s", type->series, type->parallel,
            type->range_unconfirmed ? "range?" : pack_range_name(type->range),
            pack_chemistry_name(type->chemistry));
    }
    if(type->modules && n >= 0 && (size_t)n < size) {
        n += snprintf(buf + n, size - n, ", %u modules", type->modules);
    }
    if(type->devices && n >= 0 && (size_t)n < size) {
        n += snprintf(buf + n, size - n, ", %u devices", type->devices);
    }
    if(type->temp_sensors && n >= 0 && (size_t)n < size) {
        snprintf(buf + n, size - n, ", %u temp sensors", type->temp_sensors);
    }
}

static void print_topology(const pack_type_t *type) {
    char buf[64];
    format_topology(type, buf, sizeof(buf));
    printf("%s", buf);
}

static void print_ids(const uint16_t *ids, int count) {
    for(int i=0; i<count; i++) {
        printf(" 0x%03X", ids[i]);
    }
}

void pack_types_classify(bms_model_t *model) {
    pack_classification_t c;
    if(!pack_types_evaluate(model, &c)) {
        return;
    }

    // Integrity: every BMB that answered passed its CRC and check bits
    uint8_t failed_mask = model->module_ids_present_mask & ~model->module_ids_valid_mask;
    uint16_t failed_raw = 0;
    for(int module=CELL_MAP_MODULES-1; module>=0; module--) {
        if(failed_mask & (1 << module)) {
            failed_raw = model->module_ids_raw[module];
        }
    }
    if(!confirm(failed_mask == 0, ERR_BMB_MODULE_ID_CHECK_FAILED, ((uint64_t)failed_mask << 16) | failed_raw)) {
        warning_printf("Module ID check failed: slots 0x%02X (first raw 0x%04X)\n", failed_mask, failed_raw);
    }

    // Unknown IDs are logged over serial (no filesystem to keep them)
    confirm(c.result != PACK_RESULT_UNKNOWN, ERR_BMB_MODULE_ID_UNKNOWN, c.unknown_count ? c.unknown_ids[0] : 0);
    uint64_t conflict_data = 0;
    for(int i=0; i<c.id_count && i<4; i++) {
        conflict_data |= (uint64_t)c.ids[i] << (16 * i);
    }
    confirm(c.result != PACK_RESULT_CONFLICT, ERR_BMB_MODULE_ID_CONFLICT, conflict_data);

    model->pack_type_index = (int8_t)(c.result == PACK_RESULT_CLASSIFIED ? pack_type_index(c.candidates[0]) : -1);

    if(c.id_count > 1) {
        // Believed to be one ID per pack, so worth knowing
        warning_printf("Module IDs: BMBs report %u different IDs\n", c.id_count);
    }

    switch(c.result) {
        case PACK_RESULT_NO_ID:
            warning_printf("Module IDs: no valid ID (%d BMBs answered)\n", __builtin_popcount(model->module_ids_present_mask));
            break;
        case PACK_RESULT_CLASSIFIED: {
            char topology[64];
            format_topology(c.candidates[0], topology, sizeof(topology));
            info_printf("Pack type %d: %s (id 0x%03X)\n", pack_type_index(c.candidates[0]), topology, c.ids[0]);
            break;
        }
        case PACK_RESULT_AMBIGUOUS:
            info_printf("Pack type ambiguous: %u candidates (id 0x%03X)\n", c.candidate_count, c.ids[0]);
            break;
        case PACK_RESULT_UNKNOWN:
            for(int i=0; i<c.unknown_count; i++) {
                warning_printf("Module ID 0x%03X unknown\n", c.unknown_ids[i]);
            }
            break;
        case PACK_RESULT_CONFLICT:
            warning_printf("Module IDs conflict: no topology has all %u IDs (first 0x%03X) and %d %s cells\n",
                c.id_count, c.ids[0], NUM_CELLS, pack_chemistry_name(CHEMISTRY));
            break;
        default:
            break;
    }

    if(!c.cells_ok) {
        warning_printf("Pack type is %us, but %d channels read a cell\n",
            c.candidates[0]->series, bmb3y_count_live_channels());
    }
    if(!c.devices_ok) {
        warning_printf("Pack type has %u devices, but %d BMBs answered the cell reads\n",
            c.candidates[0]->devices, bmb3y_count_cell_modules());
    }
}

void pack_types_print_verdict(const bms_model_t *model) {
    pack_classification_t c;
    if(!pack_types_evaluate(model, &c)) {
        return;
    }

    switch(c.result) {
        case PACK_RESULT_NO_ID:
            printf("Pack: NO VALID ID (%d BMBs answered)\n", __builtin_popcount(model->module_ids_present_mask));
            return;
        case PACK_RESULT_CLASSIFIED:
            printf("Pack: ");
            print_topology(c.candidates[0]);
            printf(" (1 candidate)\n");
            break;
        case PACK_RESULT_AMBIGUOUS:
            printf("Pack: AMBIGUOUS, %u candidates:\n", c.candidate_count);
            for(int i=0; i<c.candidate_count; i++) {
                printf("  ");
                print_topology(c.candidates[i]);
                printf("\n");
            }
            break;
        case PACK_RESULT_UNKNOWN:
            printf("Pack: UNKNOWN ids:");
            print_ids(c.unknown_ids, c.unknown_count);
            printf("\n");
            break;
        case PACK_RESULT_CONFLICT:
            printf("Pack: CONFLICT ids:");
            print_ids(c.ids, c.id_count);
            printf(" (profile %d cells %s)\n", NUM_CELLS, pack_chemistry_name(CHEMISTRY));
            break;
        default:
            return;
    }

    if(c.id_count > 1) {
        printf("Pack: %u different IDs:", c.id_count);
        print_ids(c.ids, c.id_count);
        printf("\n");
    }
    if(c.result == PACK_RESULT_CLASSIFIED) {
        int live = bmb3y_count_live_channels();
        int cell_modules = bmb3y_count_cell_modules();
        printf("Pack: measured %d cells %s, %d BMBs answering cell reads %s\n",
            live, live < 0 ? "(not read)" : (c.cells_ok ? "OK" : "MISMATCH"),
            cell_modules, cell_modules < 0 ? "(not read)" : (c.devices_ok ? "OK" : "MISMATCH"));
    }
    if(c.voltage_suggests_nmc) {
        printf("Pack: voltage suggests NMC\n");
    }
}
