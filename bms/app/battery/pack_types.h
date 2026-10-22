#pragma once

// Known pack topologies and the BMB module IDs (register 0x33) listed for
// each, and the classification of a pack from the IDs its BMBs report.

#include <stdbool.h>
#include <stdint.h>

typedef struct bms_model bms_model_t;

typedef enum {
    PACK_RANGE_UNKNOWN,
    PACK_RANGE_SR,
    PACK_RANGE_MR,
    PACK_RANGE_LR,
} pack_range_t;

typedef struct {
    uint8_t series;
    uint8_t parallel;
    uint8_t chemistry;          // NMC or LFP, as in config/limits.h
    pack_range_t range;
    bool range_unconfirmed;
    uint8_t modules;            // 0 = not listed
    uint8_t devices;            // BMBs on the chain, 0 = not listed
    uint8_t temp_sensors;
    const uint16_t *module_ids; // 11-bit IDs
    uint8_t module_id_count;    // 0 = "Disabled" (no IDs to match)
    bool custom;                // built from the profile's CUSTOM_PACK_MODULE_IDS
} pack_type_t;

#define PACK_TYPE_MAX_CANDIDATES 4

// Topology groups (rows with the same topology, their IDs combined) whose IDs
// cover every ID in ids[]. Each group is given by its first row. Returns the
// number found, which may be more than max_out. No IDs match nothing.
int pack_types_match(const uint16_t *ids, int count,
                     const pack_type_t **out, int max_out);
// Whether the ID is listed for any topology
bool pack_types_id_known(uint16_t module_id);
const char *pack_range_name(pack_range_t range);
const char *pack_chemistry_name(uint8_t chemistry);
// Index of a row in the table (for model->pack_type_index)
int pack_type_index(const pack_type_t *type);
const pack_type_t *pack_type_at(int index);

typedef enum {
    PACK_RESULT_NOT_READ,       // module IDs not read yet
    PACK_RESULT_NO_ID,          // no BMB gave a valid ID
    PACK_RESULT_CLASSIFIED,     // exactly one candidate
    PACK_RESULT_AMBIGUOUS,      // several candidates
    PACK_RESULT_UNKNOWN,        // an ID isn't in the table
    PACK_RESULT_CONFLICT,       // all IDs known, but no topology fits them and the profile
} pack_result_t;

// What the system around the IDs looks like, for narrowing and cross-checks
typedef struct {
    int num_cells;              // NUM_CELLS of the profile
    uint8_t chemistry;          // CHEMISTRY of the profile
    int present;                // BMBs that answered the module ID read
    int live_channels;          // channels reading a cell voltage, -1 = not read
    int cell_modules;           // BMBs that answered the cell reads, -1 = not read
    int mean_cell_mV;           // -1 = not read
} pack_context_t;

typedef struct {
    pack_result_t result;
    uint16_t ids[8];            // distinct valid IDs, in slot order
    uint8_t id_count;
    uint16_t unknown_ids[8];    // of those, the ones in no row
    uint8_t unknown_count;
    const pack_type_t *candidates[PACK_TYPE_MAX_CANDIDATES];
    uint8_t candidate_count;    // stored candidates (all of them, up to the max)
    // Cross-checks of a classified pack (true when there's nothing to check)
    bool cells_ok;              // series == live channels
    bool devices_ok;            // devices == BMBs answering the cell reads
    bool voltage_suggests_nmc;  // mean cell voltage too high for LFP
} pack_classification_t;

// Classify a set of valid IDs (one per BMB, repeats allowed)
void pack_types_classify_ids(const uint16_t *ids, int count, const pack_context_t *context,
                             pack_classification_t *result);

// Whether to read the module IDs on this idle BMB tick. Only after a full cell
// read (cell_modules >= 0): a chain still waking up may answer with fewer BMBs,
// which would look like the wrong pack. Then once, and again on each request.
bool pack_types_module_id_read_due(bool ids_read, bool requested, int cell_modules);

// Classify the IDs last read into the model. Returns false if none were read yet.
bool pack_types_evaluate(const bms_model_t *model, pack_classification_t *result);
// As above, then raise/clear the module ID events, set pack_type_index and log it
void pack_types_classify(bms_model_t *model);
// Print the pack verdict (after the per-BMB lines of bmb3y_print_module_ids)
void pack_types_print_verdict(const bms_model_t *model);
