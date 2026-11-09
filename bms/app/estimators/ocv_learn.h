#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lib/persist.h"

// Learns a correction to the OCV table from near-rest voltages, against Ah below
// settled full charge, and reads off the Ah from full down to OCV_LEARN_V_REF.
// Readings are held until the next full charge, so their Ah below full can be
// interpolated between the two (which also cancels any constant current offset).
// It never uses the EKF's own SOC, so errors can't feed back into it.

#define OCV_LEARN_BINS 13         // corrections evenly spaced from full to the factory curve's 0%
#define OCV_LEARN_MAX_READINGS 256 // one every 2 hours for 21 days
#define OCV_LEARN_FULL_SETTLE_S 600 // near the charge voltage and quiet this long counts as full

// ocv_learn_t.capacity_state
#define OCV_LEARN_CAPACITY_NOT_LEARNING 0 // no settled full charge yet
#define OCV_LEARN_CAPACITY_NO_DATA      1 // no bin below full trusted yet (or no longer), or too
                                          // uncertain (over 25%) to give a figure
#define OCV_LEARN_CAPACITY_PRELIMINARY  2 // trusted bins, but too shallow for a good figure
#define OCV_LEARN_CAPACITY_GOOD         3 // uncertainty within 5% of the capacity

typedef struct {
    uint32_t t_s;   // learner time of the reading
    float dq_Ah;    // charge counter relative to the full charge before it
    float v;        // OCV estimate (V/cell)
    float sv;       // its sigma (V)
} ocv_reading_t;

// Saved to NVM
// Reserved bytes will need adding to keep the members aligned (the compiler
// will warn if needed).
PERSIST_BEGIN
typedef struct {
    bool valid;
    uint8_t reserved_1[3];
    float s_full;   // factory-curve SOC of the charge voltage, fixed when learning starts
    double corr[OCV_LEARN_BINS];   // learned OCV correction (V)
    double P[OCV_LEARN_BINS][OCV_LEARN_BINS];
    // Full-charge cycles that gave each bin at least one reading's worth of data;
    // decays daily, so old data stops counting as confirmation
    float cycles[OCV_LEARN_BINS];
    uint8_t reserved_1516[4];
} ocv_learn_persistent_t;
PERSIST_END

// Ensure the number of bins hasn't changed (update OCV_LEARN_VERSION if it does)
_Static_assert(OCV_LEARN_BINS == 13, "OCV_LEARN_BINS changes the saved layout");

typedef struct {
    ocv_learn_persistent_t persistent;

    float capacity_ref_Ah;  // Ah from full to OCV_LEARN_V_REF; NAN until the bins there are trusted

    // Whole-pack capacity from every trusted bin: each bin's Ah below full was
    // measured, its depth below full is read off the factory curve, so deeper
    // bins lean on the curve less and count for more. NAN until a bin is trusted.
    float capacity_Ah;
    float capacity_sd_Ah;         // 1-sigma uncertainty, mostly from the factory curve
    uint8_t capacity_state;       // OCV_LEARN_CAPACITY_*
    float capacity_depth;         // factory-curve SOC below full of the deepest trusted bin
    float capacity_depth_V;       // learned OCV there (V/cell)
    float capacity_depth_cycles;  // that bin's decaying count of full charge cycles
    bool dirty;             // persistent data changed and wants saving
    bool edited;            // persistent data changed from outside (the HMI); rebuild on the next tick

    const float *factory;   // the factory OCV table, set on the first tick
    double cap;             // actual capacity (Ah), which maps Ah below full to SOC
    float table[101];       // factory + learned correction, for the EKF
    uint32_t t_s;           // seconds since boot (one tick per second)
    double charge_Ah;       // running charge count
    uint32_t quiet_s, low_s, full_s, last_reading_s;
    bool have_anchor;       // a settled full charge seen since boot
    double anchor_Ah;
    uint32_t anchor_s;
    uint16_t n_readings;
    ocv_reading_t readings[OCV_LEARN_MAX_READINGS];
} ocv_learn_t;

// Call once a second with the factory OCV table. Returns true when table has been
// (re)built. charge_V is the per-cell voltage charging stops at; a quiet pack
// within 10mV of it is full.
bool ocv_learn_tick(ocv_learn_t *l, const float factory[101], double capacity_Ah, float charge_V,
                    double charge_Ah, float current_A, float voltage_V);

// Set where full sits on the factory curve (0 < s_full <= 1), e.g. restoring a
// saved learner. Starts learning from the prior if it hadn't started. Returns
// false, changing nothing, for an out of range value.
bool ocv_learn_set_s_full(ocv_learn_t *l, float s_full);

// After changing persistent data directly: rebuild the table and save it
static inline void ocv_learn_mark_edited(ocv_learn_t *l) {
    l->edited = true;
    l->dirty = true;
}

// At a settled full charge, which sits at SOC persistent.s_full on the table
static inline bool ocv_learn_at_full(const ocv_learn_t *l) {
    return l->persistent.valid && l->full_s >= OCV_LEARN_FULL_SETTLE_S;
}
