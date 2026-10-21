#include "supervisor_logic.h"

#include "config/limits.h"

#include <string.h>

// Cell problems that block enabling, and latch once enabled
#define CELL_FAULT_REASONS (SUP_REASON_CELL_HIGH | SUP_REASON_CELL_LOW | SUP_REASON_CELL_MISSING)

// What unpopulated channels read hasn't been checked on every pack yet, so by
// default a channel the map says is empty but reads like a cell is only
// reported. Build with SUPERVISOR_ENFORCE_ABSENT_CHANNELS to treat it as a
// cell map mismatch.
#ifdef SUPERVISOR_ENFORCE_ABSENT_CHANNELS
#define SCAN_FAULT_REASONS (CELL_FAULT_REASONS | SUP_REASON_ABSENT_READS)
#else
#define SCAN_FAULT_REASONS CELL_FAULT_REASONS
#endif

uint16_t sup_check_cells(const int16_t *cell_mV, int16_t *min_mV, int16_t *max_mV, uint8_t *cells_ok) {
    uint16_t reasons = 0;
    int16_t min = INT16_MAX;
    int16_t max = INT16_MIN;
    int ok = 0;
    int present = 0;

    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        int16_t mV = cell_mV[i];

        if(!cell_map_is_present(i)) {
            if(mV != CELL_MV_NOT_READ && mV > SUPERVISOR_ABSENT_CHANNEL_MAX_mV) {
                reasons |= SUP_REASON_ABSENT_READS;
            }
            continue;
        }

        present++;
        // -1 is a channel reporting no cell, 1 a zero reading
        if(mV == CELL_MV_NOT_READ || mV <= 1) {
            reasons |= SUP_REASON_CELL_MISSING;
            continue;
        }

        if(mV < min) min = mV;
        if(mV > max) max = mV;

        if(mV > CELL_VOLTAGE_HARD_MAX_mV) {
            reasons |= SUP_REASON_CELL_HIGH;
        } else if(mV < CELL_VOLTAGE_HARD_MIN_mV) {
            reasons |= SUP_REASON_CELL_LOW;
        } else {
            ok++;
        }
    }

    if(present != NUM_CELLS) {
        // The presence mask doesn't match the cell count
        reasons |= SUP_REASON_CELL_MISSING;
    }

    *min_mV = (max >= min) ? min : 0;
    *max_mV = (max >= min) ? max : 0;
    *cells_ok = (uint8_t)ok;
    return reasons;
}

void sup_logic_init(sup_logic_t *logic) {
    memset(logic, 0, sizeof(*logic));
    logic->state = SUP_STATE_OFF;
    logic->reasons = SUP_REASON_NO_HEARTBEAT | SUP_REASON_NO_SCAN | SUP_REASON_SETTLING;
}

void sup_logic_on_status(sup_logic_t *logic, const sup_bms_status_t *status, uint32_t now) {
    if(logic->have_status) {
        bool was_lost = (now - logic->status_millis) > SUPERVISOR_HEARTBEAT_DROP_MS;
        if(status->uptime_ms < logic->status.uptime_ms || was_lost) {
            logic->bms_restarted = true;
        }
    }
    logic->status = *status;
    logic->status_millis = now;
    logic->have_status = true;
}

void sup_logic_on_scan(sup_logic_t *logic, const int16_t *cell_mV, uint32_t now) {
    logic->scan_reasons = sup_check_cells(cell_mV, &logic->cell_min_mV, &logic->cell_max_mV, &logic->cells_ok);
    logic->have_scan = true;
    logic->scan_millis = now;
    logic->scans++;

    if(logic->scan_reasons & SCAN_FAULT_REASONS) {
        logic->good_scans = 0;
        if(!logic->violating) {
            logic->violating = true;
            logic->violation_since = now;
        }
    } else {
        if(logic->good_scans < UINT8_MAX) {
            logic->good_scans++;
        }
        logic->violating = false;
    }
}

static int abs_int(int v) {
    return v < 0 ? -v : v;
}

static uint16_t blocking_reasons(const sup_logic_t *logic, uint32_t now) {
    uint16_t reasons = 0;

    if(!logic->have_status || (now - logic->status_millis) > SUPERVISOR_HEARTBEAT_ENABLE_MS) {
        reasons |= SUP_REASON_NO_HEARTBEAT;
    }
    if(logic->have_status) {
        if(logic->status.version != SUP_PROTOCOL_VERSION) {
            reasons |= SUP_REASON_VERSION;
        } else if(logic->status.map_hash != cell_map_hash()) {
            reasons |= SUP_REASON_MAP_MISMATCH;
        }
    }

    if(!logic->have_scan || (now - logic->scan_millis) > CELL_VOLTAGE_STALE_THRESHOLD_MS) {
        reasons |= SUP_REASON_NO_SCAN;
    }
    reasons |= logic->scan_reasons & SCAN_FAULT_REASONS;
    if(logic->good_scans < SUPERVISOR_GOOD_SCANS_TO_ENABLE) {
        reasons |= SUP_REASON_SETTLING;
    }

    // The BMS must see the same cells we do. Balancing skews its readings
    // relative to ours (it skips the unstable ones), so only a missing reading
    // counts then.
    if(!logic->have_status || !(logic->status.flags & SUP_BMS_FLAG_CELLS_VALID)) {
        reasons |= SUP_REASON_BMS_DISAGREE;
    } else if(logic->have_scan && !(logic->status.flags & SUP_BMS_FLAG_BALANCING) && (
        abs_int(logic->status.cell_min_mV - logic->cell_min_mV) > SUPERVISOR_BMS_AGREE_TOLERANCE_mV ||
        abs_int(logic->status.cell_max_mV - logic->cell_max_mV) > SUPERVISOR_BMS_AGREE_TOLERANCE_mV)) {
        reasons |= SUP_REASON_BMS_DISAGREE;
    }

    return reasons;
}

bool sup_logic_tick(sup_logic_t *logic, uint32_t now) {
    if(logic->bms_restarted) {
        // A restarted BMS starts with its contactors open; check everything
        // again from fresh scans, and give it a clean slate if we had latched
        logic->bms_restarted = false;
        logic->good_scans = 0;
        logic->state = SUP_STATE_OFF;
        logic->latch_reasons = 0;
    }

    uint16_t blocking = blocking_reasons(logic, now);

    switch(logic->state) {
        case SUP_STATE_OFF:
            if(blocking == 0) {
                logic->state = SUP_STATE_ON;
            }
            break;
        case SUP_STATE_ON:
            if(logic->violating && (now - logic->violation_since) >= SUPERVISOR_CELL_FAULT_DROP_MS) {
                // The BMS should long since have opened the contactors
                logic->state = SUP_STATE_LATCHED;
                logic->latch_reasons = logic->scan_reasons & SCAN_FAULT_REASONS;
            } else if((now - logic->scan_millis) > SUPERVISOR_SCAN_STALE_DROP_MS) {
                // Scans stopped. The BMS's slow mode isn't trusted to excuse
                // this (it only makes us more lenient), so the BMS doesn't use
                // it while the contactors may close. Enable again once fresh
                // scans check out.
                logic->state = SUP_STATE_OFF;
                logic->good_scans = 0;
            } else if(!logic->have_status || (now - logic->status_millis) > SUPERVISOR_HEARTBEAT_DROP_MS) {
                // BMS hung or gone. When it comes back it has restarted.
                logic->state = SUP_STATE_OFF;
            }
            break;
        case SUP_STATE_LATCHED:
            // Until the BMS restarts
            break;
    }

    // Unpopulated channels reading like cells are reported even when they
    // don't block
    logic->reasons = blocking | (logic->scan_reasons & SUP_REASON_ABSENT_READS);
    return logic->state == SUP_STATE_ON;
}

void sup_logic_reply(const sup_logic_t *logic, sup_reply_t *reply) {
    memset(reply, 0, sizeof(*reply));
    reply->version = SUP_PROTOCOL_VERSION;
    reply->state = (uint8_t)logic->state;
    reply->reasons = logic->reasons;
    reply->latch_reasons = logic->latch_reasons;
    reply->map_hash = cell_map_hash();
    reply->cell_min_mV = logic->cell_min_mV;
    reply->cell_max_mV = logic->cell_max_mV;
    reply->cells_ok = logic->cells_ok;
    reply->scans = logic->scans;
}
