#include "supervisor_logic.h"

#include "supervisor_config.h"

#include <string.h>

// Cell problems in a scan: they block enabling, and latch once they last
#define SCAN_FAULT_REASONS (SUP_REASON_CELL_HIGH | SUP_REASON_CELL_LOW | SUP_REASON_CELL_MISSING | SUP_REASON_ABSENT_READS)

// Problems with the status itself
#define STATUS_FAULT_REASONS (SUP_REASON_VERSION | SUP_REASON_MAP_INVALID | SUP_REASON_CHEMISTRY)

uint16_t sup_check_cells(const cell_map_t *map, const int16_t *cell_mV,
        int16_t *min_mV, int16_t *max_mV, uint8_t *cells_ok) {
    uint16_t reasons = 0;
    int16_t min = INT16_MAX;
    int16_t max = INT16_MIN;
    int ok = 0;
    int present = 0;

    for(int i=0; i<CELL_MAP_CHANNELS; i++) {
        int16_t mV = cell_mV[i];

        if(!cell_map_is_present(map, i)) {
            // Must read as empty wherever it was read at all, including
            // modules past the map's: a cell there would go unchecked
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

    if(present != map->num_cells) {
        // Can't happen with a map that passed cell_map_check()
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
    logic->status_version = status->version;
    if(status->version != SUP_PROTOCOL_VERSION) {
        // Nothing else in it can be read. It still counts as a heartbeat, so
        // the BMS can report the mismatch.
        logic->have_status = true;
        logic->status_millis = now;
        logic->status_reasons = SUP_REASON_VERSION;
        return;
    }

    if(logic->have_status && logic->status.version == SUP_PROTOCOL_VERSION) {
        // A BMS that booted since its last status has an uptime shorter than
        // the time since then (and in particular, shorter than before)
        uint32_t elapsed = now - logic->status_millis;
        if(status->uptime_ms < logic->status.uptime_ms || status->uptime_ms < elapsed) {
            logic->bms_restarted = true;
            // It may come back with a different map (eg. after reflashing),
            // and every check starts again from fresh scans
            logic->have_map = false;
            logic->map_changed = false;
            logic->have_scan = false;
            logic->scan_reasons = 0;
            logic->good_scans = 0;
            logic->violating = false;
        }
    }

    logic->status = *status;
    logic->status_millis = now;
    logic->have_status = true;
    logic->hmi_address = status->hmi_address;
    logic->have_hmi_address = true;

    logic->status_reasons = 0;
    if(cell_map_check(&status->map) != 0) {
        logic->status_reasons |= SUP_REASON_MAP_INVALID;
    } else if(status->map.chemistry != SUPERVISOR_CHEMISTRY) {
        logic->status_reasons |= SUP_REASON_CHEMISTRY;
    }

    if(logic->have_map) {
        if(!cell_map_equal(&status->map, &logic->map)) {
            logic->map_changed = true;
        }
    } else if(logic->status_reasons == 0) {
        // The map for this BMS boot. Scans are only checked from here on.
        logic->map = status->map;
        logic->have_map = true;
        logic->good_scans = 0;
    }
}

void sup_logic_on_scan(sup_logic_t *logic, const int16_t *cell_mV, uint16_t module_ok, uint32_t now) {
    if(!logic->have_map) {
        // Nothing to check it against yet
        return;
    }

    uint16_t mapped_modules = (uint16_t)((1U << logic->map.num_modules) - 1);
    if((module_ok & mapped_modules) != mapped_modules) {
        // A CRC error on a module the BMS reads: it will count the same read
        // as failed, so this is a missing scan rather than a missing cell
        logic->incomplete_scans++;
        return;
    }

    if(logic->have_scan && (now - logic->scan_millis) > CELL_VOLTAGE_STALE_THRESHOLD_MS) {
        // Good scans only count in a row if they are also close together
        // (not, eg., the BMS's slow mode, 41s apart)
        logic->good_scans = 0;
    }

    logic->scan_reasons = sup_check_cells(&logic->map, cell_mV, &logic->cell_min_mV, &logic->cell_max_mV, &logic->cells_ok);
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
    reasons |= logic->status_reasons;
    if(logic->map_changed) {
        reasons |= SUP_REASON_MAP_CHANGED;
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
    if(!logic->have_status || logic->status.version != SUP_PROTOCOL_VERSION ||
       !(logic->status.flags & SUP_BMS_FLAG_CELLS_VALID)) {
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
        // A restarted BMS starts with its contactors open. Give it a clean
        // slate if we had latched (the scan checks were reset on its status).
        logic->bms_restarted = false;
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
            if(logic->map_changed) {
                // The BMS isn't checking the cells it said it would. Don't
                // wait for it to open the contactors.
                logic->state = SUP_STATE_LATCHED;
                logic->latch_reasons = SUP_REASON_MAP_CHANGED;
            } else if(logic->violating && (now - logic->violation_since) >= SUPERVISOR_CELL_FAULT_DROP_MS) {
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
            } else if(blocking & STATUS_FAULT_REASONS) {
                // Its statuses stopped making sense (with a map that still
                // matches, only a protocol version change gets here)
                logic->state = SUP_STATE_OFF;
            }
            break;
        case SUP_STATE_LATCHED:
            // Until the BMS restarts
            break;
    }

    logic->reasons = blocking;
    return logic->state == SUP_STATE_ON;
}

void sup_logic_reply(const sup_logic_t *logic, sup_reply_t *reply) {
    memset(reply, 0, sizeof(*reply));
    reply->version = SUP_PROTOCOL_VERSION;
    reply->state = (uint8_t)logic->state;
    reply->reasons = logic->reasons;
    reply->latch_reasons = logic->latch_reasons;
    reply->map_hash = logic->have_map ? cell_map_hash(&logic->map) : 0;
    reply->cell_min_mV = logic->cell_min_mV;
    reply->cell_max_mV = logic->cell_max_mV;
    reply->cells_ok = logic->cells_ok;
    reply->scans = logic->scans;
    reply->chemistry = SUPERVISOR_CHEMISTRY;
}
