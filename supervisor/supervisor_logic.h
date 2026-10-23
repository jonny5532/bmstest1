#pragma once

// Decides whether the supervisor enables the contactor drivers (CTR_EN).
//
// Enabling needs, all at once:
//  - a recent status from a BMS speaking our protocol version and built for the
//    same pack (cell map hash);
//  - the last SUPERVISOR_GOOD_SCANS_TO_ENABLE snooped scans complete, recent,
//    and with every present cell read and within the hard limits;
//  - the BMS's own cell min/max agreeing with the snooped ones.
//
// Once enabled, the drivers are cut:
//  - when the BMS stops sending status (hung, flashing, powered off) or
//    restarts, or the scans stop, and re-enabled once the checks pass again;
//  - when a present cell stays out of range or unread for twice as long as
//    the BMS takes to open the contactors itself. This latches until the BMS
//    restarts.

#include "shared/cell_map.h"
#include "shared/supervisor_protocol.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    sup_state_t state;
    uint16_t reasons;       // what blocks enabling, as of the last tick
    uint16_t latch_reasons; // why it latched

    // BMS heartbeat
    bool have_status;
    uint32_t status_millis;
    sup_bms_status_t status;
    bool bms_restarted;

    // Snooped scans
    bool have_scan;
    uint32_t scan_millis;
    uint16_t scan_reasons;  // cell problems in the last scan
    uint8_t good_scans;     // consecutive scans without cell problems
    int16_t cell_min_mV;
    int16_t cell_max_mV;
    uint8_t cells_ok;
    uint32_t scans;
    bool violating;         // present cells out of range or unread
    uint32_t violation_since;
} sup_logic_t;

void sup_logic_init(sup_logic_t *logic);

// A status message arrived from the BMS
void sup_logic_on_status(sup_logic_t *logic, const sup_bms_status_t *status, uint32_t now);

// A complete scan was snooped (one reading per physical channel)
void sup_logic_on_scan(sup_logic_t *logic, const int16_t *cell_mV, uint32_t now);

// Update the state. Returns whether the contactor drivers should be enabled.
bool sup_logic_tick(sup_logic_t *logic, uint32_t now);

// Fill a reply to the BMS from the current state
void sup_logic_reply(const sup_logic_t *logic, sup_reply_t *reply);

// Check one scan against the cell map and hard limits. Returns SUP_REASON_*
// bits, and the min/max of the present cells and how many are in range.
uint16_t sup_check_cells(const int16_t *cell_mV, int16_t *min_mV, int16_t *max_mV, uint8_t *cells_ok);
