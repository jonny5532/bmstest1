#pragma once

// Decides whether the supervisor enables the contactor drivers (CTR_EN).
//
// The cell map comes from the BMS, in every status message. The supervisor
// takes the first one that passes cell_map_check() and matches its own
// chemistry after each BMS boot, and holds the BMS to it until the BMS boots
// again: a different map later on (which can only be a BMS fault) latches.
//
// Enabling needs, all at once:
//  - a recent status from a BMS speaking our protocol version, with a usable
//    cell map for our chemistry that hasn't changed since the BMS booted;
//  - the last SUPERVISOR_GOOD_SCANS_TO_ENABLE snooped scans, no more than
//    CELL_VOLTAGE_STALE_THRESHOLD_MS apart, complete (every
//    module the map covers with good CRCs), recent, every present cell read
//    and within our hard limits, and every channel the map leaves out reading
//    as empty (below SUPERVISOR_ABSENT_CHANNEL_MAX_mV), on any module that
//    answered, inside the map or not;
//  - the BMS's own cell min/max agreeing with the snooped ones.
//
// Once enabled, the drivers are cut:
//  - when the BMS stops sending status (hung, flashing, powered off), or
//    restarts, or the scans stop, and re-enabled once the checks pass again;
//  - when a cell problem above (present cell out of range or unread, absent
//    channel reading like a cell) lasts twice as long as the BMS takes to open
//    the contactors itself, or the BMS's map changes. These latch until the
//    BMS restarts.

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
    sup_bms_status_t status;   // last status in our protocol version
    uint8_t status_version;    // protocol version of the last status, ours or not
    uint16_t status_reasons;   // VERSION, MAP_INVALID, CHEMISTRY from the last status
    bool bms_restarted;
    // The BMS's HMI address, kept when the BMS stops (supervisor_hmi.h)
    bool have_hmi_address;
    uint8_t hmi_address;

    // The cell map in force since the BMS booted
    bool have_map;
    cell_map_t map;
    bool map_changed;          // the BMS sent a different one since

    // Snooped scans
    bool have_scan;
    uint32_t scan_millis;
    uint16_t scan_reasons;  // cell problems in the last scan
    uint8_t good_scans;     // consecutive scans without cell problems
    int16_t cell_min_mV;
    int16_t cell_max_mV;
    uint8_t cells_ok;
    uint32_t scans;            // complete scans checked against the map
    uint32_t incomplete_scans; // scans a mapped module failed its CRC in
    bool violating;         // a cell problem in the scans since violation_since
    uint32_t violation_since;
} sup_logic_t;

void sup_logic_init(sup_logic_t *logic);

// A status message arrived from the BMS
void sup_logic_on_status(sup_logic_t *logic, const sup_bms_status_t *status, uint32_t now);

// A scan was snooped: one reading per physical channel (CELL_MV_NOT_READ on
// modules that failed their CRC), and the modules that read cleanly
void sup_logic_on_scan(sup_logic_t *logic, const int16_t *cell_mV, uint16_t module_ok, uint32_t now);

// Update the state. Returns whether the contactor drivers should be enabled.
bool sup_logic_tick(sup_logic_t *logic, uint32_t now);

// Fill a reply to the BMS from the current state
void sup_logic_reply(const sup_logic_t *logic, sup_reply_t *reply);

// Check one complete scan against a cell map and our hard limits. Returns
// SUP_REASON_* bits, and the min/max of the present cells and how many are in
// range.
uint16_t sup_check_cells(const cell_map_t *map, const int16_t *cell_mV,
    int16_t *min_mV, int16_t *max_mV, uint8_t *cells_ok);
