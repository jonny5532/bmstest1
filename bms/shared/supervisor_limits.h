#pragma once

// Timing shared by the BMS and the supervisor (supervisor/). The supervisor
// waits longer than the BMS before acting on the same problem, so that the BMS
// always gets to open the contactors itself first. Its delays are derived from
// the BMS's own, which live here so both builds see the same values.
//
// The supervisor is general-purpose apart from the chemistry, it gets
// everything else at runtime from the BMS.

// BMS

// We only sample every 1.28s, and could have isospi/CRC issues, so be generous
// (also allow long enough for stable readings during balancing)
#define CELL_VOLTAGE_STALE_THRESHOLD_MS 15000
// How long we wait when force-opening contactors
#define CONTACTORS_FORCE_OPEN_TIMEOUT_MS 2000
// Escalation times of the BMS cell voltage events (see sys/events/events.h).
// The cells are read every 1.28s, so one bad read stays raised for at least
// that long: allow two bad reads in a row, with some margin.
#define CELL_VOLTAGE_FAULT_ESCALATION_MS 4000
#define CELL_VOLTAGES_STALE_ESCALATION_MS 1000
// The BMS watchdog reboots it (which opens the contactors) after this long
#define BMS_WATCHDOG_TIMEOUT_MS 5000

// SUPERVISOR

// Before cutting the contactor drivers over a cell fault, the supervisor waits
// twice as long as the BMS takes to open the contactors itself (fault
// escalation plus the force-open wait for the current to fall).
#define SUPERVISOR_CELL_FAULT_DROP_MS \
    (2 * (CELL_VOLTAGE_FAULT_ESCALATION_MS + CONTACTORS_FORCE_OPEN_TIMEOUT_MS))
// Likewise when the snooped scans stop. The BMS's slow mode isn't allowed to
// stretch this (the BMS doesn't use slow mode while the contactors may close).
#define SUPERVISOR_SCAN_STALE_DROP_MS \
    (2 * (CELL_VOLTAGE_STALE_THRESHOLD_MS + CELL_VOLTAGES_STALE_ESCALATION_MS + CONTACTORS_FORCE_OPEN_TIMEOUT_MS))
// Without a BMS status for this long, the BMS is taken to be hung or gone
#define SUPERVISOR_HEARTBEAT_DROP_MS (2 * BMS_WATCHDOG_TIMEOUT_MS)

// Enabling needs a BMS status at least this recent...
#define SUPERVISOR_HEARTBEAT_ENABLE_MS 1000
// ...and this many complete, in-range scans in a row
#define SUPERVISOR_GOOD_SCANS_TO_ENABLE 3
// How closely the BMS's cell min/max must match the snooped ones to enable
#define SUPERVISOR_BMS_AGREE_TOLERANCE_mV 100

// Minimum legitimate cell voltage, above which a cell counts as present (and
// must also appear in the presence mask).
#define SUPERVISOR_ABSENT_CHANNEL_MAX_mV 10

// The BMS treats the contactor drivers as disabled without a supervisor reply
// this recent
#define SUPERVISOR_REPLY_STALE_MS 2000
