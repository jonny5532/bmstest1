#pragma once

// We include the relevant pack profile header, which defines pack-specific
// constants and settings (see profiles/README.md).

#ifndef BMS_PROFILE_HEADER
#error "BMS_PROFILE_HEADER is not set (configure with -DBMS_PROFILE=<name>)"
#endif
#include BMS_PROFILE_HEADER
#include "validate.h"

#define CELL_PRESENCE_MASK { CELL_PRESENCE_WORDS }


// The board: its PCB revision and any modifications (see boards/README.md),
// orthogonal to the pack profile above

#define BMS_BOARD_REV0 1
#define BMS_BOARD_REV1 2

#ifndef BMS_BOARD_HEADER
#error "BMS_BOARD_HEADER is not set (configure with -DBMS_BOARD=<name>)"
#endif
#include BMS_BOARD_HEADER

// Check the revision with #if IS_BMS_BOARD(REV1), rather than comparing
// BMS_BOARD_REVISION directly. If this header hasn't been included, an
// undefined function-like macro is a compile error, whereas comparing would
// quietly compare two undefined (zero) values and pick the wrong board.
#define IS_BMS_BOARD(rev) (BMS_BOARD_REVISION == BMS_BOARD_##rev)
#if !IS_BMS_BOARD(REV0) && !IS_BMS_BOARD(REV1)
#error "The board must set BMS_BOARD_REVISION to BMS_BOARD_REV0 or BMS_BOARD_REV1"
#endif

// USB_VBUS_SENSE: USB VBUS reaches GPIO3 (J12 pin 2) through a 3.3k/5.1k
// divider, and the chip's D+ pull-up is only switched on while it is present.
// USB_EXTERNAL_PULLUP: an external D+ pull-up powered from VBUS, and the
// chip's own one stays off. Either way a switched-off host is never
// back-powered through D+. Neither can be detected, and either one set on a
// board without it means USB never connects. Check with #if IS_USB_VBUS_SENSE()
// and #if IS_USB_EXTERNAL_PULLUP().
#if !defined(USB_VBUS_SENSE) || !defined(USB_EXTERNAL_PULLUP)
#error "The board must define USB_VBUS_SENSE and USB_EXTERNAL_PULLUP (include a revision, e.g. builtin/rev1.h)"
#endif
#if (USB_VBUS_SENSE != 0 && USB_VBUS_SENSE != 1) || (USB_EXTERNAL_PULLUP != 0 && USB_EXTERNAL_PULLUP != 1)
#error "USB_VBUS_SENSE and USB_EXTERNAL_PULLUP must be 0 or 1"
#endif
#if USB_VBUS_SENSE && USB_EXTERNAL_PULLUP
#error "USB_VBUS_SENSE and USB_EXTERNAL_PULLUP are alternatives; a board has one or neither"
#endif
#define IS_USB_VBUS_SENSE() (USB_VBUS_SENSE)
#define IS_USB_EXTERNAL_PULLUP() (USB_EXTERNAL_PULLUP)

// Which pole carries the precharge element. Negative (via the FC negative
// contactor) is the standard; a pack wired with it on the positive sets
// PRECHARGE_ON_NEGATIVE to 0 in its profile. Check it with
// #if IS_PRECHARGE_ON_NEGATIVE(), which, like IS_BMS_BOARD, fails to compile
// if this header hasn't been included.
#ifndef PRECHARGE_ON_NEGATIVE
#define PRECHARGE_ON_NEGATIVE 1
#endif
#if PRECHARGE_ON_NEGATIVE != 0 && PRECHARGE_ON_NEGATIVE != 1
#error "PRECHARGE_ON_NEGATIVE must be 0 or 1"
#endif
#define IS_PRECHARGE_ON_NEGATIVE() (PRECHARGE_ON_NEGATIVE)

// Isolation monitor (app/isolation.h)
#define ISOLATION_DIVIDER_BOTTOM_OHMS 24000.0f // bottom resistor of each HV divider
#define ISOLATION_RESISTANCE_CAP_OHMS 100e6f   // report at most 100 Mohm
#define ISOLATION_MIN_BATTERY_V 20.0f          // below this the channels barely separate
#define ISOLATION_SETTLE_MS 2000               // contactors open this long before a reading counts
// ISOLATION_LOW below this many ohms per volt of pack voltage, with the
// contactors open (500 ohm/V is the usual minimum for AC-connected systems)
#define ISOLATION_LOW_OHMS_PER_V 500.0f


// A deliberate overestimate of internal resistance of an individual cell, for working current limits
#define WORKING_LIMIT_INTERNAL_RESISTANCE_uR 50


// Options: 
// - we could load in the cellvoltages as they come in, and use the presence mask for decoding
// - or use the presence mask to load in the cellvoltages (and modify the balance mask)
// i guess we check them more often (every model tick) vs BMB comms every 1s... so second option


#define BATTERY_VOLTAGE_HARD_MAX_mV (NUM_CELLS * CELL_VOLTAGE_HARD_MAX_mV)
#define BATTERY_VOLTAGE_HARD_MIN_mV (NUM_CELLS * CELL_VOLTAGE_HARD_MIN_mV)

// Max discrepancy between BMB cell voltages and measured terminal voltage
// (which should be calibrated away at zero current). If in slow mode there
// could be a considerable time delay between cellvoltage and pack samples.
#define VOLTAGE_MISMATCH_THRESHOLD_mV 20000

// Current limits to apply in the soft-limit region
#define OVERCHARGE_DISCHARGE_CURRENT_LIMIT_dA 50 // in 0.1A units
#define OVERDISCHARGE_CHARGE_CURRENT_LIMIT_dA 50 // in 0.1A units

// How much over the normal-region current limits we consider excessive.
#define CURRENT_LIMIT_ERROR_MARGIN_dA 10 // in 0.1A units

// How much excess normal-region charge/discharge (beyond current limits) we allow before
// cutting off the battery to protect it.
#define OVERCURRENT_BUFFER_LIMIT_dC 1000 // in 0.1Coulomb units

// How much excess charge/discharge we allow in the soft-limit region before
// cutting off the battery to protect it.

// We dont't tolerate much overcharge (1A for 200 seconds)
#define OVERCHARGE_BUFFER_LIMIT_dC 2000 // in 0.1Coulomb units

// 5000 dC (50C) is 200mA (quiescent current for a big inverter) for ~40 minutes,
// which should be long enough to get the inverter [dis]charging.
#define OVERDISCHARGE_BUFFER_LIMIT_dC 5000 // in 0.1Coulomb units










// How recent readings must be to be considered valid.

// Battery voltage samples every ~100ms
#define BATTERY_VOLTAGE_STALE_THRESHOLD_MS 1000
#define OUTPUT_VOLTAGE_STALE_THRESHOLD_MS 1000
#define CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS 1000

// The current samples every ~531ms, so this should be sufficient. Precharge
// must wait this long to ensure it isn't using pre-precharge readings.
#define CURRENT_STALE_THRESHOLD_MS 800

// Current sampling is briefly paused (~1.2s worst case between samples) every
// ~53s while the INA228 takes a shunt temperature measurement, so fault/
// readiness checks must tolerate a longer gap. The contactor logic keeps the
// tight threshold above; it never sees the pause because temperature
// measurements are deferred unless the contactors are settled.
#define CURRENT_STALE_FAULT_THRESHOLD_MS 1500

#define TEMPERATURE_STALE_THRESHOLD_MS(model) (  \
    (model)->cell_voltage_slow_mode ?              \
        CELL_TEMPERATURE_STALE_THRESHOLD_SLOW_MS  \
        : CELL_TEMPERATURE_STALE_THRESHOLD_MS )

// We only sample every 1.28s, and could have isospi/CRC issues, so be generous
// (also allow long enough for stable readings during balancing)
#define CELL_VOLTAGE_STALE_THRESHOLD_MS 15000
// In slow mode the BMBs are sampled every BMB3Y_SLOW_MODE_PERIOD_MS (at least:
// main loop overruns stretch it). Must be a power-of-two number of timesteps.
#define BMB3Y_SLOW_MODE_PERIOD_MS 40960
// Allow two bad slow-mode reads, plus up to one normal cycle on entering slow
// mode (3 x 40.96s + 1.28s = 124.2s), with some margin
#define CELL_VOLTAGE_STALE_THRESHOLD_SLOW_MS 135000
#define CELL_TEMPERATURE_STALE_THRESHOLD_MS 5000
#define CELL_TEMPERATURE_STALE_THRESHOLD_SLOW_MS 135000
_Static_assert(CELL_VOLTAGE_STALE_THRESHOLD_SLOW_MS > 3 * BMB3Y_SLOW_MODE_PERIOD_MS + 1280 &&
               CELL_TEMPERATURE_STALE_THRESHOLD_SLOW_MS > 3 * BMB3Y_SLOW_MODE_PERIOD_MS + 1280,
               "slow-mode stale thresholds must allow two bad reads");

// How long a cell may stay below the soft minimum before the BMS faults and
// opens the contactors. Long enough for a manual recovery charge, short enough
// that a forgotten flat pack stops discharging (BMB drain then falls to slow
// mode) before its cells get too low to recover.
#define CELL_VOLTAGE_LOW_ESCALATION_MS 3600000
// Event leeways are counted in 16-bit deciseconds
_Static_assert(CELL_VOLTAGE_LOW_ESCALATION_MS / 100 <= 0xFFFF, "CELL_VOLTAGE_LOW_ESCALATION_MS too long");

// Threshold for detecting a cell voltage glitch (sudden jump in one cycle)
#define CELL_VOLTAGE_GLITCH_THRESHOLD_mV 100

// Threshold for detecting a module temperature glitch (sudden jump in one cycle)
#define MODULE_TEMPERATURE_GLITCH_THRESHOLD_dC 100

// The 3V3 measurement is really just measuring the divider resistor tolerances...
#define SUPPLY_VOLTAGE_3V3_MIN_MV 3200
#define SUPPLY_VOLTAGE_3V3_MAX_MV 3600
#define SUPPLY_VOLTAGE_5V_MIN_MV 4400
#define SUPPLY_VOLTAGE_5V_MAX_MV 5600
#define SUPPLY_VOLTAGE_12V_MIN_MV 11000
#define SUPPLY_VOLTAGE_12V_MAX_MV 16000
#define SUPPLY_VOLTAGE_CONTACTOR_HARD_MIN_MV 10000 // Shut down if below this (risk of contactors opening)
#define SUPPLY_VOLTAGE_CONTACTOR_SOFT_MIN_MV 12000
#define SUPPLY_VOLTAGE_CONTACTOR_MAX_MV 16000
#define SUPPLY_VOLTAGE_STALE_THRESHOLD_MS 2000


// CONTACTOR BEHAVIOUR THRESHOLDS

// Contactor weld test thresholds

// How long to wait for voltage to settle during contactor tests
#define CONTACTORS_TEST_WAIT_MS 1000
// How long to wait after a failed precharge (for the PTCs to cool down)
#define CONTACTORS_FAILED_PRECHARGE_TIMEOUT_MS 20000
// How long to wait after a failed contactor test (to avoid rapid cycling)
#define CONTACTORS_FAILED_TEST_TIMEOUT_MS 20000
// We need at least this much voltage difference between the battery and output
// voltages before we allow 'assisted' weld checking.
#define CONTACTORS_ASSISTED_MIN_DELTA_MV 20000
// Max voltage across a contactor to consider it closed
#define CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV 2000
// Min voltage across a contactor to consider it open
#define CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV 3000 // was 5000
#if IS_BMS_BOARD(REV1)
// Rev1 measures directly across the pos (Link Positive) contactor, so the
// tight generic thresholds apply
#define CONTACTORS_POS_CLOSED_VOLTAGE_THRESHOLD_MV 2000
#define CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV 3000
// Max discrepancy between battery and link voltage while closed (measured by
// two different ADCs which may be independently [un]calibrated)
#define CONTACTORS_LINK_MISMATCH_THRESHOLD_MV 16000
// Max drop across the F4 fuse/jumper while the contactors are closed. Exceeding
// it for FUSE_DROP_FAULT_DELAY_MS means it has blown, and faults the BMS
#define FUSE_DROP_FAULT_THRESHOLD_MV 10000
#else
// Pos contactor has wider tolerances due to the way it is measured
#define CONTACTORS_POS_CLOSED_VOLTAGE_THRESHOLD_MV 5000
#define CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV 10000
#endif
// Max bus voltage allowed before we abort offline calibration.
// During offline calibration, we need to close the negative contactor. We do
// this cautiously, with precharge first (if applicable), just in case the other
// contactor is welded. We abort if either a significant current is flowing, or
// if the voltage across the output is above this threshold (indicating a live
// bus). Needs to be wide to accommodate uncalibrated voltage readings.
#define CONTACTORS_CALIBRATION_DEAD_BUS_MAX_MV 50000
// How long the F4 fuse drop must stay high before the BMS faults (the
// escalation time of ERR_FUSE_VOLTAGE_DROP_HIGH). It must be longer than the
// time a reading counts as fresh, so that a single high reading followed by a
// stalled ADC can't fault the BMS before the reading goes stale.
#define FUSE_DROP_FAULT_DELAY_MS 2000
_Static_assert(FUSE_DROP_FAULT_DELAY_MS > CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS,
    "the fuse fault delay must be longer than the contactor voltage staleness limit");


// Precharge thresholds

// It is difficult to get a good reading across the positive contactor due to
// the ADC arrangement, so we have to tolerate a wider precharge voltage
// differential than we'd like.

// Max steady-state PTC current (TDK C1451 in 2S3P) will be at 450mA at 16V.

// Thus we can close with a current < 225mA and voltage < 16V, since hot PTCs
// would result in at least 25V or higher.

// Largest allowable pack current to consider precharge successful
#define PRECHARGE_SUCCESS_MAX_MA 225
// Maximum voltage difference to consider precharge successful (allowing for uncalibrated contactors)
#define PRECHARGE_SUCCESS_MAX_MV 16000
#if IS_BMS_BOARD(REV1)
// Rev1 also senses the link rail directly, so the voltage across the PTC + FC
// negative contactor path (link vs output) can be held to a tighter limit
#define PRECHARGE_SUCCESS_LINK_MAX_MV 8000
#endif


// Contactor opening thresholds

// Current below which we can instantly open contactors
#define CONTACTORS_INSTANT_OPEN_MA 1000
// Current below which we will begrudgingly open contactors after the wait is up
#define CONTACTORS_DELAYED_OPEN_MA 5000
// How long we wait for the current to fall before failing to open contactors.
// NOTE: Some inverters (Deye!) take a long time to react to current changes (20
// seconds!).
#define CONTACTORS_OPEN_TIMEOUT_MS 30000
// How long we wait when force-opening contactors
#define CONTACTORS_FORCE_OPEN_TIMEOUT_MS 2000
