#include "contactors.h"
#include "config/limits.h"
#include "app/model.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "drivers/contactors/contactors.h"
#include "sys/time/time.h"

#include <stdio.h>

// TODO - as well as staleness, check for noisy readings (or use max value sometimes?)
// TODO - give up after so many failed precharges

static inline int32_t abs_int32(int32_t v) {
    return (v < 0) ? -v : v;
}


bool check_current_is_below(const bms_model_t *model, int32_t threshold_ma) {
    if(!millis_recent_enough(model->current_millis, CURRENT_STALE_THRESHOLD_MS)) {
        // stale reading
        //ret.event_type = ERR_CURRENT_STALE;
        //ret.data64 = model->current_millis;
        return false;
    }
    
    if(abs_int32(model->current_mA) > threshold_ma) {
        debug_printf("Erk, current %" PRId32 " mA is above threshold %" PRId32 " mA\n", abs_int32(model->current_mA), threshold_ma);
    }

    return abs_int32(model->current_mA) <= threshold_ma;

    // if(abs_int32(model->current_mA)<=threshold_ma) {
    //     ret.success = true;
    //     return ret;
    // }

    // ret.event_type = ERR_CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH;
    // ret.data64 = model->current_mA;
    // return ret;
}

bool check_precharge_successful(const bms_model_t *model, bool log_errors) {
    if(!check_or_confirm(
        millis_recent_enough(model->high_voltages.battery_millis, BATTERY_VOLTAGE_STALE_THRESHOLD_MS),
        log_errors,
        ERR_BATTERY_VOLTAGE_STALE,
        0x1000000000000000
    )) {
        return false;
    }

    if(!check_or_confirm(
        millis_recent_enough(model->high_voltages.output_millis, OUTPUT_VOLTAGE_STALE_THRESHOLD_MS),
        log_errors,
        ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH,
        0x2000000000000000
    )) {
        return false;
    }

    if(!check_or_confirm(
        millis_recent_enough(model->high_voltages.neg_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        log_errors,
        ERR_CONTACTOR_NEG_UNEXPECTED_OPEN,
        0x3000000000000000
    )) {
        return false;
    }

    if(!check_or_confirm(
        millis_recent_enough(model->current_millis, CURRENT_STALE_THRESHOLD_MS),
        log_errors,
        ERR_CURRENT_STALE,
        0x4000000000000000
    )) {
        return false;
    }

#if IS_BMS_BOARD(REV1)
    if(!check_or_confirm(
        millis_recent_enough(model->high_voltages.link_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        log_errors,
        ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH,
        0x5000000000000000
    )) {
        return false;
    }
#endif

    if(!check_or_confirm(
        fabsf(model->high_voltages.neg_contactor) <= (CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV * 0.001f),
        log_errors,
        ERR_CONTACTOR_NEG_UNEXPECTED_OPEN,
        (int32_t)(model->high_voltages.neg_contactor * 1000)
    )) {
        return false;
    }

    // Is current still too high?
    if(!check_or_confirm(
        fabsf(model->current_filtered_mA - model->contactor_sm.pre_close_current_mA) <= PRECHARGE_SUCCESS_MAX_MA,
        log_errors,
        ERR_CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH,
        (int32_t)(model->current_filtered_mA - model->contactor_sm.pre_close_current_mA)
    )) {
        return false;
    }

#if IS_BMS_BOARD(REV1)
    // Is the voltage across the PTC + FC negative contactor path (link vs
    // output) low enough? This is a direct measurement, so can be tighter than
    // the battery-vs-output check below.
    if(!check_or_confirm(
        fabsf(model->high_voltages.link - model->high_voltages.output) <= (PRECHARGE_SUCCESS_LINK_MAX_MV * 0.001f),
        log_errors,
        ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH,
        ((uint64_t)(uint32_t)(int32_t)(model->high_voltages.link * 1000) << 32) | (uint32_t)(int32_t)(model->high_voltages.output * 1000)
    )) {
        return false;
    }
#endif

    // Is voltage difference low enough?
    return check_or_confirm(
        fabsf(model->high_voltages.battery - model->high_voltages.output) <= (PRECHARGE_SUCCESS_MAX_MV * 0.001f),
        log_errors,
        ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH,
        ((uint64_t)(uint32_t)(int32_t)(model->high_voltages.battery * 1000) << 32) | (uint32_t)(int32_t)(model->high_voltages.output * 1000)
    );
}

bool confirm_battery_is_healthy(const bms_model_t *model) {
    (void)model;
    // Do we need this as well as the events system?

    return true;
}

bool confirm_contactor_neg_seems_closed(const bms_model_t *model) {
    if(!confirm(
        millis_recent_enough(model->high_voltages.neg_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_CONTACTOR_NEG_STUCK_OPEN,
        0x1000000000000000
    )) {
        return false;
    }

    return confirm(
        fabsf(model->high_voltages.neg_contactor) <= (CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV * 0.001f),
        ERR_CONTACTOR_NEG_STUCK_OPEN,
        (int32_t)(model->high_voltages.neg_contactor * 1000)
    );
}

bool check_contactor_neg_seems_open(const bms_model_t *model, bool log_errors) {
    if(!check_or_confirm(
        millis_recent_enough(model->high_voltages.neg_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        log_errors,
        ERR_CONTACTOR_NEG_STUCK_CLOSED,
        0x1000000000000000
    )) {
        return false;
    }

    float voltage = fabsf(model->high_voltages.neg_contactor);

    if(voltage < (CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV * 0.001f)) {
        debug_printf("Erk, neg contactor voltage %1.3f V is below open threshold %d mV\n", model->high_voltages.neg_contactor, CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV);
    }

    return check_or_confirm(
        voltage >= (CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV * 0.001f),
        log_errors,
        ERR_CONTACTOR_NEG_STUCK_CLOSED,
        (int32_t)(voltage * 1000)
    );

    // TODO - also check for voltage difference between battery and output?
}

bool confirm_contactor_neg_seems_open(const bms_model_t *model) {
    return check_contactor_neg_seems_open(model, true);
}

// Threshold for determining whether the positive contactor can be considered
// closed, using the previously stored open voltage as a reference (this works
// around noisy positive contactor readings on rev0): closed means collapsed to
// half or less of the open value, i.e. relative and immune to divider gain
// error. Capped at the (wider) pos open threshold so a large stored reference
// (e.g. the forced full-pack difference captured on the assisted path) doesn't
// cause the contactors to always read as closed.
static float pos_closed_fallback_limit(const bms_model_t *model) {
    float half_open = fabsf(model->contactor_sm.pre_close_pos_contactor_voltage) * 0.5f;
    float cap = CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV * 0.001f;
    return min(half_open, cap);
}

bool confirm_contactor_pos_seems_closed(const bms_model_t *model) {
    if(!confirm(
        millis_recent_enough(model->high_voltages.pos_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_CONTACTOR_POS_STUCK_OPEN,
        0x1000000000000000
    )) {
        return false;
    }

    float voltage = fabsf(model->high_voltages.pos_contactor);
    return confirm(
        (voltage <= (CONTACTORS_POS_CLOSED_VOLTAGE_THRESHOLD_MV * 0.001f))
        // Drift or miscalibration may result in a higher than desired voltage
        // reading, but we're satisfied as long as it is half or less of the open value.
        || (voltage <= pos_closed_fallback_limit(model)),
        ERR_CONTACTOR_POS_STUCK_OPEN,
        (int32_t)(voltage * 1000)
    );
}

bool check_contactor_pos_seems_open(const bms_model_t *model, bool log_errors) {
    if(!check_or_confirm(
        millis_recent_enough(model->high_voltages.pos_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        log_errors,
        ERR_CONTACTOR_POS_STUCK_CLOSED,
        0x1000000000000000
    )) {
        return false;
    }

    float voltage = fabsf(model->high_voltages.pos_contactor);

    if(voltage < (CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV * 0.001f)) {
        debug_printf("Erk, pos contactor voltage %1.3f V is below open threshold %d mV\n", model->high_voltages.pos_contactor, CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV);
    }

    return check_or_confirm(
        voltage >= (CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV * 0.001f),
        log_errors,
        ERR_CONTACTOR_POS_STUCK_CLOSED,
        (int32_t)(voltage * 1000)
    );

    // TODO - also check for voltage difference between battery and output?
}

bool confirm_contactor_pos_seems_open(const bms_model_t *model) {
    return check_contactor_pos_seems_open(model, true);
}

bool check_contactor_pos_and_neg_seems_open(const bms_model_t *model, bool log_errors) {
    bool ret = true;
    if(!check_contactor_pos_seems_open(model, log_errors)) ret = false;
    if(!check_contactor_neg_seems_open(model, log_errors)) ret = false;
    return ret;
}

bool confirm_contactor_pos_and_neg_seems_open(const bms_model_t *model) {
    return check_contactor_pos_and_neg_seems_open(model, true);
}

bool check_contactor_pre_bypass_seems_closed(const bms_model_t *model, bool log_errors) {
    // Use the aux contact reading
    return check_or_confirm(
        model->precharge_closed,
        log_errors,
        ERR_CONTACTOR_PRE_STUCK_OPEN,
        0
    );
}

bool confirm_contactor_pre_bypass_seems_closed(const bms_model_t *model) {
    return check_contactor_pre_bypass_seems_closed(model, true);
}

bool check_contactor_pre_bypass_seems_open(const bms_model_t *model, bool log_errors) {
    // Use the aux contact reading
    return check_or_confirm(
        !model->precharge_closed,
        log_errors,
        ERR_CONTACTOR_PRE_STUCK_CLOSED,
        0
    );
}

bool confirm_contactor_pre_bypass_seems_open(const bms_model_t *model) {
    return check_contactor_pre_bypass_seems_open(model, true);
}

bool confirm_contactors_staying_closed(const bms_model_t *model) {
    bool ret = true;

    // Check that the contactors still seem to be closed, and haven't opened due
    // to a contactor supply glitch. If they have opened we will need to go
    // through the checking/precharge sequence again.

    // Note, we will return true if the readings are stale, as we don't want to
    // immediately trigger contactor opening if there is a loop overrun. If it
    // remains stale for too long then the events system will trigger a FATAL
    // and open contactors anyway.

    // (an alternative would be to have another state such that we only consider
    // them open if another timeout passes)

    if(confirm(
        millis_recent_enough(model->high_voltages.pos_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_CONTACTOR_POS_UNEXPECTED_OPEN,
        0x1000000000000000
    )) {
        float pos_voltage = fabsf(model->high_voltages.pos_contactor);
        ret = confirm(
            (pos_voltage <= (CONTACTORS_POS_CLOSED_VOLTAGE_THRESHOLD_MV * 0.001f))
            || (pos_voltage <= pos_closed_fallback_limit(model)),
            ERR_CONTACTOR_POS_UNEXPECTED_OPEN,
            (int32_t)(pos_voltage * 1000)
        ) && ret;
    }

    if(confirm(
        millis_recent_enough(model->high_voltages.neg_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_CONTACTOR_NEG_UNEXPECTED_OPEN,
        0x2000000000000000
    )) {
        float neg_voltage = fabsf(model->high_voltages.neg_contactor);
        ret = confirm(
            neg_voltage <= (CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV * 0.001f),
            ERR_CONTACTOR_NEG_UNEXPECTED_OPEN,
            (int32_t)(neg_voltage * 1000)
        ) && ret;
    }

#if IS_BMS_BOARD(REV1)
    // With direct link sensing, the link rail should track the battery voltage
    // while both link contactors are closed. A large mismatch means one of
    // them has dropped out.
    if(confirm(
        millis_recent_enough(model->high_voltages.link_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS)
        && millis_recent_enough(model->high_voltages.battery_millis, BATTERY_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_LINK_VOLTAGE_MISMATCH,
        0x3000000000000000
    )) {
        ret = confirm(
            fabsf(model->high_voltages.battery - model->high_voltages.link) <= (CONTACTORS_LINK_MISMATCH_THRESHOLD_MV * 0.001f),
            ERR_LINK_VOLTAGE_MISMATCH,
            ((uint64_t)(model->high_voltages.battery * 1000) << 32) | (uint32_t)(model->high_voltages.link * 1000)
        ) && ret;
    }
#endif

    // TODO - check for voltage diference between battery and output?
    // or check for voltage across precharge?

    return ret;
}

#if IS_BMS_BOARD(REV1)
// Raise an event if the F4 fuse voltage drop is too high (ie, fuse blown).
static void check_fuse(bms_model_t *model) {
    contactors_sm_t *contactor_sm = &(model->contactor_sm);

    bool can_check = contactor_sm->state == CONTACTORS_STATE_CLOSED
        && state_timeout((sm_t*)contactor_sm, 2000)
        && millis_recent_enough(model->high_voltages.fuse_drop_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS);

    if(can_check) {
        confirm(
            fabsf(model->high_voltages.fuse_drop) <= (FUSE_DROP_FAULT_THRESHOLD_MV * 0.001f),
            ERR_FUSE_VOLTAGE_DROP_HIGH,
            (int32_t)(model->high_voltages.fuse_drop * 1000)
        );
    } else {
        clear_bms_event(ERR_FUSE_VOLTAGE_DROP_HIGH);
    }
}
#endif

// The precharge element sits on one side of the link (the "precharge side");
// the other side is the "other side". The close sequence is written in terms of
// these roles so it is independent of which physical pole carries the precharge
// element. The precharge side is always connected and tested first.

// Open all contactors
static void contactors_open_all(void) {
    contactors_set_pos_pre_neg(false, false, false);
}

// Close the precharge side via the precharge resistor
static void contactors_close_pre_side_via_precharge(void) {
    contactors_set_pos_pre_neg(false, true, false);
}

// Close the non-precharge side only
static void contactors_close_other_side_only(void) {
#if IS_PRECHARGE_ON_NEGATIVE()
    contactors_set_pos_pre_neg(true, false, false);
#else
    contactors_set_pos_pre_neg(false, false, true);
#endif
}

// Close both the precharge and the other side, to actually precharge
static void contactors_start_precharge(void) {
#if IS_PRECHARGE_ON_NEGATIVE()
    contactors_set_pos_pre_neg(true, true, false);
#else
    contactors_set_pos_pre_neg(false, true, true);
#endif
}

// Close the main contactors (post-precharge)
static void contactors_close(void) {
    contactors_set_pos_pre_neg(true, false, true);
}

// Close the negative contactor, with precharge in-circuit if applicable (on
// boards with precharge on the negative leg).
static void contactors_close_neg_only_via_precharge(void) {
#if IS_PRECHARGE_ON_NEGATIVE()
    contactors_set_pos_pre_neg(false, true, true);
#else
    contactors_set_pos_pre_neg(false, false, true);
#endif
}

// Close the negative contactor only, without precharge
static void contactors_close_neg_only(void) {
    contactors_set_pos_pre_neg(false, false, true);
}

// Check helpers, mapping the logical role onto the physical contactor reading
// and its POS/NEG event.
static bool confirm_pre_side_seems_closed(const bms_model_t *model) {
#if IS_PRECHARGE_ON_NEGATIVE()
    return confirm_contactor_neg_seems_closed(model);
#else
    return confirm_contactor_pos_seems_closed(model);
#endif
}

static bool confirm_other_side_seems_closed(const bms_model_t *model) {
#if IS_PRECHARGE_ON_NEGATIVE()
    return confirm_contactor_pos_seems_closed(model);
#else
    return confirm_contactor_neg_seems_closed(model);
#endif
}

static bool confirm_pre_side_seems_open(const bms_model_t *model) {
#if IS_PRECHARGE_ON_NEGATIVE()
    return confirm_contactor_neg_seems_open(model);
#else
    return confirm_contactor_pos_seems_open(model);
#endif
}

static bool confirm_other_side_seems_open(const bms_model_t *model) {
#if IS_PRECHARGE_ON_NEGATIVE()
    return confirm_contactor_pos_seems_open(model);
#else
    return confirm_contactor_neg_seems_open(model);
#endif
}

// Whether we can attempt an 'assisted entry', which is useful if we are unable
// to prove both contactors are open initially. An assisted entry closes the
// precharge first, which should shift the voltage difference to the other side,
// proving it non-welded. We will only attempt assisted entry if there's a
// significant difference between bus and link voltages.
static bool assisted_entry_allowed(const bms_model_t *model) {
    if(!millis_recent_enough(model->high_voltages.battery_millis, BATTERY_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }
    if(!millis_recent_enough(model->high_voltages.output_millis, OUTPUT_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }
    return fabsf(model->high_voltages.battery - model->high_voltages.output)
        >= (CONTACTORS_ASSISTED_MIN_DELTA_MV * 0.001f);
}

static float contactor_voltage_for_side(const bms_model_t *model, bool pre_side) {
#if IS_PRECHARGE_ON_NEGATIVE()
    return pre_side ? model->high_voltages.neg_contactor : model->high_voltages.pos_contactor;
#else
    return pre_side ? model->high_voltages.pos_contactor : model->high_voltages.neg_contactor;
#endif
}

// Stores the voltage across the positive contactor, if it is open, to be used
// as a reference when determining if the contactor has closed properly.
// Necessary for rev0 where the positive contactor readings are noisy.
static void remember_pos_open_reference(bms_model_t *model) {
    if(check_contactor_pos_seems_open(model, false)) {
        model->contactor_sm.pre_close_pos_contactor_voltage = model->high_voltages.pos_contactor;
    }
}

// Data tag for ERR_CONTACTOR_CLOSING_FAILED, kept aligned with the physical
// pole (NEG=0x3..., POS=0x5...) as before.
static uint64_t closed_failed_tag(bool pre_side) {
#if IS_PRECHARGE_ON_NEGATIVE()
    return pre_side ? 0x3000000000000000ULL : 0x5000000000000000ULL;
#else
    return pre_side ? 0x5000000000000000ULL : 0x3000000000000000ULL;
#endif
}

static uint64_t supervisor_event_data(const bms_model_t *model) {
    return (millis_recent_enough(model->supervisor_millis, SUPERVISOR_REPLY_STALE_MS) ? 0 : 0x1000000000000000ULL)
        | ((uint64_t)model->supervisor.state << 32)
        | ((uint64_t)model->supervisor.latch_reasons << 16)
        | model->supervisor.reasons;
}

void contactor_sm_tick(bms_model_t *model) {
    contactors_sm_t *contactor_sm = &(model->contactor_sm);

    // By default, disallow current flow
    contactor_sm->enable_current = false;

    if(contactor_sm->state != CONTACTORS_STATE_OPEN) {
        // A supervisor trip in any condition other than contactors open is
        // instantly fatal.
        confirm(
            supervisor_contactors_enabled(model),
            ERR_SUPERVISOR_DISABLED_CONTACTORS,
            supervisor_event_data(model)
        );
    }

    // React to open requests immediately
    if(model->contactor_req == CONTACTORS_REQUEST_OPEN || model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN) {
        switch(contactor_sm->state) {
            case CONTACTORS_STATE_PRECHARGING_INIT:
            case CONTACTORS_STATE_PRECHARGING:
            case CONTACTORS_STATE_TESTING_INITIALLY_OPEN:
            case CONTACTORS_STATE_TESTING_PRE_SIDE_CLOSED:
            case CONTACTORS_STATE_TESTING_REOPENED:
            case CONTACTORS_STATE_TESTING_OTHER_SIDE_CLOSED:
            case CONTACTORS_STATE_TESTING_ASSISTED_PRE:
            case CONTACTORS_STATE_TESTING_ASSISTED_OPEN:
            case CONTACTORS_STATE_TESTING_ASSISTED_OTHER:
            case CONTACTORS_STATE_CALIBRATING:
            case CONTACTORS_STATE_CALIBRATING_PRECHARGE_INIT:
            case CONTACTORS_STATE_CALIBRATING_PRECHARGE:
            case CONTACTORS_STATE_CALIBRATING_CLOSED:
            case CONTACTORS_STATE_CALIBRATING_ONLY_NEG_INIT:
            case CONTACTORS_STATE_CALIBRATING_ONLY_NEG:
                // Open contactors immediately
                model->contactor_req = CONTACTORS_REQUEST_NULL;
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
                break;
            case CONTACTORS_STATE_OPEN:
            case CONTACTORS_STATE_CLOSED:
            case CONTACTORS_STATE_AWAITING_OPEN:
            case CONTACTORS_STATE_PRECHARGE_FAILED:
            case CONTACTORS_STATE_TESTING_FAILED:
                // let normal processing handle it
                break;
        }
    }

    switch(contactor_sm->state) {
        case CONTACTORS_STATE_OPEN:
            contactors_open_all();

            bool closing_requested = (
                model->contactor_req == CONTACTORS_REQUEST_CLOSE ||
                model->contactor_req == CONTACTORS_REQUEST_CALIBRATE ||
                model->contactor_req == CONTACTORS_REQUEST_CALIBRATE_ONLY_NEG
            );
            // Hold off closing (leaving the request pending) until the
            // supervisor enables the contactor drivers
            bool held_by_supervisor = !confirm(
                !closing_requested || supervisor_contactors_enabled(model),
                ERR_SUPERVISOR_NOT_ENABLED,
                supervisor_event_data(model)
            );

            if(held_by_supervisor) {
                // waiting for the supervisor
            } else if(model->contactor_req == CONTACTORS_REQUEST_CLOSE) {
                model->contactor_req = CONTACTORS_REQUEST_NULL;

                if(confirm_battery_is_healthy(model)) {
                    // Start a self-test of the contactors before precharging
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_BYPASS_CLOSED);
                }
            } else if(model->contactor_req == CONTACTORS_REQUEST_OPEN || model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN) {
                // already open, just clear the request
                model->contactor_req = CONTACTORS_REQUEST_NULL;
            } else if(model->contactor_req == CONTACTORS_REQUEST_CALIBRATE) {
                model->contactor_req = CONTACTORS_REQUEST_NULL;
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING);
            } else if(model->contactor_req == CONTACTORS_REQUEST_CALIBRATE_ONLY_NEG) {
                model->contactor_req = CONTACTORS_REQUEST_NULL;
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING_ONLY_NEG_INIT);
            }
            break;
        case CONTACTORS_STATE_CLOSED:
            contactors_close();

            bool try_to_open = (
                // Requesting to open
                model->contactor_req == CONTACTORS_REQUEST_OPEN || 
                model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN ||
                // or settled but contactors seem to have opened unexpectedly
                (state_timeout((sm_t*)contactor_sm, 2000) && !confirm_contactors_staying_closed(model))
            );

            if(state_timeout((sm_t*)contactor_sm, 500) && !try_to_open) {
                // Enable current flow after a short delay
                contactor_sm->enable_current = true;
            }

            if(try_to_open) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_AWAITING_OPEN);
            } else if(model->contactor_req == CONTACTORS_REQUEST_CLOSE) {
                // cancel any close requests (we're already closed)
                model->contactor_req = CONTACTORS_REQUEST_NULL;
            }
             
            break;
        case CONTACTORS_STATE_AWAITING_OPEN:
            contactors_close();

            // Wait for current to fall before actually opening contactors.
            if((
                    check_current_is_below(model, CONTACTORS_INSTANT_OPEN_MA)
                    || (check_current_is_below(model, CONTACTORS_DELAYED_OPEN_MA) && state_timeout((sm_t*)contactor_sm, CONTACTORS_OPEN_TIMEOUT_MS))
            ) || (
                model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN && (
                    check_current_is_below(model, CONTACTORS_INSTANT_OPEN_MA)
                    || state_timeout((sm_t*)contactor_sm, CONTACTORS_FORCE_OPEN_TIMEOUT_MS)
                )
            )) {
                 model->contactor_req = CONTACTORS_REQUEST_NULL;
                 state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }
            break;
        case CONTACTORS_STATE_TESTING_BYPASS_CLOSED:
            // Test that the precharge contactor closes
            contactors_test_pre_bypass(true);
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_contactor_pre_bypass_seems_closed(model)) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_INITIALLY_OPEN);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_INITIALLY_OPEN:
            // Test that the contactors can all open
            contactors_open_all();

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(check_contactor_pos_and_neg_seems_open(model, false)) {
                    // Confirm again to clear any stale STUCK_CLOSED events.
                    confirm_contactor_pos_and_neg_seems_open(model);
                    // store the open pos contactor voltage for later comparison
                    contactor_sm->pre_close_pos_contactor_voltage = model->high_voltages.pos_contactor;
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_PRE_SIDE_CLOSED);
                } else if(assisted_entry_allowed(model)) {
                    // We couldn't confirm that both contactors are open (the
                    // bus may be floating close to one of the rails). Instead
                    // we can try an `assisted entry` by closing the precharge
                    // to shift the voltage difference to the other contactor.
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_ASSISTED_PRE);
                } else {
                    // Contactors either welded, or bus has voltage (TODO: allow hot join)
                    confirm_contactor_pos_and_neg_seems_open(model);
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        0x2000000000000000
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_PRE_SIDE_CLOSED:
            // Test that the precharge side can close
            contactors_close_pre_side_via_precharge();
            debug_printf("Precharge-side contactor voltage: %1.3f V\n",
                contactor_voltage_for_side(model, true));

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_pre_side_seems_closed(model)) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_REOPENED);
                } else {
                    // fault detected
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        closed_failed_tag(true)
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_REOPENED:
            // Finish testing by reopening the contactors
            contactors_open_all();

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_contactor_pos_and_neg_seems_open(model)) {
                    // all tests passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_OTHER_SIDE_CLOSED);
                } else {
                    // fault detected
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        0x4000000000000000
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_OTHER_SIDE_CLOSED:
            // Test that the non-precharge side can close
            contactors_close_other_side_only();
            debug_printf("Other-side contactor voltage: %1.3f V\n",
                contactor_voltage_for_side(model, false));

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_other_side_seems_closed(model)) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING_INIT);
                } else {
                    // fault detected
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        closed_failed_tag(false)
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_ASSISTED_PRE:
            // Close the precharge, hopefully shifting the voltage difference
            // across to the other side so we can confirm that non-welded.
            // If it is welded, then a voltage difference won't appear (and
            // we'll incidentally precharge the bus, but that's not a problem).
            
            // If the precharge bypass is not open, abort immediately to avoid a
            // dead short.
            if(!check_contactor_pre_bypass_seems_open(model, false)) {
                contactors_open_all();
                confirm_contactor_pre_bypass_seems_open(model);
                count_bms_event(
                    ERR_CONTACTOR_CLOSING_FAILED,
                    0x6000000000000000
                );
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                break;
            }
            contactors_close_pre_side_via_precharge();
            debug_printf("Assisted pre-side voltage: %1.3f V, other-side voltage: %1.3f V\n",
                contactor_voltage_for_side(model, true),
                contactor_voltage_for_side(model, false));

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                bool pre_closed = confirm_pre_side_seems_closed(model);
                bool other_open = confirm_other_side_seems_open(model);
                if(pre_closed && other_open) {
                    // With the precharge side closed, we've confirmed that the
                    // other side is not welded. Now open contactors before
                    // testing the opposite side.

                    // Store the voltage across the positive contactor (if applicable).
                    remember_pos_open_reference(model);
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_ASSISTED_OPEN);
                } else {
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        0x6000000000000000
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_ASSISTED_OPEN:
            // Open contactors before testing the opposite side, to avoid any
            // overlap.
            contactors_open_all();

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_contactor_pre_bypass_seems_open(model)) {
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_ASSISTED_OTHER);
                } else {
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        0x8000000000000000
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_ASSISTED_OTHER:
            // We already verified that the other side is not welded, so now we
            // can close it, which should transfer the voltage difference to the
            // precharge side, which we can detect to confirm that the precharge
            // side isn't welded.

            // If it is welded, we never see the voltage, plus we'll also
            // briefly supply current to the load, limited by the precharge.

            // Ensure precharge bypass never closes, abort immediately if it does.
            if(!check_contactor_pre_bypass_seems_open(model, false)) {
                contactors_open_all();
                confirm_contactor_pre_bypass_seems_open(model);
                count_bms_event(
                    ERR_CONTACTOR_CLOSING_FAILED,
                    0x7000000000000000
                );
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                break;
            }
            contactors_close_other_side_only();
            debug_printf("Assisted other-side voltage: %1.3f V, pre-side voltage: %1.3f V\n",
                contactor_voltage_for_side(model, false),
                contactor_voltage_for_side(model, true));

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                bool other_closed = confirm_other_side_seems_closed(model);
                bool pre_open = confirm_pre_side_seems_open(model);
                if(other_closed && pre_open) {
                    // We've now proven that the precharge side isn't welded
                    // (and previously proven the converse), so we're ready to
                    // precharge.
                    // Store the voltage across the positive contactor (if applicable).
                    remember_pos_open_reference(model);
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING_INIT);
                } else {
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        0x7000000000000000
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_FAILED:
            contactors_open_all();
            // Wait for some arbitrary time to avoid cycling too fast
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_FAILED_TEST_TIMEOUT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }
            break;

        case CONTACTORS_STATE_PRECHARGING_INIT:
            // Ensure the other side is connected directly, with the precharge
            // side still open, ready to start precharging.
            contactors_close_other_side_only();
            if(!confirm_contactor_pre_bypass_seems_open(model)) {
                // precharge contactor is stuck closed
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGE_FAILED);
            } else if(state_timeout((sm_t*)contactor_sm, 500)) {
                // Store current reading just before closing contactors, so we can account for sensor drift
                contactor_sm->pre_close_current_mA = model->current_filtered_mA;
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING);
            }
            break;
        case CONTACTORS_STATE_PRECHARGING:
            // Connect the precharge side through the precharge element, with the
            // other side already connected.
            contactors_start_precharge();

            debug_printf("PRECHARGING: %1.3f V, %.0f mA\n", 
                model->high_voltages.battery - model->high_voltages.output,
                model->current_filtered_mA - contactor_sm->pre_close_current_mA
            );


            if(model->contactor_req == CONTACTORS_REQUEST_OPEN || model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN) {
                // Abort precharge
                model->contactor_req = CONTACTORS_REQUEST_NULL;
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            } else if(state_timeout((sm_t*)contactor_sm, 1000) && check_precharge_successful(model, false)) {
                // Successful precharge
                info_printf("Precharge successful after %" PRIu32 " ms\n", state_time((sm_t*)contactor_sm));
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CLOSED);
            } else if(state_timeout((sm_t*)contactor_sm, 10000)) {
                // Failed to precharge!
                // Log the reason
                check_precharge_successful(model, true);
                count_bms_event(
                    ERR_CONTACTOR_CLOSING_FAILED,
                    0x1000000000000000
                );
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGE_FAILED);
            }
            break;

        case CONTACTORS_STATE_PRECHARGE_FAILED:
            contactors_open_all();
            // wait for the precharge to cool down
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_FAILED_PRECHARGE_TIMEOUT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }            
            break;            

        /* Calibration (requires contactors to be closed) */
        
        case CONTACTORS_STATE_CALIBRATING:
            contactors_open_all();

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING_PRECHARGE_INIT);
            }
            break;
        case CONTACTORS_STATE_CALIBRATING_PRECHARGE_INIT:
            contactors_close_other_side_only();
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING_PRECHARGE);
            }
            break;
        case CONTACTORS_STATE_CALIBRATING_PRECHARGE:
            contactors_start_precharge();
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                // There should be no real precharging as nothing should be
                // attached. We need a margin to accommodate for uncalibrated
                // values however.
                debug_printf("[11] checking current\n");
                if(check_current_is_below(model, 200)) {
                    debug_printf("[11] pass!\n");
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING_CLOSED);
                } else {
                    debug_printf("[11] fail!\n");
                    // Error properly?
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
                }
            }
            break;
        case CONTACTORS_STATE_CALIBRATING_CLOSED:
            contactors_close();

            if(!check_current_is_below(model, 200)) {
                // Current too high (output is meant to be open circuit!)
                // TODO - Error properly?
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }
            break;

        case CONTACTORS_STATE_CALIBRATING_ONLY_NEG_INIT:
            contactors_close_neg_only_via_precharge();

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                // Only proceed if the current is low and the bus is dead
                // (ensuring that we don't have a welded positive contactor and
                // a live or shorted bus).
                bool current_quiet = check_current_is_below(model, 200);
                bool bus_dead = millis_recent_enough(model->high_voltages.output_millis, OUTPUT_VOLTAGE_STALE_THRESHOLD_MS)
                    && fabsf(model->high_voltages.output) <= (CONTACTORS_CALIBRATION_DEAD_BUS_MAX_MV * 0.001f);
                if(current_quiet && bus_dead) {
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING_ONLY_NEG);
                } else {
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
                }
            }
            break;

        case CONTACTORS_STATE_CALIBRATING_ONLY_NEG:
            contactors_close_neg_only();

            if(!check_current_is_below(model, 200)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }
            break;

        default:
            // panic instead?
            error_printf("Invalid contactor state!");
            state_reset((sm_t*)contactor_sm);
            break;
    }

#if IS_BMS_BOARD(REV1)
    check_fuse(model);
#endif
}
