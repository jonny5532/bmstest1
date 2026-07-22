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
        debug_printf("Erk, current %d mA is above threshold %d mA\n", abs_int32(model->current_mA), threshold_ma);
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

#if BMS_BOARD == BMS_BOARD_REV1
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

#if BMS_BOARD == BMS_BOARD_REV1
    // Is the voltage across the PTC + FC negative contactor path (link vs
    // output) low enough? This is a direct measurement, so can be tighter than
    // the battery-vs-output check below.
    if(!check_or_confirm(
        fabsf(model->high_voltages.link - model->high_voltages.output) <= (PRECHARGE_SUCCESS_LINK_MAX_MV * 0.001f),
        log_errors,
        ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH,
        ((uint64_t)(model->high_voltages.link * 1000) << 32) | (uint32_t)(model->high_voltages.output * 1000)
    )) {
        return false;
    }
#endif

    // Is voltage difference low enough?
    return check_or_confirm(
        fabsf(model->high_voltages.battery - model->high_voltages.output) <= (PRECHARGE_SUCCESS_MAX_MV * 0.001f),
        log_errors,
        ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH,
        ((uint64_t)(model->high_voltages.battery * 1000) << 32) | (uint32_t)(model->high_voltages.output * 1000)
    );
}

bool confirm_battery_is_healthy(const bms_model_t *model) {
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

bool confirm_contactor_neg_seems_open(const bms_model_t *model) {
    if(!confirm(
        millis_recent_enough(model->high_voltages.neg_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_CONTACTOR_NEG_STUCK_CLOSED,
        0x1000000000000000
    )) {
        return false;
    }

    float voltage = fabsf(model->high_voltages.neg_contactor);

    if(voltage < (CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV * 0.001f)) {
        debug_printf("Erk, neg contactor voltage %1.3f V is below open threshold %d mV\n", model->high_voltages.neg_contactor, CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV);
    }

    return confirm(
        voltage >= (CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV * 0.001f),
        ERR_CONTACTOR_NEG_STUCK_CLOSED,
        (int32_t)(voltage * 1000)
    );

    // TODO - also check for voltage difference between battery and output?
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
        || (voltage <= (model->contactor_sm.pre_close_pos_contactor_voltage * 0.5f)),
        ERR_CONTACTOR_POS_STUCK_OPEN,
        (int32_t)(voltage * 1000)
    );
}

bool confirm_contactor_pos_seems_open(const bms_model_t *model) {
    if(!confirm(
        millis_recent_enough(model->high_voltages.pos_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS),
        ERR_CONTACTOR_POS_STUCK_CLOSED,
        0x1000000000000000
    )) {
        return false;
    }

    float voltage = fabsf(model->high_voltages.pos_contactor);

    if(voltage < (CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV * 0.001f)) {
        debug_printf("Erk, pos contactor voltage %1.3f V is below open threshold %d mV\n", model->high_voltages.pos_contactor, CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV);
    }

    return confirm(
        voltage >= (CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV * 0.001f),
        ERR_CONTACTOR_POS_STUCK_CLOSED,
        (int32_t)(voltage * 1000)
    );

    // TODO - also check for voltage difference between battery and output?
}

bool confirm_contactor_pos_and_neg_seems_open(const bms_model_t *model) {
    bool ret = true;
    if(!confirm_contactor_pos_seems_open(model)) ret = false;
    if(!confirm_contactor_neg_seems_open(model)) ret = false;
    return ret;
}

bool confirm_contactor_pre_seems_closed(const bms_model_t *model) {
    // Use the aux contact reading
    return confirm(
        model->precharge_closed,
        ERR_CONTACTOR_PRE_STUCK_OPEN,
        0
    );
}

bool confirm_contactor_pre_seems_open(const bms_model_t *model) {
    // Use the aux contact reading
    return confirm(
        !model->precharge_closed,
        ERR_CONTACTOR_PRE_STUCK_CLOSED,
        0
    );
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
            || (pos_voltage <= (model->contactor_sm.pre_close_pos_contactor_voltage * 0.5f)),
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

#if BMS_BOARD == BMS_BOARD_REV1
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

void contactor_sm_tick(bms_model_t *model) {
    contactors_sm_t *contactor_sm = &(model->contactor_sm);

    // By default, disallow current flow
    contactor_sm->enable_current = false;

    // React to open requests immediately
    if(model->contactor_req == CONTACTORS_REQUEST_OPEN || model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN) {
        switch(contactor_sm->state) {
            case CONTACTORS_STATE_PRECHARGING_INIT:
            case CONTACTORS_STATE_PRECHARGING:
            case CONTACTORS_STATE_TESTING_NEG_OPEN:
            case CONTACTORS_STATE_TESTING_NEG_CLOSED:
            case CONTACTORS_STATE_TESTING_POS_OPEN:
            case CONTACTORS_STATE_TESTING_POS_CLOSED:
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
            contactors_set_pos_pre_neg(false, false, false);
            if(model->contactor_req == CONTACTORS_REQUEST_CLOSE) {
                model->contactor_req = CONTACTORS_REQUEST_NULL;

                if(confirm_battery_is_healthy(model)) {
                    // Start a self-test of the contactors before precharging
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_PRE_CLOSED);
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
            contactors_set_pos_pre_neg(true, false, true);

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

#if BMS_BOARD == BMS_BOARD_REV1
            // Diagnostic only: with contactors closed and current flowing, a
            // sustained voltage drop across the F4 fuse/jumper suggests it has
            // blown or the drive-unit connection is open.
            if(state_timeout((sm_t*)contactor_sm, 2000)
                && millis_recent_enough(model->high_voltages.fuse_drop_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS)
                && millis_recent_enough(model->current_millis, CURRENT_STALE_THRESHOLD_MS)
                && abs_int32(model->current_mA) > CONTACTORS_INSTANT_OPEN_MA) {
                confirm(
                    fabsf(model->high_voltages.fuse_drop) <= (FUSE_DROP_WARNING_THRESHOLD_MV * 0.001f),
                    ERR_FUSE_VOLTAGE_DROP_HIGH,
                    (int32_t)(model->high_voltages.fuse_drop * 1000)
                );
            }
#endif

            if(try_to_open) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_AWAITING_OPEN);
            } else if(model->contactor_req == CONTACTORS_REQUEST_CLOSE) {
                // cancel any close requests (we're already closed)
                model->contactor_req = CONTACTORS_REQUEST_NULL;
            }
             
            break;
        case CONTACTORS_STATE_AWAITING_OPEN:
            contactors_set_pos_pre_neg(true, false, true);

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
        case CONTACTORS_STATE_TESTING_PRE_CLOSED:
            contactors_test_pre(true);
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_contactor_pre_seems_closed(model)) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_NEG_OPEN);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_NEG_OPEN:
            contactors_set_pos_pre_neg(false, false, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_contactor_pos_and_neg_seems_open(model)) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_NEG_CLOSED);
                } else {
                    // fault detected
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        0x2000000000000000
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_NEG_CLOSED:
#ifdef PRECHARGE_ON_NEGATIVE
            // Both megatove and precharge (since this actually leaves the
            // precharge open due to the inverted logic - we don't want to close
            // more than one contactor per state due to current draw)
            contactors_set_pos_pre_neg(false, true, true);
#else
            contactors_set_pos_pre_neg(false, false, true);
#endif

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_contactor_neg_seems_closed(model)) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_POS_OPEN);
                } else {
                    // fault detected
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        0x3000000000000000
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_POS_OPEN:
            contactors_set_pos_pre_neg(false, false, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_contactor_pos_and_neg_seems_open(model)) {
                    // all tests passed
                    // store pre-close voltage reading for later comparison
                    contactor_sm->pre_close_pos_contactor_voltage = model->high_voltages.pos_contactor;
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_POS_CLOSED);
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
        case CONTACTORS_STATE_TESTING_POS_CLOSED:
#ifdef PRECHARGE_ON_NEGATIVE
            contactors_set_pos_pre_neg(true, false, false);
#else
            // Both positive and precharge (since this actually leaves the
            // precharge open due to the inverted logic - we don't want to close
            // more than one contactor per state due to current draw)
            contactors_set_pos_pre_neg(true, true, false);
#endif

            float voltage = fabsf(model->high_voltages.pos_contactor);
            debug_printf("Pos contactor voltage: %1.3f V\n", voltage);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(confirm_contactor_pos_seems_closed(model)) {
                    // all tests passed, go to precharging
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING_INIT);
                } else {
                    // fault detected
                    count_bms_event(
                        ERR_CONTACTOR_CLOSING_FAILED,
                        0x5000000000000000
                    );
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_FAILED:
            contactors_set_pos_pre_neg(false, false, false);
            // Wait for some arbitrary time to avoid cycling too fast
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_FAILED_TEST_TIMEOUT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }
            break;

        case CONTACTORS_STATE_PRECHARGING_INIT:
#ifdef PRECHARGE_ON_NEGATIVE
            // Close positive contactor
            contactors_set_pos_pre_neg(true, false, false);
#else
            // Close negative contactor
            contactors_set_pos_pre_neg(false, false, true);
#endif
            if(!confirm_contactor_pre_seems_open(model)) {
                // precharge contactor is stuck closed
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGE_FAILED);
            } else if(state_timeout((sm_t*)contactor_sm, 500)) {
                // Store current reading just before closing contactors, so we can account for sensor drift
                contactor_sm->pre_close_current_mA = model->current_filtered_mA;
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING);
            }
            break;
        case CONTACTORS_STATE_PRECHARGING:
#ifdef PRECHARGE_ON_NEGATIVE
            // Now close precharge contactor (actually just the Bat- one)
            contactors_set_pos_pre_neg(true, true, false);
#else
            // Now close precharge contactor (actually just the Bat+ one)
            contactors_set_pos_pre_neg(false, true, true);
#endif

            debug_printf("PRECHARGING: %1.3f V, %d mA\n", 
                model->high_voltages.battery - model->high_voltages.output,
                model->current_filtered_mA - contactor_sm->pre_close_current_mA
            );


            if(model->contactor_req == CONTACTORS_REQUEST_OPEN || model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN) {
                // Abort precharge
                model->contactor_req = CONTACTORS_REQUEST_NULL;
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            } else if(state_timeout((sm_t*)contactor_sm, 1000) && check_precharge_successful(model, false)) {
                // Successful precharge
                info_printf("Precharge successful after %u ms\n", state_time((sm_t*)contactor_sm));
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
            contactors_set_pos_pre_neg(false, false, false);
            // wait for the precharge to cool down
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_FAILED_PRECHARGE_TIMEOUT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }            
            break;            

        /* Calibration (requires contactors to be closed) */
        
        case CONTACTORS_STATE_CALIBRATING:
            contactors_set_pos_pre_neg(false, false, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING_PRECHARGE_INIT);
            }
            break;
        case CONTACTORS_STATE_CALIBRATING_PRECHARGE_INIT:
#ifdef PRECHARGE_ON_NEGATIVE
            // Close positive contactor first
            contactors_set_pos_pre_neg(true, false, false);
#else
            // Close negative contactor first
            contactors_set_pos_pre_neg(false, false, true);
#endif            
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING_PRECHARGE);
            }
            break;
        case CONTACTORS_STATE_CALIBRATING_PRECHARGE:
#ifdef PRECHARGE_ON_NEGATIVE
            // Now precharge
            contactors_set_pos_pre_neg(true, true, false);
#else
            // Now precharge
            contactors_set_pos_pre_neg(false, true, true);
#endif
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
            contactors_set_pos_pre_neg(true, false, true);

            if(!check_current_is_below(model, 200)) {
                // Current too high (output is meant to be open circuit!)
                // TODO - Error properly?
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }
            break;

        case CONTACTORS_STATE_CALIBRATING_ONLY_NEG_INIT:
#ifdef PRECHARGE_ON_NEGATIVE
            // Precharge + negative (actually just negative due to inverted logic)
            contactors_set_pos_pre_neg(false, true, true);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING_ONLY_NEG);
            }
#else
            // Go straight to the next state
            state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CALIBRATING_ONLY_NEG);
#endif
            break;

        case CONTACTORS_STATE_CALIBRATING_ONLY_NEG:
            contactors_set_pos_pre_neg(false, false, true);

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
}
