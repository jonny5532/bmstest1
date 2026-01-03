#include "contactors.h"
#include "../model.h"
#include "../debug/events.h"
#include "../hw/actuators/contactors.h"
#include "../hw/chip/time.h"

#include <stdio.h>

// TODO - as well as staleness, check for noisy readings (or use max value sometimes?)

// How recent a reading must be to be considered valid. The slowest sensors are 10Hz so this should be fine.
#define STALENESS_THRESHOLD_MS 200

// Contactor testing constants

// How long to wait for voltage to settle during contactor tests
#define CONTACTORS_TEST_WAIT_MS 500
// How long to wait after a failed precharge (for the PTCs to cool down)
#define CONTACTORS_FAILED_PRECHARGE_TIMEOUT_MS 20000
// How long to wait after a failed contactor test (to avoid rapid cycling)
#define CONTACTORS_FAILED_TEST_TIMEOUT_MS 20000
// Max voltage across a contactor to consider it closed
#define CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV 1000
// Min voltage across a contactor to consider it open
#define CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV 5000

// Precharge constants

// Largest allowable pack current to consider precharge successful
#define PRECHARGE_SUCCESS_MAX_MA 2000
// Minimum voltage difference to consider precharge successful
#define PRECHARGE_SUCCESS_MIN_MV 10000

// Contactor opening constants

// Current below which we can instantly open contactors
#define CONTACTORS_INSTANT_OPEN_MA 1000
// Current below which we can open contactors (after a delay)
#define CONTACTORS_DELAYED_OPEN_MA 5000
// How long we wait for the current to fall before potentially failing to open contactors
#define CONTACTORS_OPEN_TIMEOUT_MS 2000

// A type representing the result of a check. Includes event type and data which will be populated if unsuccessful.
typedef struct {
    bool success;
    bms_event_type_t event_type;
    union {
        uint8_t data8[8];
        uint16_t data16[4];
        uint32_t data32[2];
        uint64_t data64;
    };
} check_result_t;

static inline int32_t abs_int32(int32_t v) {
    return (v < 0) ? -v : v;
}

static inline int32_t clamp(int32_t v, int32_t min, int32_t max) {
    if(v < min) {
        return min;
    } else if(v > max) {
        return max;
    }
    return v;
}

static inline bool check(check_result_t res) {
    if(res.success) {
        // hmm, how do we clear multiple event types?
        //clear_bms_event(res.event_type);
    }

    return res.success;
}

static inline bool check_or_report(check_result_t res, bms_event_level_t level) {
    if(!res.success) {
        log_bms_event(
            res.event_type,
            level,
            res.data64
        );
    }

    return res.success;
}

check_result_t current_is_below(bms_model_t *model, int32_t threshold_ma) {
    check_result_t ret = {0};
    if(!millis_recent_enough(model->current_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        ret.event_type = ERR_CURRENT_STALE;
        ret.data64 = model->current_millis;
        return ret;
    }
    
    if(abs_int32(model->current_mA)<=threshold_ma) {
        ret.success = true;
        return ret;
    }

    ret.event_type = ERR_CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH;
    ret.data64 = model->current_mA;
    return ret;
}

check_result_t precharge_successful(bms_model_t *model) {
    check_result_t ret = {0};
    if(!millis_recent_enough(model->battery_voltage_millis, STALENESS_THRESHOLD_MS)) {
        ret.event_type = ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH;
        ret.data64 = 0x1000000000000000;
        return ret;
    }
    if(!millis_recent_enough(model->output_voltage_millis, STALENESS_THRESHOLD_MS)) {
        ret.event_type = ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH;
        ret.data64 = 0x2000000000000000;
        return ret;
    }
    if(!millis_recent_enough(model->neg_contactor_voltage_millis, STALENESS_THRESHOLD_MS)) {
        ret.event_type = ERR_CONTACTOR_PRECHARGE_NEG_OPEN;
        ret.data64 = 0x3000000000000000;
        return ret;
    }
    if(!millis_recent_enough(model->current_millis, STALENESS_THRESHOLD_MS)) {
        ret.event_type = ERR_CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH;
        ret.data64 = 0x4000000000000000;
        return ret;
    }

    // Check negative contactor is still closed
    if(abs_int32(model->neg_contactor_voltage_mV) > CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV) {
        ret.event_type = ERR_CONTACTOR_PRECHARGE_NEG_OPEN;
        ret.data64 = model->neg_contactor_voltage_mV;
        return ret;
    }

    // Is current still too high?
    if(abs_int32(model->current_mA) > PRECHARGE_SUCCESS_MAX_MA) {
        ret.event_type = ERR_CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH;
        ret.data64 = model->current_mA;
        return ret;
    }

    // Is voltage difference low enough?
    if(abs_int32(model->battery_voltage_mV - model->output_voltage_mV) > PRECHARGE_SUCCESS_MIN_MV) {
        ret.event_type = ERR_CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH;
        ret.data32[0] = model->battery_voltage_mV;
        ret.data32[1] = model->output_voltage_mV;
        return ret;
    }

    ret.success = true;
    return ret;
}

check_result_t battery_is_healthy(bms_model_t *model) {
    check_result_t ret = {0};
    // if(!millis_recent_enough(model->battery_voltage_millis, STALENESS_THRESHOLD_MS)) {
    //     ret.event_type = ERR_BATTERY_VOLTAGE_STALE;
    //     ret.data64 = model->battery_voltage_millis;
    //     return ret;
    // }

    // if(!millis_recent_enough(model->cell_voltage_millis, STALENESS_THRESHOLD_MS)) {
    //     ret.event_type = ERR_BATTERY_VOLTAGE_STALE;
    //     ret.data64 = model->cell_voltage_millis;
    //     return ret;
    // }

    // if(!millis_recent_enough(model->temperature_millis, STALENESS_THRESHOLD_MS)) {
    //     ret.event_type = ERR_BATTERY_TEMPERATURE_STALE;
    //     ret.data64 = model->temperature_millis;
    //     return ret;
    // }

    ret.success = true;
    return ret;
}

check_result_t contactor_neg_seems_closed(bms_model_t *model) {
    check_result_t ret = {0};
    if(!millis_recent_enough(model->neg_contactor_voltage_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        ret.event_type = ERR_CONTACTOR_NEG_STUCK_OPEN;
        ret.data64 = 0x1000000000000000;
        return ret;
    }

    if(abs_int32(model->neg_contactor_voltage_mV) > CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV) {
        ret.event_type = ERR_CONTACTOR_NEG_STUCK_OPEN;
        ret.data64 = model->neg_contactor_voltage_mV;
        return ret;
    }

    ret.success = true;
    return ret;
}

check_result_t contactor_neg_seems_open(bms_model_t *model) {
    check_result_t ret = {0};
    if(!millis_recent_enough(model->neg_contactor_voltage_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        ret.event_type = ERR_CONTACTOR_NEG_STUCK_CLOSED;
        ret.data64 = 0x1000000000000000;
        return ret;
    }

    // FIXME testing
    ret.success = true;
    return ret;

    int32_t voltage = abs_int32(model->neg_contactor_voltage_mV);
    if(voltage <= CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV) {
        ret.event_type = ERR_CONTACTOR_NEG_STUCK_CLOSED;
        ret.data64 = voltage;
        return ret;
    }

    ret.success = true;
    return ret;
}

check_result_t contactor_pos_seems_closed(bms_model_t *model) {
    check_result_t ret = {0};
    if(!millis_recent_enough(model->pos_contactor_voltage_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        ret.event_type = ERR_CONTACTOR_POS_STUCK_OPEN;
        ret.data64 = 0x1000000000000000;
        return ret;
    }

    int32_t voltage = abs_int32(model->pos_contactor_voltage_mV);
    if(voltage > CONTACTORS_CLOSED_VOLTAGE_THRESHOLD_MV) {
        ret.event_type = ERR_CONTACTOR_POS_STUCK_OPEN;
        ret.data64 = voltage;
        return ret;
    }

    ret.success = true;
    return ret;
}

check_result_t contactor_pos_seems_open(bms_model_t *model) {
    check_result_t ret = {0};
    if(!millis_recent_enough(model->pos_contactor_voltage_millis, STALENESS_THRESHOLD_MS)) {
        // stale reading
        ret.event_type = ERR_CONTACTOR_POS_STUCK_CLOSED;
        ret.data64 = 0x1000000000000000;
        return ret;
    }

    int32_t voltage = abs_int32(model->pos_contactor_voltage_mV);
    if(voltage < CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV) {
        ret.event_type = ERR_CONTACTOR_POS_STUCK_CLOSED;
        ret.data64 = voltage;
        return ret;
    }

    ret.success = true;
    return ret;
}

void contactor_sm_tick(bms_model_t *model) {
    contactors_sm_t *contactor_sm = &(model->contactor_sm);
    check_result_t result = {0};
    switch(contactor_sm->state) {
        case CONTACTORS_STATE_OPEN:
            contactors_set_pos_pre_neg(false, false, false);
            if(model->contactor_req == CONTACTORS_REQUEST_CLOSE) {
                model->contactor_req = CONTACTORS_REQUEST_NULL;

                if(check_or_report(battery_is_healthy(model), LEVEL_CRITICAL)) {
                    // Start a self-test of the contactors before precharging
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_NEG_OPEN);
                }
            }
            break;
        case CONTACTORS_STATE_PRECHARGING_NEG:
            // Close negative contactor first
            contactors_set_pos_pre_neg(false, false, true);
            if(state_timeout((sm_t*)contactor_sm, 500)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING);
            }
            break;
        case CONTACTORS_STATE_PRECHARGING:
            // Now close precharge contactor (actually just the Bat+ one)
            contactors_set_pos_pre_neg(false, true, true);

            result = precharge_successful(model);

            if(state_timeout((sm_t*)contactor_sm, 1000) && check(result)) {
                // successful precharge
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_CLOSED);
            } else if(state_timeout((sm_t*)contactor_sm, 10000)) {
                // failed to precharge
                check_or_report(result, LEVEL_CRITICAL);
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGE_FAILED);
            }
            break;
        case CONTACTORS_STATE_CLOSED:
             contactors_set_pos_pre_neg(true, false, true);

             if((
                model->contactor_req == CONTACTORS_REQUEST_OPEN && (
                    check(current_is_below(model, CONTACTORS_INSTANT_OPEN_MA))
                    || (check(current_is_below(model, CONTACTORS_DELAYED_OPEN_MA)) && state_timeout((sm_t*)contactor_sm, CONTACTORS_OPEN_TIMEOUT_MS)))
             ) || model->contactor_req == CONTACTORS_REQUEST_FORCE_OPEN) {
                 model->contactor_req = CONTACTORS_REQUEST_NULL;
                 state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
             }
             
             break;
        case CONTACTORS_STATE_PRECHARGE_FAILED:
            contactors_set_pos_pre_neg(false, false, false);
            // wait for the precharge to cool down
            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_FAILED_PRECHARGE_TIMEOUT_MS)) {
                state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_OPEN);
            }            
            break;
        case CONTACTORS_STATE_TESTING_NEG_OPEN:
            contactors_set_pos_pre_neg(false, false, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(check_or_report(contactor_neg_seems_open(model), LEVEL_CRITICAL)) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_NEG_CLOSED);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_NEG_CLOSED:
            contactors_set_pos_pre_neg(false, false, true);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(check_or_report(contactor_neg_seems_closed(model), LEVEL_CRITICAL)) {
                    // passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_POS_OPEN);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_POS_OPEN:
            contactors_set_pos_pre_neg(false, false, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(check_or_report(contactor_pos_seems_open(model), LEVEL_CRITICAL)) {
                    // all tests passed
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_POS_CLOSED);
                } else {
                    // fault detected
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_TESTING_FAILED);
                }
            }
            break;
        case CONTACTORS_STATE_TESTING_POS_CLOSED:
            // Both positive and precharge (since this actually leaves the precharge open due to the inverted logic)
            contactors_set_pos_pre_neg(true, true, false);

            if(state_timeout((sm_t*)contactor_sm, CONTACTORS_TEST_WAIT_MS)) {
                if(check_or_report(contactor_pos_seems_closed(model), LEVEL_CRITICAL)) {
                    // all tests passed, go to precharging
                    state_transition((sm_t*)contactor_sm, CONTACTORS_STATE_PRECHARGING_NEG);
                } else {
                    // fault detected
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
        default:
            // panic instead?
            printf("Invalid contactor state!");
            state_reset((sm_t*)contactor_sm);
            break;
    }
}
