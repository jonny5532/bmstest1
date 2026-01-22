#include "system.h"

#include "../hw/sensors/ina228.h"
#include "../monitoring/events.h"
#include "../limits.h"
#include "../model.h"


bool successfully_initialized(bms_model_t *model) {
    // check we're getting readings from everything we need

    if(!millis_recent_enough(model->battery_voltage_millis, BATTERY_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }

    if(!millis_recent_enough(model->output_voltage_millis, OUTPUT_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }

    if(!millis_recent_enough(model->neg_contactor_voltage_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }

    if(!millis_recent_enough(model->pos_contactor_voltage_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }

    if(!millis_recent_enough(model->current_millis, CURRENT_STALE_THRESHOLD_MS)) {
        return false;
    }

    if(!millis_recent_enough(model->cell_voltage_millis, CELL_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }

    if(!millis_recent_enough(model->temperature_millis, TEMPERATURE_STALE_THRESHOLD_MS(model))) {
        return false;
    }

    return true;
}

#define VOLTAGE_CALIBRATION_SAMPLES 1024


void system_sm_tick(bms_model_t *model) {
    system_sm_t *system_sm = &(model->system_sm);

    if(system_sm->state != SYSTEM_STATE_FAULT) {
        // Check for fatal events
        if(get_highest_event_level() == LEVEL_FATAL) {
            // Go straight to fault state
            state_transition((sm_t*)system_sm, SYSTEM_STATE_FAULT);
        }
    }

    switch(system_sm->state) {
        case SYSTEM_STATE_UNINITIALIZED:
            state_transition((sm_t*)system_sm, SYSTEM_STATE_INITIALIZING);
            break;
        case SYSTEM_STATE_INITIALIZING:
            if(successfully_initialized(model)) {
                // wait a few seconds for supervisor
                if(state_timeout((sm_t*)system_sm, 10000)) {
                    // TODO - have a better check for calibration status?
                    if(model->neg_contactor_offset_mV) {
                        // already calibrated
                        state_transition((sm_t*)system_sm, SYSTEM_STATE_INACTIVE);
                    } else {
                        // need to calibrate
                        model->contactor_req = CONTACTORS_REQUEST_CALIBRATE;
                        model->offline_calibration_req = OFFLINE_CALIBRATION_REQUEST_START;
                        state_transition((sm_t*)system_sm, SYSTEM_STATE_CALIBRATING);
                    }
                }
            }
            // assert some events after a timeout?
            break;
        case SYSTEM_STATE_CALIBRATING:
            // TODO - use explicit set/clear flags rather than checking state?
            if(model->offline_calibration_sm.state == OFFLINE_CALIBRATION_STATE_IDLE) {
                // Calibration complete
                model->contactor_req = CONTACTORS_REQUEST_OPEN;
                state_transition((sm_t*)system_sm, SYSTEM_STATE_INACTIVE);
            } else if(state_timeout((sm_t*)system_sm, 120000)) {
                // Calibration timeout
                printf("System calibration timeout!\n");
                state_transition((sm_t*)system_sm, SYSTEM_STATE_FAULT);
            }
            break;
        case SYSTEM_STATE_INACTIVE:
            // Currently, the supervisor takes a few seconds to start up, so we
            // need a 2s delay before trying to close contactors.
            if(model->system_req == SYSTEM_REQUEST_RUN && state_timeout((sm_t*)system_sm, 2000)) {
                // leave request asserted?
                //model->system_req = SYSTEM_REQUEST_NULL;
                state_transition((sm_t*)system_sm, SYSTEM_STATE_OPERATING);
            } else if(model->system_req == SYSTEM_REQUEST_CALIBRATE) {
                model->system_req = SYSTEM_REQUEST_NULL;
                model->contactor_req = CONTACTORS_REQUEST_CALIBRATE;
                model->offline_calibration_req = OFFLINE_CALIBRATION_REQUEST_START;
                state_transition((sm_t*)system_sm, SYSTEM_STATE_CALIBRATING);
            }
            break;
        case SYSTEM_STATE_OPERATING:
            // Keep trying to close? (TODO: check failure count?)
            model->contactor_req = CONTACTORS_REQUEST_CLOSE;

            if(model->system_req == SYSTEM_REQUEST_STOP) {
                model->system_req = SYSTEM_REQUEST_NULL;
                // do we need a wait-for-open state?
                model->contactor_req = CONTACTORS_REQUEST_OPEN;
                state_transition((sm_t*)system_sm, SYSTEM_STATE_INACTIVE);
            }
            break;
        case SYSTEM_STATE_FAULT:
            model->contactor_req = CONTACTORS_REQUEST_FORCE_OPEN;
            break;
    }
}
