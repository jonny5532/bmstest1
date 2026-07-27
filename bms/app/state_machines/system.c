#include "system.h"

#include "drivers/chip/nvm.h"
#include "drivers/sensors/ina228.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "config/limits.h"
#include "app/model.h"


bool successfully_initialized(const bms_model_t *model) {
    // check we're getting readings from everything we need

    if(!millis_recent_enough(model->high_voltages.battery_millis, BATTERY_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }

    if(!millis_recent_enough(model->high_voltages.output_millis, OUTPUT_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }

    if(!millis_recent_enough(model->high_voltages.neg_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }

    if(!millis_recent_enough(model->high_voltages.pos_contactor_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }

#if BMS_BOARD == BMS_BOARD_REV1
    if(!millis_recent_enough(model->high_voltages.link_millis, CONTACTOR_VOLTAGE_STALE_THRESHOLD_MS)) {
        return false;
    }
#endif

    if(!millis_recent_enough(model->current_millis, CURRENT_STALE_FAULT_THRESHOLD_MS)) {
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


void system_sm_tick(bms_model_t *model) {
    system_sm_t *system_sm = &(model->system_sm);

    if(system_sm->state != SYSTEM_STATE_FAULT) {
        // Check for fatal events
        if(get_highest_event_level() == LEVEL_FATAL) {
            // Go straight to fault state
            model->operating = false; // TODO: avoid setting this if RESTARTING?
            nvm_schedule_save_persistent_fast(model);
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
                    state_transition((sm_t*)system_sm, SYSTEM_STATE_INACTIVE);
                }
            } else if(state_timeout((sm_t*)system_sm, 10000)) {
                // TODO: raise a specific event too?
                state_transition((sm_t*)system_sm, SYSTEM_STATE_FAULT);
            }
            break;
        case SYSTEM_STATE_CALIBRATING:
            // TODO - use explicit set/clear flags rather than checking state?
            if(model->calibration_sm.state == CALIBRATION_STATE_IDLE) {
                // Calibration complete
                model->contactor_req = CONTACTORS_REQUEST_OPEN;
                state_transition((sm_t*)system_sm, SYSTEM_STATE_INACTIVE);
            } else if(state_timeout((sm_t*)system_sm, 120000)) {
                // Calibration timeout
                error_printf("System calibration timeout!\n");
                state_transition((sm_t*)system_sm, SYSTEM_STATE_FAULT);
            }
            break;
        case SYSTEM_STATE_INACTIVE:
            if(model->system_req == SYSTEM_REQUEST_RUN) {
                model->system_req = SYSTEM_REQUEST_NULL;
                model->operating = true;
                nvm_schedule_save_persistent_fast(model);
                state_transition((sm_t*)system_sm, SYSTEM_STATE_OPERATING);
            } else if(model->system_req == SYSTEM_REQUEST_CALIBRATE_OFFLINE_LONG) {
                model->system_req = SYSTEM_REQUEST_NULL;
                model->contactor_req = CONTACTORS_REQUEST_CALIBRATE;
                model->calibration_req = CALIBRATION_REQUEST_START_OFFLINE_LONG;
                state_transition((sm_t*)system_sm, SYSTEM_STATE_CALIBRATING);
            } else if(model->system_req == SYSTEM_REQUEST_CALIBRATE_OFFLINE_SHORT) {
                model->system_req = SYSTEM_REQUEST_NULL;
                model->calibration_req = CALIBRATION_REQUEST_START_OFFLINE_SHORT;
                state_transition((sm_t*)system_sm, SYSTEM_STATE_CALIBRATING);
            } else if(model->system_req == SYSTEM_REQUEST_CALIBRATE_ONLINE) {
                model->system_req = SYSTEM_REQUEST_NULL;
                model->calibration_req = CALIBRATION_REQUEST_START_ONLINE;
            }
            break;
        case SYSTEM_STATE_OPERATING:
            // Keep trying to close? (TODO: check failure count?)
            model->contactor_req = CONTACTORS_REQUEST_CLOSE;

            if(model->system_req == SYSTEM_REQUEST_STOP) {
                model->system_req = SYSTEM_REQUEST_NULL;
                // do we need a wait-for-open state?
                model->contactor_req = CONTACTORS_REQUEST_OPEN;
                model->operating = false;
                nvm_schedule_save_persistent_fast(model);
                state_transition((sm_t*)system_sm, SYSTEM_STATE_INACTIVE);
            } else if(model->system_req == SYSTEM_REQUEST_CALIBRATE_ONLINE) {
                model->system_req = SYSTEM_REQUEST_NULL;
                model->calibration_req = CALIBRATION_REQUEST_START_ONLINE;
            }
            break;
        case SYSTEM_STATE_FAULT:
            model->contactor_req = CONTACTORS_REQUEST_FORCE_OPEN;
            break;
    }
}
