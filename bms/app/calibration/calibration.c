#include "calibration.h"
#include "common.h"
#include "drivers/chip/nvm.h"
#include "drivers/sensors/ads1115.h"
#include "drivers/sensors/ina228.h"
#include "app/model.h"
#include "sys/logging/logging.h"

extern ina228_t ina228_dev;

void calibration_sm_tick(bms_model_t *model) {
    calibration_sm_t *sm = &model->calibration_sm;

    switch(sm->state) {
        case CALIBRATION_STATE_IDLE:
            if (model->calibration_req == CALIBRATION_REQUEST_START_OFFLINE_LONG) {
                model->calibration_req = CALIBRATION_REQUEST_NULL;
                model->contactor_req = CONTACTORS_REQUEST_CALIBRATE;
                state_transition((sm_t*)sm, CALIBRATION_STATE_LONG_WAIT_BOTH);
            } else if (model->calibration_req == CALIBRATION_REQUEST_START_OFFLINE_SHORT) {
                model->calibration_req = CALIBRATION_REQUEST_NULL;
                if (model->contactor_sm.state != CONTACTORS_STATE_OPEN) {
                    error_printf("Offline short calibration aborted: contactors must be OPEN (currently %d)\n", model->contactor_sm.state);
                } else {
                    state_transition((sm_t*)sm, CALIBRATION_STATE_SHORT_WAIT_OPEN);
                }
            } else if (model->calibration_req == CALIBRATION_REQUEST_START_ONLINE) {
                model->calibration_req = CALIBRATION_REQUEST_NULL;
                if (model->contactor_sm.state != CONTACTORS_STATE_OPEN && model->contactor_sm.state != CONTACTORS_STATE_CLOSED) {
                    error_printf("Online calibration aborted: contactors in state %d\n", model->contactor_sm.state);
                } else {
                    sm->initial_contactor_state = model->contactor_sm.state;
                    state_transition((sm_t*)sm, CALIBRATION_STATE_ONLINE_STABILIZING);
                }
            }
            break;

        /* OFFLINE LONG calibration (all contactors and current) */
        case CALIBRATION_STATE_LONG_WAIT_BOTH:
            if (model->contactor_sm.state == CONTACTORS_STATE_CALIBRATING_CLOSED) {
                state_transition((sm_t*)sm, CALIBRATION_STATE_LONG_STABILIZING);
            } else if (state_timeout((sm_t*)sm, 10000)) {
                state_transition((sm_t*)sm, CALIBRATION_STATE_IDLE);
            }
            break;

        case CALIBRATION_STATE_LONG_STABILIZING:
            if (state_timeout((sm_t*)sm, 2000)) {
                ads1115_start_calibration(VOLTAGE_CALIBRATION_SAMPLES);
                ina228_dev.null_accumulator = 0;
                ina228_dev.null_counter = 0;
                state_transition((sm_t*)sm, CALIBRATION_STATE_LONG_MEASURING);
            }
            break;

        case CALIBRATION_STATE_LONG_MEASURING:
            if (ads1115_calibration_finished()) {
                float n = (float)VOLTAGE_CALIBRATION_SAMPLES;
                bool ok = true;
                if(!calibrate_battery_voltage(model, ads1115_get_calibration(0), n)) ok = false;
                if(!calibrate_output_voltage(model, ads1115_get_calibration(1), n)) ok = false;
                if(!calibrate_neg_contactor(model, ads1115_get_calibration(2), n)) ok = false;
#if BMS_BOARD == BMS_BOARD_REV1
                // Link must be calibrated before the (direct) pos contactor,
                // whose multiplier is derived from the battery and link muls
                if(!calibrate_link_voltage(model, ads1115_get_calibration(7), n)) ok = false;
                if(!calibrate_pos_contactor(model, ads1115_get_calibration(6), n)) ok = false;
#else
                if(!calibrate_pos_contactor(model, ads1115_get_calibration(3), n)) ok = false;
#endif
                
                int32_t offset = (int32_t)div_round_closest(ina228_dev.null_accumulator, ina228_dev.null_counter);
                if(!calibrate_current_offset(model, offset)) ok = false;

                if (ok) nvm_save_persistent_slow(model);
                state_transition((sm_t*)sm, CALIBRATION_STATE_IDLE);
            }
            break;

        /* OFFLINE SHORT calibration (battery voltage and negative contactor) */
        case CALIBRATION_STATE_SHORT_WAIT_OPEN:
            // Wait for all contactors to open
            if (model->contactor_sm.state == CONTACTORS_STATE_OPEN) {
                state_transition((sm_t*)sm, CALIBRATION_STATE_SHORT_STABILIZING_OPEN);
            } else if (state_timeout((sm_t*)sm, 10000)) {
                state_transition((sm_t*)sm, CALIBRATION_STATE_IDLE);
            }
            break;
        
        case CALIBRATION_STATE_SHORT_STABILIZING_OPEN:
            // Wait for 2s and then start calibrating
            if (state_timeout((sm_t*)sm, 2000)) {
                ads1115_start_calibration(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
                ina228_dev.null_accumulator = 0;
                ina228_dev.null_counter = 0;
                state_transition((sm_t*)sm, CALIBRATION_STATE_SHORT_MEASURING_OPEN);
            }
            break;
        
        case CALIBRATION_STATE_SHORT_MEASURING_OPEN:
            // Wait until calibration is finished
            if (ads1115_calibration_finished()) {
                float n = (float)SHORT_VOLTAGE_CALIBRATION_SAMPLES;
                bool ok = true;
                if(!calibrate_battery_voltage(model, ads1115_get_calibration(0), n)) ok = false;
                
                int32_t offset = (int32_t)div_round_closest(ina228_dev.null_accumulator, ina228_dev.null_counter);
                if(!calibrate_current_offset(model, offset)) ok = false;

                if (ok) {
                    model->contactor_req = CONTACTORS_REQUEST_CALIBRATE_ONLY_NEG;
                    state_transition((sm_t*)sm, CALIBRATION_STATE_SHORT_WAIT_NEG);
                } else {
                    state_transition((sm_t*)sm, CALIBRATION_STATE_IDLE);
                }
            }
            break;

        case CALIBRATION_STATE_SHORT_WAIT_NEG:
            // Wait until negative contactor only is closed
            if (model->contactor_sm.state == CONTACTORS_STATE_CALIBRATING_ONLY_NEG) {
                state_transition((sm_t*)sm, CALIBRATION_STATE_SHORT_STABILIZING_NEG);
            } else if (state_timeout((sm_t*)sm, 10000)) {
                state_transition((sm_t*)sm, CALIBRATION_STATE_IDLE);
            }
            break;

        case CALIBRATION_STATE_SHORT_STABILIZING_NEG:
            // Wait for 2s and then start calibrating
            if (state_timeout((sm_t*)sm, 2000)) {
                ads1115_start_calibration(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
                state_transition((sm_t*)sm, CALIBRATION_STATE_SHORT_MEASURING_NEG);
            }
            break;

        case CALIBRATION_STATE_SHORT_MEASURING_NEG:
            // Wait until calibration is finished
            if (ads1115_calibration_finished()) {
                float n = (float)SHORT_VOLTAGE_CALIBRATION_SAMPLES;
                bool ok = true;
                // Since Neg is closed, Out- is tied to Bat-.
                // Channel 2 (Neg contactor) should be 0V -> calibrate offset.
                if(!calibrate_neg_contactor(model, ads1115_get_calibration(2), n)) ok = false;

                if (ok) {
                    nvm_save_persistent_slow(model);
                }
                state_transition((sm_t*)sm, CALIBRATION_STATE_IDLE);
            }
            break;

        /* ONLINE calibration (both voltages and both contactors) */
        case CALIBRATION_STATE_ONLINE_STABILIZING:
            if (state_timeout((sm_t*)sm, 2000)) {
                ads1115_start_calibration(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
                ina228_dev.null_accumulator = 0;
                ina228_dev.null_counter = 0;
                state_transition((sm_t*)sm, CALIBRATION_STATE_ONLINE_MEASURING);
            }
            break;

        case CALIBRATION_STATE_ONLINE_MEASURING:
            if (model->contactor_sm.state != sm->initial_contactor_state) {
                error_printf("Online calibration aborted: contactor state changed\n");
                state_transition((sm_t*)sm, CALIBRATION_STATE_IDLE);
                break;
            }
            if (ads1115_calibration_finished()) {
                float n = (float)SHORT_VOLTAGE_CALIBRATION_SAMPLES;
                bool ok = false;
                if (model->contactor_sm.state == CONTACTORS_STATE_OPEN) {
                    if (calibrate_battery_voltage(model, ads1115_get_calibration(0), n)) {
                        int32_t offset = (int32_t)div_round_closest(ina228_dev.null_accumulator, ina228_dev.null_counter);
                        if (calibrate_current_offset(model, offset)) {
                            ok = true;
                        }
                    }
                } else if (model->contactor_sm.state == CONTACTORS_STATE_CLOSED) {
                    if (calibrate_output_voltage(model, ads1115_get_calibration(1), n)) {
                        if (calibrate_neg_contactor(model, ads1115_get_calibration(2), n)) {
#if BMS_BOARD == BMS_BOARD_REV1
                            // Link before pos (pos mul derives from battery/link muls)
                            if (calibrate_link_voltage(model, ads1115_get_calibration(7), n)) {
                                if (calibrate_pos_contactor(model, ads1115_get_calibration(6), n)) {
                                    ok = true;
                                }
                            }
#else
                            if (calibrate_pos_contactor(model, ads1115_get_calibration(3), n)) {
                                ok = true;
                            }
#endif
                        }
                    }
                }
                if (ok) nvm_save_persistent_slow(model);
                state_transition((sm_t*)sm, CALIBRATION_STATE_IDLE);
            }
            break;
    }
}
