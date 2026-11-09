#include "calibration.h"
#include "common.h"
#include "drivers/chip/nvm.h"
#include "drivers/sensors/ads1115.h"
#include "drivers/sensors/ina228.h"
#include "app/model.h"
#include "sys/logging/logging.h"

#include <stddef.h>

extern ina228_t ina228_dev;

// Calibration values from before the current sequence started. Each
// calibrate_*() call only updates its own values if it succeeds, but a sequence
// runs several of them in turn, so if any step fails (or the sequence is
// abandoned) the steps that did succeed are rolled back as well. Otherwise a
// mix of old and new values would stay in use, and be saved by the next NVM
// save.
static struct {
    calibration_data_t data;
#if IS_BMS_BOARD(REV1)
    // Stored after the other settings, to keep the NVM layout
    float pos_contactor_offset_mV;
    float link_voltage_mul;
#endif
    float isolation_zero_counts;
} calibration_before;

// The calibration values are embedded (anonymously) at the start of the
// persistent settings
_Static_assert(offsetof(bms_model_persistent_slow_t, battery_voltage_mul) == 0,
    "calibration_data_t must be at the start of bms_model_persistent_slow_t");

static calibration_data_t *calibration_data(bms_model_t *model) {
    return (calibration_data_t *)&model->persistent_slow;
}

static void start_sequence(bms_model_t *model, calibration_state_t first_state) {
    calibration_before.data = *calibration_data(model);
    calibration_before.isolation_zero_counts = model->isolation_zero_counts;
#if IS_BMS_BOARD(REV1)
    calibration_before.pos_contactor_offset_mV = model->pos_contactor_offset_mV;
    calibration_before.link_voltage_mul = model->link_voltage_mul;
#endif
    state_transition((sm_t*)&model->calibration_sm, first_state);
}

// End the sequence, saving the new values if every step succeeded, otherwise
// restoring the values from before it started
static void finish_sequence(bms_model_t *model, bool ok) {
    if (ok) {
        nvm_save_persistent_slow(model);
    } else {
        *calibration_data(model) = calibration_before.data;
        model->isolation_zero_counts = calibration_before.isolation_zero_counts;
#if IS_BMS_BOARD(REV1)
        model->pos_contactor_offset_mV = calibration_before.pos_contactor_offset_mV;
        model->link_voltage_mul = calibration_before.link_voltage_mul;
#endif
    }
    state_transition((sm_t*)&model->calibration_sm, CALIBRATION_STATE_IDLE);
}

void calibration_sm_tick(bms_model_t *model) {
    calibration_sm_t *sm = &model->calibration_sm;

    switch(sm->state) {
        case CALIBRATION_STATE_IDLE:
            if (model->calibration_req == CALIBRATION_REQUEST_START_OFFLINE_LONG) {
                model->calibration_req = CALIBRATION_REQUEST_NULL;
                model->contactor_req = CONTACTORS_REQUEST_CALIBRATE;
                start_sequence(model, CALIBRATION_STATE_LONG_WAIT_BOTH);
            } else if (model->calibration_req == CALIBRATION_REQUEST_START_OFFLINE_SHORT) {
                model->calibration_req = CALIBRATION_REQUEST_NULL;
                if (model->contactor_sm.state != CONTACTORS_STATE_OPEN) {
                    error_printf("Offline short calibration aborted: contactors must be OPEN (currently %d)\n", model->contactor_sm.state);
                } else {
                    start_sequence(model, CALIBRATION_STATE_SHORT_WAIT_OPEN);
                }
            } else if (model->calibration_req == CALIBRATION_REQUEST_START_ONLINE) {
                model->calibration_req = CALIBRATION_REQUEST_NULL;
                if (model->contactor_sm.state != CONTACTORS_STATE_OPEN && model->contactor_sm.state != CONTACTORS_STATE_CLOSED) {
                    error_printf("Online calibration aborted: contactors in state %d\n", model->contactor_sm.state);
                } else {
                    sm->initial_contactor_state = model->contactor_sm.state;
                    start_sequence(model, CALIBRATION_STATE_ONLINE_STABILIZING);
                }
            } else if (model->calibration_req == CALIBRATION_REQUEST_START_ISOLATION) {
                model->calibration_req = CALIBRATION_REQUEST_NULL;
                if (model->contactor_sm.state != CONTACTORS_STATE_OPEN) {
                    error_printf("Isolation calibration aborted: contactors must be OPEN (currently %d)\n", model->contactor_sm.state);
                } else {
                    start_sequence(model, CALIBRATION_STATE_ISOLATION_STABILIZING);
                }
            }
            break;

        /* OFFLINE LONG calibration (all contactors and current) */
        case CALIBRATION_STATE_LONG_WAIT_BOTH:
            if (model->contactor_sm.state == CONTACTORS_STATE_CALIBRATING_CLOSED) {
                state_transition((sm_t*)sm, CALIBRATION_STATE_LONG_STABILIZING);
            } else if (state_timeout((sm_t*)sm, 10000)) {
                finish_sequence(model, false);
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
#if IS_BMS_BOARD(REV1)
                // Link must be calibrated before the (direct) pos contactor,
                // whose multiplier is derived from the battery and link muls
                if(!calibrate_link_voltage(model, ads1115_get_calibration(7), n)) ok = false;
                if(!calibrate_pos_contactor(model, ads1115_get_calibration(6), n)) ok = false;
#else
                if(!calibrate_pos_contactor(model, ads1115_get_calibration(3), n)) ok = false;
#endif
                
                int32_t offset = (int32_t)div_round_closest(ina228_dev.null_accumulator, ina228_dev.null_counter);
                if(!calibrate_current_offset(model, offset)) ok = false;

                finish_sequence(model, ok);
            }
            break;

        /* OFFLINE SHORT calibration (battery voltage and negative contactor) */
        case CALIBRATION_STATE_SHORT_WAIT_OPEN:
            // Wait for all contactors to open
            if (model->contactor_sm.state == CONTACTORS_STATE_OPEN) {
                state_transition((sm_t*)sm, CALIBRATION_STATE_SHORT_STABILIZING_OPEN);
            } else if (state_timeout((sm_t*)sm, 10000)) {
                finish_sequence(model, false);
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
                    finish_sequence(model, false);
                }
            }
            break;

        case CALIBRATION_STATE_SHORT_WAIT_NEG:
            // Wait until negative contactor only is closed
            if (model->contactor_sm.state == CONTACTORS_STATE_CALIBRATING_ONLY_NEG) {
                state_transition((sm_t*)sm, CALIBRATION_STATE_SHORT_STABILIZING_NEG);
            } else if (state_timeout((sm_t*)sm, 10000)) {
                finish_sequence(model, false);
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

                finish_sequence(model, ok);
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
                finish_sequence(model, false);
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
#if IS_BMS_BOARD(REV1)
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
                finish_sequence(model, ok);
            }
            break;

        /* ISOLATION zero: the channel midpoint with the pack floating */
        case CALIBRATION_STATE_ISOLATION_STABILIZING:
        case CALIBRATION_STATE_ISOLATION_MEASURING:
            if (model->contactor_sm.state != CONTACTORS_STATE_OPEN) {
                error_printf("Isolation calibration aborted: contactors not open\n");
                finish_sequence(model, false);
            } else if (sm->state == CALIBRATION_STATE_ISOLATION_STABILIZING) {
                if (state_timeout((sm_t*)sm, ISOLATION_SETTLE_MS)) {
                    ads1115_start_calibration(SHORT_VOLTAGE_CALIBRATION_SAMPLES);
                    state_transition((sm_t*)sm, CALIBRATION_STATE_ISOLATION_MEASURING);
                }
            } else if (ads1115_calibration_finished()) {
                finish_sequence(model, calibrate_isolation_zero(model,
                    ads1115_get_calibration(4), ads1115_get_calibration(5),
                    (float)SHORT_VOLTAGE_CALIBRATION_SAMPLES));
            }
            break;
    }
}
