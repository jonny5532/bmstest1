#include "../../config/pins.h"
#include "../../config/settings.h"
#include "../chip/pwm.h"

#include "hardware/pwm.h"

#include <stdbool.h>
#include <stdint.h>

// should these go in the model? or the state machine?
uint32_t pos_level = 0;
uint32_t pre_level = 0;
uint32_t neg_level = 0;

void contactors_init() {
    init_pwm_pin(PIN_CONTACTOR_POS);
    init_pwm_pin(PIN_CONTACTOR_PRE);
    init_pwm_pin(PIN_CONTACTOR_NEG);

    // Pos is on a difference slice, advance it to desync from the others
    int slice_num = pwm_gpio_to_slice_num(PIN_CONTACTOR_POS);
    for(int i=0;i<0x800;i++) {
        pwm_advance_count(slice_num);
    }
}

static void set_with_pwm(unsigned int pin, bool enabled, uint32_t *level) {
    // Sets the given PWM pin to an on or off state. On turning on, the level
    // can start above the max, giving a short period of full on before ramping
    // down to the min level.

    const unsigned int PWM_INITIAL_LEVEL = (uint32_t)(CONTACTOR_PWM_INITIAL * 0x1000);
    const unsigned int PWM_DECREMENT = (uint32_t)(CONTACTOR_PWM_DECREMENT * 0x1000);
    const unsigned int PWM_MIN_LEVEL = (uint32_t)(CONTACTOR_PWM_MIN * 0x1000);

    if(!enabled) {
        *level = 0;
    } else if(enabled && *level==0) {
        *level = PWM_INITIAL_LEVEL;
    } else if(enabled) {
        if(*level < (PWM_MIN_LEVEL + PWM_DECREMENT)) {
            *level = PWM_MIN_LEVEL;
        } else {
            *level -= PWM_DECREMENT;
        }
        //if(*level<PWM_MIN_LEVEL) *level = PWM_MIN_LEVEL;
    }
    // if(pin==PIN_CONTACTOR_NEG) {
    //     printf("Contactor NEG PWM level: %d\n", *level);
    // }
    pwm_set(pin, *level);
}

void contactors_set_pos_pre_neg(bool pos, bool pre, bool neg) {
    // Positive needs to be on for precharging
    bool actual_pos = pos || pre;
    // Precharge contactor actually bypasses the precharge resistor, so is off
    // during precharge and on otherwise (when pos is on)
    bool actual_pre = pos && !pre;
    // Negative is as requested
    bool actual_neg = neg;

    set_with_pwm(PIN_CONTACTOR_PRE, actual_pre, &pre_level);
    set_with_pwm(PIN_CONTACTOR_POS, actual_pos, &pos_level);
    set_with_pwm(PIN_CONTACTOR_NEG, actual_neg, &neg_level);
}

void contactors_test_pre(bool closed) {
    // Independently close the precharge contactor for testing, which you can't
    // normally do without also closing the positive contactor, but we do here.

    set_with_pwm(PIN_CONTACTOR_PRE, closed, &pre_level);
    set_with_pwm(PIN_CONTACTOR_POS, false, &pos_level);
    set_with_pwm(PIN_CONTACTOR_NEG, false, &neg_level);
}