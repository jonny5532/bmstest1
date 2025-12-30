#include "../pins.h"
#include "../chip/pwm.h"

#include <stdbool.h>
#include <stdint.h>

#define PWM_INITIAL_LEVEL 0x1600
#define PWM_DECREMENT 0x20
#define PWM_MIN_LEVEL 0x200

// should these go in the model? or the state machine?
uint32_t pos_level = 0;
uint32_t pre_level = 0;
uint32_t neg_level = 0;

static void set_with_pwm(unsigned int pin, bool enabled, uint32_t *level) {
    // Sets the given PWM pin to an on or off state. On turning on, the level
    // starts above the max, giving a short period of full on before ramping
    // down to the min level.

    if(!enabled) {
        *level = 0;
    } else if(enabled && *level==0) {
        *level = PWM_INITIAL_LEVEL;
    } else if(enabled) {
        *level -= PWM_DECREMENT;
        if(*level<PWM_MIN_LEVEL) *level = PWM_MIN_LEVEL;
    }
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
