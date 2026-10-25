#include "../../config/limits.h"
#include "../../config/pins.h"
#include "../../config/settings.h"
#include "../chip/pwm.h"

#include "hardware/pwm.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef PRECHARGE_ON_NEGATIVE
#define PIN_CONTACTOR_PRE PIN_CONTACTOR_FC_NEG
#define INVERT_B false
#else
#define PIN_CONTACTOR_PRE PIN_CONTACTOR_FC_POS
#define INVERT_B true
#endif

#ifdef BMS_FC_POS_CONTACTOR
#ifndef PRECHARGE_ON_NEGATIVE
#error "BMS_FC_POS_CONTACTOR needs PRECHARGE_ON_NEGATIVE: otherwise GPIO16 is the precharge output"
#endif
#endif

// should these go in the model? or the state machine?
uint32_t pos_level = 0;
uint32_t pre_level = 0;
uint32_t neg_level = 0;
#ifdef BMS_FC_POS_CONTACTOR
uint32_t fc_pos_level = 0;
#endif

void contactors_init() {
    init_pwm_pin(PIN_CONTACTOR_POS, INVERT_B);
    init_pwm_pin(PIN_CONTACTOR_PRE, INVERT_B);
    init_pwm_pin(PIN_CONTACTOR_NEG, INVERT_B);
#ifdef BMS_FC_POS_CONTACTOR
    // FC+ (channel A) shares a slice with neg (channel B), so it runs in phase with neg
    init_pwm_pin(PIN_CONTACTOR_FC_POS, INVERT_B);
#endif

    // Since the contactors will be on most of the time, at a low duty cycle, we
    // should stagger their PWM phases to reduce peak current draw. Oddly, in
    // synchrony, the contactors actually boost the rail voltage, so we want the
    // settings that give the least boost.
   
#ifdef PRECHARGE_ON_NEGATIVE
    // They are all on different slices, so just stagger them by 120 degrees
    int slice1 = pwm_gpio_to_slice_num(PIN_CONTACTOR_POS);
    int slice2 = pwm_gpio_to_slice_num(PIN_CONTACTOR_PRE);
    for(int i=0;i<0x555;i++) {
        pwm_advance_count(slice1);
        pwm_advance_count(slice2);
        pwm_advance_count(slice2);
    }
#else
    // Pre and neg are on the same slice, meaning the only way to stagger them
    // is to invert one of them (so the contactors will be energised just before
    // and after the PWM cycle start).

    // Pos is on a difference slice, so we can advance the slice phase (into
    // roughly the middle of the pre/neg period) to stagger it. The advance is
    // chosen experimentally to minimise voltage boost.

    /* 
      Advance Voltage
            0   14703
        0x500   13767
        0x600   13528
        0x700   13630
        0x800   13896
        0x900   14155
    */
    int slice_num = pwm_gpio_to_slice_num(PIN_CONTACTOR_POS);
    for(int i=0;i<0x600;i++) {
        pwm_advance_count(slice_num);
    }
#endif
}

static void set_with_pwm(unsigned int pin, bool enabled, uint32_t *level) {
    // Sets the given PWM pin to an on or off state. On turning on, the level
    // can start above the max, giving a short period of full on before ramping
    // down to the min level.

    const unsigned int PWM_INITIAL_LEVEL = (uint32_t)(CONTACTOR_PWM_INITIAL * 0x1000);
    const unsigned int PWM_DECREMENT = (uint32_t)(CONTACTOR_PWM_DECREMENT * 0x1000);
    const unsigned int PWM_MIN_LEVEL = (uint32_t)(CONTACTOR_PWM_MIN * 0x1000);

    if(!enabled) {
        // Disabled, go straight to zero
        *level = 0;
    } else if(enabled && *level==0) {
        // Just enabled, start at initial level
        *level = PWM_INITIAL_LEVEL;
    } else if(enabled) {
        // Subsequent sets decay down to the min level
        if(*level < (PWM_MIN_LEVEL + PWM_DECREMENT)) {
            *level = PWM_MIN_LEVEL;
        } else {
            *level -= PWM_DECREMENT;
        }
    }
    pwm_set(pin, *level, INVERT_B);
}

void contactors_set_pos_pre_neg(bool pos, bool pre, bool neg) {
#ifdef PRECHARGE_ON_NEGATIVE
    // Negative needs to be on for precharging
    bool actual_neg = neg || pre;
    // Precharge contactor actually bypasses the precharge resistor, so is off
    // during precharge and on otherwise (when neg is on)
    bool actual_pre = neg && !pre;
    // Positive is as requested
    bool actual_pos = pos;
#else
    // Positive needs to be on for precharging
    bool actual_pos = pos || pre;
    // Precharge contactor actually bypasses the precharge resistor, so is off
    // during precharge and on otherwise (when pos is on)
    bool actual_pre = pos && !pre;
    // Negative is as requested
    bool actual_neg = neg;
#endif

    set_with_pwm(PIN_CONTACTOR_PRE, actual_pre, &pre_level);
    set_with_pwm(PIN_CONTACTOR_POS, actual_pos, &pos_level);
    set_with_pwm(PIN_CONTACTOR_NEG, actual_neg, &neg_level);
#ifdef BMS_FC_POS_CONTACTOR
    // FC+ connects the FC port to Link+, so it follows Link+ exactly: closed for the
    // precharge (the PTC on the negative side limits the inrush) and while closed
    set_with_pwm(PIN_CONTACTOR_FC_POS, actual_pos, &fc_pos_level);
#endif
}

void contactors_test_pre(bool closed) {
    // Independently close the precharge contactor for testing, which you can't
    // normally do without also closing the adjacent contactor, but we do here
    // for test purposes.

    set_with_pwm(PIN_CONTACTOR_PRE, closed, &pre_level);
    set_with_pwm(PIN_CONTACTOR_POS, false, &pos_level);
    set_with_pwm(PIN_CONTACTOR_NEG, false, &neg_level);
#ifdef BMS_FC_POS_CONTACTOR
    set_with_pwm(PIN_CONTACTOR_FC_POS, false, &fc_pos_level);
#endif
}