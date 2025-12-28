#include "hw/ads1115.h"
#include "hw/duart.h"
#include "hw/ina228.h"
#include "hw/pins.h"
#include "hw/pwm.h"
#include "hw/watchdog.h"
#include "state_machines/contactors.h"
#include "model.h"

ina228_t ina228_dev;
ads1115_t ads1115_dev;

void init_hw() {
    init_watchdog();

    init_adc();

    init_pwm_pin(PIN_CONTACTOR_POS);
    init_pwm_pin(PIN_CONTACTOR_PRE);
    init_pwm_pin(PIN_CONTACTOR_NEG);


    if(!ina228_init(&ina228_dev, 0x40, 0.001, 100.0f)) {
        printf("INA228 init failed!\n");
    }

    if(!ads1115_init(&ads1115_dev, 0x48, 2048)) {
        printf("ADS1115 init failed!\n");
    }
}

void init_comms() {
    init_duart(&duart1, 115200, 26, 27, true); //9375000 works!

    init_inverter();
}

void init_model() {
    sm_init((sm_t*)&model.contactor_sm, "contactors");
}

void init() {
    init_hw();
    init_comms();
    init_model();

    // This will advance the time to a non-zero value
    update_millis();

}