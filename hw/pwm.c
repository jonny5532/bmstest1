#include "hardware/gpio.h"
#include "hardware/pwm.h"

//#include "pico/stdlib.h"

void init_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 0xFFF);
    pwm_config_set_clkdiv(&config, 3.662f); // 1 = 36621Hz
    pwm_init(slice_num, &config, true);

    // between 0 and wrap
    pwm_set_gpio_level(pin, 0);
}

void pwm_set(uint pin, uint32_t level) {
    if(level>0x1000) level = 0x1000;

    pwm_set_gpio_level(pin, level);
}

