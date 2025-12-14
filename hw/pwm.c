#include "hardware/gpio.h"
#include "hardware/pwm.h"

//#include "pico/stdlib.h"

void init_pwm() {
    uint led_pin = 25;

    // Tell the LED pin that the PWM is in charge of its value.
    gpio_set_function(led_pin, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    uint slice_num = pwm_gpio_to_slice_num(led_pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 0xFFF);
    pwm_config_set_clkdiv(&config, 3.662f); // 1 = 36621Hz
    pwm_init(slice_num, &config, true);

    // between 0 and wrap
    pwm_set_gpio_level(led_pin, 0x400);
}


