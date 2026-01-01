#include "../settings.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

void init_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 0xFFF);
    pwm_config_set_clkdiv(&config, 36621.0f / PWM_FREQUENCY_HZ); // 1 = 36621Hz, 3.662f=10 kHz
    pwm_init(slice_num, &config, true);

    // Set the pin to off by default
    pwm_set_gpio_level(pin, 0);
}

void pwm_set(uint pin, uint32_t level) {
    if(level>0x1000) level = 0x1000;

    pwm_set_gpio_level(pin, level);
}
