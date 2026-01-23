#include "../../config/settings.h"

#include "hardware/gpio.h"
#include "hardware/pwm.h"

void pwm_set(uint pin, uint32_t level) {
    if(level>0x1000) level = 0x1000;

    if (pwm_gpio_to_channel(pin) == PWM_CHAN_B) {
        // Shift phase by 180 degrees (invert) but keep duty cycle the same.
        // For an inverted channel, physical high time = (Wrap + 1) - RegisterLevel.
        // With Wrap = 0xFFF, RegisterLevel = 0x1000 - level.
        pwm_set_gpio_level(pin, 0x1000 - level);
    } else {
        pwm_set_gpio_level(pin, level);
    }
}

void init_pwm_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    // Some of our GPIOs will share slices so they need to be configured the
    // with the same clock/wrap.
    uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, 0xFFF);
    pwm_config_set_clkdiv(&config, 36621.0f / PWM_FREQUENCY_HZ); // 1 = 36621Hz, 3.662f=10 kHz

    // Enable inversion for channel B so it's not in phase with channel A.
    // This allows the duty cycles to be staggered to reduce peak current.
    pwm_config_set_output_polarity(&config, false, true);

    pwm_init(slice_num, &config, true);

    // Set the pin to off by default
    pwm_set(pin, 0);
}
