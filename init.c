#include "hw/comms/duart.h"
#include "hw/sensors/ads1115.h"
#include "hw/sensors/ina228.h"
#include "hw/pins.h"
#include "hw/chip/pwm.h"
#include "hw/chip/watchdog.h"
#include "inverter/inverter.h"
#include "state_machines/contactors.h"
#include "model.h"

ina228_t ina228_dev;
ads1115_t ads1115_dev;

void init_hw() {
    init_watchdog();

    init_adc();

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    init_pwm_pin(PIN_CONTACTOR_POS);
    init_pwm_pin(PIN_CONTACTOR_PRE);
    init_pwm_pin(PIN_CONTACTOR_NEG);


    if(!ina228_init(&ina228_dev, 0x40, 0.001, 100.0f)) {
        printf("INA228 init failed!\n");
    }

    if(!ads1115_init(&ads1115_dev, 0x48, 2048)) {
        printf("ADS1115 init failed!\n");
    }

    isospi_master_setup(
        PIN_ISOSPI_TX_EN,
        PIN_ISOSPI_RX_HIGH
    );
    isosnoop_setup(
        PIN_ISOSPI_RX_HIGH,
        PIN_ISOSNOOP_RX_DEBUG,
        PIN_ISOSPI_RX_AND,
        PIN_ISOSNOOP_TIMER_DISABLE
    );
}

void init_comms() {
    init_internal_serial();

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

    int boot_count = update_boot_count();
    if (boot_count >= 0) {
        printf("Boot count from LittleFS: %d\n", boot_count);
    } else {
        printf("Failed to update boot count in LittleFS\n");
    }
}
