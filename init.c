#include "hw/comms/duart.h"
#include "hw/comms/internal_serial.h"
#include "hw/sensors/ads1115.h"
#include "hw/sensors/ina228.h"
#include "hw/sensors/internal_adc.h"
#include "hw/pins.h"
#include "hw/chip/pwm.h"
#include "hw/chip/watchdog.h"
#include "inverter/inverter.h"
#include "isospi/isosnoop.h"
#include "isospi/isospi_master.h"
#include "state_machines/contactors.h"
#include "limits.h"
#include "model.h"

ina228_t ina228_dev = {0};
ads1115_t ads1115_dev = {0};

void init_hw() {
    init_watchdog();

    // Use a strapped pin to check we are running on the correct MCU
    gpio_init(PIN_MCU_CHECK);
    gpio_set_dir(PIN_MCU_CHECK, GPIO_IN);
    bool mcu_check = gpio_get(PIN_MCU_CHECK);
    if(mcu_check) {
        printf("Error: BMS firmware is running on the supervisor MCU!\n");
        while(1);
    }

    init_internal_adc();

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    init_pwm_pin(PIN_CONTACTOR_POS);
    init_pwm_pin(PIN_CONTACTOR_PRE);
    init_pwm_pin(PIN_CONTACTOR_NEG);

    if(!ina228_init(&ina228_dev, 0x40, 0.001, 100.0f)) {
        printf("INA228 init failed!\n");
    }

    if(!ads1115_init(&ads1115_dev, 0x48)) {
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
    init_hmi_serial();

    init_inverter();
}

void init_model() {
    sm_init((sm_t*)&model.system_sm, "system");
    sm_init((sm_t*)&model.contactor_sm, "contactors");
    sm_init((sm_t*)&model.balancing_sm, "balancing");
    sm_init((sm_t*)&model.offline_calibration_sm, "offline_calibration");

    // load calibration data from NVM
    if(nvm_load_calibration(&model)) {
        printf("Calibration data loaded from NVM\n");
    } else {
        printf("No calibration data in NVM\n");
    }

    model.capacity_mC = BATTERY_CAPACITY_AH * 3600 * 1000; // in mC
}

void init() {
    init_hw();
    init_comms();
    init_model();

    // start up by default
    model.system_req = SYSTEM_REQUEST_RUN;

    // This will advance the time to a non-zero value
    update_millis();

    int boot_count = update_boot_count();
    if (boot_count >= 0) {
        printf("Boot count from LittleFS: %d\n", boot_count);
    } else {
        printf("Failed to update boot count in LittleFS\n");
    }
}
