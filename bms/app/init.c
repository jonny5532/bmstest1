#include "../drivers/comms/duart.h"
#include "../protocols/internal_serial/internal_serial.h"
#include "../drivers/sensors/ads1115.h"
#include "../drivers/sensors/ina228.h"
#include "../drivers/sensors/internal_adc.h"
#include "../config/limits.h"
#include "../config/pins.h"
#include "../drivers/chip/pwm.h"
#include "../drivers/chip/watchdog.h"
#include "../protocols/inverter/inverter.h"
#include "../drivers/isospi/isosnoop.h"
#include "../drivers/isospi/isospi_master.h"
#include "state_machines/contactors.h"
#include "model.h"

ina228_t ina228_dev = {0};
ads1115_t ads1115_dev = {0};

static void init_hw() {
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

    contactors_init();

    gpio_init(PIN_ESTOP);
    gpio_set_dir(PIN_ESTOP, GPIO_IN);
    gpio_pull_down(PIN_ESTOP);

    gpio_init(PIN_AUX_CONTACTOR_PRE);
    gpio_set_dir(PIN_AUX_CONTACTOR_PRE, GPIO_IN);
    gpio_pull_down(PIN_AUX_CONTACTOR_PRE);

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

static void init_comms() {
    init_internal_serial();
    init_hmi_serial();

    init_inverter();
}

static void init_model() {
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

    model.nameplate_capacity_mC = NAMEPLATE_CAPACITY_AH * 3600 * 1000; // in mC

    // Pretend balancing is active at startup to avoid trusting
    // cell voltages until we've definitely turned balancing off.
    model.balancing_active = true;
}

void bms_init() {
    init_hw();
    init_comms();
    init_model();

    // start up by default
    model.system_req = SYSTEM_REQUEST_RUN;

    // This will advance the time to a non-zero value
    update_millis();

    int boot_count = update_boot_count();
    // NVM operations can be slow, so allow a missed deadline
    model.ignore_missed_deadline = true;
    if (boot_count >= 0) {
        printf("Boot count from LittleFS: %d\n", boot_count);
    } else {
        printf("Failed to update boot count in LittleFS\n");
    }

}
