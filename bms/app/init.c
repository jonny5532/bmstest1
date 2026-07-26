#include "../drivers/comms/duart.h"
#include "../protocols/internal_serial/internal_serial.h"
#include "../drivers/sensors/ads1115.h"
#include "../drivers/sensors/ina228.h"
#include "../drivers/sensors/internal_adc.h"
#include "../config/limits.h"
#include "../config/pins.h"
#include "../drivers/chip/nvm.h"
#include "../drivers/chip/pwm.h"
#include "../drivers/chip/watchdog.h"
#include "../drivers/contactors/contactors.h"
#include "../protocols/hmi_serial/hmi_serial.h"
#include "../protocols/inverter/inverter.h"
#include "../drivers/isospi/isosnoop.h"
#include "../drivers/isospi/isospi_master.h"
#include "state_machines/contactors.h"
#include "model.h"
#include "sys/logging/logging.h"
#include "pico/stdlib.h"
#include <math.h>

ina228_t ina228_dev = {0};

static struct repeating_timer logging_timer;

bool logging_timer_callback(struct repeating_timer *t) {
    (void)t;
    logging_flush_to_stdout();
    return true;
}

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

    // Init precharge sense input
#ifdef PRECHARGE_ON_NEGATIVE
    gpio_init(PIN_AUX_CONTACTOR_FC_NEG);
    gpio_set_dir(PIN_AUX_CONTACTOR_FC_NEG, GPIO_IN);
    gpio_pull_down(PIN_AUX_CONTACTOR_FC_NEG);
#else
    gpio_init(PIN_AUX_CONTACTOR_FC_POS);
    gpio_set_dir(PIN_AUX_CONTACTOR_FC_POS, GPIO_IN);
    gpio_pull_down(PIN_AUX_CONTACTOR_FC_POS);
#endif

    if(!ina228_init(&ina228_dev, 0x40, 0.001, 100.0f)) {
        error_printf("INA228 init failed!\n");
    }

    if(!ads1115_init(&ads1115_dev, 0x48)) {
        error_printf("ADS1115 init failed!\n");
    }
#ifdef HAS_ADS1115_SECONDARY
    if(!ads1115_init(&ads1115_dev_b, 0x49)) {
        error_printf("ADS1115 (secondary, 0x49) init failed!\n");
    }
#endif

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
    sm_init((sm_t*)&model.calibration_sm, "calibration");

    // load calibration data from NVM
    // if(nvm_load_calibration(&model)) {
    //     info_printf("Calibration data loaded from NVM\n");
    // } else {
    //     info_printf("No calibration data in NVM\n");
    // }
    if(nvm_load_persistent_slow(&model)) {
        info_printf("Persistent slow data loaded from NVM\n");
    } else {
        info_printf("No persistent slow data in NVM\n");
    }
    if(nvm_load_persistent_fast(&model)) {
        info_printf("Persistent fast data loaded from NVM\n");
    } else {
        info_printf("No persistent fast data in NVM\n");
    }

    // If NVM contained operating=true, resume operating
    if(model.operating) {
        model.system_req = SYSTEM_REQUEST_RUN;
    }

    model.nameplate_capacity_mC = NAMEPLATE_CAPACITY_AH * 3600 * 1000; // in mC

    model.current_filtered_mA = NAN;
    model.working_charge_current_limit_filtered_dA = NAN;

    for(int i=0; i<NUM_MODULE_TEMPS; i++) {
        model.module_temperatures[i] = NAN;
    }
}

void bms_init() {
    init_hw();
    init_comms();
    init_model();

    // This will advance the time to a non-zero value
    update_millis();

    int boot_count = update_boot_count();
    // NVM operations can be slow, so allow a missed deadline
    model.ignore_missed_deadline = true;
    if (boot_count >= 0) {
        info_printf("Boot count from LittleFS: %d\n", boot_count);
    } else {
        error_printf("Failed to update boot count in LittleFS\n");
    }

    add_repeating_timer_ms(10, logging_timer_callback, NULL, &logging_timer);
}
