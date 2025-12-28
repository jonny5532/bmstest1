#include "hw/internal_adc.h"
#include "hw/duart.h"
#include "hw/ina228.h"
#include "hw/ads1115.h"
#include "hw/pins.h"
#include "hw/pwm.h"
#include "hw/time.h"
#include "hw/watchdog.h"
#include "state_machines/contactors.h"
#include "battery/balancing.h"
#include "model.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "vendor/littlefs/lfs.h"

#include <stdio.h>

#define CRC16_INIT                  ((uint16_t)-1l)
void memcpy_with_crc16(uint8_t *dest, const uint8_t *src, size_t len, uint16_t *crc16);


extern uint16_t adc_samples_raw[8];
extern uint32_t adc_samples_smooth_accum[8];
extern uint16_t adc_samples_smoothed[8];
extern uint32_t adc_samples_variance_accum[8];
extern uint16_t adc_samples_variance[8];
extern millis_t adc_samples_millis[8];


void core1_entry() {
    flash_safe_execute_core_init();
    while(true) {
        // just sleep for now
        sleep_ms(1000);
    }
}

void tick() {
    // The main tick function

    watchdog_update();

    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    contactor_sm_tick(&model);

    update_balancing(&model);

    model_tick(&model);
    inverter_tick();
}

void synchronize_time() {
    uint32_t prev = millis();
    update_millis();
    update_timestep();

    uint32_t delta = millis() - prev;
    if(delta < 20) {
        sleep_ms(20 - delta);
    } else {
        // took too long!
    }
}

// TODO - where to put this?
bool battery_ready(bms_model_t *model) {
    // Check if we have recent voltage and current readings
    if(!millis_recent_enough(model->battery_voltage_millis, 5000)) {
        return false;
    }
    if(!millis_recent_enough(model->current_millis, 5000)) {
        return false;
    }
    if(!millis_recent_enough(model->temperature_millis, 5000)) {
        return false;
    }

    // TODO - other checks?

    return true;
}

int main() {
    stdio_usb_init();

    // Deliberate delay to:
    // 1) allow time for USB to enumerate so that we can catch early prints
    // 2) allow early reprogramming, in case the program hangs later on
    sleep_ms(4000);

    multicore_launch_core1(core1_entry);

    init();
    
    int boot_count = update_boot_count();
    if (boot_count >= 0) {
        printf("Boot count from LittleFS: %d\n", boot_count);
    } else {
        printf("Failed to update boot count in LittleFS\n");
    }

    while(true) {
        tick();
        synchronize_time();
    }

    uint8_t str[] = {
        'K', 'e', 'e', 'p', 'A', 'l', 'i', 'v',
    };

    int i=0;
    int n=0;
    while(true) {
        //sprintf((char*)&str[0], "%08d", n++);
        if(n>99999999) {
            n = 0;
        }

        if(duart_send_packet(&duart1, (uint8_t *)str, 8)) {
            //printf("Sent successfully\n");
            //putchar('1');
        } else {
            //printf("Send failed\n");
            //putchar('0');
        }
        if(i++ >= 80) {
            //putchar('\n');
            printf("Temp: %d C\n", get_temperature_c_times10());
            printf("ADC 12V: raw=%u smoothed=%u variance=%u\n", adc_samples_raw[0], adc_samples_smoothed[0], adc_samples_variance[0]);
            printf("Temp Sensor: raw=%u smoothed=%u variance=%u\n", adc_samples_raw[1], adc_samples_smoothed[1], adc_samples_variance[1]);
            //printf("INA228 Charge: %.3f C\n", ina228_dev.charge_c);
            i = 0;


        }
        sleep_us(10000);

        while(true) {
            uint8_t buf[256];
            size_t len = duart_read_packet(&duart1, buf, sizeof(buf));
            if(len > 0) {
                printf("<%.*s>", len, buf);
            } else {
                break;
            }
        }


        //printf("ADC 12V: raw=%u smoothed=%u\n", adc_samples_raw[0], adc_samples_smoothed[0]);

        update_millis();

        model.contactor_req = CONTACTORS_REQUEST_CLOSE;
        contactor_sm_tick(&model);
        update_balancing(&model);

        watchdog_update();
    }


    while(true) {
        update_millis();
        
        printf("Current millis: %llu\n", millis());
        sleep_ms(1000);

    }

    return 0;
}
