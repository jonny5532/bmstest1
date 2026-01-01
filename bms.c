#include "hw/chip/pwm.h"
#include "hw/chip/time.h"
#include "hw/chip/watchdog.h"
#include "hw/comms/duart.h"
#include "hw/sensors/ina228.h"
#include "hw/sensors/internal_adc.h"
#include "hw/sensors/ads1115.h"
#include "hw/pins.h"
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


// should this go?
void read_inputs(bms_model_t *model) {
    // ADS1115 voltage readings

    // For differential readings, full scale should be 436V

    // Read battery voltage from ADS1115 channel 0
    int16_t raw_batt = ads1115_get_sample(0);
    millis_t raw_batt_millis = ads1115_get_sample_millis(0);
    model->battery_voltage_mV = raw_batt * 436000 / 32768;
    model->battery_voltage_millis = raw_batt_millis;

    // Read output voltage from ADS1115 channel 1
    int16_t raw_out = ads1115_get_sample(1);
    millis_t raw_out_millis = ads1115_get_sample_millis(1);
    model->output_voltage_mV = raw_out * 436000 / 32768;
    model->output_voltage_millis = raw_out_millis;

    int16_t raw_neg_ctr = ads1115_get_sample(2);
    millis_t raw_neg_ctr_millis = ads1115_get_sample_millis(2);
    model->neg_contactor_voltage_mV = raw_neg_ctr * 436000 / 32768;
    model->neg_contactor_voltage_millis = raw_neg_ctr_millis;

    // For absolute readings, full scale should be 1745V

    int16_t raw_bat_plus = ads1115_get_sample(3);
    millis_t raw_bat_plus_millis = ads1115_get_sample_millis(3);
    int16_t raw_out_plus = ads1115_get_sample(4);
    millis_t raw_out_plus_millis = ads1115_get_sample_millis(4);
    model->pos_contactor_voltage_mV = (raw_bat_plus - raw_out_plus) * 1745000 / 32768;
    model->pos_contactor_voltage_millis = raw_bat_plus_millis < raw_out_plus_millis ? raw_bat_plus_millis : raw_out_plus_millis; // Use older value

    // INA228 current and charge readings

    int32_t current_raw = ina228_get_current_raw();
    millis_t current_millis = ina228_get_current_millis();
    model->current_mA = current_raw; // TODO - convert to mA properly
    model->current_millis = current_millis;
    
    uint32_t charge_raw = ina228_get_charge_raw();
    millis_t charge_millis = ina228_get_charge_millis();
    model->charge_raw = charge_raw;
    model->charge_millis = charge_millis;
}

void tick() {
    // The main tick function

    watchdog_update();

    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    contactor_sm_tick(&model);

    update_balancing(&model);

    if(timestep() & 32) {
        gpio_put(PIN_LED, true);
    } else {
        gpio_put(PIN_LED, false);
    }

    read_inputs(&model);
    model_tick(&model);
    inverter_tick(&model);
    internal_serial_tick();

    if((timestep() & 63) == 32) {
        //uint32_t start = time_us_32();
        bmb3y_wakeup_blocking();
        // uint32_t end = time_us_32();
        // printf("BMB3Y wakeup took %d us\n", end - start);
    }

    if((timestep() & 63) == 0) {
        // every 64 ticks, output stuff
        isosnoop_print_buffer();

        printf("Temp: %3d dC | 3V3: %4d mV | 5V: %4d mV | 12V: %5d mV | CtrV: %5d mV\n",
            get_temperature_c_times10(),
            internal_adc_read_3v3_mv(),
            internal_adc_read_5v_mv(),
            internal_adc_read_12v_mv(),
            internal_adc_read_contactor_mv()
        );

        printf("Batt: %6dmV | Out: %6dmV | NegCtr: %6dmV | PosCtr: %6dmV\n",
            model.battery_voltage_mV,
            model.output_voltage_mV,
            model.neg_contactor_voltage_mV,
            model.pos_contactor_voltage_mV
        );
        printf("Current: %6d mA | Charge: %lld raw\n\n",
            model.current_mA,
            model.charge_raw
        );
        
        // printf("ADS1115 Battery voltage: %d (%d)\n", ads1115_get_sample(0), ads1115_get_sample_millis(0));
        // printf("ADS1115 Output voltage: %d (%d)\n", ads1115_get_sample(1), ads1115_get_sample_millis(1));
        // printf("ADS1115 Across neg ctr: %d (%d)\n", ads1115_get_sample(2), ads1115_get_sample_millis(2));
        // printf("ADS1115 Bat+: %d (%d)\n", ads1115_get_sample(3), ads1115_get_sample_millis(3));
        // printf("ADS1115 Out+: %d (%d)\n", ads1115_get_sample(4), ads1115_get_sample_millis(4));

        // printf("INA228 Current: %ld raw\n", ina228_get_current_raw());
        // printf("INA228 Charge: %ld raw\n", ina228_get_charge_raw());
        // for(int i = 0; i < 5; i++) {
        //     printf("ADS1115 Channel %d: %d (%d)\n", i, ads1115_get_sample(i), ads1115_get_sample_millis(i));
        // }
    }
}

void synchronize_time() {
    uint32_t prev = millis();
    update_millis();

    int32_t delta = millis() - prev;
    if(delta <= 20) {
        sleep_ms(20 - delta);
    } else {
        // took too long!
        printf("Warning: loop overran (%d ms)\n", delta);
    }
    update_millis();
    update_timestep();
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

    return 0;
}
