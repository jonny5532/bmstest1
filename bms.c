#include "debug/events.h"
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
#include "inverter/inverter.h"
#include "model.h"

#include "bmb3y/bmb3y.h"

#include "hardware/watchdog.h"
#include "pico/flash.h"
#include "pico/multicore.h"
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
    // Read various inputs into the model. These are mostly gathered asynchronously 
    // so this is a quick process.

    // ADS1115 voltage readings

    // For differential readings, full scale should be 436V

    // Read battery voltage from ADS1115 channel 0
    int16_t raw_batt = ads1115_get_sample(0);
    millis_t raw_batt_millis = ads1115_get_sample_millis(0);
    model->battery_voltage_mV = raw_batt * 436000 / 32768 / 8;
    model->battery_voltage_range_mV = ads1115_get_sample_range(0) * 436000 / 32768;
    model->battery_voltage_millis = raw_batt_millis;

    // Read output voltage from ADS1115 channel 1
    int16_t raw_out = ads1115_get_sample(1);
    millis_t raw_out_millis = ads1115_get_sample_millis(1);
    model->output_voltage_mV = raw_out * 436000 / 32768 / 8;
    model->output_voltage_range_mV = ads1115_get_sample_range(1) * 436000 / 32768;
    model->output_voltage_millis = raw_out_millis;

    int16_t raw_neg_ctr = ads1115_get_sample(2);
    millis_t raw_neg_ctr_millis = ads1115_get_sample_millis(2);
    model->neg_contactor_voltage_mV = raw_neg_ctr * 436000 / 32768 / 8;
    model->neg_contactor_voltage_range_mV = ads1115_get_sample_range(2) * 436000 / 32768;
    model->neg_contactor_voltage_millis = raw_neg_ctr_millis;

    // Positive contactor voltage has to be derived from the difference between (battery+
    // to output-) and (output+ to output-), since we can't sample relative to battery+.

    int16_t raw_bat_plus_to_out_neg = ads1115_get_sample(3);
    millis_t raw_bat_plus_to_out_neg_millis = ads1115_get_sample_millis(3);
    model->pos_contactor_voltage_mV = (raw_bat_plus_to_out_neg - raw_out) * 436000 / 32768 / 8;
    model->pos_contactor_voltage_range_mV = (ads1115_get_sample_range(3) * 436000 / 32768)/2 + (ads1115_get_sample_range(1) * 436000 / 32768)/2;
    model->pos_contactor_voltage_millis = raw_bat_plus_to_out_neg_millis < raw_out_millis ? raw_bat_plus_to_out_neg_millis : raw_out_millis;

        
    //     * 436000 / 32768 / 8;
    // model->pos_contactor_voltage_range_m
    // e

    // For absolute readings, full scale should be 1745V

    // int16_t raw_bat_plus = ads1115_get_sample(3);
    // millis_t raw_bat_plus_millis = ads1115_get_sample_millis(3);
    // int16_t raw_out_plus = ads1115_get_sample(4);
    // millis_t raw_out_plus_millis = ads1115_get_sample_millis(4);
    // model->pos_contactor_voltage_mV = (raw_bat_plus - raw_out_plus) * 1745000 / 32768 / 8;
    // model->pos_contactor_voltage_range_mV = (ads1115_get_sample_range(3) * 1745000 / 32768)/2 + (ads1115_get_sample_range(4) * 1745000 / 32768)/2;
    // model->pos_contactor_voltage_millis = raw_bat_plus_millis < raw_out_plus_millis ? raw_bat_plus_millis : raw_out_plus_millis; // Use older value

    // INA228 current and charge readings
    
    if((timestep() & 0x7) == 0) {
        // Only query every 160ms or so (readings are available every 531ms but
        // we want to be sure we don't miss any). This is currently blocking due
        // to the I2C transaction but doesn't have to wait for the reading
        // itself.

        // This reads current, and also updates the charge accumulator if it is
        // a new reading

        extern ina228_t ina228_dev;
        if(!ina228_read_current(&ina228_dev)) {
            printf("INA228 current read failed\n");
        }
    }
}

void tick() {
    // The main tick function

    // Phase 0: Preamble

    watchdog_update();

    if(timestep() & 32) {
        gpio_put(PIN_LED, true);
    } else {
        gpio_put(PIN_LED, false);
    }

    // Phase 1: Read sensors

    // Read INA228 current occasionally
    if((timestep() & 0x7) == 0) {
        extern ina228_t ina228_dev;
        if(!ina228_read_current(&ina228_dev)) {
            printf("INA228 current read failed\n");
        }
    }

    read_inputs(&model);

    // Phase 2: Update model

    static int32_t last_charge_raw = 0;
    model.soc = kalman_update(
        ((model.charge_raw - last_charge_raw) * 132736) / 1000000,
        model.current_mA,
        model.battery_voltage_mV
    );
    last_charge_raw = model.charge_raw;

    model_tick(&model);
    confirm_battery_safety(&model);

    // Phase 3: Outputs and communications

    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    contactor_sm_tick(&model);

    inverter_tick(&model);
    internal_serial_tick();
    hmi_serial_tick(&model);
    

    // We talk to the BMB3Y every 64 ticks (about once per second)
    if((timestep() & 0x3f) == 31) {
        uint32_t start = time_us_32();

        bmb3y_wakeup_blocking();
        bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
        //bmb3y_send_command_blocking(BMB3Y_CMD_MUTE);
        bmb3y_send_command_blocking(BMB3Y_CMD_SNAPSHOT);

        // 70 sometimes isn't enough time, 100  seems ok
        sleep_us(100);

        bmb3y_read_cell_voltages_blocking(&model);

        uint32_t end = time_us_32();
        printf("BMB3Y test took %ld us\n", end - start);

        

        //bmb3y_send_command_blocking(BMB3Y_CMD_MUTE);
        //bmb3y_set_balancing(bitmap_set, even_counter & 8);
        // even_counter++;
        // bmb3y_set_balancing(bitmap_set, even_counter & 8);
        
        if(false) {
            // We need the balancing state machine to update here so that
            // it updates in sync with the BMB sends. Otherwise the actual balancing won't be applied for as long as the state machine expects.
            balancing_sm_tick(&model);

            bmb3y_send_balancing(&model);
        }
    }

    // if((timestep() & 0x7f) == 31) {
    //     uint8_t rx_buf[100];

    //     bmb3y_wakeup_blocking();
    //     bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
    //     bmb3y_send_command_blocking(BMB3Y_CMD_SNAPSHOT);

    //     sleep_us(100);

    //     bmb3y_get_data_blocking(BMB3Y_CMD_READ_CONFIG, rx_buf, 50);
    //     for(int i=0; i<50; i++) {
    //         printf("%02X ", rx_buf[i]);
    //     }
    //     printf("\n");
    //     isosnoop_print_buffer();
    // }


    if((timestep() & 0x3f) == 32) {
        //isosnoop_print_buffer();
        for(int i=0; i<15; i++) {
            printf("[c%3d]: %4d mV | ", i, model.cell_voltages_mV[i]);
        }
        printf("\n");
        //printf("Bal mask: %02X %02X\n", bitmap_set[14], bitmap_set[15]);
    }

    if((timestep() & 31) == 0) {
        // every 64 ticks, output stuff
        //isosnoop_print_buffer();
        print_bms_events();

        printf("Temp: %3ld dC | 3V3: %4ld mV | 5V: %4ld mV | 12V: %5ld mV | CtrV: %5ld mV\n",
            get_temperature_c_times10(),
            internal_adc_read_3v3_mv(),
            internal_adc_read_5v_mv(),
            internal_adc_read_12v_mv(),
            internal_adc_read_contactor_mv()
        );

        printf("Batt: %6ldmV (%3ldmV) | Out: %6ldmV (%3ldmV) | NegCtr: %6dmV (%3ldmV) | PosCtr: %6dmV (%3ldmV)\n",
            model.battery_voltage_mV,
            model.battery_voltage_range_mV,
            model.output_voltage_mV,
            model.output_voltage_range_mV,
            model.neg_contactor_voltage_mV,
            model.neg_contactor_voltage_range_mV,
            model.pos_contactor_voltage_mV,
            model.pos_contactor_voltage_range_mV
        );
        int64_t charge_mC = (model.charge_raw * 132736) / 1000000;
        printf("Current: %6ld mA | Charge: %lld mC | SoC: %2.2f %%\n",
            model.current_mA,
            charge_mC,
            model.soc / 100.0f

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
        printf("Warning: loop overran (%ld ms)\n", delta);
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

    while(true) {
        tick();
        synchronize_time();
    }

    return 0;
}
