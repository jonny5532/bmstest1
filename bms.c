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
    // Read various inputs into the model. These are all gathered asynchronously 
    // so this is a quick process.


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

uint8_t bitmap_on[16] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                      0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t bitmap_set[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

int16_t balance_counters[120] = {0};
// mapping is a bit strange:
// 0x01 in last byte is cell 0
// 0x0f in last byte is cells 0,1,2,3
// 0xf0 is cell 4,5,6,7
// 0xf00 is cell 8,9,10,11
// 0x7000 is cell 12,13,14
// no idea how the next module maps...
    
uint8_t bitmap_off[16] = {0};
int even_counter = 0;

void tick() {
    // The main tick function

    watchdog_update();

    model.contactor_req = CONTACTORS_REQUEST_CLOSE;
    contactor_sm_tick(&model);

    if(timestep() & 32) {
        gpio_put(PIN_LED, true);
    } else {
        gpio_put(PIN_LED, false);
    }

    read_inputs(&model);
    model_tick(&model);
    inverter_tick(&model);
    internal_serial_tick();

    if(timestep() == 31) {
        //bmb3y_wakeup_blocking();
        // uint32_t start = time_us_32();
        // isospi_send_command_blocking(BMB3Y_CMD_SNAPSHOT, true);
        // uint32_t end = time_us_32();
        // printf("BMB3Y snapshot took %d us\n", end - start);
    }

    // We talk to the BMB3Y every 64 ticks (about once per second)
    if((timestep() & 0x3f) == 99931) {
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
        
        // We need the balancing state machine to update here so that
        // it updates in sync with the BMB sends. Otherwise the actual balancing won't be applied for as long as the state machine expects.
        balancing_sm_tick(&model);

        bmb3y_send_balancing(&model);
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


    if((timestep() & 0x7f) == 32) {
        //isosnoop_print_buffer();
        for(int i=0; i<15; i++) {
            printf("[c%3d]: %4d mV | ", i, model.cell_voltages_mV[i]);
        }
        printf("\n");
        //printf("Bal mask: %02X %02X\n", bitmap_set[14], bitmap_set[15]);
    }

    // every 164 seconds
    if((timestep() & 0x1fff) == 999930) {
    //if(timestep() == 100) {
        // start balancing
        int16_t min_cell = 0x7FFF;
        for(int i=0; i<15; i++) {
            if(model.cell_voltages_mV[i] < min_cell) {
                min_cell = model.cell_voltages_mV[i];
            }
        }

        for(int i=0; i<15; i++) {
            if(model.cell_voltages_mV[i] > min_cell) {
                balance_counters[i] = model.cell_voltages_mV[i] - min_cell - 3;
            }
        }

        printf("Min cell voltage: %d mV\n", min_cell);
    }

    // every 20 seconds
    if((timestep() & 0x3ff) == 99930) {
        // update balancing bitmap
        bitmap_set[14] = 0;
        bitmap_set[15] = 0;
        for(int i=0; i<15; i++) {
            if(balance_counters[i] > 0) {
                bitmap_set[15 - (i / 8)] |= (1 << (i % 8));
                balance_counters[i] -= 1;
            }
        }
    }

    // if((timestep() & 31) == 0) {
    //     bmb3y_set_balancing(bitmap_set, true);
    // } else if((timestep() & 31) == 16) {
    //     bmb3y_set_balancing(bitmap_set, false);
    // }


    if((timestep() & 63) == 99990) {
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

        printf("Batt: %6ldmV | Out: %6ldmV | NegCtr: %6dmV | PosCtr: %6dmV\n",
            model.battery_voltage_mV,
            model.output_voltage_mV,
            model.neg_contactor_voltage_mV,
            model.pos_contactor_voltage_mV
        );
        printf("Current: %6ld mA | Charge: %lld raw\n\n",
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
