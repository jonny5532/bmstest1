#include "hw/chip/pwm.h"
#include "hw/chip/time.h"
#include "hw/chip/watchdog.h"
#include "hw/comms/duart.h"
#include "hw/comms/hmi_serial.h"
#include "hw/comms/internal_serial.h"
#include "hw/sensors/ina228.h"
#include "hw/sensors/internal_adc.h"
#include "hw/sensors/ads1115.h"
#include "hw/checks.h"
#include "hw/pins.h"
#include "estimators/ekf.h"
#include "estimators/estimators.h"
#include "calibration/offline.h"
#include "state_machines/contactors.h"
#include "battery/balancing.h"
#include "battery/safety_checks.h"
#include "inverter/inverter.h"
#include "monitoring/events.h"
#include "limits.h"
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
    const int32_t full_scale_mv = 436000;

    int32_t battery_voltage_mul = model->battery_voltage_mul ? model->battery_voltage_mul : 54500;
    model->battery_voltage_millis = ads1115_get_sample_millis(0);
    model->battery_voltage_mV = ads1115_scaled_sample(0, battery_voltage_mul);//(int32_t)((float)full_scale_mv * 1.0011809966896306f));
    model->battery_voltage_range_mV = ads1115_scaled_sample_range(0, full_scale_mv);

    int32_t output_voltage_mul = model->output_voltage_mul ? model->output_voltage_mul : 54500;
    millis_t output_voltage_millis = ads1115_get_sample_millis(1);
    int32_t output_voltage_mV = ads1115_scaled_sample(1, output_voltage_mul);//(int32_t)((float)full_scale_mv  * 0.99217974180734856007f));

    // arbitrary scaling to remove error 
    //output_voltage_mV = (output_voltage_mV * 58721) / 59496;

    model->output_voltage_millis = output_voltage_millis;
    model->output_voltage_mV = output_voltage_mV;
    model->output_voltage_range_mV = ads1115_scaled_sample_range(1, full_scale_mv);

    int32_t neg_contactor_mul = model->neg_contactor_mul ? model->neg_contactor_mul : 54500;
    model->neg_contactor_voltage_millis = ads1115_get_sample_millis(2);
    model->neg_contactor_voltage_mV = ads1115_scaled_sample(2, neg_contactor_mul) + model->neg_contactor_offset_mV;
    model->neg_contactor_voltage_range_mV = ads1115_scaled_sample_range(2, full_scale_mv);

    // Positive contactor voltage has to be derived from the difference between (battery+
    // to output-) and (output+ to output-), since we can't sample relative to battery+.

    int32_t pos_contactor_mul = model->pos_contactor_mul ? model->pos_contactor_mul : 54500;
    millis_t raw_bat_pos_to_out_neg_millis = ads1115_get_sample_millis(3);
    int32_t raw_bat_plus_to_out_neg_mV = ads1115_scaled_sample(3, pos_contactor_mul);
    model->pos_contactor_voltage_millis = raw_bat_pos_to_out_neg_millis < output_voltage_millis ? raw_bat_pos_to_out_neg_millis : output_voltage_millis;
    model->pos_contactor_voltage_mV = raw_bat_plus_to_out_neg_mV - output_voltage_mV;
    model->pos_contactor_voltage_range_mV = ads1115_scaled_sample_range(3, full_scale_mv)/2 + ads1115_scaled_sample_range(1, full_scale_mv)/2;

        
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
        if(!ina228_read_current_blocking(&ina228_dev)) {
            printf("INA228 current read failed\n");
        }
    }

    /* Read supply voltages */

    model->supply_voltage_3V3_mV = internal_adc_read_3v3_mv();
    model->supply_voltage_3V3_millis = internal_adc_read_3v3_millis();
    model->supply_voltage_5V_mV = internal_adc_read_5v_mv();
    model->supply_voltage_5V_millis = internal_adc_read_5v_millis();
    model->supply_voltage_12V_mV = internal_adc_read_12v_mv();
    model->supply_voltage_12V_millis = internal_adc_read_12v_millis();
    model->supply_voltage_contactor_mV = internal_adc_read_contactor_mv();
    model->supply_voltage_contactor_millis = internal_adc_read_contactor_millis();

    model->estop_pressed = gpio_get(PIN_ESTOP);
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

    read_inputs(&model);
    model.cell_voltage_slow_mode = true;
    bmb3y_tick(&model);

    // For debugging, prepare for restart (zero current) if 'R' received on USB stdio
    if(stdio_getchar_timeout_us(0) == 'R') {
        printf("Preparing to restart due to 'R' on USB stdio\n");
        count_bms_event(ERR_RESTARTING, 1);
    }

    // Phase 2: Update model

    static int32_t last_charge_raw = 0;
    uint32_t soc = kalman_update(
        raw_charge_to_mC(model.charge_raw - last_charge_raw),
        model.current_mA,
        //model.battery_voltage_mV
        model.cell_voltage_total_mV / NUM_CELLS
    );
    if(soc != 0xFFFFFFFF) {
        model.soc = (uint16_t)soc;
        model.soc_millis = millis();
    }
    last_charge_raw = model.charge_raw;

    static int32_t last_charge_raw2 = 0;
    static millis_t last_ekf_update_millis = 0;
    millis_t now = millis();
    if(now - last_ekf_update_millis >= 1000) {
        uint32_t soc2 = ekf_tick(
            raw_charge_to_mC(model.charge_raw - last_charge_raw2),
            model.current_mA,
            model.cell_voltage_total_mV / NUM_CELLS
        );
        if(soc2 != 0xFFFFFFFF) {
            model.soc_ekf = (uint16_t)soc2;
        }
        last_charge_raw2 = model.charge_raw;
        last_ekf_update_millis = now;
    }

    model.soc_voltage_based = voltage_based_soc_estimate(&model);
    model.soc_basic_count = basic_count_soc_estimate(&model);
    model.soc_fancy_count = fancy_count_soc_estimate(&model);


    model_tick(&model);
    confirm_battery_safety(&model);
    confirm_hardware_integrity(&model);

    // Phase 3: Outputs and communications

    events_tick();
    system_sm_tick(&model);
    contactor_sm_tick(&model);
    offline_calibration_sm_tick(&model);

    inverter_tick(&model);
    internal_serial_tick();
    hmi_serial_tick(&model);


    if((timestep() & 0x3f) == 32) {
        //isosnoop_print_buffer();
        uint32_t total = 0;
        for(int i=0; i<15; i++) {
            printf("[c%3d]: %4d mV | ", i, model.cell_voltages_mV[i]);
            total += model.cell_voltages_mV[i];
            if((i % 5) == 4) {
                printf("\n");
            }
        }
        printf("Total: %lu mV | Temps: %ddC - %ddC\n\n", total, model.temperature_min_dC, model.temperature_max_dC);

        //printf("Bal mask: %02X %02X\n", bitmap_set[14], bitmap_set[15]);
    }

    if((timestep() & 0x3f) == 32) {
        // every 64 ticks, output stuff
        //isosnoop_print_buffer();
        print_bms_events();

        printf("Temp: %3ld dC | 3V3: %4ld mV | 5V: %4ld mV | 12V: %5ld mV | CtrV: %5ld mV\n",
            get_temperature_c_times10(),
            model.supply_voltage_3V3_mV,
            model.supply_voltage_5V_mV,
            model.supply_voltage_12V_mV,
            model.supply_voltage_contactor_mV
        );

        printf("Batt: %6ldmV (%3ldmV) | Out: %6ldmV (%3ldmV) | NegCtr: %6ldmV (%3ldmV) | PosCtr: %6ldmV (%3ldmV)\n",
            model.battery_voltage_mV,
            model.battery_voltage_range_mV,
            model.output_voltage_mV,
            model.output_voltage_range_mV,
            model.neg_contactor_voltage_mV,
            model.neg_contactor_voltage_range_mV,
            model.pos_contactor_voltage_mV,
            model.pos_contactor_voltage_range_mV
        );
        int64_t charge_mC = raw_charge_to_mC(model.charge_raw);
        printf("Current: %6ld mA | Charge: %lld mC | SoC: %2.2f %% | SoC(VB): %2.2f %% | SoC(BC): %2.2f %% | SoC(FC): %2.2f %% | SoC(EKF): %2.2f %%\n\n",
            model.current_mA,
            charge_mC,
            model.soc / 100.0f,
            model.soc_voltage_based / 100.0f,
            model.soc_basic_count / 100.0f,
            model.soc_fancy_count / 100.0f,
            model.soc_ekf / 100.0f
        );
    }
}

void synchronize_time() {
    uint32_t prev = millis();
    update_millis();

    int32_t delta = millis() - prev;
    if(delta <= 20) {
        sleep_ms(20 - delta);
    } else if(model.ignore_missed_deadline) {
        printf("Notice: loop overran but ignored (%ld ms)\n", delta);
    } else {
        // took too long!
        printf("Warning: loop overran (%ld ms)\n", delta);
        count_bms_event(ERR_LOOP_OVERRUN, delta);
    }
    model.ignore_missed_deadline = false;
    update_millis();
    update_timestep();
}

int main() {
    stdio_usb_init();

    // Give a chance for USB to enumerate before we start printing
    sleep_ms(1000);

    multicore_launch_core1(core1_entry);

    init();

    while(true) {
        tick();
        synchronize_time();
    }

    return 0;
}
