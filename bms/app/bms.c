#include "app/monitoring/counters.h"
#include "drivers/bmb3y/bmb3y.h"
#include "drivers/chip/nvm.h"
#include "drivers/chip/pwm.h"
#include "drivers/chip/watchdog.h"
#include "drivers/sensors/ina228.h"
#include "drivers/sensors/internal_adc.h"
#include "drivers/sensors/ads1115.h"
#include "hardware_checks.h"
#include "config/limits.h"
#include "config/pins.h"
#include "estimators/ekf.h"
#include "estimators/estimators.h"
#include "calibration/calibration.h"
#include "state_machines/contactors.h"
#include "battery/balancing.h"
#include "battery/safety_checks.h"
#include "protocols/cli/cli.h"
#include "protocols/hmi_serial/hmi_serial.h"
#include "protocols/internal_serial/internal_serial.h"
#include "protocols/inverter/inverter.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "sys/time/time.h"
#include "model.h"


#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "../vendor/littlefs/lfs.h"

#include <stdio.h>

void read_supply_voltages(supply_voltages_t *supply_voltages) {
    supply_voltages->voltage_3V3_mV = internal_adc_read_3v3_mv();
    supply_voltages->voltage_3V3_millis = internal_adc_read_3v3_millis();
    supply_voltages->voltage_5V_mV = internal_adc_read_5v_mv();
    supply_voltages->voltage_5V_millis = internal_adc_read_5v_millis();
    supply_voltages->voltage_12V_mV = internal_adc_read_12v_mv();
    supply_voltages->voltage_12V_millis = internal_adc_read_12v_millis();
    supply_voltages->voltage_contactor_mV = internal_adc_read_contactor_mv();
    supply_voltages->voltage_contactor_millis = internal_adc_read_contactor_millis();
}

// should this go?
void read_inputs(bms_model_t *model) {
    // Read various inputs into the model. These are mostly gathered asynchronously 
    // so this is a quick process.

    // ADS1115 voltage readings

    // For differential readings, full scale should be 436V
    // const int32_t full_scale_mv = 436000;

    float battery_voltage_mul = fabs(model->battery_voltage_mul) > 0.001f ? model->battery_voltage_mul : ADS1115_DEFAULT_MUL;
    model->high_voltages.battery_millis = ads1115_get_sample_millis(0);
    model->high_voltages.battery = ads1115_float_sample(0, battery_voltage_mul);
    model->high_voltages.battery_deviation = ads1115_float_deviation(0, battery_voltage_mul);

    float output_voltage_mul = fabs(model->output_voltage_mul) > 0.001f ? model->output_voltage_mul : ADS1115_DEFAULT_MUL;
    model->high_voltages.output_millis = ads1115_get_sample_millis(1);
    model->high_voltages.output = ads1115_float_sample(1, output_voltage_mul);
    model->high_voltages.output_deviation = ads1115_float_deviation(1, output_voltage_mul);

    float neg_contactor_mul = fabs(model->neg_contactor_mul) > 0.001f ? model->neg_contactor_mul : ADS1115_DEFAULT_MUL;
    model->high_voltages.neg_contactor_millis = ads1115_get_sample_millis(2);
    model->high_voltages.neg_contactor = ads1115_float_sample(2, neg_contactor_mul) + (0.001f * model->neg_contactor_offset_mV);
    model->high_voltages.neg_contactor_deviation = ads1115_float_deviation(2, neg_contactor_mul);

    float pos_contactor_mul = fabs(model->pos_contactor_mul) > 0.001f ? model->pos_contactor_mul : ADS1115_DEFAULT_MUL;

#if BMS_BOARD == BMS_BOARD_REV1
    // Rev1 measures directly across the pos (Link Positive) contactor via
    // ADC B channel 6 (BatPos - LinkPos)
    model->high_voltages.pos_contactor_millis = ads1115_get_sample_millis(6);
    model->high_voltages.pos_contactor = ads1115_float_sample(6, pos_contactor_mul) + (0.001f * model->pos_contactor_offset_mV);
    model->high_voltages.pos_contactor_deviation = ads1115_float_deviation(6, pos_contactor_mul);

    // Link rail voltage (channel 7 measures LinkNeg - LinkPos, so negate)
    float link_voltage_mul = fabs(model->link_voltage_mul) > 0.001f ? model->link_voltage_mul : ADS1115_DEFAULT_MUL;
    model->high_voltages.link_millis = ads1115_get_sample_millis(7);
    model->high_voltages.link = -ads1115_float_sample(7, link_voltage_mul);
    model->high_voltages.link_deviation = ads1115_float_deviation(7, link_voltage_mul);

    // Drop across the F4 fuse / drive-unit jumper (channel 8, OutPos - LinkPos)
    float fuse_drop_mul = fabs(model->fuse_drop_mul) > 0.001f ? model->fuse_drop_mul : ADS1115_DEFAULT_MUL;
    model->high_voltages.fuse_drop_millis = ads1115_get_sample_millis(8);
    model->high_voltages.fuse_drop = ads1115_float_sample(8, fuse_drop_mul);
    model->high_voltages.fuse_drop_deviation = ads1115_float_deviation(8, fuse_drop_mul);
#else
    // Positive contactor voltage has to be derived from the difference between (battery+
    // to output-) and (output+ to output-), since we can't sample relative to battery+.

    millis_t raw_bat_pos_to_out_neg_millis = ads1115_get_sample_millis(3);
    float raw_bat_plus_to_out_neg = ads1115_float_sample(3, pos_contactor_mul);
    
    model->high_voltages.pos_contactor_millis = raw_bat_pos_to_out_neg_millis < model->high_voltages.output_millis ? raw_bat_pos_to_out_neg_millis : model->high_voltages.output_millis;
    model->high_voltages.pos_contactor = raw_bat_plus_to_out_neg - model->high_voltages.output;
    model->high_voltages.pos_contactor_deviation = ads1115_float_deviation(3, pos_contactor_mul) + model->high_voltages.output_deviation;
#endif
    
    // INA228 current and charge readings
    
    //if((timestep() & 0x7) == 0) {
        // Only query every 160ms or so (readings are available every 531ms but
        // we want to be sure we don't miss any). This is currently blocking due
        // to the I2C transaction but doesn't have to wait for the reading
        // itself.

        // This reads current, and also updates the charge accumulator if it is
        // a new reading

        extern ina228_t ina228_dev;
        ina228_read_current_async(&ina228_dev);
    //}

    /* Read supply voltages */

    read_supply_voltages(&model->supply_voltages);

    model->estop_pressed = gpio_get(PIN_ESTOP);

#ifdef PRECHARGE_ON_NEGATIVE
    model->precharge_closed = gpio_get(PIN_AUX_CONTACTOR_FC_NEG);
#else
    model->precharge_closed = gpio_get(PIN_AUX_CONTACTOR_FC_POS);
#endif
    //printf("Pre: %d\n", model->precharge_closed);
}

uint32_t timings[10];
uint32_t max_timings[10];

void bms_tick() {
    // The main tick function

    // Phase 0: Preamble

    watchdog_update();
    internal_adc_check();

    if(timestep() & 32) {
        gpio_put(PIN_LED, true);
    } else {
        gpio_put(PIN_LED, false);
    }

    timings[0] = time_us_32();

    // Phase 1: Read sensors

    read_inputs(&model);

    timings[1] = time_us_32();

    //model.cell_voltage_slow_mode = true;
    bmb3y_tick(&model);

    timings[2] = time_us_32();
    

    // For debugging, prepare for restart (zero current) if 'R' received on USB stdio

    // int c = stdio_getchar_timeout_us(0);
    // if(c != PICO_ERROR_TIMEOUT) {
    //     debug_printf("Received char '%c' on USB stdio\n", c);
    // }
    // if(c == 'R') {
    //     debug_printf("Preparing to restart due to 'R' on USB stdio\n");
    //     count_bms_event(ERR_RESTARTING, 1);
    // } else if(c == 'T') {
    //     // Toggle system operational state
    //     if(model.system_sm.state == SYSTEM_STATE_INACTIVE) {
    //         debug_printf("Requesting operation mode\n");
    //         model.system_req = SYSTEM_REQUEST_RUN;
    //     } else if(model.system_sm.state == SYSTEM_STATE_OPERATING) {
    //         debug_printf("Requesting inactive mode\n");
    //         model.system_req = SYSTEM_REQUEST_STOP;
    //     }
    // } else if(c == 'C') {
    //     debug_printf("Requesting contactor calibration\n");
    //     model.system_req = SYSTEM_REQUEST_CALIBRATE_OFFLINE;
    // }

    cli_tick();

    timings[3] = time_us_32();

    // Phase 2: Update model

    static int32_t last_charge_raw = 0;
    millis_t now = millis();
    if(now - model.soc_millis >= 1000) {
        uint32_t soc = ekf_tick(
            &model,
            raw_charge_to_Ah(model.charge_raw - last_charge_raw),
            model.current_mA,
            model.cell_voltage_total_mV / NUM_CELLS
        );
        if(soc != 0xFFFFFFFF) {
            model.soc = (uint16_t)soc;
            model.soc_millis = now;
        }
        // Truncates, but tick-to-tick changes will be small
        last_charge_raw = model.charge_raw;
    }


    model.soc_voltage_based = voltage_based_soc_estimate(&model);
    model.soc_basic_count = basic_count_soc_estimate(&model);
    model.soc_fancy_count = fancy_count_soc_estimate(&model);

    timings[4] = time_us_32();

    model_tick(&model);

    timings[5] = time_us_32();

    // Phase 3: Checks

    confirm_battery_safety(&model);
    confirm_hardware_integrity(&model);
    events_tick();

    timings[6] = time_us_32();

    // Phase 4: Update state machines

    system_sm_tick(&model);
    contactor_sm_tick(&model);
    calibration_sm_tick(&model);

    timings[7] = time_us_32();

    // Phase 5: Comms

    nvm_tick(&model);
    inverter_tick(&model.inverter_outputs);
    internal_serial_tick();
    hmi_serial_tick(&model);

    timings[8] = time_us_32();

    // Phase 6: Debug output

    if((timestep() & 0x3f) == 32) {
        debug_printf("\033[2J\033[H"); // Clear terminal

        //isosnoop_print_buffer();
        uint32_t total = 0;
        for(int i=0; i<NUM_CELLS; i++) {
            debug_printf("[c%3d]: %4d mV | ", i, (uint32_t)model.raw_cell_voltages_mV[i]);
            total += model.raw_cell_voltages_mV[i];
            if((i % 5) == 4) {
                debug_printf("\n");
            }
        }
        debug_printf("Total: %dmV | Temps: %.1fC - %.1fC | Delta: %d mV %s%s\n\n", 
            total, 
            model.temperature_min, model.temperature_max, 
            (int)(model.cell_voltage_max_mV - model.cell_voltage_min_mV),
            model.cell_voltage_slow_mode ? " | Slow" : "",
            model.balancing_active ? " | Balancing" : ""
        );

        //printf("Bal mask: %02X %02X\n", bitmap_set[14], bitmap_set[15]);
    }

    if((timestep() & 0x3f) == 32) {
        // every 64 ticks, output stuff
        //isosnoop_print_buffer();
        print_bms_events();
        
        debug_printf("MCU: %3ld dC | 3V3: %4ld mV | 5V: %4ld mV | 12V: %5ld mV | CtrV: %5ld mV\n",
            get_temperature_c_times10(),
            model.supply_voltages.voltage_3V3_mV,
            model.supply_voltages.voltage_5V_mV,
            model.supply_voltages.voltage_12V_mV,
            model.supply_voltages.voltage_contactor_mV
        );

        {
            millis_t now = millis();
            debug_printf("ModTemps:");
            for(int i=0; i<NUM_MODULE_TEMPS; i++) {
                debug_printf(" [%d]=%.1fC", i, model.module_temperatures[i]);
            }
            debug_printf(" (age %lums) | ShuntDie: %.1fC (age %lums) | ShuntNTC: %.1fC / %.0f ohm (age %lums)\n",
                model.module_temperatures_millis ? (now - model.module_temperatures_millis) : 0,
                model.shunt_die_temperature,
                model.shunt_die_temperature_millis ? (now - model.shunt_die_temperature_millis) : 0,
                model.shunt_ntc_temperature,
                model.shunt_ntc_resistance_ohms,
                model.shunt_ntc_millis ? (now - model.shunt_ntc_millis) : 0
            );
        }

        debug_printf("Batt: %6.3fV (%3.3f) | Out: %6.3fV (%3.3f) | NegCtr: %6.3fV (%3.3f) | PosCtr: %6.3fV (%3.3f)\n",
            model.high_voltages.battery,
            model.high_voltages.battery_deviation,
            model.high_voltages.output,
            model.high_voltages.output_deviation,
            model.high_voltages.neg_contactor,
            model.high_voltages.neg_contactor_deviation,
            model.high_voltages.pos_contactor,
            model.high_voltages.pos_contactor_deviation
        );
#if BMS_BOARD == BMS_BOARD_REV1
        debug_printf("Link: %6.3fV (%3.3f) | Fuse: %6.3fV (%3.3f)\n",
            model.high_voltages.link,
            model.high_voltages.link_deviation,
            model.high_voltages.fuse_drop,
            model.high_voltages.fuse_drop_deviation
        );
#endif
        int64_t charge_mC = raw_charge_to_mC(model.charge_raw);
        debug_printf("Current: %6ld mA | Charge: %lld mC | SoC: %2.2f %% | SoC(VB): %2.2f %% | SoC(BC): %2.2f %% | SoC(FC): %2.2f %%\n",
            model.current_mA,
            charge_mC,
            model.soc / 100.0f,
            model.soc_voltage_based / 100.0f,
            model.soc_basic_count / 100.0f,
            model.soc_fancy_count / 100.0f
        );
        debug_printf("Current filtered: %6f mA (%f)\n",
            model.current_filtered_mA,
            model.current_deviation
        );
        debug_printf("CV: %2.1f/%2.1f A | W: %2.1f A | T: %2.1f/%2.1f A | Inv. min: %2.1f V | Inv. max: %2.1f V\n\n",
            model.cell_voltage_charge_current_limit_dA / 10.0f,
            model.cell_voltage_discharge_current_limit_dA / 10.0f,
            model.working_charge_current_limit_dA / 10.0f,
            model.temp_charge_current_limit_dA / 10.0f,
            model.temp_discharge_current_limit_dA / 10.0f,
            model.inverter_outputs.min_voltage_limit_dV / 10.0f,
            model.inverter_outputs.max_voltage_limit_dV / 10.0f
        );

        /*
        char ascii[127-33+1];
        for(int i=33; i<=126; i++) {
            ascii[i-33] = (char)i;
        }
        for(int i=0;i<30;i++) {
            debug_printf("%.*s\n", 126-33+1, ascii);
        }
        */

        // printf("DUART0 RX: %lu, crcfail %lu | DUART1 RX: %lu, crcfail %lu\n",
        //     debug_counters.uart0_packets_received, debug_counters.uart0_crc_errors,
        //     debug_counters.uart1_packets_received, debug_counters.uart1_crc_errors
        // );

        float cal_midpoint = 22496.4f;
        float cal_midpoint_temp_dC = 285.0f;

        cal_midpoint += (get_temperature_c_times10() - cal_midpoint_temp_dC) * 0.28f; //compensate for temp drift

        float vp = ads1115_float_sample(4, 1.0f);
        float vn = ads1115_float_sample(5, 1.0f);
        debug_printf("midpoint is %1.1f | ", (vp + vn)/2.0f);

        float vpos = (vp - cal_midpoint) * (4.096f/32768);
        float vneg = (cal_midpoint - vn) * (4.096f/32768);
        float r = vpos / vneg;
        // Cap at 1Gohm
        float r_iso = 10000000 * fminf(r / fmaxf(fabsf(1.0f - r), 0.009f), 100.0f);

        debug_printf("Vpos: %1.3f V | Vneg: %1.3f V | Riso: %1.2f Mohm\n",
            vpos, vneg, r_iso/1000000
        );

    }

    timings[9] = time_us_32();

    for(int i=0; i<9; i++) {
        uint32_t delta = timings[i+1] - timings[i];
        if(delta > max_timings[i]) {
            max_timings[i] = delta;
        }
    }

    if((timestep() & 0x3ff) == 33) {
        debug_printf("Max timings (us): ");
        for(int i=0; i<9; i++) {
            debug_printf("%lu ", max_timings[i]);
            max_timings[i] = 0;
        }
        debug_printf("\n");
    }

}

uint32_t average_loop_time_256_ms = 0;
uint32_t loop_counter = 0;

void synchronize_time() {
    uint32_t prev = millis();
    update_millis();

    int32_t delta = millis() - prev;
    if(delta <= 20) {
        sleep_ms(20 - delta);
    } else if(model.ignore_missed_deadline) {
        debug_printf("Notice: loop overran but ignored (%ld ms)\n", delta);
    } else {
        // took too long!
        debug_printf("Warning: loop overran (%ld ms)\n", delta);
        count_bms_event(ERR_LOOP_OVERRUN, delta);
    }
    model.ignore_missed_deadline = false;
    update_millis();
    update_timestep();

    // average_loop_time_256_ms += delta;
    // loop_counter++;
    // if(loop_counter >= 256) {
    //     printf("Average loop time over last 256 ticks: %lu/256 ms\n", average_loop_time_256_ms);
    //     average_loop_time_256_ms = 0;
    //     loop_counter = 0;
    // }
}
