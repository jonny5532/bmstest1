#include "cli.h"
#include "app/estimators/ekf.h"
#include "app/model.h"
#include "app/state_machines/system.h"
#include "drivers/sensors/internal_adc.h"
#if CAN_MASTER
#include "protocols/inverter/can_master.h"
#endif
#include "sys/events/events.h"
#include "sys/logging/logging.h"

#include "pico/stdlib.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>

extern float working_charge_internal_resistance_uR;
extern float working_charge_ceiling_dA;
extern float working_charge_slow_alpha;
extern float working_charge_fast_alpha;
extern float working_charge_slow_threshold_dA;
extern float working_charge_fast_threshold_dA;



static char line_buf[120];
static uint8_t line_buf_idx = 0;

static void cli_handle_command(const char *cmd) {
    // TODO - implement commands
    debug_printf("Executing command: '%s'\n", cmd);

    if(strcmp(cmd, "restart") == 0) {
        debug_printf("Requesting inactive mode\n");
        model.system_req = SYSTEM_REQUEST_STOP;
    } else if (strncmp(cmd, "set ", 4) == 0) {
        char name[64];
        float value;
        if (sscanf(cmd + 4, "%63s %f", name, &value) == 2) {
            if (strcmp(name, "working_charge_internal_resistance_uR") == 0) {
                working_charge_internal_resistance_uR = value;
                debug_printf("Set working_charge_internal_resistance_uR to %f\n", value);
            } else if (strcmp(name, "working_charge_ceiling_dA") == 0) {
                working_charge_ceiling_dA = value;
                debug_printf("Set working_charge_ceiling_dA to %f\n", value);
            } else if (strcmp(name, "working_charge_slow_alpha") == 0) {
                working_charge_slow_alpha = value;
                debug_printf("Set working_charge_slow_alpha to %f\n", value);
            } else if (strcmp(name, "working_charge_fast_alpha") == 0) {
                working_charge_fast_alpha = value;
                debug_printf("Set working_charge_fast_alpha to %f\n", value);
            } else if (strcmp(name, "working_charge_slow_threshold_dA") == 0) {
                working_charge_slow_threshold_dA = value;
                debug_printf("Set working_charge_slow_threshold_dA to %f\n", value);
            } else if (strcmp(name, "working_charge_fast_threshold_dA") == 0) {
                working_charge_fast_threshold_dA = value;
                debug_printf("Set working_charge_fast_threshold_dA to %f\n", value);
            } else if (strcmp(name, "ekf_q0") == 0) {
                model.ekf.Q[0] = value;
                debug_printf("Set ekf_q0 to %f\n", value);
            } else if (strcmp(name, "ekf_q1") == 0) {
                model.ekf.Q[1] = value;
                debug_printf("Set ekf_q1 to %f\n", value);
            } else if (strcmp(name, "ekf_q2") == 0) {
                model.ekf.Q[2] = value;
                debug_printf("Set ekf_q2 to %f\n", value);
            } else if (strcmp(name, "ekf_r") == 0) {
                model.ekf.R = value;
                debug_printf("Set ekf_r to %f\n", value);
            } else if (strcmp(name, "ekf_r0") == 0) {
                model.ekf.R0 = value;
                debug_printf("Set ekf_r0 to %f\n", value);
            } else if (strcmp(name, "ekf_r1") == 0) {
                model.ekf.R1 = value;
                debug_printf("Set ekf_r1 to %f\n", value);
            } else if (strcmp(name, "ekf_c1") == 0) {
                model.ekf.C1 = value;
                debug_printf("Set ekf_c1 to %f\n", value);
            } else {
                debug_printf("Unknown parameter: %s\n", name);
            }
        } else {
            debug_printf("Invalid set command format. Use: set <name> <val>\n");
        }
    } else if(strcmp(cmd, "calibrate") == 0) {
        debug_printf("Requesting contactor calibration (offline long)...\n");
        model.system_req = SYSTEM_REQUEST_CALIBRATE_OFFLINE_LONG;
    } else if(strcmp(cmd, "calibrate_short") == 0) {
        debug_printf("Requesting contactor calibration (offline short)...\n");
        model.system_req = SYSTEM_REQUEST_CALIBRATE_OFFLINE_SHORT;
    } else if(strcmp(cmd, "calibrate_online") == 0) {
        debug_printf("Requesting online calibration...\n");
        model.system_req = SYSTEM_REQUEST_CALIBRATE_ONLINE;
    } else if(strcmp(cmd, "toggle") == 0) {
        if(model.system_sm.state == SYSTEM_STATE_INACTIVE) {
            debug_printf("Requesting operation mode\n");
            model.system_req = SYSTEM_REQUEST_RUN;
        } else if(model.system_sm.state == SYSTEM_STATE_OPERATING) {
            debug_printf("Requesting inactive mode\n");
            model.system_req = SYSTEM_REQUEST_STOP;
        }
    } else if(strcmp(cmd, "print_ekf_state") == 0) {
        debug_printf("EKF Constants: Q=[%e, %e, %e], R=%f, R0=%f, R1=%f, C1=%f\n",
                     model.ekf.Q[0], model.ekf.Q[1], model.ekf.Q[2],
                     model.ekf.R, model.ekf.R0, model.ekf.R1, model.ekf.C1);
        debug_printf("EKF State: Ah_used=%.4f Ah, V_c1=%.4f V, Capacity=%.2f Ah\n",
           model.ekf.x[0], model.ekf.x[1], model.ekf.x[2]);
    } else if(strcmp(cmd, "adc") == 0) {
        // printf directly (not debug_printf) so the response is visible even
        // when the buffered logger is saturated by other errors
        millis_t now = millis();
        printf("ADC: now=%lu ms | Temp: %ld dC\n", now, get_temperature_c_times10());
        printf("ADC: 3V3: %4ld mV (age %lu ms)\n",
            internal_adc_read_3v3_mv(), now - internal_adc_read_3v3_millis());
        printf("ADC: 5V:  %4ld mV (age %lu ms)\n",
            internal_adc_read_5v_mv(), now - internal_adc_read_5v_millis());
        printf("ADC: 12V: %5ld mV (age %lu ms)\n",
            internal_adc_read_12v_mv(), now - internal_adc_read_12v_millis());
        printf("ADC: Ctr: %5ld mV (age %lu ms)\n",
            internal_adc_read_contactor_mv(), now - internal_adc_read_contactor_millis());
#if CAN_MASTER
    } else if(strcmp(cmd, "fleet") == 0) {
        can_master_print_status();
#endif
    } else if(strcmp(cmd, "print_working_charge_settings") == 0) {
        debug_printf("working_charge_internal_resistance_uR = %f\n", working_charge_internal_resistance_uR);
        debug_printf("working_charge_ceiling_dA = %f\n", working_charge_ceiling_dA);
        debug_printf("working_charge_slow_alpha = %f\n", working_charge_slow_alpha);
        debug_printf("working_charge_fast_alpha = %f\n", working_charge_fast_alpha);
        debug_printf("working_charge_slow_threshold_dA = %f\n", working_charge_slow_threshold_dA);
        debug_printf("working_charge_fast_threshold_dA = %f\n", working_charge_fast_threshold_dA);
    }
}

void cli_tick() {
    int c = stdio_getchar_timeout_us(0);
    if(c == PICO_ERROR_TIMEOUT) {
        return;
    }

    if(c == '\n' || c == '\r') {
        line_buf[line_buf_idx] = '\0';
        debug_printf("Received command: '%s'\n", line_buf);
        cli_handle_command(line_buf);
        line_buf_idx = 0;
    } else if(c == '\b') {
        if(line_buf_idx > 0) {
            line_buf_idx--;
        }
    } else if(line_buf_idx < sizeof(line_buf) - 1) {
        line_buf[line_buf_idx++] = (char)c;
    }






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
    //     model.system_req = SYSTEM_REQUEST_CALIBRATE;
    // }

}