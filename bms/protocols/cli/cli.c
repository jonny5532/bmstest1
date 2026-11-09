#include "cli.h"
#include "app/bms.h"
#include "app/estimators/ekf.h"
#include "app/model.h"
#include "app/state_machines/system.h"
#include "drivers/sensors/internal_adc.h"
#include "drivers/sensors/ads1115.h"
#include "drivers/sensors/ina228.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"

#include "pico/stdlib.h"

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Line-based command interface on USB stdio.
//
// Every command is echoed as "Received command: '<line>'" and finishes with a
// line that is exactly "OK" or "ERR <reason>", so scripts (tools/bms_cmd.py)
// know when the reply is complete. All output goes through the logger so it
// stays in order with the rest of the log.

extern float working_charge_internal_resistance_uR;
extern float working_charge_ceiling_dA;
extern float working_charge_slow_alpha;
extern float working_charge_fast_alpha;
extern float working_charge_slow_threshold_dA;
extern float working_charge_fast_threshold_dA;
extern ina228_t ina228_dev;

// Max characters consumed per tick, so a flood of input can't stall the loop
#define CLI_MAX_CHARS_PER_TICK 128

// Runtime-tunable parameters for set/get/params. Exactly one of f/d is set.
typedef struct {
    const char *name;
    float *f;
    double *d;
} cli_param_t;

static const cli_param_t params[] = {
    {"working_charge_internal_resistance_uR", &working_charge_internal_resistance_uR, NULL},
    {"working_charge_ceiling_dA", &working_charge_ceiling_dA, NULL},
    {"working_charge_slow_alpha", &working_charge_slow_alpha, NULL},
    {"working_charge_fast_alpha", &working_charge_fast_alpha, NULL},
    {"working_charge_slow_threshold_dA", &working_charge_slow_threshold_dA, NULL},
    {"working_charge_fast_threshold_dA", &working_charge_fast_threshold_dA, NULL},
    {"ekf_q0", NULL, &model.ekf.Q[0]},
    {"ekf_q1", NULL, &model.ekf.Q[1]},
    {"ekf_q2", NULL, &model.ekf.Q[2]},
    {"ekf_r", NULL, &model.ekf.R},
    {"ekf_min_slope", NULL, &model.ekf.min_ocv_slope},
    {"ekf_knee_slope", NULL, &model.ekf.knee_ocv_slope},
    {"ekf_r_ohm", NULL, &model.ekf.R_ohm},
    {"ekf_offset_tau", NULL, &model.ekf.offset_tau_s},
};
#define NUM_PARAMS (sizeof(params) / sizeof(params[0]))

static const cli_param_t *find_param(const char *name) {
    for(size_t i = 0; i < NUM_PARAMS; i++) {
        if(strcmp(params[i].name, name) == 0) {
            return &params[i];
        }
    }
    return NULL;
}

static double param_value(const cli_param_t *p) {
    return p->f ? (double)*p->f : *p->d;
}

static void print_param(const cli_param_t *p) {
    info_printf("%s = %g\n", p->name, param_value(p));
}

// Command handlers return NULL on success or a short error reason.
// args is the rest of the line after the command word ("" if none).

static const char *cmd_help(const char *args);

static const char *cmd_status(const char *args) {
    (void)args;
    bms_print_status();
    return NULL;
}

static const char *cmd_params(const char *args) {
    (void)args;
    for(size_t i = 0; i < NUM_PARAMS; i++) {
        print_param(&params[i]);
    }
    return NULL;
}

static const char *cmd_get(const char *args) {
    const cli_param_t *p = find_param(args);
    if(!p) {
        return "unknown parameter (see params)";
    }
    print_param(p);
    return NULL;
}

static const char *cmd_set(const char *args) {
    char name[64];
    const char *space = strchr(args, ' ');
    if(!space || (size_t)(space - args) >= sizeof(name)) {
        return "usage: set <name> <value>";
    }
    memcpy(name, args, space - args);
    name[space - args] = '\0';

    const cli_param_t *p = find_param(name);
    if(!p) {
        return "unknown parameter (see params)";
    }

    // The whole value must parse, and NaN/inf would poison the estimators
    char *end;
    double value = strtod(space + 1, &end);
    if(end == space + 1 || *end != '\0' || !isfinite(value)) {
        return "invalid value";
    }

    if(p->f) {
        *p->f = (float)value;
    } else {
        *p->d = value;
    }
    print_param(p);
    return NULL;
}

static const char *cmd_stop(const char *args) {
    (void)args;
    info_printf("Requesting inactive mode\n");
    model.system_req = SYSTEM_REQUEST_STOP;
    return NULL;
}

static const char *cmd_toggle(const char *args) {
    (void)args;
    if(model.system_sm.state == SYSTEM_STATE_INACTIVE) {
        info_printf("Requesting operation mode\n");
        model.system_req = SYSTEM_REQUEST_RUN;
    } else if(model.system_sm.state == SYSTEM_STATE_OPERATING) {
        info_printf("Requesting inactive mode\n");
        model.system_req = SYSTEM_REQUEST_STOP;
    } else {
        return "can only toggle from inactive or operating";
    }
    return NULL;
}

static const char *cmd_calibrate(const char *args) {
    (void)args;
    info_printf("Requesting contactor calibration (offline long)...\n");
    model.system_req = SYSTEM_REQUEST_CALIBRATE_OFFLINE_LONG;
    return NULL;
}

static const char *cmd_calibrate_short(const char *args) {
    (void)args;
    info_printf("Requesting contactor calibration (offline short)...\n");
    model.system_req = SYSTEM_REQUEST_CALIBRATE_OFFLINE_SHORT;
    return NULL;
}

static const char *cmd_calibrate_online(const char *args) {
    (void)args;
    info_printf("Requesting online calibration...\n");
    model.system_req = SYSTEM_REQUEST_CALIBRATE_ONLINE;
    return NULL;
}

static const char *cmd_print_ekf_state(const char *args) {
    (void)args;
    info_printf("EKF Constants: Q=[%e, %e, %e], R=%f, min_slope=%f, knee_slope=%f, R_ohm=%f, offset_tau=%f\n",
                 model.ekf.Q[0], model.ekf.Q[1], model.ekf.Q[2],
                 model.ekf.R, model.ekf.min_ocv_slope, model.ekf.knee_ocv_slope, model.ekf.R_ohm, model.ekf.offset_tau_s);
    info_printf("EKF State: Ah_used=%.4f Ah, V_off=%.4f V, Capacity=%.2f Ah, Bias=%.1f mA\n",
       model.ekf.x[0], model.ekf.x[1], model.ekf.cap, model.ekf.x[2] * 1000.0f);
    return NULL;
}

static const char *cmd_print_working_charge_settings(const char *args) {
    (void)args;
    for(size_t i = 0; i < NUM_PARAMS; i++) {
        if(strncmp(params[i].name, "working_charge_", 15) == 0) {
            print_param(&params[i]);
        }
    }
    return NULL;
}

static const char *cmd_adc(const char *args) {
    (void)args;
    millis_t now = millis();
    info_printf("ADC: now=%lu ms | Temp: %ld dC\n", now, get_temperature_c_times10());
    info_printf("ADC: 3V3: %4ld mV (age %lu ms)\n",
        internal_adc_read_3v3_mv(), now - internal_adc_read_3v3_millis());
    info_printf("ADC: 5V:  %4ld mV (age %lu ms)\n",
        internal_adc_read_5v_mv(), now - internal_adc_read_5v_millis());
    info_printf("ADC: 12V: %5ld mV (age %lu ms)\n",
        internal_adc_read_12v_mv(), now - internal_adc_read_12v_millis());
    info_printf("ADC: Ctr: %5ld mV (age %lu ms)\n",
        internal_adc_read_contactor_mv(), now - internal_adc_read_contactor_millis());
    return NULL;
}

static const char *cmd_ads(const char *args) {
    (void)args;
    millis_t now = millis();
    info_printf("ADS A (0x48): %s\n", ads1115_dev.present ? "present" : "NOT DETECTED");
#if HAS_ADS1115_SECONDARY()
    info_printf("ADS B (0x49): %s\n", ads1115_dev_b.present ? "present" : "NOT DETECTED");
#endif
    for(int ch = 0; ch < ADS1115_CHANNEL_COUNT; ch++) {
        info_printf("ADS ch%d: raw %8.1f (age %lu ms)\n",
            ch, (double)filtered_samples[ch], now - ads1115_get_sample_millis(ch));
    }
    info_printf("INA228 (0x40): %s\n", ina228_dev.present ? "present" : "NOT DETECTED");
    if(model.current_millis > 0) {
        info_printf("INA228 current: %ld mA (age %lu ms)\n",
            model.current_mA, now - model.current_millis);
    } else {
        info_printf("INA228 current: no reading yet\n");
    }
    return NULL;
}

typedef struct {
    const char *name;
    const char *(*handler)(const char *args);
    bool takes_args;
    const char *help;
} cli_command_t;

static const cli_command_t commands[] = {
    {"help", cmd_help, false, "list commands"},
    {"status", cmd_status, false, "full status dump"},
    {"params", cmd_params, false, "list tunable parameters and values"},
    {"get", cmd_get, true, "get <name>: show one parameter"},
    {"set", cmd_set, true, "set <name> <value>: change a parameter (not persisted)"},
    {"stop", cmd_stop, false, "request inactive mode"},
    {"restart", cmd_stop, false, "same as stop (used by reflash_bms.sh before flashing)"},
    {"toggle", cmd_toggle, false, "switch between inactive and operating"},
    {"calibrate", cmd_calibrate, false, "contactor calibration (offline long)"},
    {"calibrate_short", cmd_calibrate_short, false, "contactor calibration (offline short)"},
    {"calibrate_online", cmd_calibrate_online, false, "online calibration"},
    {"print_ekf_state", cmd_print_ekf_state, false, "EKF constants and state"},
    {"print_working_charge_settings", cmd_print_working_charge_settings, false, "working_charge_* parameters"},
    {"adc", cmd_adc, false, "internal ADC readings"},
    {"ads", cmd_ads, false, "ADS1115 and INA228 readings"},
};
#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))

static const char *cmd_help(const char *args) {
    (void)args;
    for(size_t i = 0; i < NUM_COMMANDS; i++) {
        info_printf("%-30s %s\n", commands[i].name, commands[i].help);
    }
    return NULL;
}

// line is trimmed and non-empty
static void cli_handle_command(char *line) {
    info_printf("Received command: '%s'\n", line);

    // Split into command word and arguments
    char *args = strchr(line, ' ');
    if(args) {
        *args++ = '\0';
        while(*args == ' ') {
            args++;
        }
    } else {
        args = "";
    }

    const char *err = "unknown command (see help)";
    for(size_t i = 0; i < NUM_COMMANDS; i++) {
        if(strcmp(line, commands[i].name) == 0) {
            if(!commands[i].takes_args && *args) {
                err = "command takes no arguments";
            } else {
                err = commands[i].handler(args);
            }
            break;
        }
    }

    if(err) {
        info_printf("ERR %s\n", err);
    } else {
        info_printf("OK\n");
    }
}

static char line_buf[120];
static uint8_t line_buf_idx = 0;
static bool line_overflow = false;

static void cli_handle_line() {
    if(line_overflow) {
        info_printf("ERR line too long (max %u chars)\n", (unsigned)sizeof(line_buf) - 1);
        return;
    }

    // Trim surrounding whitespace; ignore blank lines (e.g. the \n of \r\n)
    line_buf[line_buf_idx] = '\0';
    char *line = line_buf;
    while(isspace((unsigned char)*line)) {
        line++;
    }
    char *end = line + strlen(line);
    while(end > line && isspace((unsigned char)end[-1])) {
        *--end = '\0';
    }
    if(*line) {
        cli_handle_command(line);
    }
}

void cli_tick() {
    for(int n = 0; n < CLI_MAX_CHARS_PER_TICK; n++) {
        int c = stdio_getchar_timeout_us(0);
        if(c == PICO_ERROR_TIMEOUT) {
            return;
        }

        if(c == '\n' || c == '\r') {
            cli_handle_line();
            line_buf_idx = 0;
            line_overflow = false;
        } else if(c == '\b' || c == 0x7f) {
            if(line_buf_idx > 0) {
                line_buf_idx--;
            }
        } else if(line_buf_idx < sizeof(line_buf) - 1) {
            line_buf[line_buf_idx++] = (char)c;
        } else {
            // Drop the rest of the line rather than run a truncated command
            line_overflow = true;
        }
    }
}
