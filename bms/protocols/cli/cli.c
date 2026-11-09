#include "cli.h"
#include "app/bms.h"
#include "app/estimators/ekf.h"
#include "app/firmware_id.h"
#include "app/model.h"
#include "app/state_machines/system.h"
#include "drivers/bmb3y/bmb3y.h"
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

// Parameters for params/set. Read-only ones are shown but can't be set (the
// calibration values are persisted, and a later NVM save would keep a
// temporary change).
typedef enum { PARAM_FLOAT, PARAM_DOUBLE, PARAM_INT32 } param_type_t;

typedef struct {
    const char *name;
    param_type_t type;
    void *ptr;
    bool writable;
} cli_param_t;

#define RW(name, type, ptr) {name, type, ptr, true}
#define RO(name, type, ptr) {name, type, ptr, false}

static const cli_param_t params[] = {
    RW("working_charge_internal_resistance_uR", PARAM_FLOAT, &working_charge_internal_resistance_uR),
    RW("working_charge_ceiling_dA", PARAM_FLOAT, &working_charge_ceiling_dA),
    RW("working_charge_slow_alpha", PARAM_FLOAT, &working_charge_slow_alpha),
    RW("working_charge_fast_alpha", PARAM_FLOAT, &working_charge_fast_alpha),
    RW("working_charge_slow_threshold_dA", PARAM_FLOAT, &working_charge_slow_threshold_dA),
    RW("working_charge_fast_threshold_dA", PARAM_FLOAT, &working_charge_fast_threshold_dA),
    RW("ekf_q0", PARAM_DOUBLE, &model.ekf.Q[0]),
    RW("ekf_q1", PARAM_DOUBLE, &model.ekf.Q[1]),
    RW("ekf_q2", PARAM_DOUBLE, &model.ekf.Q[2]),
    RW("ekf_r", PARAM_DOUBLE, &model.ekf.R),
    RW("ekf_min_slope", PARAM_DOUBLE, &model.ekf.min_ocv_slope),
    RW("ekf_knee_slope", PARAM_DOUBLE, &model.ekf.knee_ocv_slope),
    RW("ekf_r_ohm", PARAM_DOUBLE, &model.ekf.R_ohm),
    RW("ekf_offset_tau", PARAM_DOUBLE, &model.ekf.offset_tau_s),
    RO("cal_battery_voltage_mul", PARAM_FLOAT, &model.battery_voltage_mul),
    RO("cal_output_voltage_mul", PARAM_FLOAT, &model.output_voltage_mul),
    RO("cal_neg_contactor_mul", PARAM_FLOAT, &model.neg_contactor_mul),
    RO("cal_neg_contactor_offset_mV", PARAM_FLOAT, &model.neg_contactor_offset_mV),
    RO("cal_pos_contactor_mul", PARAM_FLOAT, &model.pos_contactor_mul),
#if IS_BMS_BOARD(REV1)
    RO("cal_pos_contactor_offset_mV", PARAM_FLOAT, &model.pos_contactor_offset_mV),
    RO("cal_link_voltage_mul", PARAM_FLOAT, &model.link_voltage_mul),
#endif
    RO("cal_current_offset", PARAM_INT32, &model.current_offset),
};
#define NUM_PARAMS (sizeof(params) / sizeof(params[0]))

static void print_param(const cli_param_t *p) {
    const char *ro = p->writable ? "" : " (read-only)";
    if(p->type == PARAM_INT32) {
        info_printf("%s = %ld%s\n", p->name, *(int32_t *)p->ptr, ro);
    } else {
        double v = p->type == PARAM_FLOAT ? (double)*(float *)p->ptr : *(double *)p->ptr;
        info_printf("%s = %g%s\n", p->name, v, ro);
    }
}

// Command handlers return NULL on success or a short error reason. args is
// the rest of the line after the command word ("" if none), and may be
// modified.

static const char *cmd_help(char *args);

static const char *cmd_status(char *args) {
    (void)args;
    bms_print_status();
    return NULL;
}

// Lists the parameters whose name contains args (all if empty)
static const char *cmd_params(char *args) {
    bool any = false;
    for(size_t i = 0; i < NUM_PARAMS; i++) {
        if(strstr(params[i].name, args)) {
            print_param(&params[i]);
            any = true;
        }
    }
    return any ? NULL : "no matching parameter";
}

static const char *cmd_set(char *args) {
    char *value = strchr(args, ' ');
    if(!value) {
        return "usage: set <name> <value>";
    }
    *value++ = '\0';

    const cli_param_t *p = NULL;
    for(size_t i = 0; i < NUM_PARAMS; i++) {
        if(strcmp(params[i].name, args) == 0) {
            p = &params[i];
            break;
        }
    }
    if(!p) {
        return "unknown parameter (see params)";
    }
    if(!p->writable) {
        return "parameter is read-only";
    }

    // The whole value must parse, and NaN/inf would poison the estimators
    char *end;
    double v = strtod(value, &end);
    if(end == value || *end != '\0' || !isfinite(v)) {
        return "invalid value";
    }
    switch(p->type) {
        case PARAM_FLOAT: *(float *)p->ptr = (float)v; break;
        case PARAM_DOUBLE: *(double *)p->ptr = v; break;
        case PARAM_INT32:
            if(v < INT32_MIN || v > INT32_MAX) {
                return "invalid value";
            }
            *(int32_t *)p->ptr = (int32_t)(v < 0 ? v - 0.5 : v + 0.5);
            break;
    }
    print_param(p);
    return NULL;
}

static const char *const system_state_names[] = {
    "uninitialized", "initializing", "calibrating", "inactive", "operating", "fault",
};

// Requests a system state change, if the system is in one of the allowed
// states (a bitmask of 1 << SYSTEM_STATE_*), since the state machine only
// acts on a request in those states and would otherwise leave it pending.
static const char *request(system_requests_t req, unsigned allowed_states) {
    unsigned state = model.system_sm.state;
    if(!(allowed_states & (1u << state))) {
        static char err[40];
        snprintf(err, sizeof(err), "not possible while %s",
            state < sizeof(system_state_names) / sizeof(system_state_names[0])
                ? system_state_names[state] : "?");
        return err;
    }
    model.system_req = req;
    return NULL;
}

#define IN(state) (1u << SYSTEM_STATE_##state)

static const char *cmd_run(char *args) {
    (void)args;
    return request(SYSTEM_REQUEST_RUN, IN(INACTIVE));
}

static const char *cmd_stop(char *args) {
    (void)args;
    return request(SYSTEM_REQUEST_STOP, IN(OPERATING));
}

// Before reflashing: stop from any state, so the request is still honoured if
// the system only reaches operating later
static const char *cmd_restart(char *args) {
    (void)args;
    return request(SYSTEM_REQUEST_STOP, ~0u);
}

static const char *cmd_calibrate(char *args) {
    static const struct {
        const char *mode;
        system_requests_t req;
        unsigned allowed_states;
    } modes[] = {
        {"long", SYSTEM_REQUEST_CALIBRATE_OFFLINE_LONG, IN(INACTIVE)},
        {"short", SYSTEM_REQUEST_CALIBRATE_OFFLINE_SHORT, IN(INACTIVE)},
        {"online", SYSTEM_REQUEST_CALIBRATE_ONLINE, IN(INACTIVE) | IN(OPERATING)},
        {"isolation", SYSTEM_REQUEST_CALIBRATE_ISOLATION, IN(INACTIVE)},
    };
    for(size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++) {
        if(strcmp(args, modes[i].mode) == 0) {
            return request(modes[i].req, modes[i].allowed_states);
        }
    }
    return "usage: calibrate long|short|online|isolation";
}

static const char *cmd_ekf(char *args) {
    (void)args;
    info_printf("EKF Constants: Q=[%e, %e, %e], R=%f, min_slope=%f, knee_slope=%f, R_ohm=%f, offset_tau=%f\n",
                 model.ekf.Q[0], model.ekf.Q[1], model.ekf.Q[2],
                 model.ekf.R, model.ekf.min_ocv_slope, model.ekf.knee_ocv_slope, model.ekf.R_ohm, model.ekf.offset_tau_s);
    info_printf("EKF State: Ah_used=%.4f Ah, V_off=%.4f V, Capacity=%.2f Ah, Bias=%.1f mA\n",
       model.ekf.x[0], model.ekf.x[1], model.ekf.cap, model.ekf.x[2] * 1000.0f);
    return NULL;
}

static const char *cmd_version(char *args) {
    (void)args;
    info_printf("Firmware %s\n", bms_build_version);
    return NULL;
}

static const char *cmd_cells_raw(char *args) {
    (void)args;
    bmb3y_print_physical_cells();
    return NULL;
}

static const char *cmd_adc(char *args) {
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

// Rate of a counter since the previous call that took a snapshot, in Hz
static double rate_hz(uint32_t now_count, uint32_t prev_count, millis_t dt_ms) {
    return dt_ms ? (double)(now_count - prev_count) * 1000.0 / dt_ms : 0.0;
}

static const char *cmd_ads(char *args) {
    (void)args;
    // Counters at the previous call, so rates cover the time in between
    static millis_t prev_ms;
    static ads1115_stats_t prev;

    millis_t now = millis();
    ads1115_stats_t st = ads1115_stats; // racy copy, fine for diagnostics
    ads1115_stats.max_cycle_us = 0;
    millis_t dt = prev_ms ? now - prev_ms : now;

    info_printf("ADS A (0x48): %s\n", ads1115_dev.present ? "present" : "NOT DETECTED");
#if HAS_ADS1115_SECONDARY()
    info_printf("ADS B (0x49): %s\n", ads1115_dev_b.present ? "present" : "NOT DETECTED");
#endif
    info_printf("ADS scan: %lu cycles (%.2f Hz over %lu ms), %lu errors (+%lu), cycle %lu us (max %lu us)\n",
        st.cycles, rate_hz(st.cycles, prev.cycles, dt), dt,
        st.errors, st.errors - prev.errors, st.last_cycle_us, st.max_cycle_us);
    // V is at the uncalibrated default divider ratio (single-ended channels
    // read the ADC pin itself, in mV)
    for(int ch = 0; ch < ADS1115_CHANNEL_COUNT; ch++) {
        bool single = ads1115_channel_single_ended(ch);
        info_printf("ADS ch%d %-15s raw %8.1f (dev %5.1f) = %8.3f %s | %lu samples (%.2f Hz) | age %lu ms\n",
            ch, ads1115_channel_name(ch), (double)filtered_samples[ch], (double)sample_deviations[ch],
            single ? (double)filtered_samples[ch] * (4096.0 / 32768.0) : (double)filtered_samples[ch] * ADS1115_DEFAULT_MUL,
            single ? "mV" : "V",
            st.samples[ch], rate_hz(st.samples[ch], prev.samples[ch], dt),
            now - ads1115_get_sample_millis(ch));
    }
    prev_ms = now;
    prev = st;
    return NULL;
}

static const char *cmd_ina(char *args) {
    (void)args;
    static millis_t prev_ms;
    static uint32_t prev_samples;
    static int64_t prev_charge_raw;

    millis_t now = millis();
    ina228_stats_t st = ina228_stats;
    ina228_stats.max_gap_us = 0;
    millis_t dt = prev_ms ? now - prev_ms : now;

    info_printf("INA228 (0x40): %s | SHUNT_CAL %u\n",
        ina228_dev.present ? "present" : "NOT DETECTED", (unsigned)INA228_SHUNT_CAL);
    info_printf("INA228 samples: %lu (%.3f Hz over %lu ms), max gap %lu ms, I2C errors %lu\n",
        st.samples, rate_hz(st.samples, prev_samples, dt), dt,
        st.max_gap_us / 1000, st.i2c_errors);
    info_printf("INA228 raw %ld - vos %ld - offset %ld = %ld counts -> %ld mA (age %lu ms) | filtered %.2f mA (dev %.2f)\n",
        st.last_raw, st.last_vos_counts, model.current_offset,
        st.last_raw - st.last_vos_counts - model.current_offset,
        model.current_mA, model.current_millis ? now - model.current_millis : 0,
        (double)model.current_filtered_mA, (double)model.current_deviation);
    // Average current from the charge integral since the last call should
    // match the sampled current
    int64_t dq = model.charge_raw - prev_charge_raw;
    info_printf("INA228 charge: %lld mC, +%lld mC since last (avg %.2f mA)\n",
        (long long)raw_charge_to_mC(model.charge_raw), (long long)raw_charge_to_mC(dq),
        dt ? (double)dq / INA228_CHARGE_RAW_PER_MC / (dt / 1000.0) : 0.0);
    if(st.die_measurements) {
        info_printf("INA228 die: %.2f C (%lu readings, age %lu ms)\n",
            (double)model.shunt_die_temperature, st.die_measurements,
            now - model.shunt_die_temperature_millis);
    } else {
        info_printf("INA228 die: no reading yet (vos assumes 25 C)\n");
    }
    if(st.ntc_measurements) {
        info_printf("INA228 NTC: %.3f V -> %.0f ohm -> %.1f C (%lu readings, age %lu ms)\n",
            (double)st.ntc_volts, (double)model.shunt_ntc_resistance_ohms,
            (double)model.shunt_ntc_temperature, st.ntc_measurements,
            now - model.shunt_ntc_millis);
    } else {
        info_printf("INA228 NTC: no reading yet\n");
    }

    prev_ms = now;
    prev_samples = st.samples;
    prev_charge_raw = model.charge_raw;
    return NULL;
}

typedef struct {
    const char *name;
    const char *args; // usage, or NULL if the command takes no arguments
    const char *(*handler)(char *args);
    const char *help;
} cli_command_t;

// Stats and rates in ads/ina cover the time since that command was last run
static const cli_command_t commands[] = {
    {"help", NULL, cmd_help, "list commands"},
    {"status", NULL, cmd_status, "full status dump"},
    {"version", NULL, cmd_version, "firmware commit and build time"},
    {"params", "[filter]", cmd_params, "list parameters whose name contains filter"},
    {"set", "<name> <value>", cmd_set, "change a parameter (not persisted)"},
    {"run", NULL, cmd_run, "request operating mode (from inactive)"},
    {"stop", NULL, cmd_stop, "request inactive mode (from operating)"},
    {"restart", NULL, cmd_restart, "request inactive mode from any state (used by reflash_bms.sh)"},
    {"calibrate", "long|short|online|isolation", cmd_calibrate, "contactor (offline), online or isolation zero calibration"},
    {"ekf", NULL, cmd_ekf, "EKF constants and state"},
    {"adc", NULL, cmd_adc, "internal ADC readings"},
    {"ads", NULL, cmd_ads, "ADS1115 readings, scan rate and errors"},
    {"ina", NULL, cmd_ina, "INA228 current, charge, temperatures and sampling stats"},
    {"cells_raw", NULL, cmd_cells_raw, "last reading of every BMB channel (* = in the cell map)"},
};
#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))

static const char *cmd_help(char *args) {
    (void)args;
    for(size_t i = 0; i < NUM_COMMANDS; i++) {
        const cli_command_t *c = &commands[i];
        info_printf("%-10s %-18s %s\n", c->name, c->args ? c->args : "", c->help);
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
            if(!commands[i].args && *args) {
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

static void cli_handle_line(void) {
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

void cli_tick(void) {
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
