#include <stddef.h>
#include <setjmp.h>
#include <stdarg.h>
#include <cmocka.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drivers/bmb3y/bmb3y.h"
#include "drivers/bmb3y/crc.h"
#include "app/model.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "config/limits.h"

// The BMB read schedule (bmb3y_tick), against a fake isoSPI chain whose BMBs
// answer every read with a good CRC unless told otherwise.

void logging_printf(log_level_t level, const char *format, ...) {
    (void)level;
    (void)format;
}

millis_t stored_millis = 0;
millis64_t stored_millis64 = 0;
uint32_t stored_timestep = 0;

void sleep_us(uint64_t us) { (void)us; }

// Only what the BMB driver calls into the rest of the BMS
void balancing_sm_tick(bms_model_t *model) { model->balancing_sm.is_pause_cycle = false; }
void store_cell_voltage(bms_model_t *model, uint8_t logical_index, int16_t voltage_mV) {
    (void)model; (void)logical_index; (void)voltage_mV;
}
void store_thermistor_temperature(bms_model_t *model, uint8_t thermistor_index, int16_t temp_dC, bool valid) {
    (void)model; (void)thermistor_index; (void)temp_dC; (void)valid;
}
void store_bmb_die_temperature(bms_model_t *model, uint8_t module_index, int16_t temp_dC) {
    (void)model; (void)module_index; (void)temp_dC;
}

// Fake isoSPI. A read is from the snapshot the last SNAPSHOT command took;
// remember how old the oldest snapshot read from was.
static uint16_t failing_command = 0;
static uint32_t snapshot_timestep = 0;
static uint32_t oldest_snapshot_read = 0;

void isospi_send_wakeup_cs_blocking() {}

bool isospi_write_read_blocking(uint8_t *tx, uint8_t *rx, size_t len, size_t skip) {
    uint16_t command = (uint16_t)(tx[0] << 8) | tx[1];
    size_t rx_len = len - skip;

    if(command == BMB3Y_CMD_SNAPSHOT) {
        snapshot_timestep = stored_timestep;
    }
    if(rx == NULL || rx_len == 0) {
        return true;
    }
    if(command == failing_command || command == BMB3Y_CMD_READ_MODULE_ID) {
        return false;
    }

    if(stored_timestep - snapshot_timestep > oldest_snapshot_read) {
        oldest_snapshot_read = stored_timestep - snapshot_timestep;
    }

    // Cell banks are 9 bytes a module, temperatures 8: 6 data bytes (all
    // zero) then a big-endian CRC14
    bool bank = skip == 4;
    int stride = bank ? 9 : 8;
    memset(rx, 0, rx_len);
    for(size_t module = 0; module < rx_len / stride; module++) {
        uint8_t *m = &rx[module * stride];
        uint16_t crc = crc14(m, 6, bank ? 0x1000 : 0x0010, 0);
        m[6] = (uint8_t)(crc >> 8);
        m[7] = (uint8_t)crc;
    }
    return true;
}

static void tick(bms_model_t *model) {
    stored_timestep++;
    stored_millis += TIMESTEP_PERIOD_MS;
    stored_millis64 += TIMESTEP_PERIOD_MS;
    bmb3y_tick(model);
}

// The step the next tick will be at, for the given period
static int next_step(int period_mask) {
    extern int bmb3y_timestep_offset;
    return (int)((stored_timestep + 1 + bmb3y_timestep_offset + period_mask - 5) & period_mask);
}

#define SLOW_PERIOD_MASK (BMB3Y_SLOW_MODE_PERIOD_MS / TIMESTEP_PERIOD_MS - 1)
#define FAST_CYCLE_MS (64 * TIMESTEP_PERIOD_MS)

static int setup(void **state) {
    (void) state;
    stored_millis = 1000000;
    stored_millis64 = 1000000;
    for(int i = 0; i < ERR_HIGHEST; i++) {
        bms_event_slots[i] = (bms_event_slot_t){0};
    }
    extern uint16_t highest_level;
    highest_level = LEVEL_NONE;
    failing_command = 0;
    return 0;
}

// A pack idle in slow mode (inactive for over a minute, too low to balance),
// with a full slow cycle read. The next tick is at `slow_step`.
static void idle_in_slow_mode(bms_model_t *model, int slow_step) {
    *model = (bms_model_t){0};
    model->system_sm.state = SYSTEM_STATE_INACTIVE;
    model->system_sm.last_transition_time = stored_millis64 - 61000;
    model->contactor_sm.state = CONTACTORS_STATE_OPEN;
    model->cell_voltages_millis = stored_millis;
    model->cell_voltage_min_mV = get_minimum_balancing_voltage_mV(model) - 100;

    for(int i = 0; i < 2 * (SLOW_PERIOD_MASK + 1); i++) {
        tick(model);
    }
    assert_true(model->cell_voltage_slow_mode);
    while(next_step(SLOW_PERIOD_MASK) != slow_step) {
        tick(model);
    }
}

// Tick until slow mode is left, at most `max_ticks`. Returns whether it was.
static bool tick_until_slow_mode_left(bms_model_t *model, int max_ticks) {
    for(int i = 0; i < max_ticks; i++) {
        tick(model);
        if(!model->cell_voltage_slow_mode) {
            return true;
        }
    }
    return false;
}

static void assert_readings_fresh(const bms_model_t *model) {
    assert_true(millis() - model->cell_voltages_millis < FAST_CYCLE_MS);
    assert_true(millis() - model->thermistor_temperatures_millis < FAST_CYCLE_MS);
    assert_true(millis() - model->bmb_die_temperatures_millis < FAST_CYCLE_MS);
    // Read from a snapshot taken in this cycle (steps 1 to 9)
    assert_true(oldest_snapshot_read <= 9);
}

static void test_slow_mode_left_on_fresh_readings_wherever_the_switch_lands(void **state) {
    (void) state;
    // Slow mode ends late in a slow cycle (readings ~40s old), with the fast
    // cycle at each of its steps
    for(int fast_step = 0; fast_step < 64; fast_step++) {
        bms_model_t model;
        idle_in_slow_mode(&model, SLOW_PERIOD_MASK + 1 - 64 + fast_step);
        assert_true(millis() - model.cell_voltages_millis > 30000);

        // Run
        model.system_sm.state = SYSTEM_STATE_OPERATING;
        oldest_snapshot_read = 0;

        if(!tick_until_slow_mode_left(&model, 2 * 64)) {
            fail_msg("fast step %d: slow mode not left", fast_step);
        }
        assert_readings_fresh(&model);
    }
}

static void test_slow_mode_kept_after_a_failed_read(void **state) {
    (void) state;
    // A failed voltage bank, die temperature or thermistor read
    const uint16_t commands[] = {
        BMB3Y_CMD_READ_A >> 16, BMB3Y_CMD_READ_TEMPS, BMB3Y_CMD_READ_TEMPS3
    };
    for(size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        bms_model_t model;
        idle_in_slow_mode(&model, 1000);

        model.system_sm.state = SYSTEM_STATE_OPERATING;
        failing_command = commands[i];
        if(tick_until_slow_mode_left(&model, 64)) {
            fail_msg("command 0x%04X failing: slow mode left", commands[i]);
        }

        // Left on the next good cycle
        failing_command = 0;
        oldest_snapshot_read = 0;
        assert_true(tick_until_slow_mode_left(&model, 64));
        assert_readings_fresh(&model);
    }
}

int main(void) {
    const struct CMUnitTest tests[] = {
        cmocka_unit_test_setup(test_slow_mode_left_on_fresh_readings_wherever_the_switch_lands, setup),
        cmocka_unit_test_setup(test_slow_mode_kept_after_a_failed_read, setup),
    };

    return cmocka_run_group_tests(tests, NULL, NULL);
}
