#include "bmb3y.h"
#include "crc.h"

#include "config/limits.h"
#include "app/model.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "drivers/isospi/isospi_master.h"

#include "pico/stdlib.h"

#include <stdint.h>

const uint8_t CELLS_PER_MODULE = 15;
const uint8_t CELLS_PER_BANK = 3;
const uint8_t MODULE_COUNT = 8;

// Low level functions

void bmb3y_send_command_blocking(uint16_t cmd_word) {
    // Send a 16-bit command with normal CS pattern
    uint8_t tx[2];
    uint8_t rx[2] = {0};
    
    // Split 16-bit command into 2 bytes (MSB first)
    tx[0] = (cmd_word >> 8) & 0xFF;
    tx[1] = cmd_word & 0xFF;
    
    isospi_write_read_blocking(tx, rx, 2, 2);
}

bool bmb3y_long_command_get_data_blocking(uint32_t cmd, uint8_t *response, int response_len) {
    // Send a four-byte command and read data from BMBs into the supplied buffer.
    // Returns true if the read was successful.
    
    // Assumes the BMB is already awake.

    uint8_t tx[104] = {0};
    if(response_len > 100) {
        // Transmit buffer isn't large enough
        return false;
    }

    // CRC is CRC8 with polynomial 2f (AUTOSAR) but we just bake it into the commands

    tx[0] = (cmd >> 24) & 0xFF;
    tx[1] = (cmd >> 16) & 0xFF;
    tx[2] = (cmd >> 8) & 0xFF;
    tx[3] = cmd & 0xFF;

    // tx[0] = reg_cmd; // command byte
    // tx[1] = 0;       // dummy byte
    // tx[2] = 0x70;    // CRC8 (poly=0x2F, init=0x10)
    // tx[3] = 0;       // 
    
    return isospi_write_read_blocking(tx, response, 4 + response_len, 4);
}

bool bmb3y_short_command_get_data_blocking(uint16_t cmd, uint8_t *response, int response_len) {
    // Send a two-byte command and read data from BMBs into the supplied buffer.
    // Returns true if the read was successful.

    // Assumes the BMB is already awake.

    uint8_t tx[104] = {0};
    if(response_len > 100) {
        // Transmit buffer isn't large enough
        return false;
    }

    tx[0] = (cmd >> 8) & 0xFF;
    tx[1] = cmd & 0xFF;
    
    return isospi_write_read_blocking(tx, response, 2 + response_len, 2);
}

void bmb3y_send_wakeup_cs_blocking() {
    // Wakes up the BMBs for further communication. The BMBs will fall back to sleep
    // after about 5ms of inactivity.

    isospi_send_wakeup_cs_blocking();
}

void bmb3y_send_super_wakeup() {
    // First do the normal wakeup, which works most of the time
    isospi_send_wakeup_cs_blocking();

    // But also, as per the normal BMS, send 9 dummy commands, which will always
    // wake all the BMSes (even if some were already awake and thus didn't
    // propagate the CS wakeup)
    for(int i=0; i<9; i++) {
        bmb3y_send_command_blocking(0);
        sleep_us(2);
    }
}

void bmb3y_send_balancing_blocking(bms_model_t *model) {
    balancing_sm_t *balancing_sm = &model->balancing_sm;

    uint8_t tx_buf[150] = {0};
    
    tx_buf[0] = (BMB3Y_CMD_WRITE_CONFIG >> 8) & 0xFF;
    tx_buf[1] = BMB3Y_CMD_WRITE_CONFIG & 0xFF;

    /* 
    The balancing mask consists of four 32-bit ints, with a bit for each cell -
    the LSB of the first int is cell 0.

    However the BMB modules consume the mask 15-bits at a time, so it is
    necessary to re-pack the bits accordingly.
    */

    uint32_t cell_presence_mask[] = CELL_PRESENCE_MASK;

    int logical_index = 0;
    for (int word_idx = 0; word_idx < 4; word_idx++) {
        uint32_t presence = cell_presence_mask[word_idx];
        while (presence) {
            // Find the index of the next cell in the balance mask
            int bit_in_word = __builtin_ctz(presence);
            // Clear that bit so the next iteration will find the next cell
            presence &= ~(1U << bit_in_word);

            // Are we requesting to balance this cell?
            if (balancing_sm->balance_request_mask[logical_index / 32] & (1U << (logical_index & 0x1F))) {
                int physical_index = (word_idx * 32) | bit_in_word;

                // The balancing register has an extra padding bit after every 15
                // cells, compared to our cell presence mask.
                int balance_index = physical_index + (physical_index / 15);
                int module = 7 - (balance_index >> 4);
                int module_cell_index = balance_index & 0x0F;

                tx_buf[2 + 2 + module * 6 + (module_cell_index / 8)] |= (1 << (module_cell_index & 0x07));
            }
            logical_index++;
        }
    }

    // Fill in config bytes and calculate CRCs
    for(int module=0; module<8; module++) {
        tx_buf[2 + 0 + module*6] = 0xf3;
        tx_buf[2 + 1 + module*6] = 0;

        // bytes 2, 3 are already filled in by the loop above

        uint16_t calc_crc = crc14(&tx_buf[2 + 0 + module*6], 4, 0x0010, 0);

        tx_buf[2 + 4 + module*6] = (calc_crc >> 8) & 0xFF;
        tx_buf[2 + 5 + module*6] = calc_crc & 0xFF;
    }

    // Write only - we skip all of the response bytes
    isospi_write_read_blocking(tx_buf, NULL, 50, 50);
}


// Higher level functions

static void bmb3y_update_balancing_active(bms_model_t *model) {
    balancing_sm_t *balancing_sm = &model->balancing_sm;

    // Update the balancing active flag based on what we previously requested to
    // balance.

    model->balancing_active = balancing_sm->balance_request_mask[0] != 0 ||
                  balancing_sm->balance_request_mask[1] != 0 ||
                  balancing_sm->balance_request_mask[2] != 0 ||
                  balancing_sm->balance_request_mask[3] != 0;
}

static const uint32_t READ_COMMANDS[] = {
    BMB3Y_CMD_READ_A,
    BMB3Y_CMD_READ_B,
    BMB3Y_CMD_READ_C,
    BMB3Y_CMD_READ_D,
    BMB3Y_CMD_READ_E
};

static const uint16_t SHORT_READ_COMMANDS[] = {
    BMB3Y_CMD_READ_A_SHORT,
    BMB3Y_CMD_READ_B_SHORT,
    BMB3Y_CMD_READ_C_SHORT,
    BMB3Y_CMD_READ_D_SHORT,
    BMB3Y_CMD_READ_E_SHORT
};

bool bmb3y_read_cell_voltage_bank_blocking(bms_model_t *model, int bank_index) {
    uint8_t rx_buf[72];
    uint32_t cmd = READ_COMMANDS[bank_index];
    isosnoop_flush();
    if (!bmb3y_long_command_get_data_blocking(cmd, rx_buf, 72)) {
        error_printf("BMB3Y read failed for cmd 0x%02lX\n", cmd);
        count_bms_event(ERR_BMB_READ_ERROR, 0x0100000000000000 | bank_index);
        return false;
    }
    // Short commands need different CRC handling
    // uint16_t cmd = SHORT_READ_COMMANDS[bank_index];
    // if (!bmb3y_short_command_get_data_blocking(cmd, rx_buf, 72)) {
    //     error_printf("BMB3Y read failed for cmd 0x%02X\n", cmd);
    //     count_bms_event(ERR_BMB_READ_ERROR, 0x0100000000000000 | bank_index);
    //     return false;
    // }

    uint8_t cell_offset = bank_index * 3;
    uint32_t cell_presence_mask[] = CELL_PRESENCE_MASK;
    bool crc_ok = true;

    for(int module=0; module<NUM_MODULE_VOLTAGES; module++) {
        uint16_t module_crc = (uint16_t)(rx_buf[module * 9 + 6] << 8) | (uint16_t)(rx_buf[module * 9 + 7]);
        //uint16_t calc_crc = crc14(&rx_buf[module * 9], 6, 0x1000);
        uint16_t calc_crc = crc14(&rx_buf[module * 9], 6, 0x1000, module_crc);

        if((module_crc & 0x3fff) != calc_crc) {
            error_printf("CRC: %s %d %02X %02X %02X %02X %02X %02X %02X %02X %02X calc %04X\n",
                module_crc == calc_crc ? "OK" : "FAIL",
                bank_index,
                rx_buf[module * 9 + 0],
                rx_buf[module * 9 + 1],
                rx_buf[module * 9 + 2],
                rx_buf[module * 9 + 3],
                rx_buf[module * 9 + 4],
                rx_buf[module * 9 + 5],
                rx_buf[module * 9 + 6],
                rx_buf[module * 9 + 7],
                rx_buf[module * 9 + 8],
                calc_crc
            );
            isosnoop_print_buffer();
            count_bms_event(ERR_BMB_CRC_MISMATCH, 0x0100000000000000 | ((uint64_t)bank_index << 48) | ((uint64_t)module << 40) | ((uint32_t)module_crc << 16) | calc_crc);
            crc_ok = false;
            continue;
        }

        // Go through each cell (three per bank)
        for(int cell=0; cell<3; cell++) {
            int physical_index = (module * 15) + cell_offset + cell;

            // Physical index is the channel number of the cell on the BMBs,
            // however not every channel is populated.

            // Logical index is the actual cell number (with no gaps).

            // To calculate logical from physical, we need to count how many
            // bits are set in the cell presence mask up to the physical index.

            uint8_t logical_index = 0;
            // Calculate which presence mask word the physical index is in
            int word_offset = physical_index / 32;
            // Is this cell actually present according to the presence mask?
            if((cell_presence_mask[word_offset] & (1U << (physical_index % 32))) == 0) {
                // Not present, skip
                continue;
            }
            // Count all the bits in previous presence mask words
            for(int word=0; word<word_offset; word++) {
                logical_index += __builtin_popcount(cell_presence_mask[word]);
            }
            // Count the bits in the presence mask word up to the physical index
            logical_index += __builtin_popcount(cell_presence_mask[word_offset] & ((1 << (physical_index % 32)) - 1));

            uint16_t voltage = 
                (uint16_t)(rx_buf[module * 9 + cell * 2]) |
                (uint16_t)(rx_buf[module * 9 + cell * 2 + 1] << 8);

            int16_t converted = 0;

            if(voltage == 0xFFFF) {
                // A non-existent cell
                converted = -1;
            } else if(voltage == 0) {
                // Record as 1mV to distinguish from unmeasured
                converted = 1;
            } else {
                converted = (voltage * 2) / 25;
            }

            model->raw_cell_voltages_mV[logical_index] = converted;

            // Don't store voltages during balancing, as they are unstable
            if(!model->balancing_active) {
                store_cell_voltage(logical_index, converted);
            }
        }
    }
    return crc_ok;
}

/* 
NMC 

TEMPS readings:
Temp hex: 73 94 5B 05 67 F3 21 BA
Temp hex: 78 94 62 05 AA F4 08 A9
Temp hex: B2 93 68 05 47 F4 E0 DE
Temp hex: 91 94 71 05 99 F3 59 92
Temp hex: A3 94 53 05 1E F4 CD 5C
Temp hex: B6 93 57 05 A6 F4 C3 5D
Temp hex: 5A 93 63 05 E9 F3 03 13
Temp hex: 7C 94 5B 05 DE F3 5E D3

TEMPS3 readings:
Temp hex: D0 16 31 F2 E2 18 32 88
Temp hex: E6 17 18 F6 E6 17 25 14
Temp hex: 5E 16 D8 F1 52 19 E2 FF
Temp hex: A2 17 ED F4 CB 17 69 B2
Temp hex: DB 15 37 F2 E6 16 F3 72
Temp hex: 33 17 81 F3 FF 17 D6 9E
Temp hex: 8F 16 97 F1 D8 17 02 09
Temp hex: C2 16 BB F1 9A 18 4A 9A

TEMPS4 (actually register F, probably not temps)
Temp hex: 03 00 AA 88 5C E3 
          03 00 32 95 4E A2 
          03 00 35 95 00 30
          02 00 9F A1 AD D3
          02 00 18 95 1C F4 
          02 00 66 A1 1C 31
          03 00 C0 88 47 7F
          03 00 06 95 AB 8D

TEMPS2
Temp hex: 80 20 C0 BD 5C 70
          80 20 C0 E7 5E 66
          80 20 C0 9F 28 1F
          80 20 C0 ED 83 68
          80 20 C0 B8 32 F7
          80 20 C0 BB 29 43
          80 20 C0 EA 7B 81
          80 20 C0 E0 A6 8F

*/


// Read some temperature-like values
bool bmb3y_read_more_temps_blocking(bms_model_t *model) {
    uint8_t rx_buf[90];

    if(!bmb3y_short_command_get_data_blocking(BMB3Y_CMD_READ_TEMPS3, rx_buf, 64)) {
        error_printf("BMB3Y temperature3 read failed\n");
        count_bms_event(ERR_BMB_READ_ERROR, 0x0200000000000000);
        return false;
    }

    bool crc_ok = true;
    for(int module=0; module<NUM_MODULE_TEMPS; module++) {
        uint16_t module_crc = (uint16_t)(rx_buf[module * 8 + 6] << 8) | (uint16_t)(rx_buf[module * 8 + 7]);
        //uint16_t calc_crc = crc14(&rx_buf[module * 8], 6, 0x0010);
        uint16_t calc_crc = crc14(&rx_buf[module * 8], 6, 0x0010, module_crc);
        if((module_crc & 0x3fff) != calc_crc) {
        //if(!crc_matches(module_crc, calc_crc)) {
            error_printf("Bad Temp CRC on module %d: msg 0x%04X calc 0x%04X xor %04x or %04x\n", 
                module, module_crc, calc_crc, calc_crc ^ 0x425b, calc_crc ^ 0xc6ed);
            crc_ok = false;
            if(millis64() > 2000) {
                count_bms_event(ERR_BMB_CRC_MISMATCH, 0x0200000000000000 | ((uint64_t)module << 48) | ((uint32_t)module_crc << 16) | calc_crc);
            }
            continue;
        }

        model->raw_temperatures[16+module] = (
            (int16_t)(rx_buf[module * 8 + 0]) |
            (int16_t)(rx_buf[module * 8 + 1] << 8)
        );
        model->raw_temperatures[24+module] = (
            (int16_t)(rx_buf[module * 8 + 2]) |
            (int16_t)(rx_buf[module * 8 + 3] << 8)
        );
        model->raw_temperatures[32+module] = (
            (int16_t)(rx_buf[module * 8 + 4]) |
            (int16_t)(rx_buf[module * 8 + 5] << 8)
        );
    }

    return crc_ok;
}

bool bmb3y_read_temperatures_blocking(bms_model_t *model) {
    uint8_t rx_buf[90];

    // LFP seems to have a temp value in BMB3Y_CMD_READ_TEMPS3 0:1
    // NMC seems to use TEMPS and 2:3 like D/T's code

    if(!bmb3y_short_command_get_data_blocking(BMB3Y_CMD_READ_TEMPS, rx_buf, 64)) {
        error_printf("BMB3Y temperature read failed\n");
        count_bms_event(ERR_BMB_READ_ERROR, 0x0200000000000000);
        return false;
    }

    bool crc_ok = true;
    for(int module=0; module<NUM_MODULE_TEMPS; module++) {
        // Each module has 8 bytes: 6 bytes data, 2 bytes CRC
        // The temp is in bytes 2 and 3 (little-endian)
        
        uint16_t module_crc = (uint16_t)(rx_buf[module * 8 + 6] << 8) | (uint16_t)(rx_buf[module * 8 + 7]);
        //uint16_t calc_crc = crc14(&rx_buf[module * 8], 6, 0x0010);
        uint16_t calc_crc = crc14(&rx_buf[module * 8], 6, 0x0010, module_crc);

        // debug_printf("Temp hex: ");
        // for(int i=0;i<8;i++) {
        //     debug_printf("%02X ", rx_buf[module * 8 + i]);
        // }
        // debug_printf("\n");
        //isosnoop_print_buffer();

        if((module_crc & 0x3fff) != calc_crc) {
        //if(!crc_matches(module_crc, calc_crc)) {
            error_printf("Bad Temp CRC on module %d: msg 0x%04X calc 0x%04X xor %04x or %04x\n", 
                module, module_crc, calc_crc, calc_crc ^ 0x425b, calc_crc ^ 0xc6ed);
            crc_ok = false;
            if(millis64() > 2000) {
                count_bms_event(ERR_BMB_CRC_MISMATCH, 0x0200000000000000 | ((uint64_t)module << 48) | ((uint32_t)module_crc << 16) | calc_crc);
            }
            continue;
        }

        // D/T's algorithm: (TEMPS)
        if(true) {
            // // Unlike everything else, temps are little-endian??
            int16_t raw_temp = 
                (int16_t)(rx_buf[module * 8 + 2]) |
                (int16_t)(rx_buf[module * 8 + 3] << 8);

            store_module_temperature(module, raw_temp - 1131);
        }
        
        // LFP algo: (TEMPS3)
        if(false) {
            int16_t raw_temp = 
                (int16_t)(rx_buf[module * 8 + 0]) |
                (int16_t)(rx_buf[module * 8 + 1] << 8);

            // super-crude cal, 28c = 0x4300, 100c = 0x6e00
            // FIXME - do proper thermistor conversion
            int16_t temp_dC = ((raw_temp - 0x4300) * (1000 - 280)) / (0x6e00 - 0x4300) + 280;

            store_module_temperature(module, temp_dC);
        }

        model->raw_temperatures[module] = (
            (int16_t)(rx_buf[module * 8 + 0]) |
            (int16_t)(rx_buf[module * 8 + 1] << 8)
        );
        model->raw_temperatures[8+module] = (
            (int16_t)(rx_buf[module * 8 + 4]) |
            (int16_t)(rx_buf[module * 8 + 5] << 8)
        );
    }

    if(crc_ok) {
        model->module_temperatures_millis = millis();
    }

    return crc_ok;
}

static void bmb3y_read_cell_voltages_blocking(bms_model_t *model) {
    // Read all cell voltage banks in one go

    bool all_crc_ok = true;
    for(int bank=0; bank<5; bank++) {
        all_crc_ok = bmb3y_read_cell_voltage_bank_blocking(model, bank) && all_crc_ok;
    }
    if(all_crc_ok && !model->balancing_active) {
        model->cell_voltages_millis = millis();
    }
}

static bool should_stop(bms_model_t *model) {
    // Determine whether we should stop communicating with the BMBs to save power.
    (void)model;

    if((get_event_level(ERR_BATTERY_VOLTAGE_LOW) == LEVEL_FATAL) ||
       (get_event_level(ERR_CELL_VOLTAGE_VERY_LOW) == LEVEL_FATAL)) {
        // Have locked out due to low battery/cell voltage
        // (after stopping we will soon gain a cell-voltages-stale error also)
        return true;
    }

    return false;
}

static bool should_use_slow_mode(bms_model_t *model) {
    // Determine whether we should enter slow mode to reduce battery self-discharge.

    if(model->system_sm.state == SYSTEM_STATE_INACTIVE && 
       state_time(&model->system_sm) > 60000 && 
       (model->cell_voltages_millis > 0 && model->cell_voltage_min_mV < get_minimum_balancing_voltage_mV(model))) {
        // Inactive (contactors open) for a minute, and too low to balance, enter slow mode
        return true;
    }

    if(get_event_level(ERR_BATTERY_VOLTAGE_LOW) != LEVEL_NONE ||
       get_event_level(ERR_CELL_VOLTAGE_LOW) != LEVEL_NONE) {
        // Low battery/cell voltage - use slow mode
        return true;
    }

    return false;
}


int bmb3y_timestep_offset = 0;
// Use a shorter period when we're in a balancing pause cycle, to get back to balancing sooner.
const int PAUSE_CYCLE_LENGTH = 10; // (min 10)
bool crc_failed = false;
bool started = false;

void bmb3y_tick(bms_model_t *model) {
    int period_mask = 0x3f;

    const int slow_mode_period_mask = 0x7ff;

    if(should_stop(model)) {
        // Stop talking to the BMBs to save power
        return;
    } else if(should_use_slow_mode(model)) {
        // In slow mode, sample less frequently
        if(!model->cell_voltage_slow_mode) {
            info_printf("BMB3Y entering slow mode\n");
            model->cell_voltage_slow_mode = true;
        }
        period_mask = slow_mode_period_mask;
    }

    // Derive a step number which loops every 'period' timesteps
    int step = ((timestep() + bmb3y_timestep_offset + period_mask - 5) & period_mask);

    if(step == 0) {
        // Wake up, send a hello (unclear what this actually does).

        bmb3y_send_super_wakeup();
        bmb3y_send_command_blocking(BMB3Y_CMD_HELLO);
    } else if(step == 1) {
        // Wake up BMBs, take snapshot.
        
        bmb3y_send_super_wakeup();
        bmb3y_send_command_blocking(BMB3Y_CMD_SNAPSHOT);

        // Record whether the past cycle has had balancing active (so we can
        // ignore the unstable voltage readings)
        bmb3y_update_balancing_active(model);
    } else if(step == 2) {
        // Wake up, resume balancing (if active). Our voltage/temp snapshot
        // will remain readable even while balancing.

        bmb3y_send_super_wakeup();
        balancing_sm_tick(model);
        bmb3y_send_balancing_blocking(model);

        crc_failed = false;
    } else if(step >= 3 && step <= 7) {
        // Wake up and read a voltage bank

        bmb3y_send_super_wakeup();

        if(!bmb3y_read_cell_voltage_bank_blocking(model, step - 3)) {
            // Uh oh, we didn't read successfully
            crc_failed = true;
        }

        if(step == 7 && !crc_failed && !model->balancing_active) {
            // All banks read successfully
            model->cell_voltages_millis = millis();
        }
    } else if(step == 8) {
        // Wake up, read temperatures

        bmb3y_send_super_wakeup();
        bmb3y_read_temperatures_blocking(model);
    } else if(step == 9) {
        // Wake up, read more temps

        bmb3y_send_super_wakeup();
        bmb3y_read_more_temps_blocking(model);

        if(!crc_failed && !model->balancing_active && model->cell_voltage_slow_mode && !should_use_slow_mode(model)) {
            // Exit slow mode now that we have fresh readings (both voltage and temp)
            model->cell_voltage_slow_mode = false;
            info_printf("BMB3Y exiting slow mode\n");
        }
    } else if(step == PAUSE_CYCLE_LENGTH && model->balancing_sm.is_pause_cycle) {
        // Cut the cycle short if we're in a pause cycle by adjusting the
        // offset. This allows us to get back to balancing sooner, we just need
        // to have left enough time for voltages to settle.
        bmb3y_timestep_offset = (bmb3y_timestep_offset + (period_mask + 1) - PAUSE_CYCLE_LENGTH - 1) & period_mask;
    }
}


// bool last_read_crc_failed = false;
// int balancing_pause_timer = 0;

// void old_bmb3y_tick(bms_model_t *model) {
//     // TODO - maybe split this in half, into separate 'read' and 'write' phases?
//     // Otherwise we're writing out balancing during the read phase, (even though
//     // it'll be related to data discovered on previous cycles anyway)

//     // We do a full communication cycle every 64 ticks (1.28s).
//     // We offset the steps by 5 to interleave with other BMS tasks.

//     // Slow mode samples less frequently to reduce battery self-drain in low
//     // voltage situations. It only takes effect once we have a valid reading,
//     // and aren't having CRC issues.
//     bool use_slow_mode = model->cell_voltage_slow_mode && model->cell_voltages_millis > 0 &&
//         !last_read_crc_failed;

//     // 81.92s in slow mode, 1.28s in normal mode
//     int period = use_slow_mode ? 0xfff : 0x3f;
//     int step = ((timestep() + bmb3y_timestep_offset) & period) - 5; // was 3f

//     // We can 'MUTE' the BMBs which pauses the balancing, but we then have to
//     // wait for a while before requesting a snapshot for the voltages to settle:
//     //    without muting:
//     //     cell 23: 3958 - 4137 mV
//     //    with 1 timestep between mute and snapshot
//     //     cell 23: 4022 - 4107
//     //    2 timesteps:
//     //     cell 23: 4052 - 4094
//     //    3 timesteps:
//     //     cell 23: 4066 - 4086
//     //    4 timesteps:
//     //     cell 23: 4073 - 4083
//     //    5 timesteps:
//     //     cell 23: 4077 - 4081
//     //    6 timesteps:
//     //     cell 23: 4078 - 4081
//     //    7 timesteps:
//     //     cell 23: 4079 - 4080
//     //    8 timesteps:
//     //     cell 23: 4080 - 4080
//     //    9 timesteps:
//     //     cell 23: 4080 - 4080 (other cells still bouncing by 1mV)

//     // 9 timesteps is 180ms, a bit much to pause every 1.28s?
//     // alternatively, we could:
//     //   mute during a pair of balance cycles every so often (one odd, one even)
//     //     for say 6 timesteps each (120ms)
//     //   and average the readings from those two cycles
//     // could get down to 240ms every, say 5.12s, or 5% of the time, or whatever

//     int mute_steps = 0;

//     uint32_t start = time_us_32();

//     if(mute_steps>0 && step==0) {
//         // Wake up and stop balancing
//         bmb3y_send_wakeup_cs_blocking();
//         //pause_balancing(&model->balancing_sm);
//         //bmb3y_send_balancing(model);
//         bmb3y_send_command_blocking(BMB3Y_CMD_MUTE);

//     } else if(step==(mute_steps + 0)) {
//         // Wake up and request snapshot

//         bmb3y_send_wakeup_cs_blocking();

//         // The BMB responses include CRC14s, but these often seem to be
//         // corrupted unless you perform the operations in a very particular
//         // sequence.

//         // The corruption takes the form of a XOR with one of three patterns, so
//         // is easily reversible, at the cost of slightly reduced integrity.

//         // Thus we don't bother with the sequencing anymore.

//         // The sequencing changes also if you want too long between cycles -
//         // normally we use 1.28s cycles, but slow mode requires a different
//         // pattern.
        
//         // read config
//         //uint8_t rx_buf[72];
//         //bmb3y_get_data_blocking(BMB3Y_CMD_READ_CONFIG, rx_buf, 72);
//         // for(int i=0; i<72; i++) {
//         //     printf("%02X ", rx_buf[i]);
//         // }
//         // printf("\n");

//         if(model->cell_voltages_millis && model->cell_voltage_min_mV < CELL_VOLTAGE_SOFT_MIN_mV && !model->cell_voltage_slow_mode && !last_read_crc_failed) {
//             // We're not in slow mode yet, but a cell is low - switch to slow mode
//             // model->cell_voltage_slow_mode = true;
//             // use_slow_mode = true;
//         }


//         if(use_slow_mode) {
//             // In slow mode we have to set balancing config first, and use a
//             // different wakeup pattern, else we get bad CRCs back on the
//             // subsequent reads.

//             // Forcibly prevent balancing in slow mode
//             pause_balancing(&model->balancing_sm);
//             bmb3y_send_balancing(model);

//             // If attempting to keep the sequence, send three IDLE_WAKEs
//             // bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
//             // bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
//             // bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
//         //} else if(last_read_crc_failed) {
//             // Last read failed - try a single IDLE_WAKE to re-sync
//             //bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
//         } else {
//             // Normal case - two IDLE_WAKEs keep the CRCs happy
//             // bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
//             // bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
//         }


//         last_read_crc_failed = false;
//         // muting doesn't seem to work? cellvoltages still bouncy during balance
//         // bmb3y_send_command_blocking(BMB3Y_CMD_MUTE);
//         // sleep_us(100);
//         if(mute_steps==0) {
//             bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);
//         } else {
//             bmb3y_send_command_blocking(BMB3Y_CMD_MUTE);
//         }
//         bmb3y_send_command_blocking(BMB3Y_CMD_SNAPSHOT);

//         // If muting, could wait 300us and wake now, which would give more
//         // balancing-enabled time...

//         // As we wait longer than 5ms before the next command, the BMB will go
//         // to comms-idle and need a wakeup. A single CS wakeup each time seems
//         // to work fine, although need to check how multiple BMBs affect things.

//         // uint32_t end = time_us_32();
//         // printf("BMB3Y snapshot took %ld us\n", end - start);

//     } else if(step>=(mute_steps+1) && step <= (mute_steps+5)) {
//         // Read each of the five cellvoltage banks

//         if(step==(mute_steps+1)) {
//             bmb3y_send_wakeup_cs_blocking();

//             // Resume balancing
//             bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);

//             for(int bank=0; bank<5; bank++) {
//                 if(!bmb3y_read_cell_voltage_bank_blocking(model, bank)) {
//                     // Read failed
//                     last_read_crc_failed = true;
//                 } else if(bank==4 && !last_read_crc_failed) {
//                     // All banks read successfully
//                     //printf("CRC: GOOD!!!\n");

//                     model->raw_cell_voltages_millis = millis();

//                     // If balancing was not active, normal cell voltages have now been updated.
//                     if(!model->balancing_active) {
//                         // The staleness threshold will need to be large enough to cope
//                         // with balancing, during which this won't get updated.
//                         model->cell_voltages_millis = millis();
//                     }
//                 }
//             }
            
//             uint32_t end = time_us_32();
//             printf("BMB3Y bank %d read took %ld us\n", step - 1, end - start);
//         }
//     } else if(step==(mute_steps+6)) {
//         // Module temperatures

//         bmb3y_send_wakeup_cs_blocking();

//         //bmb3y_send_command_blocking(BMB3Y_CMD_IDLE_WAKE);

//         bmb3y_read_temperatures_blocking(model);

//         // See what is in F...
//         if(false) {
//             uint8_t rx_buf[100];
//             bmb3y_long_command_get_data_blocking(BMB3Y_CMD_READ_F, rx_buf, 20);
//             printf("F hex: ");
//             // supposedly bytes 2-3 (LE) might contain voltage, reads 0xabff at 56468mV tho
//             // could be /0.8? pr *1.2825? or 12328mV offset? who knows...
//             for(int i=0;i<20;i++) {
//                 printf("%02X ", rx_buf[i]);
//             }
//             printf("\n");
//         }

//         uint32_t end = time_us_32();
//         printf("BMB3Y temp read took %ld us\n", end - start);

//     } else if(step==(mute_steps+7)) {
//         // Enable/disable per-cell balancing as needed.

//         if(!use_slow_mode) {
//             bmb3y_send_wakeup_cs_blocking();
//             balancing_sm_tick(model);
//             bmb3y_send_balancing(model);
//         }

//         uint32_t end = time_us_32();
//         printf("BMB3Y balancing took %ld us\n", end - start);
//     // } else if(step > 7) {
//     //     if(use_slow_mode && model->cell_voltages_millis && model->cell_voltage_min_mV >= CELL_VOLTAGE_SOFT_MIN_mV) {
//     //         // Exit slow mode
//     //         model->cell_voltage_slow_mode = false;
//     //         use_slow_mode = true;
//     //     }
//     }
// }