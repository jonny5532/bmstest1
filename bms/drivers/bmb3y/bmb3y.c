#include "bmb3y.h"
#include "crc.h"

#include "config/limits.h"
#include "app/model.h"
#include "sys/events/events.h"
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
    
    // TODO: Check whether we need to wake up?

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
    
    // TODO: Check whether we need to wake up?

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
    isospi_send_wakeup_cs_blocking();
}

// Higher level functions

bool bmb3y_set_balancing(uint8_t bitmap[16], bool even) {
    uint8_t tx_buf[50] = {0};
    
    tx_buf[0] = (BMB3Y_CMD_WRITE_CONFIG >> 8) & 0xFF;
    tx_buf[1] = BMB3Y_CMD_WRITE_CONFIG & 0xFF;

    uint8_t balance_mask = even ? 0xAA : 0x55;

    for(int module=0; module<8; module++) {
        // was f3
        tx_buf[2 + module*6] = 0xf3;
        // was 00
        tx_buf[3 + module*6] = 0;

        tx_buf[4 + module*6] = bitmap[module*2 + 1] & balance_mask;
        tx_buf[5 + module*6] = bitmap[module*2] & balance_mask;

        uint16_t calc_crc = crc14(&tx_buf[2 + module*6], 4, 0x0010);

        tx_buf[6 + module*6] = (calc_crc >> 8) & 0xFF;
        tx_buf[7 + module*6] = calc_crc & 0xFF;
    }

    // Balance command:
    // for(int i=0; i<50; i++) {
    //     printf("%02X ", tx_buf[i]);
    // }
    // printf("\n");

    // We skip all of the response bytes
    isospi_write_read_blocking(tx_buf, NULL, 50, 50);

    return true;
}

// void bmb3y_clear_balancing(bms_model_t *model) {
//     balancing_sm_t *balancing_sm = &model->balancing_sm;
//     pause_balancing();

//     // // Clear all balancing requests
//     // for(int i=0; i<4; i++) {
//     //     balancing_sm->balance_request_mask[i] = 0;
//     // }
// }

void bmb3y_send_balancing(bms_model_t *model) {
    balancing_sm_t *balancing_sm = &model->balancing_sm;

    uint8_t tx_buf[150] = {0};
    
    tx_buf[0] = (BMB3Y_CMD_WRITE_CONFIG >> 8) & 0xFF;
    tx_buf[1] = BMB3Y_CMD_WRITE_CONFIG & 0xFF;

    /* 
    The balancing mask consists of four 32-bit ints, with a bit for each cell -
    the LSB of the last int is cell 0.

    However the BMB modules consume the mask 15-bits at a time, so it is
    necessary to re-pack the bits accordingly.
    */

    uint32_t cell_presence_mask[] = CELL_PRESENCE_MASK;

    // balancing_sm->balance_request_mask[0] = 0;
    // balancing_sm->balance_request_mask[1] = 0;
    // balancing_sm->balance_request_mask[2] = 1<<20;
    // balancing_sm->balance_request_mask[3] = 0;

    if(false) {
        uint64_t window = 0;
        // We will throw away the top 8 bits of the first mask int.
        int bits_in_window = -8;
        int mask_idx = 4;

        //printf("Sending mask ");

        for(int module=0; module<8; module++) {
            tx_buf[2 + module*6] = 0xf3;
            tx_buf[3 + module*6] = 0;

            // Pull more bits into the window if needed
            while (bits_in_window < 15 && mask_idx > 0) {
                window |= ((uint64_t)balancing_sm->balance_request_mask[--mask_idx]) << (32 - bits_in_window);
                bits_in_window += 32;
            }

            // Extract 15 bits for this module
            uint16_t balance_bits = (uint16_t)(window >> 49);
            window <<= 15;
            bits_in_window -= 15;

            tx_buf[2 + 2 + module*6] = (uint8_t)(balance_bits & 0xFF);
            tx_buf[2 + 3 + module*6] = (uint8_t)((balance_bits >> 8) & 0xFF);
            
            /* old code (assumes 16 bits per module rather than 15) */
            // int mask_idx2 = 3 - (module / 2);
            // uint32_t mask = balancing_sm->balance_request_mask[mask_idx2];
            // if (module % 2 == 0) {
            //     tx_buf[4 + module*6] = (uint8_t)((mask >> 16) & 0xFF);
            //     tx_buf[5 + module*6] = (uint8_t)((mask >> 24) & 0xFF);
            // } else {
            //     tx_buf[4 + module*6] = (uint8_t)(mask & 0xFF);
            //     tx_buf[5 + module*6] = (uint8_t)((mask >> 8) & 0xFF);
            // }

            //printf("0x%02X%02X ", tx_buf[5 + module*6], tx_buf[4 + module*6]);

            uint16_t calc_crc = crc14(&tx_buf[2 + module*6], 4, 0x0010);

            tx_buf[6 + module*6] = (calc_crc >> 8) & 0xFF;
            tx_buf[7 + module*6] = calc_crc & 0xFF;
        }
    } else {
        int cell_index = 0;
        for(int balance_index=0; balance_index<128; balance_index++) {
            // The balancing register has an extra padding bit after every 15
            // cells, compared to our cell presence mask.
            
            // How many padding bits have been added so far?
            int padding_bits = balance_index / 16;
            // Is the current index a padding bit?
            int is_padding_bit = (balance_index % 16) == 15;

            if(is_padding_bit) {
                // Skip this bit
                continue;
            }

            // Find the position in the cell presence mask (padding bits excluded)
            int mask_word_index = (balance_index - padding_bits) / 32;
            int bit_index = (balance_index - padding_bits) % 32;
            if(!(cell_presence_mask[mask_word_index] & (1 << bit_index))) {
                // This cell is not present, skip it
                continue;
            }

        // for(int balance_index=0; balance_index<128; balance_index++) {
        //     int mask_word_index = balance_index / 32;
        //     int bit_index = balance_index % 32;
        //     if(!(balance_presence_mask[mask_word_index] & (1 << bit_index))) {
        //         // This cell is not present, skip it
        //         continue;
        //     }

            if(balancing_sm->balance_request_mask[cell_index / 32] & (1 << (cell_index % 32))) {
                // This cell is to be balanced

                int module = 7 - (balance_index / 16);
                int module_cell_index = balance_index % 16;

                //printf("Balancing cell %d (mod %d mci %d)\n", cell_index, module, module_cell_index);

                if(module_cell_index < 8) {
                    tx_buf[2 + 2 + module*6] |= (1 << module_cell_index);
                } else {
                    tx_buf[2 + 3 + module*6] |= (1 << (module_cell_index - 8));
                }
            }

            cell_index++;
        }

        // Fill in config bytes and calculate CRCs
        for(int module=0; module<8; module++) {
            tx_buf[2 + 0 + module*6] = 0xf3;
            tx_buf[2 + 1 + module*6] = 0;

            // bytes 2, 3 are already filled in by the loop above

            uint16_t calc_crc = crc14(&tx_buf[2 + 0 + module*6], 4, 0x0010);

            tx_buf[2 + 4 + module*6] = (calc_crc >> 8) & 0xFF;
            tx_buf[2 + 5 + module*6] = calc_crc & 0xFF;
        }

        // testing
        // for(int module=0; module<8; module++) {
        //     tx_buf[2 + module*6] = 0xf3;
        //     tx_buf[3 + module*6] = 0;

        //     // Just balance one cell per module for testing
        //     tx_buf[4 + module*6] = module==7 ? 1 : 0;
        //     tx_buf[5 + module*6] = 0;

        //     uint16_t calc_crc = crc14(&tx_buf[2 + module*6], 4, 0x0010);
        //     tx_buf[6 + module*6] = (calc_crc >> 8) & 0xFF;
        //     tx_buf[7 + module*6] = calc_crc & 0xFF;
        // }
    }

    model->balancing_active = balancing_sm->balance_request_mask[0] != 0 ||
                  balancing_sm->balance_request_mask[1] != 0 ||
                  balancing_sm->balance_request_mask[2] != 0 ||
                  balancing_sm->balance_request_mask[3] != 0;

    // We skip all of the response bytes
    isospi_write_read_blocking(tx_buf, NULL, 50, 50);

    
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

// For some reason, the returned BMB3Y CRCs often seem to be XORed with one of
// three different patterns. It is unclear whether this is by design or not, and
// it doesn't seem possible to predict which pattern will be used when it
// happens. However this totally resolves all CRC mismatch issues and sequencing
// workarounds.
bool crc_matches(uint16_t received_crc, uint16_t calculated_crc) {
    if(received_crc == calculated_crc) {
        //printf("CRC matched directly\n");
        return true;
    }
    // This is the CRC14 polynomial (with the 15th bit set)
    if((received_crc ^ 0x425b) == calculated_crc) {
        //printf("CRC matched with 0x425b xor\n");
        return true;
    }
    // This is the above, left shifted by 1
    if((received_crc ^ 0x84b6) == calculated_crc) {
        //printf("CRC matched with 0x84b6 xor\n");
        return true;
    }
    // This is the two previous patterns xored together
    if((received_crc ^ 0xc6ed) == calculated_crc) {
        //printf("CRC matched with 0xc6ed xor\n");
        return true;
    }
    printf("CRC mismatch, xor: %04X\n", received_crc ^ calculated_crc);
    return false;
}

bool bmb3y_read_cell_voltage_bank_blocking(bms_model_t *model, int bank_index) {
    uint8_t rx_buf[72];
    uint32_t cmd = READ_COMMANDS[bank_index];
    if (!bmb3y_long_command_get_data_blocking(cmd, rx_buf, 72)) {
        printf("BMB3Y read failed for cmd 0x%02lX\n", cmd);
        count_bms_event(ERR_BMB_READ_ERROR, 0x0100000000000000 | bank_index);
        return false;
    }
    // Short commands need different CRC handling
    // uint16_t cmd = SHORT_READ_COMMANDS[bank_index];
    // if (!bmb3y_short_command_get_data_blocking(cmd, rx_buf, 72)) {
    //     printf("BMB3Y read failed for cmd 0x%02X\n", cmd);
    //     count_bms_event(ERR_BMB_READ_ERROR, 0x0100000000000000 | bank_index);
    //     return false;
    // }

    uint8_t cell_offset = bank_index * 3;
    uint32_t cell_presence_mask[] = CELL_PRESENCE_MASK;
    bool crc_ok = true;

    for(int module=0; module<NUM_MODULE_VOLTAGES; module++) {
        uint16_t module_crc = (uint16_t)(rx_buf[module * 9 + 6] << 8) | (uint16_t)(rx_buf[module * 9 + 7]);
        uint16_t calc_crc = crc14(&rx_buf[module * 9], 6, 0x1000);

        if(!crc_matches(module_crc, calc_crc)) {
            printf("CRC: %s %d %02X %02X %02X %02X %02X %02X %02X %02X %02X calc %04X xor %04X or %04X\n",
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
                calc_crc,
                calc_crc ^ 0x425b,
                calc_crc ^ 0xc6ed
            );
            count_bms_event(ERR_BMB_CRC_MISMATCH, 0x0100000000000000 | ((uint64_t)bank_index << 48) | ((uint64_t)module << 40) | (module_crc << 16) | calc_crc);
            crc_ok = false;
            continue;
        }

        // Go through each cell (three per bank)
        for(int cell=0; cell<3; cell++) {
            int raw_cell_index = (module * 15) + cell_offset + cell;

            uint8_t cell_index = 0;
            // TODO, do this better
            for(int bit=0;bit<128;bit++) {
                uint32_t mask_word = cell_presence_mask[bit / 32];
                if(mask_word & (1 << (bit % 32))) {
                    if(bit == raw_cell_index) {
                        break;
                    }
                    cell_index++;
                }
            }

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

            model->raw_cell_voltages_mV[cell_index] = converted;

            // Don't store voltages during balancing, as they are unstable
            // (TODO: can't use the unstable voltages flag here since it doesn't
            // get unset until we've finished reading)
            if(!model->balancing_active) {
                model->cell_voltages_mV[cell_index] = converted;
            }
            
            cell_index++;
        }
    }
    return crc_ok;
}

bool bmb3y_read_temperatures_blocking(bms_model_t *model) {
    uint8_t rx_buf[90];

    // LFP seems to have a temp value in BMB3Y_CMD_READ_TEMPS3 0:1
    // NMC seems to use TEMPS and 2:3 like D/T's code

    if(!bmb3y_short_command_get_data_blocking(BMB3Y_CMD_READ_TEMPS, rx_buf, 64)) {
        printf("BMB3Y temperature read failed\n");
        count_bms_event(ERR_BMB_READ_ERROR, 0x0200000000000000);
        // printf("temptest failed hex: ");
        // for(int i=0;i<8;i++) {
        //     printf("%02X ", rx_buf[i]);
        // }
        // printf("\n");
        // isosnoop_print_buffer();
        return false;
    }

    bool crc_ok = true;
    for(int module=0; module<NUM_MODULE_TEMPS; module++) {
        // Each module has 8 bytes: 6 bytes data, 2 bytes CRC
        // The temp is in bytes 2 and 3 (little-endian)
        
        uint16_t module_crc = (uint16_t)(rx_buf[module * 8 + 6] << 8) | (uint16_t)(rx_buf[module * 8 + 7]);
        uint16_t calc_crc = crc14(&rx_buf[module * 8], 6, 0x0010);

        // printf("temptest gooded hex: ");
        // for(int i=0;i<8;i++) {
        //     printf("%02X ", rx_buf[module * 8 + i]);
        // }
        // printf("\n");
        //isosnoop_print_buffer();

        if(!crc_matches(module_crc, calc_crc)) {
            printf("Bad Temp CRC on module %d: msg 0x%04X calc 0x%04X xor %04x or %04x\n", 
                module, module_crc, calc_crc, calc_crc ^ 0x425b, calc_crc ^ 0xc6ed);
            crc_ok = false;
            if(millis64() > 2000) {
                count_bms_event(ERR_BMB_CRC_MISMATCH, 0x0200000000000000 | ((uint64_t)module << 48) | (module_crc << 16) | calc_crc);
            }
            continue;
        }

        // D/T's algorithm: (TEMPS)
        if(true) {
            // // Unlike everything else, temps are little-endian??
            int16_t raw_temp = 
                (int16_t)(rx_buf[module * 8 + 2]) |
                (int16_t)(rx_buf[module * 8 + 3] << 8);

            model->module_temperatures_dC[module] = raw_temp - 1131;
            //printf("module temp %d is %d dC\n", module, model->module_temperatures_dC[module]);
        }
        
        // LFP algo: (TEMPS3)
        if(false) {
            int16_t raw_temp = 
                (int16_t)(rx_buf[module * 8 + 0]) |
                (int16_t)(rx_buf[module * 8 + 1] << 8);

            // super-crude cal, 28c = 0x4300, 100c = 0x6e00
            // FIXME - do proper thermistor conversion
            model->module_temperatures_dC[module] = ((raw_temp - 0x4300) * (1000 - 280)) / (0x6e00 - 0x4300) + 280;
        }
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


int bmb3y_timestep_offset = 0;
// Cut balancing pause cycles short so we can get back to balancing sooner.
const int PAUSE_CYCLE_PERIOD = 10;

void bmb3y_tick(bms_model_t *model) {
    int period_mask = 0x3f;
    int step = ((timestep() + bmb3y_timestep_offset) & period_mask) - 5; // was 3f

    uint32_t start = time_us_32();

    if(step == 0) {
        // Wake up BMBs, take snapshot
        // Takes about 90us
        bmb3y_send_wakeup_cs_blocking();
        bmb3y_send_command_blocking(BMB3Y_CMD_SNAPSHOT);
    } else if(step == 1) {
        // Wake up BMBs, read voltages and temperatures, setup balancing
        // Takes about 6ms
        bmb3y_send_wakeup_cs_blocking();
        bmb3y_read_cell_voltages_blocking(model);
        bmb3y_read_temperatures_blocking(model);
        balancing_sm_tick(model);
        bmb3y_send_balancing(model);
    } else if(step == PAUSE_CYCLE_PERIOD && model->balancing_sm.is_pause_cycle) {
        // Cut the cycle short if we're in a pause cycle by adjusting the offset.
        bmb3y_timestep_offset = (bmb3y_timestep_offset + (period_mask + 1) - PAUSE_CYCLE_PERIOD - 1) & period_mask;
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