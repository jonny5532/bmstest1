#include "balancing.h"

#include "../model.h"

#define AUTO_BALANCING_PERIOD_MS 60000 // how long to wait between auto-balancing sessions
#define PERIODS_PER_MV 1 // how many balancing periods per mV above minimum
#define BALANCE_MIN_OFFSET_MV 3 // minimum voltage difference to balance

static bool good_conditions_for_balancing(bms_model_t *model) {
    // Check if conditions are suitable for balancing
    return true;
}

// Update the balance request mask based on the remaining balance times and
// whether even or odd cells are being balanced this cycle. Decrement the
// remaining balance times by the given amount.
static void update_balance_requests(balancing_sm_t *balancing_sm, int16_t decrement) {
    for(int cell=0; cell<120; cell++) {
        int mask_index = cell / 32;
        int bit_index = cell % 32;

        if(balancing_sm->balance_time_remaining[cell] > 0 && (balancing_sm->even_cells == ((cell % 2) == 0))) {
            balancing_sm->balance_time_remaining[cell] -= decrement;
            balancing_sm->balance_request_mask[mask_index] |= (1 << bit_index);
        } else {
            balancing_sm->balance_request_mask[mask_index] &= ~(1 << bit_index);
        }
    }
    printf("Balance mask now: %08X %08X %08X %08X\n",
        balancing_sm->balance_request_mask[3],
        balancing_sm->balance_request_mask[2],
        balancing_sm->balance_request_mask[1],
        balancing_sm->balance_request_mask[0]
    );
}

// Calculate how long to balance a cell for, based on its voltage above the
// minimum cell voltage, in BMB update periods.
int16_t calculate_balance_time(int16_t voltage_mV, int16_t min_voltage_mV) {
    // Calculate balance time in BMB-update-periods based on voltage difference

    // TODO - figure out multiplier, and base on SoC/OCV curve

    int16_t diff = voltage_mV - min_voltage_mV - BALANCE_MIN_OFFSET_MV;
    if(diff < 0) {
        return 0;
    }
    return diff*PERIODS_PER_MV;
}

// Start the balancing process by determining which cells need balancing and for
// how long, and updating the times and balance request mask accordingly.
static bool start_balancing(bms_model_t *model) {
    balancing_sm_t *balancing_sm = &model->balancing_sm;

    // TODO - check staleness of cell voltages

    // Determine which cells need balancing
    int16_t min_cell_voltage = 0x7FFF;
    for(int cell=0; cell<120; cell++) {
        // TODO - ignore unused cells
        if(model->cell_voltages_mV[cell] > 2500 && model->cell_voltages_mV[cell] < min_cell_voltage) {
            min_cell_voltage = model->cell_voltages_mV[cell];
        }
    }

    // Set balance times based on how far above minimum voltage each cell is
    for(int cell=0; cell<120; cell++) {
        int16_t voltage = model->cell_voltages_mV[cell];
        // 3mV hysteresis, negatives will be ignored
        model->balancing_sm.balance_time_remaining[cell] = calculate_balance_time(voltage, min_cell_voltage);
    }

    update_balance_requests(balancing_sm, 0);

    return true;
}


// Decrement balance times by one period, check if individual cells are done
// balancing, and update the request mask accordingly.
static void decrement_balance_times_and_update(balancing_sm_t *balancing_sm) {
    update_balance_requests(balancing_sm, 1);
}

// Check if the balance mask is all zero, indicating balancing is finished.
static bool finished_balancing(balancing_sm_t *balancing_sm) {
    for(int i=0; i<4; i++) {
        if(balancing_sm->balance_request_mask[i] != 0) {
            return false;
        }
    }
    return true;
}

// Note - this is called every BMB send, not every timestep, since it needs to
// be synchronized with the BMB sends.
void balancing_sm_tick(bms_model_t *model) {
    balancing_sm_t *balancing_sm = &model->balancing_sm;

    switch(balancing_sm->state) {
        case BALANCING_STATE_IDLE:
            if(state_timeout((sm_t*)balancing_sm, AUTO_BALANCING_PERIOD_MS) && good_conditions_for_balancing(model)) {
                // Start balancing
                if(start_balancing(model)) {
                    state_transition((sm_t*)balancing_sm, BALANCING_STATE_ACTIVE);
                }
            }
            break;

        case BALANCING_STATE_ACTIVE:
            // Update the mask and decrement the times.
            update_balance_requests(balancing_sm, 1);
            // Switch even/odd cells for next time
            balancing_sm->even_cells = !balancing_sm->even_cells;

            if(finished_balancing(balancing_sm)) {
                state_transition((sm_t*)balancing_sm, BALANCING_STATE_IDLE);
            }
            break;
        
        default:
            // unknown state, go to idle
            state_transition((sm_t*)balancing_sm, BALANCING_STATE_IDLE);
            break;
    }
}











// Bleed resistors are 90 ohm, so balancing rate should be:
// 11mC per volt per second
// (but we have voltages in mV and time in ms, so scale accordingly)
//#define BALANCE_RATE_NUMERATOR 11
//#define BALANCE_RATE_DENOMINATOR 1000000



// void update_balancing(bms_model_t *model) {
//     // Work out how long has elapsed since last update
//     uint32_t elapsed = millis() - model->balancing.last_update_millis;

//     if(elapsed < 1000) {
//         // only update every second
//         return;
//     }

//     model->balancing.last_update_millis = millis();

//     // Go through each cell in turn
//     for(int i=0; i<120; i++) {
//         // Look up whether balancing is active for this cell
//         int mask_index = i / 32;
//         int bit_index = i % 32;
//         bool balancing_active = (model->balancing.balance_active_mask[mask_index] & (1 << bit_index)) != 0;

//         if(balancing_active) {
//             // Decrement the balance charge based on cell voltage and elapsed time.
//             model->balancing.balance_charge_mC[i] -= (model->cell_voltages_mV[i] * elapsed * BALANCE_RATE_NUMERATOR) / BALANCE_RATE_DENOMINATOR;

//             if(model->balancing.balance_charge_mC[i] <= 0) {
//                 // Stop balancing this cell
//                 model->balancing.balance_request_mask[mask_index] &= ~(1 << bit_index);
//             }
//         }
//     }
// }