#include "balancing.h"

#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "config/limits.h"
#include "app/model.h"

#include "pico/stdlib.h"

//#define AUTO_BALANCING_PERIOD_MS 30000 // how long to wait between auto-balancing sessions
#define PERIODS_PER_MV 5 //50 // how many balancing periods per mV above minimum
#define BALANCE_MIN_OFFSET_MV 2 // minimum voltage difference to balance
#define PAUSE_AFTER_N_PERIODS 4 // pause balancing for a shortened period after N periods to get a good voltage reading

static bool good_conditions_for_balancing(bms_model_t *model) {
    // Check if conditions are suitable for balancing

    // check for target balance point (mV or SoC?)

    // check for non-stale values

    // check for low current variance?

    if(get_highest_event_level() == LEVEL_FATAL) {
        // Don't balance if there are fatal faults
        return false;
    }

    if(model->cell_voltage_max_mV > get_cell_voltage_soft_max_mV(model)) {
        // Don't balance if any cell is above the soft max voltage

        // TODO - maybe only restrict this if we're also charging? The balancing
        // itself isn't the problem, it is the reduced frequency of voltage
        // updates during balancing meaning we will react more slowly to an
        // overvoltage (or recovery).

        return false;
    }

    if(model->cell_voltage_min_mV < get_minimum_balancing_voltage_mV(model)) {
        // Don't balance if any cell is below the minimum balancing voltage
        return false;
    }

    if(model->cell_voltage_slow_mode) {
        // Don't balance in slow mode
        return false;
    }

    return model->auto_balancing_period_ms > 0; // Only balance if auto-balancing is enabled
    
    //return true;
}

// Update the balance request mask based on the remaining balance times and
// whether even or odd cells are being balanced this cycle. Decrement the
// remaining balance times by the given amount.
static void update_balance_requests(balancing_sm_t *balancing_sm, int16_t decrement, bool skipping) {
    uint32_t start = time_us_32();
    // Clear all balancing requests
    for(int i=0; i<4; i++) {
        balancing_sm->balance_request_mask[i] = 0;
    }

    bool any_balancing = false;
    int last_balancing_physical_index = -2;

    uint32_t cell_presence_mask[] = CELL_PRESENCE_MASK;
    int logical_index = 0;

    // Go through each of the masks in order
    for (int i = 0; i < 4; i++) {
        uint32_t mask = cell_presence_mask[i];
        while (mask) {
            // Find the next present cell in the mask
            int bit_index = __builtin_ctz(mask);
            // Calculate the physical index of that cell
            int physical_index = i * 32 + bit_index;

            if (logical_index < 120) {
                // We can't balance physically adjacent cells (due to the BMB resistor arrangement)
                bool is_physically_adjacent = (physical_index - last_balancing_physical_index) == 1;

                if (balancing_sm->balance_time_remaining[logical_index] > 0 && !is_physically_adjacent) {
                    balancing_sm->balance_time_remaining[logical_index] -= decrement;
                    balancing_sm->balance_request_mask[logical_index / 32] |= (1 << (logical_index % 32));
                    any_balancing = true;
                    last_balancing_physical_index = physical_index;
                }
            }

            logical_index++;
            // Clear the bit we just processed
            mask &= ~(1u << bit_index);
        }
    }

    uint32_t end = time_us_32();
    debug_printf("Updating balance requests took %zu us\n", (size_t)(end - start));

    // if(!any_balancing && !skipping) {
    //     // Nothing to balance on this cycle, skip to the next one
    //     balancing_sm->even_cells = !balancing_sm->even_cells;
    //     update_balance_requests(balancing_sm, decrement, true);
    //     return;
    // }


    debug_printf("Balance mask now: %08lX %08lX %08lX %08lX\n",
        balancing_sm->balance_request_mask[3],
        balancing_sm->balance_request_mask[2],
        balancing_sm->balance_request_mask[1],
        balancing_sm->balance_request_mask[0]
    );
}

// Calculate how long to balance a cell for, based on its voltage above the
// minimum cell voltage, in BMB update periods.
int32_t calculate_balance_time(int16_t voltage_mV, int16_t min_voltage_mV, bms_model_t *model) {
    // Calculate balance time in BMB-update-periods based on voltage difference

    // TODO - figure out multiplier, and base on SoC/OCV curve

    //return 10;

    int32_t diff = voltage_mV - min_voltage_mV - model->balance_min_offset_mV;
    if(diff < 0) {
        return 0;
    }
    return diff * model->balancing_periods_per_mV;
}

// Start the balancing process by determining which cells need balancing and for
// how long, and updating the times and balance request mask accordingly.
static bool start_balancing(bms_model_t *model) {
    balancing_sm_t *balancing_sm = &model->balancing_sm;

    if(model->cell_voltage_max_mV < model->balancing_start_voltage_mV) {
        // Don't start balancing if we haven't reached the start voltage threshold
        return false;
    }

    // TODO - check staleness of cell voltages

    // Determine which cells need balancing

    int16_t min_cell_voltage = INT16_MAX;
    for(int cell=0; cell<NUM_CELLS; cell++) {
        if(model->cell_voltages_mV[cell] > 2500 && model->cell_voltages_mV[cell] < min_cell_voltage) {
            min_cell_voltage = model->cell_voltages_mV[cell];
        }
    }

    // Temporary higher-res storage
    uint32_t balance_time_remaining[120];
    int32_t max_balance_time = 0;

    // Set balance times based on how far above minimum voltage each cell is

    for(int cell=0; cell<NUM_CELLS; cell++) {
        int16_t voltage = model->cell_voltages_mV[cell];
        balance_time_remaining[cell] = calculate_balance_time(voltage, min_cell_voltage, model);
        if(balance_time_remaining[cell] > max_balance_time) {
            max_balance_time = balance_time_remaining[cell];
        }
    }

    // Make sure we fit within int16_t for the balance times
    if(max_balance_time > INT16_MAX) {
        // We don't fit, scale all the times down proportionally
        float scale = (float)INT16_MAX / max_balance_time;
        for(int cell=0; cell<NUM_CELLS; cell++) {
            int32_t scaled_time = (int32_t)(balance_time_remaining[cell] * scale);
            model->balancing_sm.balance_time_remaining[cell] = (int16_t)(scaled_time > INT16_MAX ? INT16_MAX : scaled_time);
        }
    } else {
        for(int cell=0; cell<NUM_CELLS; cell++) {
            model->balancing_sm.balance_time_remaining[cell] = (int16_t)balance_time_remaining[cell];
        }
    }

    update_balance_requests(balancing_sm, 0, false);

    return true;
}

// Decrement balance times by one period, check if individual cells are done
// balancing, and update the request mask accordingly.
static void decrement_balance_times_and_update(balancing_sm_t *balancing_sm) {
    update_balance_requests(balancing_sm, 1, false);
}

void clear_balancing_times(balancing_sm_t *balancing_sm) {
    for(int cell=0; cell<120; cell++) {
        balancing_sm->balance_time_remaining[cell] = 0;
    }
}

void pause_balancing(balancing_sm_t *balancing_sm) {
    // Clear all balancing requests
    for(int i=0; i<4; i++) {
        balancing_sm->balance_request_mask[i] = 0;
    }
}

// Check if the there is any balancing still to be done.
static bool finished_balancing(balancing_sm_t *balancing_sm) {
    // Check if any cells still have balance time remaining
    for(int cell=0; cell<120; cell++) {
        if(balancing_sm->balance_time_remaining[cell] > 0) {
            return false;
        }
    }

    // We might be in the last period where we have already decremented the
    // times, but the mask is still set, in which case we would not have
    // finished yet.
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

    balancing_sm->is_pause_cycle = false;

    switch(balancing_sm->state) {
        case BALANCING_STATE_IDLE:
            if(state_timeout((sm_t*)balancing_sm, model->auto_balancing_period_ms) && good_conditions_for_balancing(model)) {
                // Start balancing
                if(start_balancing(model)) {
                    // Start the pause cadence fresh each session
                    balancing_sm->pause_counter = 0;
                    state_transition((sm_t*)balancing_sm, BALANCING_STATE_ACTIVE);
                }
            }
            break;

        case BALANCING_STATE_ACTIVE:
        if(!good_conditions_for_balancing(model)) {
                // Stop balancing if conditions are no longer good
                pause_balancing(balancing_sm);
                state_transition((sm_t*)balancing_sm, BALANCING_STATE_IDLE);
                clear_balancing_times(balancing_sm);
                return;
            }

            if(PAUSE_AFTER_N_PERIODS>0 && (balancing_sm->pause_counter++) == PAUSE_AFTER_N_PERIODS) {
                // Skip balancing this period
                pause_balancing(balancing_sm);
                balancing_sm->pause_counter = 0;
                balancing_sm->is_pause_cycle = true;
                break;
            }

            // Update the mask and decrement the times.
            update_balance_requests(balancing_sm, 1, false);
            // Switch even/odd cells for next time
            //balancing_sm->even_cells = !balancing_sm->even_cells;

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
