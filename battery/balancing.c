#include "../model.h"

// Bleed resistors are 90 ohm, so balancing rate should be:
// 11mC per volt per second
// (but we have voltages in mV and time in ms, so scale accordingly)
#define BALANCE_RATE_NUMERATOR 11
#define BALANCE_RATE_DENOMINATOR 1000000

void update_balancing(bms_model_t *model) {
    // Work out how long has elapsed since last update
    uint32_t elapsed = millis() - model->balancing.last_update_millis;

    if(elapsed < 1000) {
        // only update every second
        return;
    }

    model->balancing.last_update_millis = millis();

    // Go through each cell in turn
    for(int i=0; i<120; i++) {
        // Look up whether balancing is active for this cell
        int mask_index = i / 32;
        int bit_index = i % 32;
        bool balancing_active = (model->balancing.balance_active_mask[mask_index] & (1 << bit_index)) != 0;

        if(balancing_active) {
            // Decrement the balance charge based on cell voltage and elapsed time.
            model->balancing.balance_charge_mC[i] -= (model->cell_voltages_mV[i] * elapsed * BALANCE_RATE_NUMERATOR) / BALANCE_RATE_DENOMINATOR;

            if(model->balancing.balance_charge_mC[i] <= 0) {
                // Stop balancing this cell
                model->balancing.balance_request_mask[mask_index] &= ~(1 << bit_index);
            }
        }
    }
}