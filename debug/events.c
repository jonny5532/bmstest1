#include "events.h"

bms_event_slot_t bms_event_slots[BMS_EVENT_TYPE_COUNT] = {0};
uint16_t bms_event_log_next_index = 0;

uint16_t highest_level = BMS_EVENT_LEVEL_INFO;

uint16_t get_highest_event_level() {
    return highest_level;
}

void log_bms_event(uint16_t level, uint16_t type, uint64_t data) {
    if (type >= BMS_EVENT_TYPE_COUNT) return;

    bms_event_slot_t *slot = &bms_event_slots[type];
    millis_t now = millis();

    slot->timestamp = now;
    slot->data = data;
    slot->level = level;
    slot->count += 1;

    // Recalculate highest level
    highest_level = BMS_EVENT_LEVEL_INFO;
    for(int i = 0; i < BMS_EVENT_TYPE_COUNT; i++) {
        if (bms_event_slots[i].level > highest_level) {
            highest_level = bms_event_slots[i].level;
        }
    }
}
