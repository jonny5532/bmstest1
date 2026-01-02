#include "events.h"

bms_event_slot_t bms_event_slots[ERR_HIGHEST] = {0};
uint16_t bms_event_log_next_index = 0;

uint16_t highest_level = LEVEL_NONE;

uint16_t get_highest_event_level() {
    return highest_level;
}

static void recalculate_highest_level() {
    highest_level = LEVEL_NONE;
    for(int i = 0; i < ERR_HIGHEST; i++) {
        if (bms_event_slots[i].level > highest_level) {
            highest_level = bms_event_slots[i].level;
        }
    }
}

void log_bms_event(uint16_t type, uint16_t level, uint64_t data) {
    if (type >= ERR_HIGHEST) return;

    bms_event_slot_t *slot = &bms_event_slots[type];
    millis_t now = millis();

    slot->timestamp = now;
    slot->data64 = data;
    slot->count += 1;

    if(level != slot->level) {
        slot->level = level;
        recalculate_highest_level();
    }
}

void clear_bms_event(uint16_t type) {
    if (type >= ERR_HIGHEST) return;

    bms_event_slot_t *slot = &bms_event_slots[type];
    if(slot->level != LEVEL_NONE) {
        slot->level = LEVEL_NONE;
        recalculate_highest_level();
    }
}

void print_bms_events() {
    for(int i = 0; i < ERR_HIGHEST; i++) {
        bms_event_slot_t *slot = &bms_event_slots[i];
        if (slot->count > 0) {
            printf("Event %d: Level %d, Count %u, Last Timestamp %u, Data 0x%016llX\n",
                   i, slot->level, slot->count, slot->timestamp, slot->data64);
        }
    }
}
