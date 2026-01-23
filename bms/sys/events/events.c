#include "events.h"
#include "../../lib/math.h"

#include <stdio.h>

const char* EVENT_TYPE_NAMES[] = {
#define X(name, _1, _2) #name,
    EVENT_TYPES(X)
#undef X
};

const bms_event_level_t EVENT_TYPE_LEVELS[] = {
#define X(_1, level, _2) level,
    EVENT_TYPES(X)
#undef X
};

// Leeway times in deciseconds (tenths of a second)
const uint16_t EVENT_TYPE_LEEWAY[] = {
#define X(_1, _2, leeway) leeway/100,
    EVENT_TYPES(X)
#undef X
};

const char* EVENT_LEVELS[] = {
    "NONE",
    "INFO",
    "WARNING",
    "CRITICAL",
    "FATAL"
};

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

void record_bms_event(bms_event_type_t type, uint64_t data, bool repeat) {
    if (type >= ERR_HIGHEST) return;

    bms_event_slot_t *slot = &bms_event_slots[type];
    millis_t now = millis();

    slot->timestamp = now;
    slot->data64 = data;
    if(slot->level == LEVEL_NONE || repeat) {
        // Only increment count if new event or repeating
        slot->count = sadd_u16(slot->count, 1);
    }

    bms_event_level_t new_level = EVENT_TYPE_LEVELS[type];

    if(new_level == LEVEL_WARNING && EVENT_TYPE_LEEWAY[type] > 0 && slot->count >= EVENT_TYPE_LEEWAY[type]) {
        // Escalate if warning event has occurred too many times
        new_level = LEVEL_FATAL;
    }

    if(slot->level == LEVEL_NONE) {
        slot->level = new_level;
        recalculate_highest_level();
    }
}

void clear_bms_event(bms_event_type_t type) {
    if (type >= ERR_HIGHEST) return;

    bms_event_slot_t *slot = &bms_event_slots[type];
    if(slot->level == LEVEL_FATAL) {
        // Can't clear a fatal event
        return;
    }

    if(slot->level != LEVEL_NONE) {
        slot->level = LEVEL_NONE;
        recalculate_highest_level();
    }
}

void print_bms_events() {
    for(int i = 0; i < ERR_HIGHEST; i++) {
        bms_event_slot_t *slot = &bms_event_slots[i];
        if (slot->count > 0) {
            printf("Event %s: Level %s, Count %u, Last Timestamp %lu, Data 0x%016llX\n",
                   EVENT_TYPE_NAMES[i], EVENT_LEVELS[slot->level], slot->count, slot->timestamp, slot->data64);
        }
    }
}

millis_t last_tick_at;
void events_tick() {
    millis_t now = millis();
    // Get elapsed time in deciseconds
    uint16_t elapsed_ds = (now - last_tick_at)/100;
    // Update last tick time by the truncated elapsed time (to avoid drift)
    last_tick_at += elapsed_ds * 100;

    bool escalated = false;
    for(int i = 0; i < ERR_HIGHEST; i++) {
        bms_event_slot_t *slot = &bms_event_slots[i];
        if(EVENT_TYPE_LEVELS[i] != LEVEL_CRITICAL) {
            // Only critical type events can escalate
            continue;
        }

        uint16_t max_accumulator = EVENT_TYPE_LEEWAY[i];
        if(slot->level == LEVEL_CRITICAL) {
            // event currently critical, increment the accumulator
            if(sadd_u16(slot->accumulator, elapsed_ds) >= max_accumulator) {
                // we've hit the leeway limit, escalate
                slot->accumulator = max_accumulator;
                slot->level = LEVEL_FATAL;
                escalated = true;
            } else {
                // just increment
                slot->accumulator += elapsed_ds;
            }
        } else {
            // event noncritical, or already fatal, decrement accumulator
            slot->accumulator = ssub_u16(slot->accumulator, elapsed_ds);
        }
    }

    if(escalated) recalculate_highest_level();
}
