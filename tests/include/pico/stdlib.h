#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PIO0_IRQ_0 0
#define SYS_CLK_HZ 150000000

static inline void irq_set_exclusive_handler(int irq, void (*handler)(void)) {
    // Mock implementation
}

static inline void irq_set_priority(int irq, int priority) {
    // Mock implementation
}

static inline void irq_set_enabled(int irq, bool enabled) {
    // Mock implementation
}

uint32_t time_us_32(void);

typedef unsigned int uint;

// Timer API, implemented by tests that need it
typedef int32_t alarm_id_t;
typedef int64_t (*alarm_callback_t)(alarm_id_t id, void *user_data);
struct repeating_timer;
typedef bool (*repeating_timer_callback_t)(struct repeating_timer *rt);
struct repeating_timer {
    repeating_timer_callback_t callback;
    void *user_data;
};
alarm_id_t add_alarm_in_ms(uint32_t ms, alarm_callback_t callback, void *user_data, bool fire_if_past);
bool add_repeating_timer_ms(int32_t delay_ms, repeating_timer_callback_t callback, void *user_data, struct repeating_timer *out);
bool add_repeating_timer_us(int64_t delay_us, repeating_timer_callback_t callback, void *user_data, struct repeating_timer *out);
void sleep_ms(uint32_t ms);