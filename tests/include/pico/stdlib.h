#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PIO0_IRQ_0 0
#define PIO1_IRQ_0 1
#define PIO2_IRQ_0 2
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