#pragma once

#include <stdbool.h>

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