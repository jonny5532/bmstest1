#include <stdio.h>

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

void watchdog_init() {
    if (watchdog_enable_caused_reboot()) {
        printf("Rebooted by Watchdog!\n");
    } else {
        printf("Clean boot\n");
    }

    watchdog_enable(5000, 0); // second arg enables pause on debug

    
}