#include "config/limits.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include <stdio.h>

void init_watchdog() {
    bool was_watchdog = watchdog_enable_caused_reboot();

    if(was_watchdog) {
        // Give time to switch to programming mode in case we're stuck in a reboot loop
        sleep_ms(3000);
    }

    watchdog_enable(BMS_WATCHDOG_TIMEOUT_MS, 0); // second arg enables pause on debug

    if (was_watchdog) {
        warning_printf("Rebooted by Watchdog!\n");
        count_bms_event(ERR_BOOT_WATCHDOG, 0);
    } else {
        info_printf("Clean boot\n");
        count_bms_event(ERR_BOOT_NORMAL, 0);
    }
}
