#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "app/bms.h"

#ifdef BMS_USB_VBUS_SENSE
#include "config/pins.h"
#include "tusb.h"

// USB VBUS reaches PIN_USB_VBUS_SENSE through a 3.3k/5.1k divider. The chip's
// D+ pull-up is only switched on while the host supplies VBUS, so a switched-off
// host is never back-powered through D+. A reboot drops the pull-up with the
// USB controller, so the host also sees every reboot.
#define USB_VBUS_DEBOUNCE_US 50000

static bool usb_vbus_connected;
static bool usb_vbus_pending;
static absolute_time_t usb_vbus_changed_at;

static void usb_vbus_sense_init(void) {
    gpio_init(PIN_USB_VBUS_SENSE);
    gpio_set_dir(PIN_USB_VBUS_SENSE, GPIO_IN);
    gpio_disable_pulls(PIN_USB_VBUS_SENSE);
    sleep_us(100);

    // The USB stack switches the pull-up on at init
    usb_vbus_connected = gpio_get(PIN_USB_VBUS_SENSE);
    if(!usb_vbus_connected) {
        tud_disconnect();
    }
}

static void usb_vbus_sense_poll(void) {
    bool vbus = gpio_get(PIN_USB_VBUS_SENSE);
    if(vbus == usb_vbus_connected) {
        usb_vbus_pending = false;
        return;
    }
    if(!usb_vbus_pending) {
        usb_vbus_pending = true;
        usb_vbus_changed_at = get_absolute_time();
        return;
    }
    if(absolute_time_diff_us(usb_vbus_changed_at, get_absolute_time()) < USB_VBUS_DEBOUNCE_US) {
        return;
    }

    usb_vbus_pending = false;
    usb_vbus_connected = vbus;
    if(vbus) {
        tud_connect();
    } else {
        tud_disconnect();
    }
}
#endif

#include <stdbool.h>

void core1_entry() {
    flash_safe_execute_core_init();
    while(true) {
        // just sleep for now
        sleep_ms(1000);
    }
}

int main() {
    stdio_usb_init();

#ifdef BMS_USB_VBUS_SENSE
    usb_vbus_sense_init();
#endif

    // Give a chance for USB to enumerate before we start printing.
    sleep_ms(1000);

    multicore_launch_core1(core1_entry);

    bms_init();

    while(true) {
        bms_tick();
#ifdef BMS_USB_VBUS_SENSE
        usb_vbus_sense_poll();
#endif
        synchronize_time();
    }

    return 0;
}
