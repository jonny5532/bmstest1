#include "duart.h"
#include "../pins.h"
#include "../chip/time.h"

const uint8_t str[] = {
    'K', 'e', 'e', 'p', 'A', 'l', 'i', 'v',
};

void init_internal_serial() {
    init_duart(&duart0, 115200, PIN_INTERNAL_SERIAL_TX, PIN_INTERNAL_SERIAL_RX, false); //9375000 works!
}

void internal_serial_tick() {
    if((timestep() & 63) == 0) {
        // Send "KeepAliv" message periodically
        duart_send_packet(&duart0, (uint8_t *)str, 8);
    }
}
