#include "duart.h"
#include "../pins.h"

const uint8_t str[] = {
    'K', 'e', 'e', 'p', 'A', 'l', 'i', 'v',
};

void init_internal_serial() {
    init_duart(&duart1, 115200, PIN_INTERNAL_SERIAL_TX, PIN_INTERNAL_SERIAL_RX, true); //9375000 works!
}

void internal_serial_tick() {
    // Send "KeepAliv" message periodically
    duart_send_packet(&duart1, (uint8_t *)str, 8);
}
