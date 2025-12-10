#include "hw/duart.h"
#include "hw/time.h"
#include "state_machines/contactors.h"

#include "pico/stdlib.h"

#include <stdio.h>

int main() {
    stdio_usb_init();

    sleep_ms(4000);

    init_duart(&duart1, uart1, 115200, 26, 27, false);

    // contactors_sm_t contactor_sm;
    // sm_init((sm_t*)&contactor_sm, "contactors");

    char str[] = "Lorem ipsum dolor sit amet, consectetur adipiscing elit. "
                "Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. ";

    int i=0;
    while(true) {
        if(duart_send(&duart1, (const uint8_t *)str, sizeof(str)-1)) {
            //printf("Sent successfully\n");
            putchar('1');
        } else {
            //printf("Send failed\n");
            putchar('0');
        }
        if(i++ >= 80) {
            putchar('\n');
            i = 0;
        }
        sleep_us(1);
    }


    while(true) {
        update_millis();
        
        printf("Current millis: %llu\n", current_millis());
        sleep_ms(1000);

        //contactor_sm_tick();

        duart_send_blocking(&duart1, (const uint8_t *)"Hello from BMS!\n", 18);

    }

    return 0;
}
