#include "hw/internal_adc.h"
#include "hw/duart.h"
#include "hw/time.h"
#include "state_machines/contactors.h"

#include "pico/stdlib.h"

#include <stdio.h>

#define CRC16_INIT                  ((uint16_t)-1l)
void memcpy_with_crc16(uint8_t *dest, const uint8_t *src, size_t len, uint16_t *crc16);

int main() {
    stdio_usb_init();

    sleep_ms(4000);

    init_adc();
    //230400
    //init_duart(&duart0, 230400, 0, 1, true);
    init_duart(&duart1, 9375000, 26, 27, true);

    contactors_sm_t contactor_sm;
    sm_init((sm_t*)&contactor_sm, "contactors");

    //char str[] = "Lorem. Lorem. Lorem. Lorem.\n";
    uint8_t str[64];
    // ipsum dolor sit amet, consectetur adipiscing elit. "
      //          "Sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. ";

    int i=0;
    int n=0;
    while(true) {
        sprintf(&str[0], "%08d", n++);
        // str[0] = 0xff; // sync byte
        // str[1] = 8-1; // length byte

        // // Add CRC16
        // uint8_t buf[64];
        // *((uint16_t*)&buf[10]) = CRC16_INIT;
        // memcpy_with_crc16(&buf[0], &str[0], 10, (uint16_t*)&buf[10]);

        if(n>99999999) {
            n = 0;
        }

        // printf("Sending: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
        //     buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
        //     buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]);

        if(duart_send_packet(&duart1, (uint8_t *)str, 8)) {
            //printf("Sent successfully\n");
            //putchar('1');
        } else {
            //printf("Send failed\n");
            //putchar('0');
        }
        if(i++ >= 80) {
            //putchar('\n');
            printf("Temp: %d C\n", get_temperature_c_times10());
            i = 0;


        }
        sleep_us(1);

        /*
        uint8_t *buf;
        size_t available;
        while(true) {
            size_t len = duart_peek(&duart1, &buf, &available);
            if(len > 0) {
                // if(buf[0] != 0xff || buf[1] == 0) {
                //     // Invalid start byte, flush one byte
                //     duart_flush(&duart1, 1);
                //     continue;
                // }
                size_t payload_len = buf[0] + 1;

                // if(payload_len>250) {
                //     // Invalid length, flush one byte
                //     duart_flush(&duart1, 1);
                //     continue;
                // }
                if((payload_len+1) <= len) {
                    // We have a full contiguous message
                    printf("[%.*s]", payload_len, &buf[1]);
                    duart_flush(&duart1, payload_len+1);
                } else if((payload_len+1) <= available) {
                    // We have a full message, but it wraps around the end of the buffer
                    printf("(%.*s", len-1, &buf[1]);
                    duart_flush(&duart1, len);
                    
                    duart_peek(&duart1, &buf, &available);
                    printf("%.*s)", payload_len - (len-1), &buf[0]);
                    duart_flush(&duart1, payload_len - (len-1));

                    //printf("wa %d\n", dma_channel_hw_addr(duart1.rx_dma_channel)->write_addr - (uint)duart1.rx_buffer);
                } else {
                    // Incomplete message
                    break;
                }
            } else {
                break;
            }
        }*/

        while(true) {
            uint8_t buf[256];
            size_t len = duart_read_packet(&duart1, buf, sizeof(buf));
            if(len > 0) {
                printf("<%.*s>", len, buf);
            } else {
                break;
            }
        }


        //printf("ADC 12V: raw=%u smoothed=%u\n", adc_samples_raw[0], adc_samples_smoothed[0]);

        update_millis();

        contactor_sm_tick(&contactor_sm);
    }


    while(true) {
        update_millis();
        
        printf("Current millis: %llu\n", current_millis());
        sleep_ms(1000);

    }

    return 0;
}
