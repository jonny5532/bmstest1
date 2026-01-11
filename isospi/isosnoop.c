#include "isosnoop.h"
#include "isosnoop.pio.h"

#include "../hw/allocations.h"

#include "hardware/dma.h"
#include "hardware/pio.h"

#include <stdio.h>

#define ISOSNOOP_SM 0

#define DMA_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
#define PIO_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
static uint dma_channel_rx;
dma_channel_hw_t *dma_chan;

#define ISOSNOOP_BUFFER_BITS 10 // 2**n byte buffer

char read_buffer[1<<ISOSNOOP_BUFFER_BITS] __attribute__((aligned(1<<ISOSNOOP_BUFFER_BITS)));
uint32_t last_write_addr;

void isosnoop_dma_setup(PIO pio, uint sm);

void isosnoop_setup(uint rx_pin_base, uint sampling_pin, uint rx_and_pin, uint timer_disable_pin) {
    isosnoop_pio_setup(ISOSNOOP_PIO, rx_pin_base, sampling_pin, rx_and_pin, timer_disable_pin);
    isosnoop_dma_setup(ISOSNOOP_PIO, ISOSNOOP_SM);

    dma_chan = dma_channel_hw_addr(dma_channel_rx);
    last_write_addr = dma_chan->write_addr;
    //uint32_t buf_end = (uint32_t)(read_buffer + sizeof(read_buffer));
}

void isosnoop_dma_setup(PIO pio, uint sm) {
    dma_channel_rx = dma_claim_unused_channel(false);
    if (dma_channel_rx < 0) {
        panic("No free dma channels");
    }

    for(int i=0; i<sizeof(read_buffer); i++) {
        read_buffer[i] = i;
    }

    dma_channel_config config_rx = dma_channel_get_default_config(dma_channel_rx);
    channel_config_set_transfer_data_size(&config_rx, DMA_SIZE_8);
    channel_config_set_read_increment(&config_rx, false);
    channel_config_set_write_increment(&config_rx, true);

    channel_config_set_dreq(&config_rx, pio_get_dreq(pio, sm, false));

    // enable dma in ring buffer mode with specified bit size
    channel_config_set_ring(&config_rx, true, ISOSNOOP_BUFFER_BITS);
    // configure dma to read from pio fifo
    dma_channel_configure(dma_channel_rx, &config_rx, read_buffer, (io_rw_8*)&pio->rxf[sm] + 0, dma_encode_endless_transfer_count(), true); // dma started
}

void isosnoop_print_buffer() {
    uint32_t write_addr = dma_chan->write_addr;
    while(last_write_addr != write_addr) {
        uint8_t b = *((uint8_t *)last_write_addr);
        // Increment last_write_addr, wrapping within the (aligned) buffer
        last_write_addr = 
            (last_write_addr & ~((1<<ISOSNOOP_BUFFER_BITS)-1))
            | ((last_write_addr + 1) & ((1<<ISOSNOOP_BUFFER_BITS)-1));
        for(int i=0;i<2;i++) {
            uint8_t chunk = b & 0xf0;
            b <<= 4;
            
            // switch(chunk>>4) {
            // case 0xa:
            //     printf("CS1 ");
            //     break;
            // case 0x5:
            //     printf("CS0 ");
            //     break;
            // case 0x9:
            //     printf("1 ");
            //     break;
            // case 0x6:
            //     printf("0 ");
            //     break;
            // case 0x0:
            //     printf("_ ");
            //     break;
            // default:
            //     printf("? ");
            //     break;
            // }    
            switch(chunk>>4) {
            case 0b0000:
                printf("_ ");
                break;
            case 0b0001:
                printf("0001 ");
                break;
            case 0b0010:
                printf("0010 ");
                break;
            case 0b0011:
                printf("0011 ");
                break;
            case 0b0100:
                printf("0100 ");
                break;
            case 0b0101:
                printf("CS0 ");
                break;
            case 0b0110:
                printf("0 ");
                break;
            case 0b0111:
                printf("0111 ");
                break;
            case 0b1000:
                printf("1000 ");
                break;
            case 0b1001:
                printf("1 ");
                break;
            case 0b1010:
                printf("CS1 ");
                break;
            case 0b1011:
                printf("1011 ");
                break;
            case 0b1100:
                printf("1100 ");
                break;
            case 0b1101:
                printf("1101 ");
                break;
            case 0b1110:
                printf("1110 ");
                break;
            case 0b1111:
                printf("1111 ");
                break;
            }    




            // if(chunk==0xa0) {
            //     printf("CS1 ");
            // } else if(chunk==0x50) {
            //     printf("CS0 ");
            // } else if(chunk==0x90) {
            //     printf("1 ");
            // } else if(chunk==0x60) {
            //     printf("0 ");
            // } else if(chunk==0x00) {
            //     printf("_ ");
            // } else {
            //     printf("? ");
            // }
        }
    }

    // empty the ISR (this will throw away the sample that may be sitting in it)
    //pio_sm_exec(ISOSNOOP_PIO, ISOSNOOP_SM, pio_encode_mov(pio_isr, pio_null));

    printf("\n");
}

void isosnoop_flush() {
    last_write_addr = dma_chan->write_addr;
}