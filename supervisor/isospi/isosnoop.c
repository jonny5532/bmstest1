#include "isosnoop.h"
#include "isosnoop.pio.h"

#include "../allocations.h"

#include "hardware/dma.h"
#include "hardware/pio.h"

#include <stdio.h>

#define ISOSNOOP_SM 0

#define DMA_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
#define PIO_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
static uint dma_channel_rx;
dma_channel_hw_t *dma_chan;

#define ISOSNOOP_BUFFER_BITS 15 // 2**n byte buffer

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

size_t isosnoop_read(uint8_t *out, size_t max_len) {
    uint32_t write_addr = dma_chan->write_addr;
    size_t len = 0;

    while(last_write_addr != write_addr && len < max_len) {
        out[len++] = *((uint8_t *)last_write_addr);
        // Increment last_write_addr, wrapping within the (aligned) buffer
        last_write_addr =
            (last_write_addr & ~((1<<ISOSNOOP_BUFFER_BITS)-1))
            | ((last_write_addr + 1) & ((1<<ISOSNOOP_BUFFER_BITS)-1));
    }
    return len;
}
