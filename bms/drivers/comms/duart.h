#include "lib/ringbuf.h"

#include "hardware/dma.h"
#include "hardware/uart.h"

#include <stdatomic.h>
#include <stdbool.h>

// should be a power of 2
#define DUART_TX_BUFFER_LEN 256
#define DUART_RX_BUFFER_BITS 9

typedef struct {
    uart_inst_t *uart;
    uint tx_dma_channel;
    uint rx_dma_channel;
    dma_channel_config tx_dma_config;
    dma_channel_config rx_dma_config;

    volatile atomic_flag tx_active;
    volatile atomic_flag tx_buffer_lock;

    struct ringbuf tx_ringbuf;
    uint8_t tx_buffer[DUART_TX_BUFFER_LEN];
    size_t tx_sending;

    uint8_t rx_buffer[1<<DUART_RX_BUFFER_BITS] __attribute__((aligned(1<<DUART_RX_BUFFER_BITS)));
    _Atomic size_t rx_pointer;

    uint tx_pin;
    uint rx_pin;
    bool deassert_tx_when_idle;

} duart;

extern duart duart0;
extern duart duart1;


bool init_duart(duart *u, uint baud_rate, uint tx_pin, uint rx_pin, bool deassert_tx_when_idle);
size_t duart_read_packet(duart *u, uint8_t *buf, size_t buf_size);
bool duart_send(duart *u, const uint8_t *data, size_t len);
bool duart_send_blocking(duart *u, const uint8_t *data, size_t len);
bool duart_send_packet(duart *u, const uint8_t *payload, size_t payload_len);
