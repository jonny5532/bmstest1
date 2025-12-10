#include "../lib/ringbuf.h"

#include "hardware/dma.h"
#include "hardware/uart.h"

#include <stdatomic.h>
#include <stdbool.h>

// should be a power of 2
#define DUART_TX_BUFFER_LEN 256

typedef struct {
    uart_inst_t *uart;
    uint tx_dma_channel;
    dma_channel_config tx_dma_config;

    volatile atomic_flag tx_active;
    volatile atomic_flag tx_buffer_lock;

    uint tx_head;
    uint tx_tail;

    struct ringbuf tx_ringbuf;
    uint8_t tx_buffer[DUART_TX_BUFFER_LEN];
    size_t tx_sending;

    uint tx_pin;
    uint rx_pin;
    bool deassert_tx_when_idle;

} duart;

extern duart duart0;
extern duart duart1;


bool init_duart(duart *u, uart_inst_t *uart_hw, uint baud_rate, uint tx_pin, uint rx_pin, bool deassert_tx_when_idle);
bool duart_send(duart *u, const uint8_t *data, size_t len);
bool duart_send_blocking(duart *u, const uint8_t *data, size_t len);
