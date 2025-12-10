#include "duart.h"

#include "hardware/gpio.h"
#include "pico/time.h"

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#define UART0_DMA_IRQ 0
#define UART1_DMA_IRQ 1

#define UART0_DMA_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
#define UART1_DMA_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY

#define CRC16_INIT                  ((uint16_t)-1l)

duart duart0;
duart duart1;

int chars_rxed = 0;

static void enable_tx_pin(duart *u) {
    // Set TX pin function to UART, which will switch the pin from an input with
    // pull-up to a high output
    gpio_set_function(u->tx_pin, UART_FUNCSEL_NUM(u->uart, u->tx_pin)); // 0 = TX
}

static void disable_tx_pin(duart *u) {
    // Set TX pin back to SIO (GPIO), which will switch back to a input with
    // pull-up
    gpio_set_function(u->tx_pin, GPIO_FUNC_SIO);
}

static void __duart_send(duart *u, const uint8_t *data, size_t len) {
    // Record what we are sending, so we know how much to consume from the
    // ringbuf once the DMA transfer is complete
    u->tx_sending = len;

    dma_channel_configure(
        u->tx_dma_channel,
        &u->tx_dma_config,
        &uart_get_hw(u->uart)->dr,
        (uint8_t *)data,
        dma_encode_transfer_count(len), 
        true
    );
}

static void on_uart0_rx() {
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);
        // Can we send it back?
        // if (uart_is_writable(INTERNAL_UART)) {
        //     // Change it slightly first!
        //     ch++;
        //     uart_putc(INTERNAL_UART, ch);
        // }
        chars_rxed++;
    }
}

static void on_uart1_rx() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);
        // Can we send it back?
        // if (uart_is_writable(INTERNAL_UART)) {
        //     // Change it slightly first!
        //     ch++;
        //     uart_putc(INTERNAL_UART, ch);
        // }
        chars_rxed++;
    }
}

bool _send_buffered_data(duart *u) {
    uint head = u->tx_head;
    if(u->tx_head != u->tx_tail) {
        // There's buffered data to send
        if(u->tx_head > u->tx_tail) {
            // single contiguous block
            __duart_send(u, &u->tx_buffer[u->tx_tail], u->tx_head - u->tx_tail);
            u->tx_tail = u->tx_head;
        } else {
            // send to end of buffer first
            __duart_send(u, &u->tx_buffer[u->tx_tail], DUART_TX_BUFFER_LEN - u->tx_tail);
            u->tx_tail = 0;
        }

        return true;
    }
    return false;
}

void _end_send(duart *u) {
    if(u->deassert_tx_when_idle) {
        disable_tx_pin(u);
    }
    atomic_flag_clear(&u->tx_active);
}

int64_t on_uart_wait_for_tx_buffer(alarm_id_t id, void *user_data) {
    duart *u = (duart *) user_data;

    if(atomic_flag_test_and_set(&u->tx_buffer_lock)) {
        return 5; // Try again in 5us
    }

    bool sent = _send_buffered_data(u);

    atomic_flag_clear(&u->tx_buffer_lock);

    if(!sent) {
        // No more data to send, end transmission
        _end_send(u);
    }

    // Don't fire again
    return 0;
}

static void on_uart_tx_complete(duart *u, uint irq_index) {
    if(dma_irqn_get_channel_status(irq_index, u->tx_dma_channel)) {
        dma_irqn_acknowledge_channel(irq_index, u->tx_dma_channel);

        if(u->tx_sending > 0) {
            // Finished sending previous chunk
            ringbuf_read(&u->tx_ringbuf, NULL, u->tx_sending);
            u->tx_sending = 0;
        }

        uint8_t *buf = NULL;
        size_t available = ringbuf_peek(&u->tx_ringbuf, &buf);

        if(available>0) {
            __duart_send(u, buf, available);
            return;
        }

        _end_send(u);


        // return;

        //uint32_t sniffed_crc = dma_sniffer_get_data_accumulator();
        //printf("sniffed CRC16: 0x%04x\n", sniffed_crc);

        // if(atomic_flag_test_and_set(&u->tx_buffer_lock)) {
        //     add_alarm_in_us(5, on_uart_wait_for_tx_buffer, (void *)u, true);
        //     return;
        // }

        // if(_send_buffered_data(u)) {
        //     // Sent more data, keep tx_active set
        //     atomic_flag_clear(&u->tx_buffer_lock);
        //     return;
        // }

        // uint tail = atomic_load_explicit(&u->tx_tail, memory_order_relaxed);
        // uint head = atomic_load_explicit(&u->tx_head, memory_order_acquire);

        // if(tail != head) {
        //     // There's buffered data to send
        //     if(head > tail) {
        //         // single contiguous block
        //         __duart_send(u, &u->tx_buffer[tail], head - tail);
        //         tail = head;
        //     } else {
        //         // send to end of buffer first
        //         __duart_send(u, &u->tx_buffer[tail], DUART_TX_BUFFER_LEN - tail);
        //         tail = 0;
        //     }

        //     atomic_store_explicit(&u->tx_tail, tail, memory_order_release);

        //     return;
        // }

        // _end_send(u);
    }
}

static void on_uart0_tx_complete() {
    //printf("on_uart0_tx_complete\n");
    on_uart_tx_complete(&duart0, UART0_DMA_IRQ);
}

static void on_uart1_tx_complete() {
    //printf("on_uart1_tx_complete\n");
    on_uart_tx_complete(&duart1, UART1_DMA_IRQ);
}


static bool _duart_send(duart *u, const uint8_t *data, size_t len) {
    if (len > sizeof(u->tx_buffer)) {
        // Too big to send
        return false;
    }

    // Enable before memcpy to give some set-up time
    enable_tx_pin(u);

    memcpy(u->tx_buffer, data, len);

    __duart_send(u, u->tx_buffer, len);

    // Successfully started
    return true;
}

bool duart_send(duart *u, const uint8_t *data, size_t len) {
    // Try to send data. Returns false if there is no room in the transmit
    // buffer to enqueue the data.


    if(atomic_flag_test_and_set(&u->tx_active)) {
        // Tx is busy. Try to enqueue data in buffer

        // Spin wait for buffer lock
        while(atomic_flag_test_and_set(&u->tx_buffer_lock));

        bool ret = ringbuf_write(&u->tx_ringbuf, data, len);

        // uint head = atomic_load_explicit(&u->tx_head, memory_order_relaxed);
        // uint tail = atomic_load_explicit(&u->tx_tail, memory_order_acquire);

        // size_t space_available;
        // if(head >= tail) {
        //     space_available = DUART_TX_BUFFER_LEN - (head - tail) - 1;
        // } else {
        //     space_available = tail - head - 1;
        // }
        // if(len > space_available) {
        //     // Not enough space
        //     atomic_flag_clear(&u->tx_buffer_lock);
        //     return false;
        // }

        // // Copy data into buffer
        // for(size_t i=0; i<len; i++) {
        //     u->tx_buffer[head] = data[i];
        //     head = (head + 1) % DUART_TX_BUFFER_LEN;
        // }

        // atomic_store_explicit(&u->tx_head, head, memory_order_release);

        atomic_flag_clear(&u->tx_buffer_lock);

        // TODO - it is possible that the tx has just completed and there is no
        // pending dma transfer.

        return ret;
    }

    if(ringbuf_write(&u->tx_ringbuf, data, len)) {
        // Successfully buffered data, start sending
        enable_tx_pin(u);

        uint8_t *buf = NULL;
        size_t available = ringbuf_peek(&u->tx_ringbuf, &buf);
        __duart_send(u, buf, available);
        return true;
    }

    return false;
    //return _duart_send(u, data, len);
}

bool duart_send_blocking(duart *u, const uint8_t *data, size_t len) {
    // Waits until previous send is complete before sending.
    // Doesn't wait for the current send to complete.
    // Returns false if data is too large to send.

    while(true) {
        if(!atomic_flag_test_and_set(&u->tx_active)) {
            // Was unset (ie, we claimed it), so we're ready to send
            break;
        }
        // Wait for previous send to complete
        dma_channel_wait_for_finish_blocking(u->tx_dma_channel);
    }
    
    return _duart_send(u, data, len);
}


bool init_duart(duart *u, uart_inst_t *uart_hw, uint baud_rate, uint tx_pin, uint rx_pin, bool deassert_tx_when_idle) {
    u->uart = uart_hw;
    u->tx_pin = tx_pin;
    u->rx_pin = rx_pin;
    u->deassert_tx_when_idle = deassert_tx_when_idle;
    u->tx_head = 0;
    u->tx_tail = 0;

    ringbuf_init(&u->tx_ringbuf, u->tx_buffer, DUART_TX_BUFFER_LEN);

    uart_init(u->uart, baud_rate);
    gpio_set_function(u->rx_pin, UART_FUNCSEL_NUM(u->uart, u->rx_pin)); // 1 = RX
    if(u->deassert_tx_when_idle) {
        gpio_pull_up(u->tx_pin);
        disable_tx_pin(u);
    } else {
        enable_tx_pin(u);
    }

    // Disable flow control
    uart_set_hw_flow(u->uart, false, false);

    // Set our data format
    uart_set_format(u->uart, 8, 1, UART_PARITY_NONE);

    // Disable FIFOs
    uart_set_fifo_enabled(u->uart, false);

    if(u->uart==uart0) {
        irq_set_exclusive_handler(UART0_IRQ, on_uart0_rx);
        irq_set_enabled(UART0_IRQ, true);
        irq_add_shared_handler(dma_get_irq_num(UART0_DMA_IRQ), on_uart0_tx_complete, UART0_DMA_IRQ_PRIORITY);
        irq_set_enabled(dma_get_irq_num(UART0_DMA_IRQ), true);
    } else {
        irq_set_exclusive_handler(UART1_IRQ, on_uart1_rx);
        irq_set_enabled(UART1_IRQ, true);
        irq_add_shared_handler(dma_get_irq_num(UART1_DMA_IRQ), on_uart1_tx_complete, UART1_DMA_IRQ_PRIORITY);
        irq_set_enabled(dma_get_irq_num(UART1_DMA_IRQ), true);
    }
    uart_set_irq_enables(u->uart, true, false);
    
    // Setup DMA for TX
    u->tx_dma_channel = dma_claim_unused_channel(false);
    if(!u->tx_dma_channel < 0) {
        panic("No free dma channels");
        return false;
    }
    u->tx_dma_config = dma_channel_get_default_config(u->tx_dma_channel);
    channel_config_set_transfer_data_size(&u->tx_dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&u->tx_dma_config, true);
    channel_config_set_write_increment(&u->tx_dma_config, false);
    channel_config_set_dreq(&u->tx_dma_config, uart_get_dreq(u->uart, true));

    channel_config_set_sniff_enable(&u->tx_dma_config, true);
    dma_sniffer_enable(u->tx_dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, true);

    dma_irqn_set_channel_enabled(u->uart==uart0 ? UART0_DMA_IRQ : UART1_DMA_IRQ, u->tx_dma_channel, true);

    atomic_flag_clear(&u->tx_active);
    atomic_flag_clear(&u->tx_buffer_lock);

    return true;
}