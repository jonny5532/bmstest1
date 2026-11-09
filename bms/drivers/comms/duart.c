#include "duart.h"
#include "app/monitoring/counters.h"
#include "shared/duart_frame.h"
#include "sys/logging/logging.h"

#include "hardware/gpio.h"
#include "pico/time.h"

#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

#define UART0_DMA_IRQ 0
#define UART1_DMA_IRQ 1

#define UART0_DMA_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY
#define UART1_DMA_IRQ_PRIORITY PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY

duart duart0;
duart duart1;

int chars_rxed = 0;

static void enable_tx_pin(duart *u) {
    // Set TX pin function to UART, which will switch the pin from an input with
    // pull-up to a high output.
    gpio_set_function(u->tx_pin, UART_FUNCSEL_NUM(u->uart, u->tx_pin));
}

static void disable_tx_pin(duart *u) {
    // Set TX pin back to SIO (GPIO), which will switch back to a input with
    // pull-up.
    gpio_set_function(u->tx_pin, GPIO_FUNC_SIO);
}

static void _duart_send(duart *u, const uint8_t *data, size_t len) {
    // Record how much we are sending, so we know how much to consume from the
    // ringbuf once the DMA transfer is complete.
    u->tx_sending = len;

    dma_channel_configure(
        u->tx_dma_channel,
        &u->tx_dma_config,
        &uart_get_hw(u->uart)->dr,
        (const uint8_t *)data,
        dma_encode_transfer_count(len), 
        true
    );
}

int64_t disable_tx_callback(alarm_id_t id, void *user_data) {
    duart *u = (duart *)user_data;

    (void)id;

    if(!atomic_flag_test_and_set(&u->tx_active)) {
        // No longer active, safe to disable TX pin

        disable_tx_pin(u);

        // Release the active flag we just grabbed
        atomic_flag_clear(&u->tx_active);
    }

    return 0;
}

static void __not_in_flash_func(on_uart_tx_complete)(duart *u, uint irq_index) {
    if(dma_irqn_get_channel_status(irq_index, u->tx_dma_channel)) {
        dma_irqn_acknowledge_channel(irq_index, u->tx_dma_channel);

        if(u->tx_sending > 0) {
            // Finished sending previous chunk, consume it from the ringbuf
            ringbuf_consume(&u->tx_ringbuf, u->tx_sending);
            u->tx_sending = 0;
        }

        // We grab a pointer to the next chunk of data to send, as we'll be
        // sending it with DMA.
        uint8_t *buf = NULL;
        size_t available = ringbuf_peek(&u->tx_ringbuf, &buf);

        if(available>0) {
            // Send the next chunk
            _duart_send(u, buf, available);
            return;
        }

        // Nothing else to send, finish up.

        if(u->deassert_tx_when_idle) {
            // FIXME: Wait for UART to finish transmitting last byte(s) before
            // doing this (there's at least one still in the tx buffer, and
            // probably another in the shift register?)

            // Maybe enable the TX interrupt and disable the tx pin there?
            // although then need to make sure the DMA hasn't started another
            // transfer in the meantime... (atomic flag?)

            // Hmm no, this won't work since there's no interrupt when the
            // transmit actually completes. We will need to set a timer or
            // something to check when the UART is idle.

            // Note - the delay has to be long enough for at least one byte to
            // finish sending, maybe two?
            add_alarm_in_us(150, disable_tx_callback, u, false);
        }

        atomic_flag_clear(&u->tx_active);
    } else {
        error_printf("no tx irq?\n");
    // if(dma_irqn_get_channel_status(irq_index, u->rx_dma_channel)) {
    //     dma_irqn_acknowledge_channel(irq_index, u->rx_dma_channel);
    }
}

static void __not_in_flash_func(on_uart0_tx_complete)() {
    on_uart_tx_complete(&duart0, UART0_DMA_IRQ);
}

static void __not_in_flash_func(on_uart1_tx_complete)() {
    on_uart_tx_complete(&duart1, UART1_DMA_IRQ);
}

bool duart_send(duart *u, const uint8_t *data, size_t len) {
    // Try to send data. Returns false if there is no room in the transmit
    // buffer to enqueue the data. Doesn't allow partial writes.

    if(atomic_flag_test_and_set(&u->tx_active)) {
        // Tx is busy. Try to enqueue data in buffer

        // Spin wait for buffer lock (ringbuf is single-writer).
        while(atomic_flag_test_and_set(&u->tx_buffer_lock));

        bool ret = ringbuf_write(&u->tx_ringbuf, data, len);

        atomic_flag_clear(&u->tx_buffer_lock);

        // TODO - it is possible that the tx has just completed and there is no
        // pending DMA transfer. In this case this data will remain unsent until
        // the next call.

        // This will only happen if it is possible for this function and the ISR
        // to run concurrently (ie, on separate cores).

        return ret;
    }

    // Write data to buffer
    bool ret = ringbuf_write(&u->tx_ringbuf, data, len);

    // We grab a pointer to the next chunk of data to send, as we need to
    // initiate the send from the buffer.
    uint8_t *buf = NULL;
    size_t available = ringbuf_peek(&u->tx_ringbuf, &buf);

    if(available>0) {
        // There is something to send, start the TX process. It might not
        // necessarily have been the data we just added though, if there was
        // already data in the buffer.
        
        if(u->deassert_tx_when_idle) {
            enable_tx_pin(u);
        }
        _duart_send(u, buf, available);
    } else {
        // Nothing was sent, clear the active flag.
        atomic_flag_clear(&u->tx_active);
    }

    return ret;
}

bool duart_send_blocking(duart *u, const uint8_t *data, size_t len) {
    // Waits for room in the transmit buffer. Doesn't wait for the data to be
    // fully sent, though.

    if(len > DUART_TX_BUFFER_LEN) {
        // Too big to send
        return false;
    }

    while(true) {
        if(duart_send(u, data, len)) {
            return true;
        }

        dma_channel_wait_for_finish_blocking(u->tx_dma_channel);
    }
}

static size_t duart_rx_available(duart *u) {
    // The DMA write address, relative to the (aligned) ring buffer
    dma_channel_hw_t *dma_chan = dma_channel_hw_addr(u->rx_dma_channel);
    return (dma_chan->write_addr - u->rx_pointer) & ((1<<DUART_RX_BUFFER_BITS)-1);
}

static void duart_flush(duart *u, size_t len) {
    u->rx_pointer = (u->rx_pointer + len) & ((1<<DUART_RX_BUFFER_BITS)-1);
}

size_t duart_read_packet(duart *u, uint8_t *buf, size_t buf_size) {
    // Reads a full packet (shared/duart_frame.h) from the receive buffer. Returns the
    // payload length, or 0 if no full packet is available. Packets longer than
    // buf_size are skipped.
    duart_frame_parse_t r = duart_frame_parse(u->rx_buffer, 1<<DUART_RX_BUFFER_BITS, u->rx_pointer,
        duart_rx_available(u), buf, buf_size);
    duart_flush(u, r.consumed);

    if(r.crc_errors) {
        // Corrupted frames, or false syncs (0xff in a payload) after losing our
        // place in the stream
        debug_printf("DUART: %lu bad CRC\n", (unsigned long)r.crc_errors);
        if(u==&duart0) {
            debug_counters.uart0_crc_errors += r.crc_errors;
        } else {
            debug_counters.uart1_crc_errors += r.crc_errors;
        }
    }
    if(r.payload_len) {
        if(u==&duart0) {
            debug_counters.uart0_packets_received++;
        } else {
            debug_counters.uart1_packets_received++;
        }
    }
    return r.payload_len;
}

bool duart_send_packet(duart *u, const uint8_t *payload, size_t payload_len) {
    // Sends a full packet with the given payload. Returns true on success.

    uint8_t buf[DUART_FRAME_MAX_PAYLOAD_LEN + DUART_FRAME_OVERHEAD];
    size_t len = duart_frame_encode(payload, payload_len, buf, sizeof(buf));
    if(len == 0) {
        // The length byte encodes len-1, so 0 can't be sent (and > 256 won't fit)
        return false;
    }

    bool ret = duart_send(u, buf, len);
    if(ret) {
        if(u==&duart0) {
            debug_counters.uart0_packets_sent++;
        } else {
            debug_counters.uart1_packets_sent++;
        }
    }
    return ret;
}

size_t duart_get_tx_free_space(duart *u) {
    return ringbuf_free_space(&u->tx_ringbuf);
}

bool init_duart(duart *u, uint baud_rate, uint tx_pin, uint rx_pin, bool deassert_tx_when_idle) {
    if(u==&duart0) {
        u->uart = uart0;
    } else {
        u->uart = uart1;
    }
    u->tx_pin = tx_pin;
    u->rx_pin = rx_pin;
    u->deassert_tx_when_idle = deassert_tx_when_idle;

    ringbuf_init(&u->tx_ringbuf, u->tx_buffer, DUART_TX_BUFFER_LEN);

    uart_init(u->uart, baud_rate);
    gpio_set_function(u->rx_pin, UART_FUNCSEL_NUM(u->uart, u->rx_pin));
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
        // irq_set_exclusive_handler(UART0_IRQ, on_uart0_single_tx_complete);
        // irq_set_enabled(UART0_IRQ, true);

        //irq_set_exclusive_handler(UART0_IRQ, on_uart0_rx);
        //irq_set_enabled(UART0_IRQ, true);
        irq_add_shared_handler(dma_get_irq_num(UART0_DMA_IRQ), on_uart0_tx_complete, UART0_DMA_IRQ_PRIORITY);
        irq_set_enabled(dma_get_irq_num(UART0_DMA_IRQ), true);
    } else {
        // irq_set_exclusive_handler(UART1_IRQ, on_uart1_single_tx_complete);
        // irq_set_enabled(UART1_IRQ, true);

        //irq_set_exclusive_handler(UART1_IRQ, on_uart1_rx);
        //irq_set_enabled(UART1_IRQ, true);
        irq_add_shared_handler(dma_get_irq_num(UART1_DMA_IRQ), on_uart1_tx_complete, UART1_DMA_IRQ_PRIORITY);
        irq_set_enabled(dma_get_irq_num(UART1_DMA_IRQ), true);
    }
    //uart_set_irq_enables(u->uart, true, false);
    
    // Setup DMA for TX
    u->tx_dma_channel = dma_claim_unused_channel(true);
    u->tx_dma_config = dma_channel_get_default_config(u->tx_dma_channel);
    channel_config_set_transfer_data_size(&u->tx_dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&u->tx_dma_config, true);
    channel_config_set_write_increment(&u->tx_dma_config, false);
    channel_config_set_dreq(&u->tx_dma_config, uart_get_dreq(u->uart, true));
    // channel_config_set_sniff_enable(&u->tx_dma_config, true);
    // dma_sniffer_enable(u->tx_dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC16, true);
    dma_irqn_set_channel_enabled(u->uart==uart0 ? UART0_DMA_IRQ : UART1_DMA_IRQ, u->tx_dma_channel, true);

    // Setup DMA for RX
    u->rx_dma_channel = dma_claim_unused_channel(true);
    u->rx_dma_config = dma_channel_get_default_config(u->rx_dma_channel);
    channel_config_set_transfer_data_size(&u->rx_dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&u->rx_dma_config, false);
    channel_config_set_write_increment(&u->rx_dma_config, true);
    channel_config_set_dreq(&u->rx_dma_config, uart_get_dreq(u->uart, false));
    channel_config_set_ring(&u->rx_dma_config, true, DUART_RX_BUFFER_BITS);
    //dma_irqn_set_channel_enabled(u->uart==uart0 ? UART0_DMA_IRQ : UART1_DMA_IRQ, u->rx_dma_channel, true);
    dma_channel_configure(
        u->rx_dma_channel,
        &u->rx_dma_config,
        u->rx_buffer,
        &uart_get_hw(u->uart)->dr,
        dma_encode_endless_transfer_count(),
        true
    );

    atomic_flag_clear(&u->tx_active);
    atomic_flag_clear(&u->tx_buffer_lock);

    return true;
}