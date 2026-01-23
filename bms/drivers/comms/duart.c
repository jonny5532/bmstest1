#include "duart.h"
#include "app/monitoring/counters.h"

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

// static void __isr __not_in_flash_func(on_uart0_rx)() {
//     while (uart_is_readable(uart0)) {
//         uint8_t ch = uart_getc(uart0);
//         // Can we send it back?
//         // if (uart_is_writable(INTERNAL_UART)) {
//         //     // Change it slightly first!
//         //     ch++;
//         //     uart_putc(INTERNAL_UART, ch);
//         // }
//         chars_rxed++;
//     }
// }

// static void __isr __not_in_flash_func(on_uart1_rx)() {
//     while (uart_is_readable(uart1)) {
//         uint8_t ch = uart_getc(uart1);
//         // Can we send it back?
//         // if (uart_is_writable(INTERNAL_UART)) {
//         //     // Change it slightly first!
//         //     ch++;
//         //     uart_putc(INTERNAL_UART, ch);
//         // }
//         chars_rxed++;
//     }
// }

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
            ringbuf_read(&u->tx_ringbuf, NULL, u->tx_sending);
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
        printf("no tx irq?\n");
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

size_t duart_peek(duart *u, uint8_t **buf, size_t *all_available) {
    dma_channel_hw_t *dma_chan = dma_channel_hw_addr(u->rx_dma_channel);
    size_t available = (dma_chan->write_addr - u->rx_pointer) & ((1<<DUART_RX_BUFFER_BITS)-1);

    // if(available > (1<<(DUART_RX_BUFFER_BITS-1))) {
    //     // Buffer is more than half full!


    //     return 0;
    // }

    *all_available = available;
    size_t remaining_to_end = (1<<DUART_RX_BUFFER_BITS) - u->rx_pointer;

    if(available>0) {
        if(available > remaining_to_end) {
            available = remaining_to_end;
        }
        
        *buf = &u->rx_buffer[u->rx_pointer];
        //printf("{p%d %d}", available, *all_available);
        return available;
    }

    return 0;
}

void duart_flush(duart *u, size_t len) {
    // Zero out len bytes from the receive buffer
    // size_t remaining_to_end = (1<<DUART_RX_BUFFER_BITS) - u->rx_pointer;
    // if(len <= remaining_to_end) {
    //     memset(&u->rx_buffer[u->rx_pointer], 0, len);
    // } else {
    //     memset(&u->rx_buffer[u->rx_pointer], 0, remaining_to_end);
    //     memset(u->rx_buffer, 0, len - remaining_to_end);
    // }


    //printf("{f%d %d}", u->rx_pointer, len);
    u->rx_pointer = (u->rx_pointer + len) & ((1<<DUART_RX_BUFFER_BITS)-1);
}

void memcpy_with_crc16_slow(uint8_t *dest, const uint8_t *src, size_t len, uint16_t *crc16) {
    // Copies data from src to dest, updating the CRC16 value as we go.

    for(size_t i=0; i<len; i++) {
        uint8_t b = src[i];
        dest[i] = b;

        *crc16 ^= b;
        for(int j=0; j<8; j++) {
            if(*crc16 & 1) {
                *crc16 = (*crc16 >> 1) ^ 0xA001;
            } else {
                *crc16 >>= 1;
            }
        }
    }
}

void memcpy_with_crc16(uint8_t *dest, const uint8_t *src, size_t len, uint16_t *crc16) {
    static const uint16_t crc_table[] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 
    };

    for(size_t i=0; i<len; i++) {
        uint8_t b = src[i];
        dest[i] = b;

        uint16_t tbl_idx = (*crc16 ^ b) & 0xFF;
        *crc16 = (uint16_t)((*crc16 >> 8) ^ crc_table[tbl_idx]);
    }
}



size_t duart_read_packet(duart *u, uint8_t *buf, size_t buf_size) {
    // Reads a full packet from the receive buffer. Returns the number of bytes
    // read, or 0 if no full packet is available.

    // Packet format:
    // 0xff (sync byte)
    // <length-1 byte> (not including sync, CRC bytes or length byte itself)
    // <payload bytes> (min 1 byte)
    // <crc16 bytes>

    // The initial sync allows quicker recovery from transmission errors, both
    // if byte synchronisation is lost (in which case the receiver will search
    // for the next 0xff byte before continuing), and if bit synchronisation is
    // lost (since 0xFF is sent as a series of low levels on the line, so the
    // receiver can re-lock its bit timing on the next start bit).


    while(true) {
        size_t available;
        uint8_t *data;
        size_t len = duart_peek(u, &data, &available);
        if(available < 5) {
            //printf("rx: not enough data\n");
            return 0;
        }
        if(data[0] != 0xff) {
            // Invalid start byte, flush one byte
            duart_flush(u, 1);
            continue;
        }

        // Pop the sync byte, wrapping if necessary
        data = (uint8_t*)(((uintptr_t)data & ~((1<<DUART_RX_BUFFER_BITS)-1)) + ((uintptr_t)(data+1) % (1<<DUART_RX_BUFFER_BITS)));

        size_t payload_len = data[0] + 1;
        if((payload_len + 4) > available) {
            // Message not fully available yet
            //printf("rx: unfinished msg\n");
            return 0;
        }
        if(payload_len > buf_size) {
            // Payload too big for buffer, flush the entire packet
            duart_flush(u, payload_len + 4);
            continue;
        }

        // Pop the length byte, wrapping if necessary
        data = (uint8_t*)(((uintptr_t)data & ~((1<<DUART_RX_BUFFER_BITS)-1)) + ((uintptr_t)(data+1) % (1<<DUART_RX_BUFFER_BITS)));
        
        if(len<=2) {
            // We will have already wrapped, so whole message is now at start of buffer
            len = available;
        }

        // printf("rx: len %d avail %d payload %d\n", len, available, payload_len);
        // duart_flush(u, payload_len + 4);
        // return 0;

        uint16_t crc16 = CRC16_INIT;
        uint8_t first_two_bytes[2];
        first_two_bytes[0] = 0xff;
        first_two_bytes[1] = payload_len - 1;
        memcpy_with_crc16(&buf[0], &first_two_bytes[0], 2, &crc16);

        if((payload_len + 4) <= len) {
            // We have a full contiguous message
            memcpy_with_crc16(&buf[0], &data[0], payload_len, &crc16);

            //printf("DUART rx1: FF %02X ", payload_len - 1);
            // for(size_t i=0; i<payload_len; i++) {
            //     printf("%02X ", buf[i]);
            // }
            //printf("%02X %02X", data[payload_len], data[payload_len + 1]);
            //printf("\n");

            // Read CRC16 from message
            uint16_t msg_crc16 = data[payload_len] | (data[payload_len + 1] << 8);
            duart_flush(u, payload_len + 4);
            if(crc16 != msg_crc16) {
                // CRC mismatch
                printf("DUART crc err1: %d calc %04x msg %04x\n", payload_len, crc16, msg_crc16);
                if(u==&duart0) {
                    debug_counters.uart0_crc_errors++;
                } else {
                    debug_counters.uart1_crc_errors++;
                }
                continue;
                //return 0;
            }
        } else {

            // We have a full message, but it wraps around the end of the buffer
            size_t first_part = len - 2;

            // Maybe it's just the CRC that wraps?
            if(first_part > payload_len) {
                first_part = payload_len;
            }

            // if(first_part > payload_len) {
            //     printf("blah: first_part %d payload_len %d\n", first_part, payload_len);
            // }


            // duart_flush(u, payload_len + 4);
            // return 0;


            memcpy_with_crc16(&buf[0], &data[0], first_part, &crc16);
            duart_flush(u, first_part + 2);

            size_t second_part = payload_len - first_part;
            duart_peek(u, &data, &available);
            memcpy_with_crc16(&buf[first_part], &data[0], second_part, &crc16);

            // print message bytes
            // printf("DUART rx2: FF %02X ", payload_len - 1);
            // for(size_t i=0; i<payload_len; i++) {
            //     printf("%02X ", buf[i]);
            // }

            // printf("%02X %02X second_part is %d", data[second_part], data[second_part + 1], second_part);
            // printf("%02X %02X", data[second_part], data[second_part + 1]);
            // printf("\n");

            // Read CRC16 from message
            uint16_t msg_crc16 = data[second_part] | (data[second_part + 1] << 8);
            duart_flush(u, second_part + 2);



            if(crc16 != msg_crc16) {
                // CRC mismatch
                printf("DUART crc err2: %d calc %04x msg %04x sp %d\n", payload_len, crc16, msg_crc16, second_part);
                if(u==&duart0) {
                    debug_counters.uart0_crc_errors++;
                } else {
                    debug_counters.uart1_crc_errors++;
                }
                continue;
                //return 0;
            }
        }
        if(u==&duart0) {
            debug_counters.uart0_packets_received++;
        } else {
            debug_counters.uart1_packets_received++;
        }
        return payload_len;
    }
}

bool duart_send_packet(duart *u, const uint8_t *payload, size_t payload_len) {
    // Sends a full packet with the given payload. Returns true on success.

    if(payload_len > 256) {
        // Too big
        return false;
    }

    uint8_t buf[DUART_TX_BUFFER_LEN];
    buf[0] = 0xff; // sync byte
    buf[1] = payload_len - 1; // length byte


    // TODO: adapt the ringbuf to perform the CRC16 calculation, avoiding the
    // extra copy

    uint16_t crc16 = CRC16_INIT;
    memcpy_with_crc16(&buf[0], &buf[0], 2, &crc16);
    memcpy_with_crc16(&buf[2], payload, payload_len, &crc16);
    buf[2 + payload_len] = crc16 & 0xff;
    buf[2 + payload_len + 1] = (crc16 >> 8);

    bool ret = duart_send(u, buf, payload_len + 4);
    if(ret) {
        if(u==&duart0) {
            debug_counters.uart0_packets_sent++;
        } else {
            debug_counters.uart1_packets_sent++;
        }
    }
    return ret;
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