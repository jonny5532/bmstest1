#include "isospi_master.pio.h"

#include "config/allocations.h"

#include "pico/stdlib.h"

#include <stdio.h>

#define ISOSPI_MASTER_SM 0

void isospi_master_setup(unsigned int tx_pin_base, uint rx_pin_base) {
    // tx_pin_base      is the driver enable pin (active high)
    // tx_pin_base + 1  is the tx data pin (noninverting)

    // rx_pin_base      is the high rx data pin
    // rx_pin_base + 1  is the low rx data pin

    isospi_master_program_init(ISOSPI_MASTER_PIO, tx_pin_base, rx_pin_base);
}

void isospi_master_flush() {
    // flush any remaining data in the PIO RX FIFO
    while(!pio_sm_is_rx_fifo_empty(ISOSPI_MASTER_PIO, ISOSPI_MASTER_SM)) {
        pio_sm_get_blocking(ISOSPI_MASTER_PIO, ISOSPI_MASTER_SM);
    }

    // empty the ISR
    pio_sm_exec(ISOSPI_MASTER_PIO, ISOSPI_MASTER_SM, pio_encode_mov(pio_isr, pio_null));
}

/**
 * Perform a blocking write/read transaction over isoSPI.
 *
 * All receptions are delayed by 1 bit to match the Tesla BMB. Returns true if all
 * non-skipped received bits were well-formed isoSPI pulses.
 *
 * The first received byte after the skipped ones will be stored at in_buf[0].
 *
 * @param tx_buf  Buffer of bytes to send out
 * @param rx_buf  Buffer to receive bytes into (can be smaller than tx_buf if skip > 0)
 * @param len     Number of bytes to send/receive
 * @param skip    Number of initial bytes to skip receiving
 */
bool isospi_write_read_blocking(uint8_t* tx_buf, uint8_t* rx_buf, size_t len, size_t skip) {
    isospi_master_cs(true);

    sleep_us(1);

    // xtra
    //sleep_us(5);

    //char log[1000];

    bool valid = true;
    uint32_t carry_word = 0;
    for(size_t i=0; i<len; i++) {
        // We write 8 bits at a time
        pio_sm_put_blocking(ISOSPI_MASTER_PIO, ISOSPI_MASTER_SM, tx_buf[i] << 24);

        // Each response bit is encoded as a nibble in a 32 bit word
        uint32_t v = pio_sm_get_blocking(ISOSPI_MASTER_PIO, ISOSPI_MASTER_SM);
        
        // Use the MSB nibble from the carry, and store the LSB nibble as the
        // next carry.
        int new_carry = (v & 0xf) << 28;
        v = (v >> 4) | carry_word;
        carry_word = new_carry;

        if(i < skip) {
            // skip receiving this byte
            rx_buf[i] = 0;
            continue;
        }

        uint8_t byte = 0;
        for(int r=0; r<8; r++) {
            uint8_t nibble = (v >> 28) & 0xf;
            v <<= 4;
            //log[i*8 + r] = nibble > 9 ? 'a' + (nibble - 10) : '0' + nibble;
            if(nibble==0b1001) {
                // bit 1
                byte = (byte << 1) | 0x1;
            } else if(nibble==0b0110) {
                // bit 0
                byte = (byte << 1) | 0x0;
            } else {
                // invalid
                printf("Invalid isoSPI nibble 0x%X on byte %d bit %d\n", nibble, (int)i, r);
                valid = false;
                byte = (byte << 1) | 0x0;
            }
        }
        rx_buf[i - skip] = byte;

        // an inter-byte delay gives the other end time to process
        //sleep_us(2);
    }

    sleep_us(1);

    // perform final ending chip select
    isospi_master_cs(false);

    // flush any remaining data
    isospi_master_flush();

    return valid;
}

void isospi_send_wakeup_cs_blocking() {
    isospi_master_cs(false);

    // 0us = first BMB fine
    // 10/15us = first 6 BMBs ok, 7,8 partial
    // 20us = all 8 BMBs ok

    sleep_us(20);
}
