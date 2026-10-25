#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "hardware/pio.h"

#define ISOSPI_MASTER_SM 0

// Defined in the c-sdk block of isospi_master.pio, which only isospi_master.c
// includes (it also defines globals, so it can't be included twice)
void isospi_master_program_init(PIO pio, uint tx_pin_base, uint rx_pin_base);
void isospi_master_cs(bool start);
// Defined in isospi_master.c
void isospi_master_flush();

void isospi_master_setup(unsigned int tx_pin_base, unsigned int rx_pin_base);
void isospi_send_wakeup_cs_blocking();
bool isospi_write_read_blocking(uint8_t* out_buf, uint8_t* in_buf, size_t len, size_t skip);

void isospi_master_async_setup(unsigned int tx_pin_base, unsigned int rx_pin_base);
void isospi_write_read_async(uint8_t* tx_buf, uint8_t* rx_buf, size_t len, size_t skip);
bool isospi_is_busy();
bool isospi_is_valid();
