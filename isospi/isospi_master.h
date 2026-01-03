#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

void isospi_master_setup(unsigned int tx_pin_base, unsigned int rx_pin_base);
void isospi_send_wakeup_cs_blocking();
bool isospi_write_read_blocking(uint8_t* out_buf, uint8_t* in_buf, size_t len, size_t skip);
