#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

void isospi_master_setup(unsigned int tx_pin_base, unsigned int rx_pin_base);
bool isospi_write_read_blocking(char* out_buf, char* in_buf, size_t len, size_t skip);
