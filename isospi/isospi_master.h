#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

void isospi_master_setup(unsigned int tx_pin_base, unsigned int rx_pin_base);
void isospi_send_wakeup_cs_blocking();
bool isospi_write_read_blocking(char* out_buf, char* in_buf, size_t len, size_t skip);
void isospi_send_command_blocking(uint16_t cmd_word);
bool isospi_get_data_blocking(uint32_t cmd, uint8_t *response, size_t response_len);
