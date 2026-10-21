#include <stdint.h>

uint16_t crc14(uint8_t *data, int len, uint16_t initial_crc, uint16_t extra_mix);

// The 5 check bits of an 11-bit BMB module ID (bits 11-15 of the ID register)
uint8_t module_id_check_bits(uint16_t module_id);
