#pragma once
#include <stddef.h>
#include <stdint.h>

void isosnoop_setup(unsigned int rx_pin_base, unsigned int sampling_pin, unsigned int rx_and_pin, unsigned int timer_disable_pin);
// Copy out up to max_len bytes (two symbols each) received since the last read
size_t isosnoop_read(uint8_t *out, size_t max_len);
