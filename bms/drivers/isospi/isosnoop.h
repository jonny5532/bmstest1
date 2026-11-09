#pragma once

void isosnoop_flush();
void isosnoop_setup(unsigned int rx_pin_base, unsigned int sampling_pin, unsigned int timer_disable_pin);
void isosnoop_print_buffer();
