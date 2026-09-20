#pragma once

// ISOSPI bus snooper (debugging aid). Compiled to no-ops when BMS_INTERCAN is
// enabled, as the inter-BMS CAN bus then occupies its PIO block.
void isosnoop_setup(unsigned int rx_pin_base, unsigned int sampling_pin, unsigned int rx_and_pin, unsigned int timer_disable_pin);
void isosnoop_print_buffer();
void isosnoop_flush();
