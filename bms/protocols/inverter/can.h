#pragma once

#include "can2040.h"

void init_inverter_can(can2040_rx_cb rx_cb);
// Stop and restart the CAN stack, discarding any queued transmits (for
// example frames stuck retrying because nobody is acknowledging them).
void inverter_can_reset(void);
int inverter_can_transmit(const struct can2040_msg *msg);
void inverter_can_get_stats(struct can2040_stats *stats);
