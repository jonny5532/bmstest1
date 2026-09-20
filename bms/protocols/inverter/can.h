#pragma once

#include "can2040.h"

// The board has two CAN interfaces, each driven by its own can2040 instance
// on its own PIO block:
//
//   CAN_BUS_INVERTER  PIN_CAN_*       the inverter protocol (byd_can)
//   CAN_BUS_INTER     PIN_INTERCAN_*  the inter-BMS fleet protocol
//                                     (custom_can slaves, can_master)
//
// A bus is brought up on first use and may have up to two receive listeners,
// each of which filters frames for itself.
typedef enum {
    CAN_BUS_INVERTER = 0,
    CAN_BUS_INTER = 1,
    CAN_BUS_COUNT
} can_bus_t;

// Start the bus if it is not running yet and register a receive listener
// (rx_cb may be NULL to just start the bus).
void can_bus_init(can_bus_t bus, can2040_rx_cb rx_cb);
int can_bus_transmit(can_bus_t bus, const struct can2040_msg *msg);

// Convenience wrappers for the inverter bus, used by the inverter protocols.
static inline void init_inverter_can(can2040_rx_cb rx_cb) {
    can_bus_init(CAN_BUS_INVERTER, rx_cb);
}
static inline int inverter_can_transmit(const struct can2040_msg *msg) {
    return can_bus_transmit(CAN_BUS_INVERTER, msg);
}
