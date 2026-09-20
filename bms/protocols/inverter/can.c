#include "protocols/inverter/can.h"

#include "config/allocations.h"
#include "config/pins.h"

#include "can2040.h"

#include <stdbool.h>
#include <stddef.h>

#define MAX_LISTENERS 2
#define CAN_BITRATE 500000 // 500 kbps on both buses

typedef struct {
    struct can2040 cd;
    can2040_rx_cb listeners[MAX_LISTENERS];
    int num_listeners;
    bool started;
    uint32_t pio_num;
    uint32_t pio_irq;
    uint32_t gpio_rx, gpio_tx;
} can_bus_state_t;

static can_bus_state_t buses[CAN_BUS_COUNT] = {
    [CAN_BUS_INVERTER] = { .pio_num = CAN2040_PIO_NUM, .pio_irq = CAN2040_PIO_IRQ0, .gpio_rx = PIN_CAN_RX, .gpio_tx = PIN_CAN_TX },
    [CAN_BUS_INTER] = { .pio_num = INTERCAN_PIO_NUM, .pio_irq = INTERCAN_PIO_IRQ0, .gpio_rx = PIN_INTERCAN_RX, .gpio_tx = PIN_INTERCAN_TX },
};

static void inverter_irq_handler(void) {
    can2040_pio_irq_handler(&buses[CAN_BUS_INVERTER].cd);
}

static void inter_irq_handler(void) {
    can2040_pio_irq_handler(&buses[CAN_BUS_INTER].cd);
}

// Fan received frames out to every listener on that bus.
static void dispatch(can_bus_state_t *bus, uint32_t notify, struct can2040_msg *msg) {
    for(int i = 0; i < bus->num_listeners; i++) {
        bus->listeners[i](&bus->cd, notify, msg);
    }
}

static void inverter_rx(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    (void)cd;
    dispatch(&buses[CAN_BUS_INVERTER], notify, msg);
}

static void inter_rx(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    (void)cd;
    dispatch(&buses[CAN_BUS_INTER], notify, msg);
}

void can_bus_init(can_bus_t bus_id, can2040_rx_cb rx_cb) {
    if(bus_id >= CAN_BUS_COUNT) return;
    can_bus_state_t *bus = &buses[bus_id];

    if(rx_cb != NULL && bus->num_listeners < MAX_LISTENERS) {
        bus->listeners[bus->num_listeners++] = rx_cb;
    }
    if(bus->started) return;
    bus->started = true;

    can2040_setup(&bus->cd, bus->pio_num);
    can2040_callback_config(&bus->cd, bus_id == CAN_BUS_INVERTER ? inverter_rx : inter_rx);

    irq_set_exclusive_handler(bus->pio_irq, bus_id == CAN_BUS_INVERTER ? inverter_irq_handler : inter_irq_handler);
    irq_set_priority(bus->pio_irq, 1);
    irq_set_enabled(bus->pio_irq, 1);

    can2040_start(&bus->cd, SYS_CLK_HZ, CAN_BITRATE, bus->gpio_rx, bus->gpio_tx);
}

int can_bus_transmit(can_bus_t bus_id, const struct can2040_msg *msg) {
    if(bus_id >= CAN_BUS_COUNT || !buses[bus_id].started) return -1;
    return can2040_transmit(&buses[bus_id].cd, msg);
}
