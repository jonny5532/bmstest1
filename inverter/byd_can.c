#include "../hw/allocations.h"
#include "../hw/pins.h"
#include "../monitoring/events.h"
#include "../model.h"

#include "can2040.h"

#include <pico/stdlib.h>

static const int battery_capacity_Wh = 60000;
static const int FW_MAJOR_VERSION = 0x03;
static const int FW_MINOR_VERSION = 0x29;
  
static struct can2040 cbus;
static bool inverter_present = false;
static bool inverter_initialized = false;
static int inverter_init_state = 0;
// offsets to avoid sending all messages on the same timestep
static uint32_t timestep_1 = 0;
static uint32_t timestep_2 = 1;
static uint32_t timestep_3 = 2;

static const struct can2040_msg byd_250 = {
    .id = 0x250,
    .dlc = 4,
    .data = {
        FW_MAJOR_VERSION, FW_MINOR_VERSION, 0x00, 0x66,
        (uint8_t)((battery_capacity_Wh / 100) >> 8),
        (uint8_t)(battery_capacity_Wh / 100), 0x02,
        0x09
    }
};

static const struct can2040_msg byd_290 = {
    .id = 0x290,
    .dlc = 8,
    .data = {0x06, 0x37, 0x10, 0xD9, 0x00, 0x00, 0x00, 0x00}
};
static const struct can2040_msg byd_2d0 = {
    .id = 0x2D0,
    .dlc = 8,
    .data = {0x00, 'B', 'Y', 'D', 0x00, 0x00, 0x00, 0x00}
};

static void PIOx_IRQHandler(void)
{
    can2040_pio_irq_handler(&cbus);
}

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    if (notify != CAN2040_NOTIFY_RX) return;
    (void)cd;
    
    // Add message processing code here...
    // rx 151 (contains brand name)
    // rx 91 (contains voltage/current/temp)
    // rx d1 (contains inverter SoC?)
    // rx 111 (contains time)

    switch(msg->id) {
        case 0x151:
            // process brand name
            break;
        case 0x91:
            // process voltage/current/temp
            break;
        case 0xd1:
            // process inverter SoC
            break;
        case 0x111:
            // process time
            break;
        default:
            // skip this message
            return;
    }

    if(!inverter_present) {
        count_bms_event(ERR_INVERTER_DETECTED, 0);
    }

    inverter_present = true;


    // TODO: store last received time for timeout detection
}

void init_inverter() {
    const int can_bitrate = 500000; // 500 kbps

    can2040_setup(&cbus, CAN2040_PIO_NUM);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(CAN2040_PIO_IRQ0, PIOx_IRQHandler);
    irq_set_priority(CAN2040_PIO_IRQ0, 1);
    irq_set_enabled(CAN2040_PIO_IRQ0, 1);

    // Start canbus
    can2040_start(&cbus, SYS_CLK_HZ, can_bitrate, PIN_CAN_RX, PIN_CAN_TX);
}

static void send_inverter_init_messages() {
    struct can2040_msg msg;

    if(inverter_init_state == 0) {
        if(can2040_transmit(&cbus, &byd_250)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 1) {
        if(can2040_transmit(&cbus, &byd_290)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 2) {
        if(can2040_transmit(&cbus, &byd_2d0)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    msg.id = 0x3D0;
    msg.dlc = 8;

    if(inverter_init_state == 3) {
        msg.data[0] = 0x00;
        msg.data[1] = 'B';
        msg.data[2] = 'a';
        msg.data[3] = 't';
        msg.data[4] = 't';
        msg.data[5] = 'e';
        msg.data[6] = 'r';
        msg.data[7] = 'y';
        if(can2040_transmit(&cbus, &msg)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 4) {
        msg.data[0] = 0x01;
        msg.data[1] = '-';
        msg.data[2] = 'B';
        msg.data[3] = 'o';
        msg.data[4] = 'x';
        msg.data[5] = ' ';
        msg.data[6] = 'P';
        msg.data[7] = 'r';
        if(can2040_transmit(&cbus, &msg)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 5) {
        msg.data[0] = 0x02;
        msg.data[1] = 'e';
        msg.data[2] = 'm';
        msg.data[3] = 'i';
        msg.data[4] = 'u';
        msg.data[5] = 'm';
        msg.data[6] = ' ';
        msg.data[7] = 'H';
        if(can2040_transmit(&cbus, &msg)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 6) {
        msg.data[0] = 0x03;
        msg.data[1] = 'V';
        msg.data[2] = 'S';
        msg.data[3] = 0x00;
        msg.data[4] = 0x00;
        msg.data[5] = 0x00;
        msg.data[6] = 0x00;
        msg.data[7] = 0x00;
        if(can2040_transmit(&cbus, &msg)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    inverter_initialized = true;
    inverter_init_state = 0;
    timestep_1 = timestep();
    timestep_2 = timestep_1 + 1;
    timestep_3 = timestep_1 + 2;
}

static int send_110(bms_model_t *model) {
    struct can2040_msg msg;
    msg.id = 0x110;
    msg.dlc = 8;
    msg.data[0] = (model->max_voltage_limit_dV >> 8) & 0xFF;
    msg.data[1] = model->max_voltage_limit_dV & 0xFF;
    msg.data[2] = (model->min_voltage_limit_dV >> 8) & 0xFF;
    msg.data[3] = model->min_voltage_limit_dV & 0xFF;
    msg.data[4] = (model->discharge_current_limit_dA >> 8) & 0xFF;
    msg.data[5] = model->discharge_current_limit_dA & 0xFF;
    msg.data[6] = (model->charge_current_limit_dA >> 8) & 0xFF;
    msg.data[7] = model->charge_current_limit_dA & 0xFF;
    return can2040_transmit(&cbus, &msg);
}

static int send_150(bms_model_t *model) {
    if(model->soc_millis==0) {
        // no valid data yet, don't send anything
        return -1;
    }

    struct can2040_msg msg;
    msg.id = 0x150;
    msg.dlc = 8;
    
    //const uint16_t soc = 5000; // 50.00%
    msg.data[0] = (model->soc >> 8) & 0xFF;
    msg.data[1] = model->soc & 0xFF;
    // TODO: workaround for Deye?
    //const uint16_t soh = 10000; // 100.00%
    const uint16_t soh = 9900; // 99.00%
    msg.data[2] = (soh >> 8) & 0xFF;
    msg.data[3] = soh & 0xFF;
    const uint16_t remaining_capacity_Ah = 120; // 12.0Ah
    msg.data[4] = (remaining_capacity_Ah >> 8) & 0xFF;
    msg.data[5] = remaining_capacity_Ah & 0xFF;
    const uint16_t full_capacity_Ah = 240; // 24.0Ah
    msg.data[6] = (full_capacity_Ah >> 8) & 0xFF;
    msg.data[7] = full_capacity_Ah & 0xFF;
    return can2040_transmit(&cbus, &msg);
}

static int send_1d0(bms_model_t *model) {
    if(model->battery_voltage_millis==0 || model->current_millis==0 || model->temperature_millis==0) {
        // no valid data yet, don't send anything
        return -1;
    }

    struct can2040_msg msg;
    msg.id = 0x1D0;
    msg.dlc = 8;

    // TODO: battery voltage or cell voltage total?
    const uint16_t pack_voltage_dV = model->battery_voltage_mV / 100; // in 0.1V units
    msg.data[0] = (pack_voltage_dV >> 8) & 0xFF;
    msg.data[1] = pack_voltage_dV & 0xFF;
    // TODO: check current direction
    const int16_t pack_current_dA = model->current_mA / 100; // in 0.1A units
    msg.data[2] = (pack_current_dA >> 8) & 0xFF;
    msg.data[3] = pack_current_dA & 0xFF;
    const int16_t temperature_midpoint_dC = 250; // 25.0C
    msg.data[4] = (temperature_midpoint_dC >> 8) & 0xFF;
    msg.data[5] = temperature_midpoint_dC & 0xFF;
    msg.data[6] = 0x03;
    msg.data[7] = 0x08;
    return can2040_transmit(&cbus, &msg);
}

static int send_210(bms_model_t *model) {
    if(model->temperature_millis==0) {
        // no valid temperature data, don't send anything
        return -1;
    }

    // TODO: Do we need to check staleness? the events system should already deal with that

    struct can2040_msg msg;
    msg.id = 0x210;
    msg.dlc = 8;

    const int16_t temperature_max_dC = model->temperature_max_dC; // in 0.1C units
    msg.data[0] = (temperature_max_dC >> 8) & 0xFF;
    msg.data[1] = temperature_max_dC & 0xFF;
    const int16_t temperature_min_dC = model->temperature_min_dC; // in 0.1C units
    msg.data[2] = (temperature_min_dC >> 8) & 0xFF;
    msg.data[3] = temperature_min_dC & 0xFF;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    return can2040_transmit(&cbus, &msg);
}

static int send_190(bms_model_t *model) {
    // Alarms
    struct can2040_msg msg;
    msg.id = 0x190;
    msg.dlc = 8;
    (void)model;

    msg.data[0] = 0x00;
    msg.data[1] = 0x00;
    msg.data[2] = 0x03;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    return can2040_transmit(&cbus, &msg);
}

static uint8_t transmit_cycle = 0;

void inverter_tick(bms_model_t *model) {
    // This should get called every 100ms

    if(!inverter_present) {
        // We haven't received any CAN messages from the inverter yet
        return;
    }

    if(!inverter_initialized) {
        if(!model->contactor_sm.enable_current) {
            // Don't initialize until first contactor close
            return;
        }
        
        // We haven't done the inverter init sequence yet
        send_inverter_init_messages();
        return;
    }

    if(timestep_every_ms(100, &timestep_1)) {
        // send regular messages every 100ms
        send_110(model);
    }
    if(timestep_every_ms(10000, &timestep_2)) {
        // send regular messages every 1000ms
        send_150(model);
        send_1d0(model);
        send_210(model);
    }
    if(timestep_every_ms(60000, &timestep_3)) {
        // send regular messages every 60000ms
        send_190(model);
    }

    // transmit_cycle++;
    // if((transmit_cycle % 20) == 0) { // every 2s
    //     send_110();
    // }
    // if((transmit_cycle % 100) == 0) { // every 10s
    //     send_150();
    //     send_1d0();
    //     send_210();
    // }
    // if((transmit_cycle % 600) == 0) { // every 60s
    //     send_190();
    //     transmit_cycle = 0;
    // }
}
