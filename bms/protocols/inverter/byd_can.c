#include "config/allocations.h"
#include "config/limits.h"
#include "config/pins.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"
#include "app/model.h"

#include "can2040.h"
#include "protocols/inverter/can.h"
#include "protocols/inverter/inverter.h"

#include <assert.h>
#include <pico/stdlib.h>

static const int battery_capacity_Wh = 60000;
static const int FW_MAJOR_VERSION = 0x03;
static const int FW_MINOR_VERSION = 0x29;

// The inverter sends 0x151/0x91/0xD1/0x111 periodically. If we hear nothing
// for this long, treat it as gone. Kept short so that an inverter restart
// while the BMS is running is noticed before the inverter comes back up;
// otherwise we would burst stale regular frames at it before it has
// identified us, and it never accepts the battery until the link is reset.
static const uint32_t INVERTER_TIMEOUT_MS = 10000;
// If we have queued a frame and nothing has acknowledged it for this long, the
// bus is dead (unplugged, or the inverter is rebooting). can2040 has no
// bus-off state and retries the head-of-queue frame forever, so we have to
// detect this ourselves.
static const uint32_t INVERTER_TX_TIMEOUT_MS = 2000;
// Don't let a chatty inverter flood the log with reinit requests
static const uint32_t REINIT_LOG_INTERVAL_MS = 1000;
  
// Written from the CAN callback (ISR context) and read from the main loop
static volatile bool inverter_present = false;
static volatile bool inverter_initialized = false;
static int inverter_init_state = 0;
static volatile uint32_t last_received_millis = 0;
// Set by the main loop when a frame is queued while nothing was pending;
// cleared by the ISR whenever any frame is acknowledged on the bus. Stays set
// with a frozen tx_pending_since when nobody is acknowledging us.
static volatile bool tx_pending = false;
static volatile uint32_t tx_pending_since = 0;
static volatile uint32_t last_tx_success_millis = 0;
// Reinit request bookkeeping (ISR context)
static volatile uint32_t last_reinit_request_millis = 0;
static volatile uint32_t last_reinit_log_millis = 0;
static volatile uint32_t reinit_request_count = 0;
static uint32_t inverter_lost_count = 0;
// offsets to avoid sending all messages on the same timestep
static uint32_t timestep_1 = 0;
static uint32_t timestep_2 = 1;
static uint32_t timestep_3 = 2;

static const struct can2040_msg byd_250 = {
    .id = 0x250,
    .dlc = 8,
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

#ifndef INVERTER_MODEL_STRING
#define INVERTER_MODEL_STRING "Battery-Box Premium HVS\x00\x00\x00\x00"
#endif

//static const char INVERTER_MODEL[] = "Battery-Box Premium HVS\x00\x00\x00\x00";
  static const char INVERTER_MODEL[] = INVERTER_MODEL_STRING;
static_assert(sizeof(INVERTER_MODEL) == 28, "INVERTER_MODEL must be exactly 28 characters including null terminator");

static void handle_reinit_request(const struct can2040_msg *msg) {
    uint32_t now = millis();
    uint32_t since_last = last_reinit_request_millis ? (now - last_reinit_request_millis) : 0;
    last_reinit_request_millis = now;
    reinit_request_count++;

    if(last_reinit_log_millis == 0 || (now - last_reinit_log_millis) >= REINIT_LOG_INTERVAL_MS) {
        last_reinit_log_millis = now;
        info_printf("BYD_CAN: [%lu ms] Reinitialization requested by inverter (#%lu, +%lu ms) 151/%u: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            (unsigned long)now, (unsigned long)reinit_request_count, (unsigned long)since_last, msg->dlc,
            msg->data[0], msg->data[1], msg->data[2], msg->data[3],
            msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
    }

    inverter_initialized = false;
}

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg)
{
    (void)cd;

    if (notify == CAN2040_NOTIFY_TX) {
        // One of our frames was acknowledged on the bus, so somebody is there
        tx_pending = false;
        last_tx_success_millis = millis();
        return;
    }

    if (notify != CAN2040_NOTIFY_RX) return;
    
    // rx 151 (contains brand name)
    // rx 91 (contains voltage/current/temp)
    // rx d1 (contains inverter SoC?)
    // rx 111 (contains time)

    switch(msg->id) {
        case 0x151:
            if (msg->data[0] & 0x01) { 
                // Inverter wants the battery to (re)send its identification
                handle_reinit_request(msg);
            }
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

    raise_bms_event(ERR_INVERTER_DETECTED, msg->id);
    inverter_present = true;
    last_received_millis = millis();
}

void init_inverter() {
    init_inverter_can(can2040_cb);
}

// Queue a frame, tracking whether anything we've queued is still awaiting
// acknowledgement. Returns the can2040_transmit result (<0 if the queue is full).
static int byd_transmit(const struct can2040_msg *msg) {
    int ret = inverter_can_transmit(msg);
    if(ret == 0 && !tx_pending) {
        tx_pending_since = millis();
        tx_pending = true;
    }
    return ret;
}

// Forget the inverter and drop anything queued for it. The next frame we hear
// from it will start a fresh identification sequence, exactly as on a BMS
// cold start.
static void mark_inverter_lost(const char *reason, uint32_t age_ms) {
    inverter_lost_count++;
    info_printf("BYD_CAN: [%lu ms] Inverter lost (%s for %lu ms), resetting CAN (#%lu)\n",
        (unsigned long)millis(), reason, (unsigned long)age_ms, (unsigned long)inverter_lost_count);

    inverter_present = false;
    inverter_initialized = false;
    inverter_init_state = 0;
    tx_pending = false;

    inverter_can_reset();
}

static void send_inverter_init_messages() {
    struct can2040_msg msg;

    if(inverter_init_state == 0) {
        if(tx_pending) {
            // Regular frames are still queued; wait for them to go out so the
            // identification frames are the next thing the inverter sees. With
            // a live inverter this takes well under one tick.
            return;
        }
        if(byd_transmit(&byd_250)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 1) {
        if(byd_transmit(&byd_290)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 2) {
        if(byd_transmit(&byd_2d0)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    msg.id = 0x3D0;
    msg.dlc = 8;

    if(inverter_init_state == 3) {
        msg.data[0] = 0x00;
        msg.data[1] = INVERTER_MODEL[0];
        msg.data[2] = INVERTER_MODEL[1];
        msg.data[3] = INVERTER_MODEL[2];
        msg.data[4] = INVERTER_MODEL[3];
        msg.data[5] = INVERTER_MODEL[4];
        msg.data[6] = INVERTER_MODEL[5];
        msg.data[7] = INVERTER_MODEL[6];
        if(byd_transmit(&msg)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 4) {
        msg.data[0] = 0x01;
        msg.data[1] = INVERTER_MODEL[7];
        msg.data[2] = INVERTER_MODEL[8];
        msg.data[3] = INVERTER_MODEL[9];
        msg.data[4] = INVERTER_MODEL[10];
        msg.data[5] = INVERTER_MODEL[11];
        msg.data[6] = INVERTER_MODEL[12];
        msg.data[7] = INVERTER_MODEL[13];
        if(byd_transmit(&msg)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 5) {
        msg.data[0] = 0x02;
        msg.data[1] = INVERTER_MODEL[14];
        msg.data[2] = INVERTER_MODEL[15];
        msg.data[3] = INVERTER_MODEL[16];
        msg.data[4] = INVERTER_MODEL[17];
        msg.data[5] = INVERTER_MODEL[18];
        msg.data[6] = INVERTER_MODEL[19];
        msg.data[7] = INVERTER_MODEL[20];
        if(byd_transmit(&msg)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    if(inverter_init_state == 6) {
        msg.data[0] = 0x03;
        msg.data[1] = INVERTER_MODEL[21];
        msg.data[2] = INVERTER_MODEL[22];
        msg.data[3] = INVERTER_MODEL[23];
        msg.data[4] = INVERTER_MODEL[24];
        msg.data[5] = INVERTER_MODEL[25];
        msg.data[6] = INVERTER_MODEL[26];
        msg.data[7] = INVERTER_MODEL[27];
        if(byd_transmit(&msg)<0) {
            // failed to send
            return;
        }
        inverter_init_state++;
    }

    info_printf("BYD_CAN: [%lu ms] Identification sequence sent\n", (unsigned long)millis());

    inverter_initialized = true;
    inverter_init_state = 0;
    timestep_1 = timestep() + 1;
    timestep_2 = timestep_1 + 1;
    timestep_3 = timestep_1 + 2;
}

static int send_110(inverter_outputs_t *outputs) {
    struct can2040_msg msg;
    msg.id = 0x110;
    msg.dlc = 8;

    msg.data[0] = (outputs->max_voltage_limit_dV >> 8) & 0xFF;
    msg.data[1] = outputs->max_voltage_limit_dV & 0xFF;
    msg.data[2] = (outputs->min_voltage_limit_dV >> 8) & 0xFF;
    msg.data[3] = outputs->min_voltage_limit_dV & 0xFF;
    msg.data[4] = (outputs->discharge_current_limit_dA >> 8) & 0xFF;
    msg.data[5] = outputs->discharge_current_limit_dA & 0xFF;
    msg.data[6] = (outputs->charge_current_limit_dA >> 8) & 0xFF;
    msg.data[7] = outputs->charge_current_limit_dA & 0xFF;

    return byd_transmit(&msg);
}

static int send_150(inverter_outputs_t *outputs) {
    if(outputs->soc_millis==0) {
        // no valid data yet, don't send anything
        return -1;
    }

    struct can2040_msg msg;
    msg.id = 0x150;
    msg.dlc = 8;

    msg.data[0] = (outputs->soc >> 8) & 0xFF;
    msg.data[1] = outputs->soc & 0xFF;
    //const uint16_t soh = 10000; // 100.00%
    const uint16_t soh = 9900; // 99.00%
    msg.data[2] = (soh >> 8) & 0xFF;
    msg.data[3] = soh & 0xFF;

    msg.data[4] = (outputs->remaining_capacity_dAh >> 8) & 0xFF;
    msg.data[5] = outputs->remaining_capacity_dAh & 0xFF;

    msg.data[6] = (outputs->full_capacity_dAh >> 8) & 0xFF;
    msg.data[7] = outputs->full_capacity_dAh & 0xFF;

    return byd_transmit(&msg);
}

static int send_1d0(inverter_outputs_t *outputs) {
    if(outputs->battery_voltage_millis==0 || outputs->current_millis==0 || outputs->temperature_millis==0) {
        // no valid data yet, don't send anything
        return -1;
    }

    struct can2040_msg msg;
    msg.id = 0x1D0;
    msg.dlc = 8;

    // TODO: battery voltage or cell voltage total?
    const uint16_t pack_voltage_dV = (uint16_t)(outputs->battery_voltage * 10.0f); // in 0.1V units
    msg.data[0] = (pack_voltage_dV >> 8) & 0xFF;
    msg.data[1] = pack_voltage_dV & 0xFF;
    // TODO: check current direction
    const int16_t pack_current_dA = outputs->current_mA / 100; // in 0.1A units
    msg.data[2] = (pack_current_dA >> 8) & 0xFF;
    msg.data[3] = pack_current_dA & 0xFF;
    const int16_t temperature_midpoint_dC = (int16_t)((outputs->temperature_min + outputs->temperature_max) * 0.5f * 10.0f); // in 0.1C units
    msg.data[4] = (temperature_midpoint_dC >> 8) & 0xFF;
    msg.data[5] = temperature_midpoint_dC & 0xFF;
    msg.data[6] = 0x03;
    msg.data[7] = 0x08;
    return byd_transmit(&msg);
}

static int send_210(inverter_outputs_t *outputs) {
    if(outputs->temperature_millis==0) {
        // no valid temperature data, don't send anything
        return -1;
    }

    // TODO: Do we need to check staleness? the events system should already deal with that

    struct can2040_msg msg;
    msg.id = 0x210;
    msg.dlc = 8;

    const int16_t temperature_max = (int16_t)(outputs->temperature_max * 10.0f); // in 0.1C units
    msg.data[0] = (temperature_max >> 8) & 0xFF;
    msg.data[1] = temperature_max & 0xFF;
    const int16_t temperature_min = (int16_t)(outputs->temperature_min * 10.0f); // in 0.1C units
    msg.data[2] = (temperature_min >> 8) & 0xFF;
    msg.data[3] = temperature_min & 0xFF;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    return byd_transmit(&msg);
}

static int send_190(inverter_outputs_t *outputs) {
    // Alarms
    struct can2040_msg msg;
    msg.id = 0x190;
    msg.dlc = 8;
    (void)outputs;

    msg.data[0] = 0x00;
    msg.data[1] = 0x00;
    msg.data[2] = 0x03;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    return byd_transmit(&msg);
}

void inverter_tick(inverter_outputs_t *outputs) {
    // Called every main loop tick (TIMESTEP_PERIOD_MS)

    if(inverter_present) {
        uint32_t now = millis();
        uint32_t rx_age = now - last_received_millis;
        if(rx_age > INVERTER_TIMEOUT_MS) {
            // Inverter has gone quiet
            mark_inverter_lost("no RX", rx_age);
        } else if(tx_pending && (now - tx_pending_since) > INVERTER_TX_TIMEOUT_MS) {
            // Nobody is acknowledging our frames: bus unplugged or inverter down
            mark_inverter_lost("no TX ack", now - tx_pending_since);
        }
    }

    if(!inverter_present) {
        // Stay silent until the inverter speaks to us
        return;
    }

    if(!inverter_initialized) {
        // Send the identification sequence before anything else
        send_inverter_init_messages();
        return;
    }

    if(timestep_every_ms(100, &timestep_1)) {
        // send regular messages every 100ms
        send_110(outputs);
    }
    if(timestep_every_ms(1000, &timestep_2)) {
        // send regular messages every 1s (note: was 10s)
        send_150(outputs);
        send_1d0(outputs);
        send_210(outputs);
    }
    if(timestep_every_ms(60000, &timestep_3)) {
        // send regular messages every 60s
        send_190(outputs);
    }
}

void inverter_debug_print(void) {
    uint32_t now = millis();
    struct can2040_stats stats;
    inverter_can_get_stats(&stats);

    debug_printf("Inverter: %s%s init_state=%d | RX age: %lu ms | TX pending: %s (%lu ms) | reinit req: %lu | lost: %lu | can rx=%lu tx=%lu attempts=%lu parse_err=%lu\n",
        inverter_present ? "present" : "absent",
        inverter_initialized ? ", identified" : ", not identified",
        inverter_init_state,
        (unsigned long)(last_received_millis ? (now - last_received_millis) : 0),
        tx_pending ? "yes" : "no",
        (unsigned long)(tx_pending ? (now - tx_pending_since) : 0),
        (unsigned long)reinit_request_count,
        (unsigned long)inverter_lost_count,
        (unsigned long)stats.rx_total,
        (unsigned long)stats.tx_total,
        (unsigned long)stats.tx_attempt,
        (unsigned long)stats.parse_error);
}
