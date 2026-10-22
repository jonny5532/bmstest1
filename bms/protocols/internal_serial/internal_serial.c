#include "internal_serial.h"

#include "app/model.h"
#include "config/allocations.h"
#include "config/limits.h"
#include "config/pins.h"
#include "drivers/comms/duart.h"
#include "shared/cell_map.h"
#include "shared/supervisor_protocol.h"
#include "sys/logging/logging.h"
#include "sys/time/time.h"

// Send a status to the supervisor every 5 ticks (100ms). It answers each one.
#define STATUS_PERIOD_TIMESTEPS 5

void init_internal_serial() {
    init_duart(&INTERNAL_SERIAL_DUART, 115200, PIN_INTERNAL_SERIAL_TX, PIN_INTERNAL_SERIAL_RX, false); //9375000 works!
}

static void send_status(const bms_model_t *model) {
    sup_bms_status_t status = {
        .uptime_ms = millis64(),
        .map_hash = cell_map_hash(),
        .cell_min_mV = (int16_t)model->cell_voltage_min_mV,
        .cell_max_mV = (int16_t)model->cell_voltage_max_mV,
    };
    if(millis_recent_enough(model->cell_voltage_millis,
            model->cell_voltage_slow_mode ? CELL_VOLTAGE_STALE_THRESHOLD_SLOW_MS : CELL_VOLTAGE_STALE_THRESHOLD_MS)) {
        status.flags |= SUP_BMS_FLAG_CELLS_VALID;
    }
    if(model->cell_voltage_slow_mode) {
        status.flags |= SUP_BMS_FLAG_SLOW_MODE;
    }
    if(model->balancing_active) {
        status.flags |= SUP_BMS_FLAG_BALANCING;
    }

    uint8_t payload[SUP_BMS_STATUS_LEN];
    size_t len = sup_pack_bms_status(&status, payload, sizeof(payload));
    duart_send_packet(&INTERNAL_SERIAL_DUART, payload, len);
}

static void receive_replies(bms_model_t *model) {
    // Big enough to also take a reply in another protocol version, so the
    // mismatch shows up in the model
    uint8_t payload[64];
    size_t len;
    while((len = duart_read_packet(&INTERNAL_SERIAL_DUART, payload, sizeof(payload))) > 0) {
        sup_reply_t reply;
        if(!sup_unpack_reply(payload, len, &reply)) {
            continue;
        }

        if(model->supervisor_millis == 0 || reply.state != model->supervisor.state ||
           reply.reasons != model->supervisor.reasons) {
            info_printf("Supervisor: state %u reasons 0x%04x latch 0x%04x (v%u)\n",
                reply.state, reply.reasons, reply.latch_reasons, reply.version);
        }
        model->supervisor = reply;
        model->supervisor_millis = millis();
    }
}

void internal_serial_tick(bms_model_t *model) {
    receive_replies(model);

    if((timestep() % STATUS_PERIOD_TIMESTEPS) == 0) {
        send_status(model);
    }
}
