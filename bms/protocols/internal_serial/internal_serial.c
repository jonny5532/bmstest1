#include "internal_serial.h"

#include "app/model.h"
#include "config/allocations.h"
#include "config/limits.h"
#include "config/pins.h"
#include "drivers/comms/duart.h"
#include "protocols/hmi_serial/hmi_serial.h"
#include "shared/cell_map.h"
#include "shared/supervisor_protocol.h"
#include "sys/logging/logging.h"
#include "sys/time/time.h"

// Send a status to the supervisor every 5 ticks (100ms). It answers each one.
#define STATUS_PERIOD_TIMESTEPS 5

void init_internal_serial() {
    init_duart(&INTERNAL_SERIAL_DUART, 115200, PIN_INTERNAL_SERIAL_TX, PIN_INTERNAL_SERIAL_RX, false); //9375000 works!

    uint8_t errors = cell_map_check(bms_cell_map());
    if(errors) {
        // The supervisor will refuse it, so the contactors won't close
        error_printf("Cell map from the profile is inconsistent (0x%02x): the supervisor won't enable the contactors\n", errors);
    }
}

static void send_status(const bms_model_t *model) {
    sup_bms_status_t status = {
        .version = SUP_PROTOCOL_VERSION,
        .uptime_ms = millis64(),
        .cell_min_mV = (int16_t)model->cell_voltage_min_mV,
        .cell_max_mV = (int16_t)model->cell_voltage_max_mV,
        // The compiled map, never the observed one: the supervisor checks the
        // pack against it
        .map = *bms_cell_map(),
        .hmi_address = hmi_serial_device_address(),
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
    // Queued for DMA; dropped (not waited for) if the buffer is ever full
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
           reply.latch_reasons != model->supervisor.latch_reasons) {
            // Only on state changes: the reasons can change every reply while
            // the supervisor settles (see the supervisor CLI command)
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
