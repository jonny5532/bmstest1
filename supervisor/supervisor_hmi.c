#include "supervisor_hmi.h"

#include "supervisor_config.h"
#include "protocols/hmi_serial/hmi_buf.h"
#include "protocols/hmi_serial/hmi_serial.h"

static uint32_t age_ms(bool have, uint32_t since, uint32_t now) {
    return have ? now - since : 0xFFFFFFFF;
}

size_t sup_hmi_handle(const sup_logic_t *logic, const uint8_t *payload, size_t len,
        uint32_t now, uint8_t *out, size_t out_size) {
    if(len < 2 || payload[0] != HMI_MSG_READ_SUPERVISOR) {
        // Everything else is for the BMS
        return 0;
    }
    if(!logic->have_hmi_address || payload[1] != logic->hmi_address) {
        // Not our BMS's address (or we don't know it yet)
        return 0;
    }
    if(out_size < SUP_HMI_RESPONSE_LEN) {
        return 0;
    }

    size_t idx = 0;
    out[idx++] = HMI_MSG_READ_SUPERVISOR_RESPONSE;
    out[idx++] = logic->hmi_address;
    out[idx++] = SUP_PROTOCOL_VERSION;
    out[idx++] = (uint8_t)logic->state;
    idx += hmi_buf_append_uint16(&out[idx], logic->reasons);
    idx += hmi_buf_append_uint16(&out[idx], logic->latch_reasons);
    idx += hmi_buf_append_uint16(&out[idx], (uint16_t)logic->cell_min_mV);
    idx += hmi_buf_append_uint16(&out[idx], (uint16_t)logic->cell_max_mV);
    out[idx++] = logic->cells_ok;
    out[idx++] = logic->have_map ? logic->map.num_cells : 0;
    out[idx++] = SUPERVISOR_CHEMISTRY;
    idx += hmi_buf_append_uint32(&out[idx], logic->scans);
    idx += hmi_buf_append_uint32(&out[idx], age_ms(logic->have_scan, logic->scan_millis, now));
    idx += hmi_buf_append_uint32(&out[idx], age_ms(logic->have_status, logic->status_millis, now));
    return idx;
}
