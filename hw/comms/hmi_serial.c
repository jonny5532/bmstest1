#include "hmi_serial.h"
#include "duart.h"
#include "../allocation.h"
#include "../pins.h"
#include "../chip/time.h"
#include "../../model.h"

#include "pico/unique_id.h"

void init_hmi_serial() {
    // 937500 baud (close to 1Mbit)
    init_duart(&HMI_SERIAL_DUART, 460800, PIN_HMI_SERIAL_TX, PIN_HMI_SERIAL_RX, true); //9375000 works!
}

static inline uint8_t hmi_buf_append_uint64(uint8_t *buf, uint64_t value) {
    uint8_t idx = 0;
    buf[idx++] = (value >> 0) & 0xFF;
    buf[idx++] = (value >> 8) & 0xFF;
    buf[idx++] = (value >> 16) & 0xFF;
    buf[idx++] = (value >> 24) & 0xFF;
    buf[idx++] = (value >> 32) & 0xFF;
    buf[idx++] = (value >> 40) & 0xFF;
    buf[idx++] = (value >> 48) & 0xFF;
    buf[idx++] = (value >> 56) & 0xFF;
    return idx;
}

static inline uint8_t hmi_buf_append_uint32(uint8_t *buf, uint32_t value) {
    uint8_t idx = 0;
    buf[idx++] = (value >> 0) & 0xFF;
    buf[idx++] = (value >> 8) & 0xFF;
    buf[idx++] = (value >> 16) & 0xFF;
    buf[idx++] = (value >> 24) & 0xFF;
    return idx;
}

static inline uint8_t hmi_buf_append_uint16(uint8_t *buf, uint16_t value) {
    uint8_t idx = 0;
    buf[idx++] = (value >> 0) & 0xFF;
    buf[idx++] = (value >> 8) & 0xFF;
    return idx;
}

void hmi_serial_tick(bms_model_t *model) {
    uint8_t tx_buf[120];
    uint8_t idx=0;

    if((timestep() & 63) == 10) {
        tx_buf[idx++] = HMI_MSG_REGISTER_BROADCAST;

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_SERIAL);
        tx_buf[idx++] = HMI_TYPE_UINT64;
        pico_get_unique_board_id((pico_unique_board_id_t*)&tx_buf[idx]);
        idx += 8;

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_MILLIS);
        tx_buf[idx++] = HMI_TYPE_UINT64;
        idx += hmi_buf_append_uint64(&tx_buf[idx], millis());

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_CURRENT);
        tx_buf[idx++] = HMI_TYPE_INT32;
        idx += hmi_buf_append_uint32(&tx_buf[idx], model->current_mA);

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_CHARGE);
        tx_buf[idx++] = HMI_TYPE_INT64;
        int64_t charge_mC = (model->charge_raw * 132736) / 1000000;
        idx += hmi_buf_append_uint64(&tx_buf[idx], charge_mC);

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_BATTERY_VOLTAGE);
        tx_buf[idx++] = HMI_TYPE_INT32;
        idx += hmi_buf_append_uint32(&tx_buf[idx], model->battery_voltage_mV);

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_OUTPUT_VOLTAGE);
        tx_buf[idx++] = HMI_TYPE_INT32;
        idx += hmi_buf_append_uint32(&tx_buf[idx], model->output_voltage_mV);

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_POS_CONTACTOR_VOLTAGE);
        tx_buf[idx++] = HMI_TYPE_INT32;
        idx += hmi_buf_append_uint32(&tx_buf[idx], model->pos_contactor_voltage_mV);

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_NEG_CONTACTOR_VOLTAGE);
        tx_buf[idx++] = HMI_TYPE_INT32;
        idx += hmi_buf_append_uint32(&tx_buf[idx], model->neg_contactor_voltage_mV);

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_TEMPERATURE_MIN);
        tx_buf[idx++] = HMI_TYPE_INT16;
        idx += hmi_buf_append_uint16(&tx_buf[idx], model->temperature_min_dC);

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_TEMPERATURE_MAX);
        tx_buf[idx++] = HMI_TYPE_INT16;
        idx += hmi_buf_append_uint16(&tx_buf[idx], model->temperature_max_dC);

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_CELL_VOLTAGE_MIN);
        tx_buf[idx++] = HMI_TYPE_INT16;
        idx += hmi_buf_append_uint16(&tx_buf[idx], model->cell_voltage_min_mV);

        idx += hmi_buf_append_uint16(&tx_buf[idx], HMI_REG_CELL_VOLTAGE_MAX);
        tx_buf[idx++] = HMI_TYPE_INT16;
        idx += hmi_buf_append_uint16(&tx_buf[idx], model->cell_voltage_max_mV);

        duart_send_packet(&HMI_SERIAL_DUART, tx_buf, idx);
    }
}
