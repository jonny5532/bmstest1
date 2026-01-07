#include "hmi_serial.h"
#include "duart.h"
#include "../allocations.h"
#include "../pins.h"
#include "../chip/time.h"
#include "../../model.h"

#include "pico/unique_id.h"

uint8_t device_address = 0;
uint32_t next_announce_timestep = 0;
uint32_t announce_period = 64;

void init_hmi_serial() {
    // 937500 baud (close to 1Mbit)
    init_duart(&HMI_SERIAL_DUART, 460800, PIN_HMI_SERIAL_TX, PIN_HMI_SERIAL_RX, true); //9375000 works!

    // Stagger announcements to reduce collisions
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    announce_period = 56 + (id.id[7] % 16);
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

static inline uint16_t hmi_buf_get_uint16(const uint8_t *buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static inline uint32_t hmi_buf_get_uint32(const uint8_t *buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static inline uint64_t hmi_buf_get_uint64(const uint8_t *buf) {
    return (uint64_t)buf[0] | ((uint64_t)buf[1] << 8) | ((uint64_t)buf[2] << 16) | ((uint64_t)buf[3] << 24) |
           ((uint64_t)buf[4] << 32) | ((uint64_t)buf[5] << 40) | ((uint64_t)buf[6] << 48) | ((uint64_t)buf[7] << 56);
}

static uint8_t hmi_get_type_size(uint8_t type) {
    return type & 0x0F;
}

static uint8_t hmi_append_register_value(uint8_t *buf, uint16_t reg_id, bms_model_t *model) {
    uint8_t idx = 0;
    idx += hmi_buf_append_uint16(&buf[idx], reg_id);

    switch (reg_id) {
        case HMI_REG_SERIAL:
            buf[idx++] = HMI_TYPE_UINT64;
            union {
                pico_unique_board_id_t id;
                uint64_t serial;
            } u;
            pico_get_unique_board_id(&u.id);
            idx += hmi_buf_append_uint64(&buf[idx], u.serial);
            break;
        case HMI_REG_MILLIS:
            buf[idx++] = HMI_TYPE_UINT64;
            idx += hmi_buf_append_uint64(&buf[idx], millis());
            break;
        case HMI_REG_SOC:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], model->soc);
            break;
        case HMI_REG_CURRENT:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)model->current_mA);
            break;
        case HMI_REG_CHARGE:
            buf[idx++] = HMI_TYPE_INT64;
            // TODO - confirm conversion
            int64_t charge_mC = (model->charge_raw * 132736) / 1000000;
            idx += hmi_buf_append_uint64(&buf[idx], (uint64_t)charge_mC);
            break;
        case HMI_REG_BATTERY_VOLTAGE:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)model->battery_voltage_mV);
            break;
        case HMI_REG_OUTPUT_VOLTAGE:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)model->output_voltage_mV);
            break;
        case HMI_REG_POS_CONTACTOR_VOLTAGE:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)model->pos_contactor_voltage_mV);
            break;
        case HMI_REG_NEG_CONTACTOR_VOLTAGE:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)model->neg_contactor_voltage_mV);
            break;
        case HMI_REG_TEMPERATURE_MIN:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->temperature_min_dC);
            break;
        case HMI_REG_TEMPERATURE_MAX:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->temperature_max_dC);
            break;
        case HMI_REG_CELL_VOLTAGE_MIN:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->cell_voltage_min_mV);
            break;
        case HMI_REG_CELL_VOLTAGE_MAX:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->cell_voltage_max_mV);
            break;
        default:
            if (reg_id >= HMI_REG_CELL_VOLTAGES_START && reg_id <= HMI_REG_CELL_VOLTAGES_END) {
                uint16_t cell_idx = reg_id - HMI_REG_CELL_VOLTAGES_START;
                if (cell_idx < 120) {
                    buf[idx++] = HMI_TYPE_INT16;
                    idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->cell_voltages_mV[cell_idx]);
                } else {
                    // Unknown cell
                    idx -= 2; // rollback reg_id
                }
            } else {
                // Unknown register
                idx -= 2; // rollback reg_id
            }
            break;
    }
    return idx;
}

// Send a regular announce device message including our current address and
// unique ID. The HMI can use this to discover devices on the bus.
void hmi_send_announce_device() {
    uint8_t tx_buf[32];
    uint8_t idx=0;

    tx_buf[idx++] = HMI_MSG_ANNOUNCE_DEVICE;

    tx_buf[idx++] = HMI_ANNOUNCE_DEVICE_TYPE_BMS;
    tx_buf[idx++] = device_address;

    pico_get_unique_board_id((pico_unique_board_id_t*)&tx_buf[idx]);
    idx += 8;

    // Should we include more info here (eg, uptime?, version?)

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, idx);
}

static void hmi_handle_set_device_address(const uint8_t *rx_buf, size_t len) {
    if (len < 11) return;
    uint8_t current_addr = rx_buf[1];
    uint8_t new_addr = rx_buf[2];
    uint64_t serial = hmi_buf_get_uint64(&rx_buf[3]);

    union {
        pico_unique_board_id_t id;
        uint64_t serial;
    } u;
    pico_get_unique_board_id(&u.id);

    if (serial == u.serial && (current_addr == device_address)) {
        device_address = new_addr;
        // Respond with announce to confirm
        hmi_send_announce_device();
    }
}

static void hmi_handle_read_registers(const uint8_t *rx_buf, size_t len, bms_model_t *model) {
    if (len < 2) return;
    uint8_t addr = rx_buf[1];
    if (addr != device_address) return;

    uint8_t tx_buf[256];
    uint16_t tx_idx = 0;
    tx_buf[tx_idx++] = HMI_MSG_READ_REGISTERS_RESPONSE;
    tx_buf[tx_idx++] = device_address;

    for (size_t i = 2; i + 1 < len; i += 2) {
        uint16_t reg_id = hmi_buf_get_uint16(&rx_buf[i]);
        tx_idx += hmi_append_register_value(&tx_buf[tx_idx], reg_id, model);
    }

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_idx);
}

static void hmi_handle_write_registers(const uint8_t *rx_buf, size_t len, bms_model_t *model) {
    if (len < 2) return;
    uint8_t addr = rx_buf[1];
    if (addr != device_address) return;

    uint16_t rx_idx = 2;
    uint8_t tx_buf[256];
    uint16_t tx_idx = 0;
    tx_buf[tx_idx++] = HMI_MSG_READ_REGISTERS_RESPONSE;
    tx_buf[tx_idx++] = device_address;

    while (rx_idx + 2 < len) {
        uint16_t reg_id = hmi_buf_get_uint16(&rx_buf[rx_idx]);
        rx_idx += 2;
        if (rx_idx >= len) break;
        uint8_t type = rx_buf[rx_idx++];
        uint8_t size = hmi_get_type_size(type);
        if (rx_idx + size > len) break;

        // Perform write if applicable
        if (reg_id == HMI_REG_SOC && type == HMI_TYPE_UINT32) {
            model->soc = hmi_buf_get_uint32(&rx_buf[rx_idx]);
        }
        // Add more writable registers here as needed

        rx_idx += size;
        
        // Always append the current (possibly new) value to the response
        tx_idx += hmi_append_register_value(&tx_buf[tx_idx], reg_id, model);
    }

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_idx);
}

static void hmi_handle_read_cell_voltages(const uint8_t *rx_buf, size_t len, bms_model_t *model) {
    // if (len < 2) return;
    // uint8_t addr = rx_buf[1];
    // if (addr != device_address) return;

    uint8_t tx_buf[243];
    uint16_t tx_idx = 0;

    tx_buf[tx_idx++] = HMI_MSG_READ_CELL_VOLTAGES_RESPONSE;
    tx_buf[tx_idx++] = device_address;
    tx_buf[tx_idx++] = 120; // number of cell voltages

    // Note: Delta coding for cell voltages
    //   Absolute voltages sent as two-byte big-endian signed integers, with the MSB set
    //   Delta voltages sent as single-byte signed integers in the range -64 to +63, with MSB clear
    //   Initial voltage is zero (if first byte is a delta).

    int16_t last_cell_voltage = 0;
    for(int cell_idx = 0; cell_idx < 120; cell_idx++) {
        const int16_t cell_voltage = model->cell_voltages_mV[cell_idx];
        const int16_t delta = cell_voltage - last_cell_voltage;
        if(delta >= -64 && delta <= 63) {
            // can encode as delta
            tx_buf[tx_idx++] = (delta & 0x7F);
        } else {
            // encode as absolute
            tx_buf[tx_idx++] = ((cell_voltage >> 8) & 0xFF) | 0x80; // set high bit for absolute values
            tx_buf[tx_idx++] = (cell_voltage >> 0) & 0xFF;
        }
        last_cell_voltage = cell_voltage;
    }

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_idx);
}

void hmi_serial_tick(bms_model_t *model) {
    uint8_t rx_buf[256];

    // Periodically announce ourselves. 
    if(timestep() >= next_announce_timestep) {
        next_announce_timestep = timestep() + announce_period;
        hmi_send_announce_device();
    }

    // Randomize the offset slightly based on address to reduce collisions.
    if((timestep() & 511) == (device_address * 17) % 512) {
        hmi_send_announce_device();
    }

    size_t len = duart_read_packet(&HMI_SERIAL_DUART, rx_buf, sizeof(rx_buf));
    if(len > 0) {
        uint8_t msg_type = rx_buf[0];
        switch (msg_type) {
            case HMI_MSG_SET_DEVICE_ADDRESS:
                hmi_handle_set_device_address(rx_buf, len);
                break;
            case HMI_MSG_READ_REGISTERS:
                hmi_handle_read_registers(rx_buf, len, model);
                break;
            case HMI_MSG_WRITE_REGISTERS:
                hmi_handle_write_registers(rx_buf, len, model);
                break;
            case HMI_MSG_READ_CELL_VOLTAGES:
                hmi_handle_read_cell_voltages(rx_buf, len, model);
                break;
            default:
                // Ignore other messages (responses or unknown)
                break;
        }
    }
}
