#include "hmi_serial.h"
#include "hmi_arrays.h"
#include "drivers/comms/duart.h"
#include "config/allocations.h"
#include "config/pins.h"
#include "sys/time/time.h"
#include "drivers/sensors/ina228.h"
#include "app/estimators/ekf.h"
#include "app/model.h"
#include "app/monitoring/counters.h"
#include "config/limits.h"
#include "drivers/chip/nvm.h"
#include "sys/events/events.h"
#include "sys/logging/logging.h"

#include "pico/stdlib.h"
#include "pico/unique_id.h"

// BMS defaults to address 1
uint8_t device_address = 1;
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

static inline uint8_t hmi_buf_append_float(uint8_t *buf, float value) {
    uint8_t idx = 0;
    union {
        float f;
        uint32_t u;
    } u;
    u.f = value;
    idx += hmi_buf_append_uint32(buf, u.u);
    return idx;
}

static inline uint16_t hmi_buf_get_uint16(const uint8_t *buf) {
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static inline uint32_t hmi_buf_get_uint32(const uint8_t *buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static inline float hmi_buf_get_float(const uint8_t *buf) {
    union {
        float f;
        uint32_t u;
    } u;
    u.u = hmi_buf_get_uint32(buf);
    return u.f;
}

static inline uint64_t hmi_buf_get_uint64(const uint8_t *buf) {
    return (uint64_t)buf[0] | ((uint64_t)buf[1] << 8) | ((uint64_t)buf[2] << 16) | ((uint64_t)buf[3] << 24) |
           ((uint64_t)buf[4] << 32) | ((uint64_t)buf[5] << 40) | ((uint64_t)buf[6] << 48) | ((uint64_t)buf[7] << 56);
}

static uint8_t hmi_get_type_size(uint8_t type) {
    return type & 0x0F;
}

static bool hmi_register_is_available(uint16_t reg_id, bms_model_t *model) {
    // Currently only checks for initial data availability on power-on, not
    // staleness

    switch (reg_id) {
        case HMI_REG_SOC:
            return model->soc_millis > 0;
        case HMI_REG_CURRENT:
            return model->current_millis > 0;
        case HMI_REG_CHARGE:
            return model->charge_millis > 0;
        case HMI_REG_BATTERY_VOLTAGE:
            return model->high_voltages.battery_millis > 0;
        case HMI_REG_OUTPUT_VOLTAGE:
            return model->high_voltages.output_millis > 0;
        case HMI_REG_POS_CONTACTOR_VOLTAGE:
            return model->high_voltages.pos_contactor_millis > 0;
        case HMI_REG_NEG_CONTACTOR_VOLTAGE:
            return model->high_voltages.neg_contactor_millis > 0;
        case HMI_REG_LINK_VOLTAGE:
            return model->high_voltages.link_millis > 0;
        case HMI_REG_TEMPERATURE_MIN:
        case HMI_REG_TEMPERATURE_MAX:
            return model->temperature_millis > 0;
        case HMI_REG_EKF_CAPACITY:
            return model->ekf.initialized;
        case HMI_REG_CELL_VOLTAGE_MIN:
        case HMI_REG_CELL_VOLTAGE_MAX:
            return model->cell_voltage_millis > 0;
        case HMI_REG_SUPPLY_VOLTAGE_3V3:
            return model->supply_voltages.voltage_3V3_millis > 0;
        case HMI_REG_SUPPLY_VOLTAGE_5V:
            return model->supply_voltages.voltage_5V_millis > 0;
        case HMI_REG_SUPPLY_VOLTAGE_12V:
            return model->supply_voltages.voltage_12V_millis > 0;
        case HMI_REG_SUPPLY_VOLTAGE_CTR:
            return model->supply_voltages.voltage_contactor_millis > 0;
        case HMI_REG_INVERTER_SOC:
        case HMI_REG_INVERTER_FULL_CAPACITY_DAH:
        case HMI_REG_INVERTER_REMAINING_CAPACITY_DAH:
        case HMI_REG_INVERTER_MIN_VOLTAGE_LIMIT_DV:
        case HMI_REG_INVERTER_MAX_VOLTAGE_LIMIT_DV:
            return model->soc_millis > 0;
    }
    return true;
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
            idx += hmi_buf_append_uint64(&buf[idx], millis64());
            break;
        case HMI_REG_SOC:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->soc);
            break;
        case HMI_REG_CURRENT:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)model->current_mA);
            break;
        case HMI_REG_CHARGE:
            buf[idx++] = HMI_TYPE_INT64;
            idx += hmi_buf_append_uint64(&buf[idx], (uint64_t)raw_charge_to_mC(model->charge_raw));
            break;
        case HMI_REG_BATTERY_VOLTAGE:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)(model->high_voltages.battery * 1000));
            break;
        case HMI_REG_OUTPUT_VOLTAGE:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)(model->high_voltages.output * 1000));
            break;
        case HMI_REG_POS_CONTACTOR_VOLTAGE:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)(model->high_voltages.pos_contactor * 1000));
            break;
        case HMI_REG_NEG_CONTACTOR_VOLTAGE:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)(model->high_voltages.neg_contactor * 1000));
            break;
        case HMI_REG_LINK_VOLTAGE:
            buf[idx++] = HMI_TYPE_INT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)(model->high_voltages.link * 1000));
            break;
        case HMI_REG_EKF_CAPACITY:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->ekf.x[2]);
            break;
        case HMI_REG_SOC_FANCY_COUNT:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->soc_fancy_count);
            break;
        case HMI_REG_OVERCURRENT_CHARGE_BUFFER:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->excess_charge_buffer_dC * 100.0f / OVERCURRENT_BUFFER_LIMIT_dC);
            break;
        case HMI_REG_OVERCURRENT_DISCHARGE_BUFFER:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->excess_discharge_buffer_dC * 100.0f / OVERCURRENT_BUFFER_LIMIT_dC);
            break;
        case HMI_REG_SOFT_CHARGE_BUFFER: {
            int32_t b = model->soft_limit_charge_buffer_dC;
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], b >= 0 ? b * 100.0f / OVERCHARGE_BUFFER_LIMIT_dC
                                                          : b * 100.0f / OVERDISCHARGE_BUFFER_LIMIT_dC);
            break;
        }
        case HMI_REG_UART0_PACKETS_RECEIVED:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], debug_counters.uart0_packets_received);
            break;
        case HMI_REG_UART0_PACKETS_SENT:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], debug_counters.uart0_packets_sent);
            break;
        case HMI_REG_UART0_CRC_ERRORS:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], debug_counters.uart0_crc_errors);
            break;
        case HMI_REG_UART1_PACKETS_RECEIVED:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], debug_counters.uart1_packets_received);
            break;
        case HMI_REG_UART1_PACKETS_SENT:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], debug_counters.uart1_packets_sent);
            break;
        case HMI_REG_UART1_CRC_ERRORS:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], debug_counters.uart1_crc_errors);
            break;
        case HMI_REG_CAN_FRAMES_SENT:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], debug_counters.can_frames_sent);
            break;
        case HMI_REG_CAN_FRAMES_RECEIVED:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], debug_counters.can_frames_received);
            break;
        case HMI_REG_BALANCING_STATE:
            buf[idx++] = HMI_TYPE_UINT8;
            buf[idx++] = (uint8_t)model->balancing_sm.state;
            break;
        case HMI_REG_CALIBRATION_STATE:
            buf[idx++] = HMI_TYPE_UINT8;
            buf[idx++] = (uint8_t)model->calibration_sm.state;
            break;
        case HMI_REG_TEMPERATURE_MIN:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], (uint16_t)model->temperature_min);
            break;
        case HMI_REG_TEMPERATURE_MAX:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], (uint16_t)model->temperature_max);
            break;
        case HMI_REG_CELL_VOLTAGE_MIN:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->cell_voltage_min_mV);
            break;
        case HMI_REG_CELL_VOLTAGE_MAX:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->cell_voltage_max_mV);
            break;
        case HMI_REG_SYSTEM_STATE:
            buf[idx++] = HMI_TYPE_UINT8;
            buf[idx++] = (uint8_t)model->system_sm.state;
            break;
        case HMI_REG_CONTACTORS_STATE:
            buf[idx++] = HMI_TYPE_UINT8;
            buf[idx++] = (uint8_t)model->contactor_sm.state;
            break;
        case HMI_REG_SOC_VOLTAGE_BASED:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->soc_voltage_based);
            break;
        case HMI_REG_SOC_BASIC_COUNT:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->soc_basic_count);
            break;
        case HMI_REG_CAPACITY:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], get_actual_capacity_mC(model));
            break;
        case HMI_REG_NAMEPLATE_CAPACITY:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], (uint32_t)(NAMEPLATE_CAPACITY_AH * 3600000.0 + 0.5));
            break;
        case HMI_REG_SUPPLY_VOLTAGE_3V3:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->supply_voltages.voltage_3V3_mV);
            break;
        case HMI_REG_SUPPLY_VOLTAGE_5V:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->supply_voltages.voltage_5V_mV);
            break;
        case HMI_REG_SUPPLY_VOLTAGE_12V:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->supply_voltages.voltage_12V_mV);
            break;
        case HMI_REG_SUPPLY_VOLTAGE_CTR:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->supply_voltages.voltage_contactor_mV);
            break;
        case HMI_REG_CELL_VOLTAGE_WORKING_MIN:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], get_cell_voltage_working_min_mV(model));
            break;
        case HMI_REG_CELL_VOLTAGE_WORKING_MAX:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], get_cell_voltage_working_max_mV(model));
            break;
        case HMI_REG_SOC_SCALING_MIN:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->soc_scaling_min);
            break;
        case HMI_REG_SOC_SCALING_MAX:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->soc_scaling_max);
            break;
        case HMI_REG_VOLTAGE_LIMIT_OFFSET_LOWER:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->pack_voltage_limit_lower_offset_dV);
            break;
        case HMI_REG_VOLTAGE_LIMIT_OFFSET_UPPER:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->pack_voltage_limit_upper_offset_dV);
            break;
        case HMI_REG_CELL_VOLTAGE_SOFT_MIN:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], get_cell_voltage_soft_min_mV(model));
            break;
        case HMI_REG_CELL_VOLTAGE_SOFT_MAX:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], get_cell_voltage_soft_max_mV(model));
            break;
        case HMI_REG_AUTO_BALANCING_PERIOD_MS:
            buf[idx++] = HMI_TYPE_UINT32;
            idx += hmi_buf_append_uint32(&buf[idx], model->auto_balancing_period_ms);
            break;
        case HMI_REG_BALANCING_PERIODS_PER_MV:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->balancing_periods_per_mV);
            break;
        case HMI_REG_BALANCE_MIN_OFFSET_MV:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->balance_min_offset_mV);
            break;
        case HMI_REG_MINIMUM_BALANCING_VOLTAGE_MV:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], get_minimum_balancing_voltage_mV(model));
            break;
        case HMI_REG_BALANCING_START_VOLTAGE_MV:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->balancing_start_voltage_mV);
            break;
        case HMI_REG_INVERTER_SOC:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->inverter_outputs.soc);
            break;
        case HMI_REG_INVERTER_FULL_CAPACITY_DAH:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->inverter_outputs.full_capacity_dAh);
            break;
        case HMI_REG_INVERTER_REMAINING_CAPACITY_DAH:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->inverter_outputs.remaining_capacity_dAh);
            break;
        case HMI_REG_INVERTER_MIN_VOLTAGE_LIMIT_DV:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->inverter_outputs.min_voltage_limit_dV);
            break;
        case HMI_REG_INVERTER_MAX_VOLTAGE_LIMIT_DV:
            buf[idx++] = HMI_TYPE_INT16;
            idx += hmi_buf_append_uint16(&buf[idx], (uint16_t)model->inverter_outputs.max_voltage_limit_dV);
            break;

        case HMI_REG_USER_CHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->user_charge_current_limit_dA - 1);
            break;
        case HMI_REG_USER_DISCHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->user_discharge_current_limit_dA - 1);
            break;
        case HMI_REG_CHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->charge_current_limit_dA);
            break;
        case HMI_REG_DISCHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->discharge_current_limit_dA);
            break;
        case HMI_REG_TEMP_CHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->temp_charge_current_limit_dA);
            break;
        case HMI_REG_TEMP_DISCHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->temp_discharge_current_limit_dA);
            break;
        case HMI_REG_CELL_VOLTAGE_CHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->cell_voltage_charge_current_limit_dA);
            break;
        case HMI_REG_CELL_VOLTAGE_DISCHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->cell_voltage_discharge_current_limit_dA);
            break;
        case HMI_REG_WORKING_CHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->working_charge_current_limit_dA);
            break;
        case HMI_REG_WORKING_DISCHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            // There isn't actually a field for this, fake it
            uint16_t discharge_limit = model->below_working_min ? 0 : 0xffff;
            idx += hmi_buf_append_uint16(&buf[idx], discharge_limit);
            break;
        case HMI_REG_DELTA_CHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->delta_charge_current_limit_dA);
            break;
        case HMI_REG_HARD_CHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], CHARGE_MAX_CURRENT_dA);
            break;
        case HMI_REG_HARD_DISCHARGE_CURRENT_LIMIT_DA:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], DISCHARGE_MAX_CURRENT_dA);
            break;
        case HMI_REG_PROTOCOL_VERSION:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], HMI_PROTOCOL_VERSION);
            break;
#ifdef OCV_LEARN
        case HMI_REG_OCV_LEARN_S_FULL:
            if (!model->ocv_learn.persistent.valid) {
                // Not learning yet
                idx -= 2; // rollback reg_id
                break;
            }
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->ocv_learn.persistent.s_full);
            break;
        case HMI_REG_OCV_LEARN_CAPACITY_REF:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->ocv_learn.capacity_ref_Ah);
            break;
        case HMI_REG_OCV_LEARN_CAPACITY_STATE:
            buf[idx++] = HMI_TYPE_UINT8;
            buf[idx++] = model->ocv_learn.capacity_state;
            break;
        case HMI_REG_OCV_LEARN_CAPACITY:
        case HMI_REG_OCV_LEARN_CAPACITY_SD:
        case HMI_REG_OCV_LEARN_CAPACITY_DEPTH:
        case HMI_REG_OCV_LEARN_CAPACITY_DEPTH_V:
        case HMI_REG_OCV_LEARN_CAPACITY_DEPTH_CYCLES: {
            const ocv_learn_t *l = &model->ocv_learn;
            float v = reg_id == HMI_REG_OCV_LEARN_CAPACITY       ? l->capacity_Ah
                    : reg_id == HMI_REG_OCV_LEARN_CAPACITY_SD    ? l->capacity_sd_Ah
                    : reg_id == HMI_REG_OCV_LEARN_CAPACITY_DEPTH ? l->capacity_depth
                    : reg_id == HMI_REG_OCV_LEARN_CAPACITY_DEPTH_V ? l->capacity_depth_V
                                                                   : l->capacity_depth_cycles;
            // Zero before the learner's first tick, so go by the state
            if (l->capacity_state < OCV_LEARN_CAPACITY_PRELIMINARY) v = NAN;
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], v);
            break;
        }
#endif

        default:
            if (reg_id >= HMI_REG_CELL_VOLTAGES_START && reg_id <= HMI_REG_CELL_VOLTAGES_END) {
                uint16_t cell_idx = reg_id - HMI_REG_CELL_VOLTAGES_START;
                if (cell_idx < 120) {
                    buf[idx++] = HMI_TYPE_INT16;
                    idx += hmi_buf_append_uint16(&buf[idx], model->cell_voltages_mV[cell_idx]);
                } else {
                    // Unknown cell
                    idx -= 2; // rollback reg_id
                }
            } else if (reg_id >= HMI_REG_MODULE_TEMPS_START && reg_id <= HMI_REG_MODULE_TEMPS_END) {
                uint16_t temp_idx = reg_id - HMI_REG_MODULE_TEMPS_START;
                if (temp_idx < 8) {
                    buf[idx++] = HMI_TYPE_FLOAT;
                    idx += hmi_buf_append_float(&buf[idx], model->module_temperatures[temp_idx]);
                } else {
                    // Unknown temp
                    idx -= 2; // rollback reg_id
                }
            } else if (reg_id >= HMI_REG_RAW_TEMPS_START && reg_id <= HMI_REG_RAW_TEMPS_END) {
                uint16_t temp_idx = reg_id - HMI_REG_RAW_TEMPS_START;
                if (temp_idx < (16+24+8)) {
                    buf[idx++] = HMI_TYPE_UINT16;
                    idx += hmi_buf_append_uint16(&buf[idx], model->raw_temperatures[temp_idx]);
                } else {
                    // Unknown temp
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
        if (!hmi_register_is_available(reg_id, model)) {
            // Skip unavailable registers
            continue;
        }
        // Max single register: 2 (id) + 1 (type) + 8 (value) = 11 bytes
        if((size_t)tx_idx + 11 > sizeof(tx_buf)) {
            // prevent buffer overflow
            break;
        }
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

    bool needs_flashing = false;

    while (((unsigned)rx_idx + 2) < len) {
        uint16_t reg_id = hmi_buf_get_uint16(&rx_buf[rx_idx]);
        rx_idx += 2;
        if (rx_idx >= len) break;
        uint8_t type = rx_buf[rx_idx++];
        uint8_t size = hmi_get_type_size(type);
        if (rx_idx + size > len) break;

        if(type == HMI_TYPE_UINT8) {
            switch(reg_id) {
                case HMI_REG_SYSTEM_REQUEST:
                    model->system_req = (system_requests_t)rx_buf[rx_idx];
                    break;
            }
        } else if(type == HMI_TYPE_UINT16) {
            uint16_t v = hmi_buf_get_uint16(&rx_buf[rx_idx]);
            switch(reg_id) {
                case HMI_REG_SOC:
                    ekf_set_soc(model, v);
                    break;
                case HMI_REG_CELL_VOLTAGE_WORKING_MIN:
                    if(get_cell_voltage_working_min_mV(model) != v) {
                        model->cell_voltage_working_min_mV = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_CELL_VOLTAGE_WORKING_MAX:
                    if(get_cell_voltage_working_max_mV(model) != v) {
                        model->cell_voltage_working_max_mV = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_CELL_VOLTAGE_SOFT_MIN:
                    if(get_cell_voltage_soft_min_mV(model) != v) {
                        model->cell_voltage_soft_min_mV = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_CELL_VOLTAGE_SOFT_MAX:
                    if(get_cell_voltage_soft_max_mV(model) != v) {
                        model->cell_voltage_soft_max_mV = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_BALANCING_PERIODS_PER_MV:
                    if(model->balancing_periods_per_mV != v) {
                        model->balancing_periods_per_mV = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_BALANCE_MIN_OFFSET_MV:
                    if(model->balance_min_offset_mV != v) {
                        model->balance_min_offset_mV = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_MINIMUM_BALANCING_VOLTAGE_MV:
                    if(get_minimum_balancing_voltage_mV(model) != v) {
                        model->minimum_balancing_voltage_mV = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_BALANCING_START_VOLTAGE_MV:
                    if(model->balancing_start_voltage_mV != v) {
                        model->balancing_start_voltage_mV = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_USER_CHARGE_CURRENT_LIMIT_DA:
                    // These are stored as +1 to allow 0 to indicate "no limit"
                    if(model->user_charge_current_limit_dA != (v + 1)) {
                        model->user_charge_current_limit_dA = v + 1;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_USER_DISCHARGE_CURRENT_LIMIT_DA:
                    if(model->user_discharge_current_limit_dA != (v + 1)) {
                        model->user_discharge_current_limit_dA = v + 1;
                        needs_flashing = true;
                    }
                    break;
            }
        } else if(type == HMI_TYPE_INT16) {
            int16_t v = (int16_t)hmi_buf_get_uint16(&rx_buf[rx_idx]);
            switch(reg_id) {
                case HMI_REG_SOC_SCALING_MIN:
                    if(model->soc_scaling_min != v) {
                        model->soc_scaling_min = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_SOC_SCALING_MAX:
                    if(model->soc_scaling_max != v) {
                        model->soc_scaling_max = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_VOLTAGE_LIMIT_OFFSET_LOWER:
                    if(model->pack_voltage_limit_lower_offset_dV != v) {
                        model->pack_voltage_limit_lower_offset_dV = v;
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_VOLTAGE_LIMIT_OFFSET_UPPER:
                    if(model->pack_voltage_limit_upper_offset_dV != v) {
                        model->pack_voltage_limit_upper_offset_dV = v;
                        needs_flashing = true;
                    }
                    break;
            }
        } else if(type == HMI_TYPE_UINT32) {
            uint32_t v = hmi_buf_get_uint32(&rx_buf[rx_idx]);
            switch(reg_id) {
                case HMI_REG_CAPACITY:
                    // 0 goes back to the profile's; otherwise 1 to 1000Ah
                    if(model->actual_capacity_mC != v && (v == 0 || (v >= 3600000u && v <= 3600000000u))) {
                        ekf_set_capacity(model, v);
                        needs_flashing = true;
                    }
                    break;
                case HMI_REG_AUTO_BALANCING_PERIOD_MS:
                    if(model->auto_balancing_period_ms != v) {
                        model->auto_balancing_period_ms = v;
                        needs_flashing = true;
                    }
                    break;
            }
        } else if(type == HMI_TYPE_FLOAT) {
            float v = hmi_buf_get_float(&rx_buf[rx_idx]);
            switch(reg_id) {
#ifdef OCV_LEARN
                case HMI_REG_OCV_LEARN_S_FULL:
                    // Saved to NVM by the learner
                    if(!model->ocv_learn.persistent.valid || model->ocv_learn.persistent.s_full != v) {
                        ocv_learn_set_s_full(&model->ocv_learn, v);
                    }
                    break;
#endif
            }
        }

        rx_idx += size;
        
        // Always append the current (possibly new) value to the response
        if((size_t)tx_idx + 11 <= sizeof(tx_buf)) {
            tx_idx += hmi_append_register_value(&tx_buf[tx_idx], reg_id, model);
        }
    }

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_idx);

    if(needs_flashing) {
        nvm_save_persistent_slow(model);
    }
}

static void hmi_handle_read_cell_voltages(const uint8_t *rx_buf, size_t len, bms_model_t *model) {
    if (len < 2) return;
    uint8_t addr = rx_buf[1];
    if (addr != device_address) return;

    if(model->cell_voltages_millis == 0) {
        // No valid data yet, don't reply
        return;
    }

    uint8_t tx_buf[243];
    uint16_t tx_idx = 0;

    tx_buf[tx_idx++] = HMI_MSG_READ_CELL_VOLTAGES_RESPONSE;
    tx_buf[tx_idx++] = device_address;
    tx_buf[tx_idx++] = 120; // number of cell voltages

    tx_idx += hmi_encode_int16_delta_array(&tx_buf[tx_idx], sizeof(tx_buf) - tx_idx, model->cell_voltages_mV, 120, NULL);

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_idx);
}

static void hmi_handle_read_balancing_times(const uint8_t *rx_buf, size_t len, bms_model_t *model) {
    if (len < 2) return;
    uint8_t addr = rx_buf[1];
    if (addr != device_address) return;

    uint8_t tx_buf[243];
    uint16_t tx_idx = 0;

    tx_buf[tx_idx++] = HMI_MSG_READ_BALANCING_TIMES_RESPONSE;
    tx_buf[tx_idx++] = device_address;
    tx_buf[tx_idx++] = 120; // number of cell balancing times

    tx_idx += hmi_encode_int16_delta_array(&tx_buf[tx_idx], sizeof(tx_buf) - tx_idx, model->balancing_sm.balance_time_remaining, 120, NULL);

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_idx);
}

static void hmi_handle_read_array(const uint8_t *rx_buf, size_t len, bms_model_t *model) {
    if (len < 5) return;
    uint8_t addr = rx_buf[1];
    if (addr != device_address) return;

    uint8_t array_id = rx_buf[2];
    uint16_t start_index = hmi_buf_get_uint16(&rx_buf[3]);

    uint8_t tx_buf[256];
    size_t tx_len = hmi_build_read_array_response(tx_buf, sizeof(tx_buf), device_address, model, array_id, start_index);

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_len);
}

static void hmi_handle_write_array(const uint8_t *rx_buf, size_t len, bms_model_t *model) {
    if (len < HMI_ARRAY_WRITE_HEADER_LEN) return;
    uint8_t addr = rx_buf[1];
    if (addr != device_address) return;

    uint8_t array_id = rx_buf[2];
    uint16_t start_index = hmi_buf_get_uint16(&rx_buf[4]);
    if (!hmi_apply_write_array(model, rx_buf, len)) {
        info_printf("HMI: refused write to array %u from %u\n", array_id, start_index);
    }

    // Reply with the values now in place, whether or not the write was applied
    uint8_t tx_buf[256];
    size_t tx_len = hmi_build_read_array_response(tx_buf, sizeof(tx_buf), device_address, model, array_id, start_index);

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_len);
}

static void hmi_handle_read_log(const uint8_t *rx_buf, size_t len) {
    if (len < 2) return;
    uint8_t addr = rx_buf[1];
    if (addr != device_address) return;

    for (int i = 0; i < 20; i++) {
        // Space check: each packet is (payload_len + 4) bytes. 
        // Max payload is 256. 256 + 4 = 260 bytes.
        if (duart_get_tx_free_space(&HMI_SERIAL_DUART) < 260) {
            break;
        }

        uint8_t tx_buf[256];
        uint16_t tx_idx = 0;

        tx_buf[tx_idx++] = HMI_MSG_READ_LOG_RESPONSE;
        tx_buf[tx_idx++] = device_address;

        size_t read = logging_read(&tx_buf[tx_idx], 254);
        
        if (read == 0) {
            if (i == 0) {
                // To acknowledge the request, we send an empty response if no data is available.
                duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_idx);
            }
            break;
        }

        tx_idx += (uint16_t)read;
        duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_idx);

        if (read < 254) {
            // Not enough data to fill another packet.
            break;
        }
    }
}

static void hmi_handle_read_events(const uint8_t *rx_buf, size_t len, bms_model_t *model) {
    (void)model;
    if (len < 4) return;
    uint8_t addr = rx_buf[1];
    if (addr != device_address) return;

    uint16_t start_index = hmi_buf_get_uint16(&rx_buf[2]);
    if (start_index > ERR_HIGHEST) start_index = ERR_HIGHEST;

    uint8_t tx_buf[256];
    uint16_t tx_idx = 0;

    tx_buf[tx_idx++] = HMI_MSG_READ_EVENTS_RESPONSE;
    tx_buf[tx_idx++] = device_address;
    
    // next_index and number of events in this packet
    uint16_t next_index_ptr = tx_idx;
    tx_idx += 2;
    uint16_t count_ptr = tx_idx;
    tx_idx += 2;

    uint16_t count = 0;
    uint16_t i;
    for (i = start_index; i < ERR_HIGHEST; i++) {
        bms_event_slot_t *slot = &bms_event_slots[i];
        if (slot->count > 0) {
            // event_type (2), level (2), count (2), timestamp (8), data (8) = 22 bytes
            if (tx_idx + 22 > 250) {
                break;
            }

            tx_idx += hmi_buf_append_uint16(&tx_buf[tx_idx], i);
            tx_idx += hmi_buf_append_uint16(&tx_buf[tx_idx], slot->level);
            tx_idx += hmi_buf_append_uint16(&tx_buf[tx_idx], slot->count);
            tx_idx += hmi_buf_append_uint64(&tx_buf[tx_idx], slot->timestamp);
            tx_idx += hmi_buf_append_uint64(&tx_buf[tx_idx], slot->data64);
            count++;
        }
    }

    hmi_buf_append_uint16(&tx_buf[next_index_ptr], i);
    hmi_buf_append_uint16(&tx_buf[count_ptr], count);

    duart_send_packet(&HMI_SERIAL_DUART, tx_buf, tx_idx);
}

void hmi_serial_tick(bms_model_t *model) {
    uint8_t rx_buf[256];

    // Periodically announce ourselves. 
    if(timestep() >= next_announce_timestep) {
        next_announce_timestep = timestep() + announce_period;
        hmi_send_announce_device();
    }

    // Handling a message seems to take 50-100us. We handle one message per tick
    // (giving a max effective throughput of ~100kbps with 256 byte messages).

    size_t len = duart_read_packet(&HMI_SERIAL_DUART, rx_buf, sizeof(rx_buf));
    if(len > 0) {
        uint8_t msg_type = rx_buf[0];
        switch (msg_type) {
            case HMI_MSG_SET_DEVICE_ADDRESS:
                hmi_handle_set_device_address(rx_buf, len);
                break;
            case HMI_MSG_READ_REGISTERS:
                hmi_handle_read_registers(rx_buf, len, model);
                // Postpone the next announce, so that if we are polled
                // frequently enough we never risk causing a collision.
                next_announce_timestep = timestep() + announce_period;
                break;
            case HMI_MSG_WRITE_REGISTERS:
                hmi_handle_write_registers(rx_buf, len, model);
                break;
            case HMI_MSG_READ_CELL_VOLTAGES:
                hmi_handle_read_cell_voltages(rx_buf, len, model);
                break;
            case HMI_MSG_READ_EVENTS:
                hmi_handle_read_events(rx_buf, len, model);
                break;
            case HMI_MSG_READ_BALANCING_TIMES:
                hmi_handle_read_balancing_times(rx_buf, len, model);
                break;
            case HMI_MSG_READ_LOG:
                hmi_handle_read_log(rx_buf, len);
                break;
            case HMI_MSG_READ_ARRAY:
                hmi_handle_read_array(rx_buf, len, model);
                break;
            case HMI_MSG_WRITE_ARRAY:
                hmi_handle_write_array(rx_buf, len, model);
                break;
            default:
                // Ignore other messages (responses or unknown)
                break;
        }
    }
}
