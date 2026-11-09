#include "hmi_registers.h"
#include "hmi_buf.h"
#include "hmi_serial.h"
#include "sys/time/time.h"
#include "drivers/sensors/ina228.h"

#include <math.h>

#include "pico/unique_id.h"

bool hmi_register_is_available(uint16_t reg_id, bms_model_t *model) {
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
        case HMI_REG_SHUNT_DIE_TEMP:
            return model->shunt_die_temperature_millis > 0;
        case HMI_REG_SHUNT_NTC_TEMP:
        case HMI_REG_SHUNT_NTC_RESISTANCE:
            return model->shunt_ntc_millis > 0;
        case HMI_REG_TEMPERATURE_MIN:
        case HMI_REG_TEMPERATURE_MAX:
            return model->temperature_millis > 0;
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

uint8_t hmi_append_register_value(uint8_t *buf, uint16_t reg_id, bms_model_t *model) {
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
        case HMI_REG_SHUNT_DIE_TEMP:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->shunt_die_temperature);
            break;
        case HMI_REG_SHUNT_NTC_TEMP:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->shunt_ntc_temperature);
            break;
        case HMI_REG_SHUNT_NTC_RESISTANCE:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->shunt_ntc_resistance_ohms);
            break;
        case HMI_REG_BOARD_REVISION:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->board_id);
            break;
        case HMI_REG_ISOLATION_OPEN_RESISTANCE:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->isolation_open_millis ? model->isolation_open_resistance_ohms : NAN);
            break;
        case HMI_REG_PACK_POS_TO_CHASSIS:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->isolation_pos_to_chassis_V);
            break;
        case HMI_REG_PACK_NEG_TO_CHASSIS:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->isolation_neg_to_chassis_V);
            break;
        case HMI_REG_ISOLATION_OPEN_AGE:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->isolation_open_millis
                ? (float)(millis() - model->isolation_open_millis) / 1000.0f : NAN);
            break;
        case HMI_REG_ISOLATION_ZEROED:
            buf[idx++] = HMI_TYPE_UINT8;
            buf[idx++] = isolation_zeroed(model) ? 1 : 0;
            break;
        case HMI_REG_BOARD_STRAP_COUNTS:
            buf[idx++] = HMI_TYPE_UINT16;
            idx += hmi_buf_append_uint16(&buf[idx], model->board_id_counts);
            break;
        case HMI_REG_TEMPERATURE_MIN:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->temperature_min);
            break;
        case HMI_REG_TEMPERATURE_MAX:
            buf[idx++] = HMI_TYPE_FLOAT;
            idx += hmi_buf_append_float(&buf[idx], model->temperature_max);
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
