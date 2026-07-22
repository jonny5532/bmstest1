#pragma once

#include "app/model.h"

// cal mul value should be around 0.013 Volts/count
#define MIN_CALIBRATION_MUL 0.008f
#define MAX_CALIBRATION_MUL 0.018f

// offset should be within a few volts
#define MIN_NEG_CONTACTOR_OFFSET_MV -5000
#define MAX_NEG_CONTACTOR_OFFSET_MV 5000
#define MIN_POS_CONTACTOR_OFFSET_MV -5000
#define MAX_POS_CONTACTOR_OFFSET_MV 5000

// current offset should be within +/- 200
#define MIN_CURRENT_OFFSET -200
#define MAX_CURRENT_OFFSET 200

bool calibrate_battery_voltage(bms_model_t *model, int32_t raw_val, float sample_count);
bool calibrate_output_voltage(bms_model_t *model, int32_t raw_val, float sample_count);
bool calibrate_neg_contactor(bms_model_t *model, int32_t raw_val, float sample_count);
bool calibrate_pos_contactor(bms_model_t *model, int32_t raw_val, float sample_count);
#if BMS_BOARD == BMS_BOARD_REV1
bool calibrate_link_voltage(bms_model_t *model, int32_t raw_val, float sample_count);
#endif
bool calibrate_current_offset(bms_model_t *model, int32_t raw_offset);
