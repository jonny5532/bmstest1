#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NVM_SIZE (128 * 1024)

typedef struct bms_model bms_model_t;

int update_boot_count(void);
bool nvm_format(void);
bool nvm_save_persistent_slow(bms_model_t *model);
bool nvm_load_persistent_slow(bms_model_t *model);
bool nvm_save_persistent_fast(bms_model_t *model);
bool nvm_load_persistent_fast(bms_model_t *model);
void nvm_schedule_save_persistent_fast(bms_model_t *model);
bool nvm_load_ocv_learn(bms_model_t *model);
void nvm_tick(bms_model_t *model);
