#ifndef HW_NVM_H
#define HW_NVM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NVM_SIZE (64 * 1024)

typedef struct bms_model bms_model_t;

int update_boot_count(void);
bool nvm_save_calibration(bms_model_t *model);
bool nvm_load_calibration(bms_model_t *model);

#endif // HW_NVM_H
