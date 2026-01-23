#include "nvm.h"

#include "../../app/model.h"

#include "hardware/flash.h"
#include "pico/flash.h"
#include "../vendor/littlefs/lfs.h"
#include <string.h>
#include <stdio.h>

// Use the last part of the flash. Assuming 2MB flash, 1.5MB offset is safe.
#define NVM_FLASH_OFFSET (1536 * 1024)
#define FLASH_MMAP_ADDR (XIP_BASE + NVM_FLASH_OFFSET)

typedef struct {
    uint32_t addr;
    const uint8_t *data;
    size_t len;
} flash_op_params_t;

static void nvm_flash_program_helper(void *param) {
    flash_op_params_t *p = (flash_op_params_t *)param;
    flash_range_program(p->addr, p->data, p->len);
}

static void nvm_flash_erase_helper(void *param) {
    flash_op_params_t *p = (flash_op_params_t *)param;
    flash_range_erase(p->addr, p->len);
}

bool nvm_read(uint32_t offset, void *dest, size_t size) {
    if (offset + size > NVM_SIZE) return false;
    memcpy(dest, (const void *)(FLASH_MMAP_ADDR + offset), size);
    return true;
}

bool nvm_write(uint32_t offset, const void *src, size_t size) {
    if (offset + size > NVM_SIZE) return false;
    if ((offset % FLASH_PAGE_SIZE) != 0 || (size % FLASH_PAGE_SIZE) != 0) return false;
    flash_op_params_t p = {NVM_FLASH_OFFSET + offset, (const uint8_t *)src, size};
    return flash_safe_execute(nvm_flash_program_helper, &p, 100) == PICO_OK;
}

bool nvm_erase_all(void) {
    flash_op_params_t p = {NVM_FLASH_OFFSET, NULL, NVM_SIZE};
    return flash_safe_execute(nvm_flash_erase_helper, &p, 100) == PICO_OK;
}

// LittleFS callbacks
static int lfs_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size) {
    uint32_t abs_off = block * c->block_size + off;
    memcpy(buffer, (const void *)(FLASH_MMAP_ADDR + abs_off), size);
    return LFS_ERR_OK;
}

static int lfs_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size) {
    uint32_t abs_off = block * c->block_size + off;
    flash_op_params_t p = {NVM_FLASH_OFFSET + abs_off, (const uint8_t *)buffer, size};
    if (flash_safe_execute(nvm_flash_program_helper, &p, 100) != PICO_OK) return LFS_ERR_IO;
    return LFS_ERR_OK;
}

static int lfs_erase(const struct lfs_config *c, lfs_block_t block) {
    uint32_t abs_off = block * c->block_size;
    flash_op_params_t p = {NVM_FLASH_OFFSET + abs_off, NULL, c->block_size};
    if (flash_safe_execute(nvm_flash_erase_helper, &p, 100) != PICO_OK) return LFS_ERR_IO;
    return LFS_ERR_OK;
}

static int lfs_sync(const struct lfs_config *c) {
    return LFS_ERR_OK;
}

// LittleFS configuration
static const struct lfs_config cfg = {
    .read  = lfs_read,
    .prog  = lfs_prog,
    .erase = lfs_erase,
    .sync  = lfs_sync,

    .read_size = 1,
    .prog_size = FLASH_PAGE_SIZE,
    .block_size = FLASH_SECTOR_SIZE,
    .block_count = NVM_SIZE / FLASH_SECTOR_SIZE,
    .cache_size = FLASH_PAGE_SIZE,
    .lookahead_size = 32,
    .block_cycles = 500,
};

static lfs_t lfs;

int update_boot_count(void) {
    int err = lfs_mount(&lfs, &cfg);
    if (err) {
        lfs_format(&lfs, &cfg);
        err = lfs_mount(&lfs, &cfg);
        if (err) return -1;
    }

    uint32_t boot_count = 0;
    lfs_file_t file;
    err = lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    if (err) return -1;

    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));
    boot_count++;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));
    lfs_file_close(&lfs, &file);

    lfs_unmount(&lfs);
    return (int)boot_count;
}

typedef struct __attribute__((packed)) {
    uint32_t version;

    // ADS1115 voltage calibration
    int32_t battery_voltage_mul;
    int32_t output_voltage_mul;
    int32_t neg_contactor_mul;
    int32_t neg_contactor_offset_mV;
    int32_t pos_contactor_mul;

    // Current calibration
    int32_t current_offset;
} calibration_data_t;

bool nvm_save_calibration(bms_model_t *model) {
    int err = lfs_mount(&lfs, &cfg);
    if (err) return false;

    lfs_file_t file;
    err = lfs_file_open(&lfs, &file, "calibration", LFS_O_WRONLY | LFS_O_CREAT);
    if (err) return false;

    calibration_data_t data = {
        .version = 1,
        .battery_voltage_mul = model->battery_voltage_mul,
        .output_voltage_mul = model->output_voltage_mul,
        .neg_contactor_mul = model->neg_contactor_mul,
        .neg_contactor_offset_mV = model->neg_contactor_offset_mV,
        .pos_contactor_mul = model->pos_contactor_mul,
        .current_offset = model->current_offset,
    };
    lfs_file_write(&lfs, &file, &data, sizeof(data));
    lfs_file_close(&lfs, &file);

    lfs_unmount(&lfs);
    return true;
}

bool nvm_load_calibration(bms_model_t *model) {
    int err = lfs_mount(&lfs, &cfg);
    if (err) return false;

    lfs_file_t file;
    err = lfs_file_open(&lfs, &file, "calibration", LFS_O_RDONLY);
    if (err) return false;

    calibration_data_t data = {0};
    lfs_file_read(&lfs, &file, &data, sizeof(data));
    lfs_file_close(&lfs, &file);

    lfs_unmount(&lfs);

    if (data.version != 1) {
        return false;
    }

    model->battery_voltage_mul = data.battery_voltage_mul;
    model->output_voltage_mul = data.output_voltage_mul;
    model->neg_contactor_mul = data.neg_contactor_mul;
    model->neg_contactor_offset_mV = data.neg_contactor_offset_mV;
    model->pos_contactor_mul = data.pos_contactor_mul;
    model->current_offset = data.current_offset;

    return true;
}

struct __attribute__((packed)) {
    uint32_t version;

    float ekf_x[3];
    float ekf_P[3][3];

    // TODO - should we just use float for this?
    int32_t basic_count_charge_raw;
    
    // store events here? or separately?
} runtime_state_t;
