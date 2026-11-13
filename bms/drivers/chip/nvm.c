#include "nvm.h"

#include "../../app/model.h"
#include "sys/logging/logging.h"

#include "hardware/flash.h"
#include "pico/flash.h"
#include "pico/stdlib.h"
#include "../vendor/littlefs/lfs.h"
#include <stddef.h>
#include <string.h>
#include <stdio.h>

// Use the last part of the flash. Assuming 2MB flash, start 128KB before the end.
#define NVM_FLASH_OFFSET ((2048 - 128) * 1024)
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
    (void)c;
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

// True if LittleFS's two superblocks (blocks 0 and 1) are erased, as on a
// board that has never stored anything here.
static bool nvm_area_blank(void) {
    const uint32_t *words = (const uint32_t *)FLASH_MMAP_ADDR;
    for (size_t i = 0; i < (2 * FLASH_SECTOR_SIZE) / sizeof(uint32_t); i++) {
        if (words[i] != 0xFFFFFFFFu) return false;
    }
    return true;
}

bool nvm_format(void) {
    int err = lfs_format(&lfs, &cfg);
    if (err) {
        error_printf("NVM: format failed (%d)\n", err);
        return false;
    }
    info_printf("NVM: formatted\n");
    return true;
}

int update_boot_count(void) {
    int err = lfs_mount(&lfs, &cfg);
    if (err) {
        // Only a blank area is formatted. Anything else may hold the
        // settings, calibration and lifetime counters: leave it for a look
        // (the BMS runs on defaults and saves nothing) rather than wipe it.
        // `nvm_format yes` on the USB console formats it on purpose.
        if (!nvm_area_blank()) {
            error_printf("NVM: mount failed (%d) and the area isn't blank: not formatting it. "
                         "Settings are at their defaults and won't be saved.\n", err);
            return -1;
        }
        info_printf("NVM: blank, formatting\n");
        if (!nvm_format()) return -1;
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

// A 4 byte version is stored first, immediately followed by the data.
static bool nvm_save(const char *name, uint32_t version, const void *data, size_t size) {
    int err = lfs_mount(&lfs, &cfg);
    if (err) {
        error_printf("NVM: mount failed saving '%s' (%d)\n", name, err);
        return false;
    }

    lfs_file_t file;
    // Truncate so that downgraded firmware will remove stale fields.
    err = lfs_file_open(&lfs, &file, name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (err) {
        error_printf("NVM: open failed saving '%s' (%d)\n", name, err);
        lfs_unmount(&lfs);
        return false;
    }

    bool ok = lfs_file_write(&lfs, &file, &version, sizeof(version)) == (lfs_ssize_t)sizeof(version)
        && lfs_file_write(&lfs, &file, data, size) == (lfs_ssize_t)size;
    // Closing flushes, so it can fail too
    ok = (lfs_file_close(&lfs, &file) == 0) && ok;
    if (!ok) error_printf("NVM: write failed saving '%s'\n", name);

    lfs_unmount(&lfs);
    return ok;
}

// Loads the data if the file's version matches, leaving it untouched otherwise.
// An older, shorter save leaves the fields it didn't have zero. If the read
// fails partway, the data is zeroed (every saved struct takes zero as "unset").
static bool nvm_load(const char *name, uint32_t version, void *data, size_t size) {
    int err = lfs_mount(&lfs, &cfg);
    if (err) {
        error_printf("NVM: mount failed loading '%s' (%d)\n", name, err);
        return false;
    }

    lfs_file_t file;
    err = lfs_file_open(&lfs, &file, name, LFS_O_RDONLY);
    if (err) {
        error_printf("NVM: open failed loading '%s' (%d)\n", name, err);
        lfs_unmount(&lfs);
        return false;
    }

    bool ok = false;
    uint32_t stored_version = 0;
    if (lfs_file_read(&lfs, &file, &stored_version, sizeof(stored_version)) == (lfs_ssize_t)sizeof(stored_version)
        && stored_version == version) {
        memset(data, 0, size);
        ok = lfs_file_read(&lfs, &file, data, size) >= 0;
        if (!ok) {
            memset(data, 0, size);
            error_printf("NVM: read failed loading '%s'\n", name);
        }
    }
    lfs_file_close(&lfs, &file);

    lfs_unmount(&lfs);
    return ok;
}

// typedef struct __attribute__((packed)) {
//     uint32_t version;

//     // ADS1115 voltage calibration
//     int32_t battery_voltage_mul;
//     int32_t output_voltage_mul;
//     int32_t neg_contactor_mul;
//     int32_t neg_contactor_offset_mV;
//     int32_t pos_contactor_mul;

//     // Current calibration
//     int32_t current_offset;
// } calibration_data_t;

bool nvm_save_persistent_fast(bms_model_t *model) {
    // Allow missing a deadline due to NVM write slowness
    model->ignore_missed_deadline = true;

    return nvm_save("fast", BMS_MODEL_PERSISTENT_FAST_VERSION, &model->persistent_fast, sizeof(model->persistent_fast));
}

bool nvm_load_persistent_fast(bms_model_t *model) {
    return nvm_load("fast", BMS_MODEL_PERSISTENT_FAST_VERSION, &model->persistent_fast, sizeof(model->persistent_fast));
}

bool nvm_save_persistent_slow(bms_model_t *model) {
    // Allow missing a deadline due to NVM write slowness
    model->ignore_missed_deadline = true;

    return nvm_save("slow", BMS_MODEL_PERSISTENT_SLOW_VERSION, &model->persistent_slow, sizeof(model->persistent_slow));
}

bool nvm_load_persistent_slow(bms_model_t *model) {
    return nvm_load("slow", BMS_MODEL_PERSISTENT_SLOW_VERSION, &model->persistent_slow, sizeof(model->persistent_slow));
}

static const uint32_t OCV_LEARN_VERSION = 1;

static bool nvm_save_ocv_learn(bms_model_t *model) {
    // Allow missing a deadline due to NVM write slowness
    model->ignore_missed_deadline = true;

    return nvm_save("ocv_learn", OCV_LEARN_VERSION, &model->ocv_learn.persistent, sizeof(model->ocv_learn.persistent));
}

bool nvm_load_ocv_learn(bms_model_t *model) {
    return nvm_load("ocv_learn", OCV_LEARN_VERSION, &model->ocv_learn.persistent, sizeof(model->ocv_learn.persistent));
}

void nvm_schedule_save_persistent_fast(bms_model_t *model) {
    // TODO: prevent too many saves in a short time?

    // Schedule immediate save of 'fast' persistent data
    model->nvm_fast_saved_timestep = timestep() - (300000 / TIMESTEP_PERIOD_MS) + 1;
}

void nvm_tick(bms_model_t *model) {
    // Saving takes a few ms (or longer, if erase is needed).

    // Save 'fast' persistent data every 5 minutes
    if(timestep_every_ms(300000, &model->nvm_fast_saved_timestep)) {
        uint32_t start = time_us_32();
        if(!nvm_save_persistent_fast(model)) {
            error_printf("NVM: failed to save persistent fast data\n");
        } else {
            debug_printf("NVM: saved persistent fast data in %ld us\n", time_us_32() - start);
        }
    } else if(model->ocv_learn.dirty) {
        // At most daily: after each settled full charge and each daily decay of the counts
        model->ocv_learn.dirty = false;
        nvm_save_ocv_learn(model);
    }
}