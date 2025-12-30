#include "nvm.h"
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
