#ifndef HW_NVM_H
#define HW_NVM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define NVM_SIZE (64 * 1024)

int update_boot_count(void);

#endif // HW_NVM_H
