#pragma once

#include <stdint.h>

// Which firmware this is: the commit it was built from and when (see cmake/firmware_id.cmake).

// Commit's first 7 hex digits; bit 31 set if built with uncommitted changes. 0 = unknown.
extern const uint32_t bms_build_hash;
// Build time, Unix seconds UTC. 0 = unknown.
extern const uint32_t bms_build_time;
// "8b0c547" or "8b0c547-dirty"
extern const char bms_build_describe[];
// "2026-10-21 10:02 UTC"
extern const char bms_build_time_text[];

#define BMS_BUILD_HASH_DIRTY (1u << 31)
