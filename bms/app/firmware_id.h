#pragma once

#include <stdint.h>

// Commit's first 7 hex digits, with bit 31 set if built with uncommitted
// changes. 0 = unknown.
extern const uint32_t bms_build_hash;
#define BMS_BUILD_HASH_DIRTY (1u << 31)

// Build time, Unix seconds UTC
extern const uint32_t bms_build_time;

// Human readable, eg. "8b0c547-dirty, built 2026-10-21 10:02 UTC"
extern const char bms_build_version[];
