#pragma once

// The supervisor is built per chemistry, not per pack: the pack layout (the
// cell map) comes from the BMS at runtime (shared/supervisor_protocol.h).
//
// The chemistry header gives CHEMISTRY and the hard cell voltage limits the
// supervisor enforces; the BMS can't change those. Select it with
// -DSUPERVISOR_CHEMISTRY=lfp|nmc (supervisor/CMakeLists.txt).

#ifndef SUPERVISOR_CHEMISTRY_HEADER
#error "SUPERVISOR_CHEMISTRY_HEADER is not set (configure with -DSUPERVISOR_CHEMISTRY=lfp|nmc)"
#endif
#include SUPERVISOR_CHEMISTRY_HEADER

#include "shared/supervisor_limits.h"

#if !defined(CHEMISTRY) || (CHEMISTRY != LFP && CHEMISTRY != NMC)
#error "The supervisor chemistry header must define CHEMISTRY as LFP or NMC"
#endif
#if !defined(CELL_VOLTAGE_HARD_MIN_mV) || !defined(CELL_VOLTAGE_HARD_MAX_mV)
#error "The supervisor chemistry header must define the hard cell voltage limits"
#endif

#define SUPERVISOR_CHEMISTRY CHEMISTRY
#define SUPERVISOR_CHEMISTRY_NAME (CHEMISTRY == LFP ? "LFP" : "NMC")
