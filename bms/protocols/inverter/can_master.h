#pragma once

/*
 * Master mode: this BMS controls its own battery and a fleet of slave BMSs
 * over the inter-BMS bus (CAN2, PIN_INTERCAN_*), presenting the whole parallel
 * HV bus to the inverter (on CAN1) as one battery.
 *
 * Build with -DCAN_MASTER=ON (see CMakeLists.txt). The slaves are ordinary
 * builds with INVERTER_PROTOCOL=custom_can; they speak the fleet protocol
 * (docs/custom_can_protocol.md) on their CAN2 port. The inverter protocol
 * (byd_can) is unchanged and keeps CAN1 to itself.
 */

#include <stdint.h>

#include "protocols/inverter/can.h"
#include "protocols/inverter/custom_can.h"

typedef struct bms_model bms_model_t;

// Prefix for the fleet traffic (CAN2 is dedicated, so any legal value works).
#ifndef CAN_MASTER_PREFIX
#define CAN_MASTER_PREFIX 0x0300
#endif

#ifndef CAN_MASTER_BUS
#define CAN_MASTER_BUS CAN_BUS_INTER
#endif

#ifndef CAN_MASTER_MAX_SLAVES
#define CAN_MASTER_MAX_SLAVES 8
#endif

// Flags given to slaves in ASSIGN. Default: slaves keep running if the master
// goes quiet (the inverter loses its BMS at the same time and stops on its
// own). Add CUSTOM_CAN_ASSIGN_FLAG_STOP_ON_TIMEOUT to make them disconnect.
#ifndef CAN_MASTER_ASSIGN_FLAGS
#define CAN_MASTER_ASSIGN_FLAGS 0u
#endif

#ifndef CAN_MASTER_HEARTBEAT_MS
#define CAN_MASTER_HEARTBEAT_MS CUSTOM_CAN_COMMAND_RECOMMENDED_PERIOD_MS
#endif

// A slave offline for this long gives up its node number
#ifndef CAN_MASTER_RETIRE_MS
#define CAN_MASTER_RETIRE_MS 60000
#endif

// Brings up the inter-BMS bus and starts listening for slaves.
void init_can_master(void);

// Call every tick, before inverter_tick(&model->fleet_outputs). Talks to the
// slaves and fills model->fleet_outputs / model->fleet.
void can_master_tick(bms_model_t *model);

// Dump the slave table to the debug log.
void can_master_print_status(void);
