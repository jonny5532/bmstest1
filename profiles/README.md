# Pack profiles

A pack profile is a header that sets the chemistry and pack-specific settings
(cell count and presence mask, capacity, shunt calibration, current limits...).
The rest of the BMS limits, which don't depend on the pack, stay in
`bms/config/limits.h`, and the PCB revision and board modifications are a
separate choice (`boards/README.md`).

```
profiles/
  chemistry/   base settings for each cell chemistry (lfp.h, nmc.h)
  builtin/     profiles for the packs we support
  custom/      your own profiles; ignored by git, so a pull won't touch them
  validate.h   sanity checks run on every build
```

## Selecting a profile

Pass the profile's file name (without `.h`):

```
cmake -S bms -B bms/build -DBMS_PROFILE=lfp1 -DBMS_BOARD=rev0
./build_bms.sh lfp1 rev0
```

The default is `bluetesla`. The profile is found in `profiles/custom/` or
`profiles/builtin/`, plus `BMS_PROFILE_DIR` if you set it
(`-DBMS_PROFILE_DIR=/path/to/my/profiles`, handy for keeping profiles outside
the repo altogether, where even `git clean -fdx` can't reach them). The same
name in two places is an error, so give custom profiles their own names.

The profile name is logged at boot and shows up in `picotool info`.

## Writing a custom profile

Include the profile you're starting from, then `#undef` and redefine whatever
you want to change:

```c
// profiles/custom/my_lfp.h
#pragma once

#include "builtin/lfp1.h"

#undef  NAMEPLATE_CAPACITY_AH
#define NAMEPLATE_CAPACITY_AH 155
#undef  ACTUAL_CAPACITY_AH
#define ACTUAL_CAPACITY_AH 155

#undef  DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV
#define DEFAULT_CELL_VOLTAGE_WORKING_MAX_mV 3320
```

Then build with `-DBMS_PROFILE=my_lfp`. There's a starting point in
`custom/example.h.sample`.

The `#undef` is required: redefining a setting without one fails the build, so
nothing gets overridden by accident. To describe a new pack from scratch,
include `chemistry/lfp.h` or `chemistry/nmc.h` instead and define the pack
settings yourself; see the built-in profiles for the full list.

The SOC estimator's tuning (EKF noise settings, the cell model, OCV
learning) lives in the chemistry files and can be overridden the same way,
e.g. to set `EKF_R_OHM` for your cells. What each setting does is described
in `bms/app/estimators/ekf.c`.

Precharge on the negative pole (via the FC negative contactor) is the
standard. A pack wired with the precharge element on the positive sets
`PRECHARGE_ON_NEGATIVE` to 0.

`validate.h` then checks the result: that every required setting is present,
that the voltage and temperature limits nest properly (hard outside soft,
soft outside working), and that `CELL_PRESENCE_WORDS` has exactly `NUM_CELLS`
bits set.
