# Boards

A board header says which PCB revision the BMS is built for and how that
particular board has been modified. Pack settings (cells, chemistry, capacity,
how the pack is wired) belong in a pack profile instead (`profiles/`).

```
boards/
  builtin/   the revisions (rev0.h, rev1.h, rev1_1.h) and the modified boards we have
  custom/    your own boards; ignored by git, so a pull won't touch them
```

## Selecting a board

Pass the board's file name (without `.h`). There is no default:

```
cmake -S bms -B bms/build -DBMS_PROFILE=bluetesla -DBMS_BOARD=rev1
```

| Board | Revision | Modifications |
|---|---|---|
| `rev0` | Rev0 | none |
| `rev1` | Rev1 | none |
| `rev1_1` | Rev1.1 | none (USB VBUS is sensed on GPIO3 by the board's own divider) |
| `desk` | Rev1 | USB VBUS sensed on GPIO3 (J12 pin 2) through a 3.3k/5.1k divider |
| `ck0` | Rev1 | external D+ pull-up powered from the host's VBUS |

The board is found in `boards/custom/` or `boards/builtin/`, plus
`BMS_BOARD_DIR` if you set it. The name is logged at boot and shows up in
`picotool info`.

The revision is also checked at boot against the PCB's strap: firmware built
for the wrong revision refuses to operate. Modifications can't be detected, so
a modified board must be built with its own board name. Getting a USB setting
wrong means USB never connects, and the board can't be reflashed remotely.

## Settings

Every revision file defines all of these, so a modified board changes them
with `#undef` and `#define`, as pack profiles do:

| Setting | Meaning |
|---|---|
| `BMS_BOARD_REVISION` | `BMS_BOARD_REV0` or `BMS_BOARD_REV1`; code checks it with `#if IS_BMS_BOARD(REV1)` |
| `HAS_ADS1115_SECONDARY()` | 1 if there is a second ADS1115 (0x49) |
| `ADS1115_DEFAULT_MUL` | HV divider scaling, volts per ADS1115 count |
| `ISOLATION_ZERO_DEFAULT_COUNTS` | isolation monitor zero until the board is zeroed |
| `USB_VBUS_SENSE` | 1: the chip's D+ pull-up is only on while VBUS is sensed on GPIO3 |
| `USB_EXTERNAL_PULLUP` | 1: an external VBUS-powered D+ pull-up; the chip's own stays off |

Either USB setting stops a switched-off host being back-powered through D+.
A board has one or neither, never both.

## Writing a custom board

Include the revision it is built on (as `boards/builtin/<name>.h`), then
`#undef` and redefine what differs:

```c
// boards/custom/my_board.h
#pragma once

#include "boards/builtin/rev1.h"

#undef  USB_VBUS_SENSE
#define USB_VBUS_SENSE 1
```

Then build with `-DBMS_BOARD=my_board`. Redefining a setting without `#undef`
fails the build.
