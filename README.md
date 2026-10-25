## Safety mechanisms

### Events

There are four levels of event severity:

- INFO
- WARNING
- CRITICAL
- FATAL

**INFO** events are informational, and don't indicate a problem.

**WARNING** events indicate a potential problem which is not yet serious.

**CRITICAL** events signal imminent contactor opening and escalate to FATAL
after a timeout (which differs by event type). To catch intermittent (flapping)
faults, a cumulative time buffer increments while the event is active and
decrements when cleared; if the buffer fills, the event escalates to FATAL.

**FATAL** events immediately trigger the contactor opening sequence, and require
manual intervention to reset. Unlike the other event types, FATAL events do not
clear when the triggering condition stops.


### Current limits

The BMS continuously calculates the maximum allowed charge and discharge current
in its present state, and these limits are transmitted to the inverter.

The limits are the lesser of several different calculations:

**Temperature** derating, which linearly tapers the maximum current such that it
reaches zero at the maximum temperature limit. The limits are different for
charging and discharging, according to the cell chemistry.

**State of charge** derating




### Voltage limits

There are six separate voltage limits, which are defined in reference to the
voltage of an individual cell. These limits operate progressively, and are
designed to ensure safety whilst making it usually possible to recover a pack
that has gone beyond the normal voltage range.

#### Working limits

The battery's operating range is defined by the working voltage limits,
representing 0% and 100% State of Charge (SoC). 

The BMS will taper the charge current at the top to attempt to keep the cell
'virtual OCV' at the working limit, which allows for absorption charge using
current limits. This means the terminal voltage can go slightly higher than the
working limits.

The discharge current limit will drop to zero at the lower limit, with 100mV
hysteresis band before it recovers.

Since some inverters prevent discharging below a specific threshold (e.g., 10%),
the Minimum SoC setting allows you to define a percentage that will correspond
to the lower voltage limit.

#### Soft voltage limits

If a cell's voltage goes outside the soft limits, the BMS immediately issues a
WARNING. It will also restrict current to prevent further charging (at the upper
limit) or discharging (at the lower limit).

The BMS will disconnect the battery if:
- A current continues to flow against these restrictions.
- The state persists for more than one hour.

This gives you a chance to correct the problem - if the battery has
disconnected, you can restart the BMS, and will have one hour to
charge/discharge the battery back into the normal range.


#### Hard voltage limits

If a cell exceeds its hard voltage limits, the BMS will immediately disconnect
the battery. These limits are set to prevent permanent damage to the cell
chemistry. Recovery would require bypassing or manually energizing the
contactors. 

Be careful if you decide to manually recover or recharge the battery, as it may
be damaged, and you may need to bypass safety mechanisms to do so.


### Response time

If a critical condition arises there is usually a short timeout (a second or
two) to see if it clears, after which the contactor opening sequence will start.
This sets the current limits to zero, and then waits for the current to fall and
opens the contactors. If the current hasn't fallen below the threshold after two
seconds, the contactors open anyway.

Thus the BMS will react to fault conditions within a timescale of seconds,
whilst avoiding nuisance trips due to brief glitches, and avoiding contactor
damage where possible. External safety equipment should be used for more
time-critical protection against overcurrent (fuses) or leakage (an RCD).


### Supervisor

A second MCU, the supervisor (firmware in `supervisor/`), holds the contactor
gate drivers disabled (`CTR_EN`) unless its own checks pass. It doesn't trust
the BMS's readings: it snoops the BMB cell voltage reads off the isoSPI bus and
checks them against the same cell map and hard limits as the BMS
(`bms/shared/cell_map.h`, from `BMS_PROFILE` in `bms/config/limits.h`).

It enables the drivers once the BMS is sending status over the internal serial
link, both were built for the same pack (the cell map hash matches), three scans
in a row have every populated cell read and within the hard limits, and the
BMS's cell min/max agree with the snooped ones. The BMS holds off closing the
contactors until the supervisor reports that it is enabled.

Once enabled, the supervisor cuts the drivers if:
- the BMS stops sending status for twice its watchdog timeout, or the scans
  stop for twice as long as the BMS tolerates stale cell data (e.g. the BMB
  chain unplugged). It re-enables once fresh scans check out again. The BMS's
  slow mode doesn't extend this, so the BMS only uses slow mode while idle with
  the contactors open;
- a populated cell stays out of range or unread for twice as long as the BMS
  takes to open the contactors itself. This latches until the BMS restarts.

The doubled delays mean the BMS always gets to open the contactors first.

Build the supervisor with the same `BMS_PROFILE` and `BMS_BOARD` as the BMS:
use the same preset (see [Build configurations](#build-configurations)), e.g.

```bash
cd supervisor
cmake --preset desk && cmake --build --preset desk
```

Over its USB serial, `s` prints the status and `r` the last snooped scan (with
`*` on the populated channels). The BMS CLI has `supervisor` and `cells_raw`.


## Build configurations

`BMS_PROFILE` picks the pack and `BMS_BOARD` the PCB. Both default to the old
setup (BlueTesla, Rev0), so a plain `cmake ..` is wrong for a Rev1 board: it
scales the pack voltage for Rev0's dividers and raises `VOLTAGE_MISMATCH`.

Each board's configuration is a CMake preset, with the same name in
`CMakePresets.json` (BMS) and `supervisor/CMakePresets.json` (supervisor). The
pack itself is a `BMS_PROFILE` in `bms/config/limits.h`; a preset only picks
the profile and adds the board and USB options. A preset builds into
`build-<preset>/`.

| Preset | Pack (`BMS_PROFILE`) | PCB (`BMS_BOARD`) | USB |
|---|---|---|---|
| `desk` | Desk bench, 15 cells (1) | Rev1 (2) | VBUS sense on GPIO3 |
| `ck0-bluetesla` | BlueTesla 96s NMC (2) | Rev1 (2) | external VBUS-powered D+ pull-up |
| `ck1-lfp-rev0` | LFP1 108s (3) | Rev0 (1) | stock |
| `rev1-nmc-jpwhit` | BlueTesla 96s NMC (2) | Rev1 (2) | stock: no pull-up mod, no VBUS sense |

```bash
cmake --preset rev1-nmc-jpwhit && cmake --build --preset rev1-nmc-jpwhit
cd supervisor && cmake --preset rev1-nmc-jpwhit && cmake --build --preset rev1-nmc-jpwhit
```

Use `BMS_USB_EXTERNAL_PULLUP` or `BMS_USB_VBUS_SENSE` only on boards with that
modification: an unmodified board built with either one never connects over USB.
A new board gets a new preset rather than a hand-typed `cmake` line.

**Output contactors.** By default the inverter is on the FC port, which gets its positive
from Link+ through a jumper on the pack's 63 A PCS / heat-pump fuse (F4); the FC+ contactor
is not used. `rev1-nmc-jpwhit` also sets `BMS_FC_POS_CONTACTOR`, for a pack without that
jumper: the FC+ contactor (K4) then connects the FC port and follows Link+ (closed for the
precharge and while running). Its coil must be wired to the board's FC+ channel. The F4
voltage-drop warning is off in that build.

### Building on Windows

Each preset has a `-win` twin (e.g. `desk-win`), listed only on Windows. It
builds natively with the tools the Raspberry Pi Pico VS Code extension installs
in `%USERPROFILE%\.pico-sdk`, so nothing needs to be on `PATH`.
`cmake/pico_extension_tools.cmake` picks the versions at configure time:

- **pioasm and picotool:** the versions matching the Pico SDK in `../pico-sdk`
  (SDK 2.2.0 uses `tools/2.2.0` and `picotool/2.2.0-*`)
- **Arm GCC and Ninja:** the newest installed

Nothing needs editing after an SDK or extension update. If the extension hasn't
installed the tools for your SDK version yet, the configure step says which one
is missing: open the project in VS Code and let the extension install them, or
set that variable yourself. A tool set on the command line wins, e.g.
`-DPICO_TOOLCHAIN_PATH=...` for a different Arm GCC.

```powershell
cmake --preset desk-win; cmake --build --preset desk-win
```

A clean build of both chips takes seconds this way. A WSL build of a checkout on
the Windows drive (`/mnt/c`) takes minutes, mostly in file access and in
compiling picotool from source for every new build folder.

Run `version` on the BMS console to check what a board is running. Built
images can be kept in `firmware/`, which git ignores.


## Balancing

There are five settings that control balancing:

1. Auto balancing interval (ms)
   This is how regularly the balancing routine tries to start a new balancing cycle. It acts as a short interval between balancing cycles, which can be useful to let cell voltages recover before the voltages are sampled for the next cycle. Setting to 0 disables balancing. Values of 10000ms - 60000ms are typical.

2. Balance periods per mV (periods/mV)
   This affects how long each cell is balanced for, during a balancing cycle, according on its voltage delta above the minimum - higher cells get balanced for longer. A period is 1.28s long. This can be set low (eg: 1), which results in regular short balancing cycles of a minute or less, or very high (eg: 1000) which can result in a single balancing cycle lasting 20 hours or more. 

3. Balancing minimum offset (mv)
   This is the minimum delta, below which balancing won't start. To avoid unnecessary balancing cycles triggered by noise, this should be at least 2mV.

4. Minimum balance voltage (mV)
   If any cell falls below this voltage during balancing, the balancing will stop immediately.

5. Balance start voltage (mV)
   Balancing won't start until a single cell voltage is above this threshold.

It is usual to top-balance a pack, evening the cell voltages when the pack is full. Conventionally this needs the pack to be kept nearly full during the balancing process. However CellKeeper allows balancing cycles to run for hours, whilst the pack is discharging, as long as they start when the pack is near full (at least one cell is above the 'Balance start voltage' threshold). This is especially useful with LFP, where the delta is only visible above 99% SoC.




## Internal architecture

The BMS has a conventional single-threaded loop architecture, with a 50Hz loop
frequency. Sensor reading and communication is performed asynchronously
using interrupts/DMA to avoid blocking where possible.






The main loop of the BMS runs every 20 milliseconds. It will therefore take at
least 20ms for the BMS to react to a change, up to several multiples of 20ms if
the signal propagates through the model in stages.



Most operations performed in the main loop should easily complete within the
20ms deadline. However the BMS performs occasional flash memory writes which can
take much longer (the worst case block-erase time of the SPI flash chip is 2
seconds). The normal WARNING that occurs if a loop tick overruns is suppressed
when flash memory operations are performed.

A system-wide watchdog timer is set up with a 5 second deadline, which gets
reset every tick. If the main loop stalls for more than 5 seconds, the chip will
reboot and raise a WARNING event.


