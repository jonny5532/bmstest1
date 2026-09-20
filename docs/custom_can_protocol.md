# Custom multi-battery CAN protocol

This protocol lets several BMS-managed batteries share one DC bus and one CAN
bus with an upstream *controller* (for example
[Battery Emulator](https://github.com/dalathegreat/Battery-Emulator)). The
controller aggregates the batteries and talks to the inverter itself; the
batteries only ever talk to the controller.

Design goals, in priority order:

1. **Safe by construction** – the battery keeps all contactor sequencing
   local. The controller can only *ask* for RUN or STOP; the BMS decides how
   and when (e.g. it waits for current to fall before opening contactors).
2. **Zero configuration** – batteries announce themselves; the controller
   picks addresses. Nothing is configured on the battery.
3. **Robust to restarts** of either side, with explicit liveness checking in
   both directions.
4. **Complete visibility** – everything the BMS knows (cell voltages,
   balancing, events, supply rails, contactor voltages…) is on the bus, spread
   over time so the bus is never flooded.
5. **Coexistence** – only a small discovery block of identifiers is fixed; the
   controller chooses where everything else lives, so the bus can be shared
   with other equipment.
6. **Hard to misimplement** – level-based commands that are repeated, idempotent
   assignment, big-endian everywhere, "ignore what you don't understand".

The authoritative byte layouts live in
[`bms/protocols/inverter/custom_can.h`](../bms/protocols/inverter/custom_can.h),
which has no dependencies and can be dropped into a controller project. The
battery-side implementation is
[`bms/protocols/inverter/custom_can.c`](../bms/protocols/inverter/custom_can.c);
its host-side tests in [`tests/test_custom_can.c`](../tests/test_custom_can.c)
double as worked examples of every exchange described here.

Build the firmware with it using:

```bash
cmake -DINVERTER_PROTOCOL=custom_can ..
```

(the default `byd_can` emulates a BYD Battery-Box directly to an inverter).

On CellKeeper hardware this protocol runs on the **inter-BMS port, CAN2**
(GPIO 4 TX / GPIO 5 RX, `PIN_INTERCAN_*`). CAN1 (`PIN_CAN_*`) remains the
inverter interface. Because can2040 needs a whole PIO block, CAN2 takes PIO2
from the `isosnoop` ISOSPI debugging aid, which is compiled out in any build
that enables CAN2 (`BMS_INTERCAN=1`, set automatically by CMake).

Two controllers are available for the CAN2 side: another CellKeeper built in
**master mode** (section 9), or an external controller such as Battery
Emulator following section 8.

---

## 1. Bus conventions

| Item | Value |
|---|---|
| Bit rate | 500 kbit/s |
| Identifiers | 29-bit extended only. Standard (11-bit) and RTR frames are ignored. |
| Byte order | Big-endian (network order) for every multi-byte field. |
| DLC | Frames are 8 bytes unless stated. Receivers must accept *longer* frames than they expect and ignore the extra bytes. |
| Reserved bytes | Sent as 0, ignored on receipt. |
| Unknown message types | Ignored. New types may appear in later protocol versions. |
| "No data" | `int16` telemetry fields use `0x7FFF` (`CUSTOM_CAN_INT16_NA`). Whole frames whose source data is not yet valid are simply not sent. |

## 2. Identifier layout and priority

Every identifier is split into three fields:

```
 28                     14 13        8 7          0
+-------------------------+-----------+------------+
|  prefix (15 bits)       | type (6)  | node (8)   |
+-------------------------+-----------+------------+
```

* **prefix** – chosen by the controller when it assigns a battery. All
  batteries under one controller normally share a prefix, but they need not.
  `base_id = prefix << 14`.
* **type** – fixed by this protocol (section 6). Lower is more important.
  `0x00–0x0F` are controller→battery, `0x10–0x3F` battery→controller.
* **node** – chosen by the controller, `1..254`. `0` means unassigned and
  `0xFF` addresses every battery on the prefix (controller→battery only).

CAN arbitration favours the numerically lowest identifier, and it compares
from the MSB. Because *type* sits above *node*, a `STATUS` frame from **any**
battery wins against a `CELL_VOLTAGES` page from **any** other battery, and
controller commands (type `0x00`) win against everything the batteries send.
Priority between battery traffic and unrelated bus traffic is set by the
controller's choice of prefix: pick a low prefix if the batteries matter more
than the other traffic, a high one otherwise.

### Filters

* Controller: accept `id & 0x1FFFC000 == prefix << 14` (one filter per
  prefix) plus the discovery block `id & 0x1FFFFF00 == 0x1CE10000`.
* Battery: accept `ASSIGN` (`0x1CE10100`) plus controller-type frames on its
  prefix whose node is its own or `0xFF`.

## 3. Discovery and assignment

The **discovery block** is the only fixed part of the address space: prefix
`0x7384`, i.e. identifiers `0x1CE10000–0x1CE13FFF`. Controllers must never
assign this prefix.

### 3.1 Serial number

Each battery derives a 32-bit **serial32** from its 64-bit RP2350 board id
using 32-bit FNV-1a (`custom_can_serial32()` in the header). It is printed at
boot, sent in `ANNOUNCE`, and the raw 64-bit id is available in the `SERIAL`
telemetry frame. Controllers should key persistent per-battery settings
(friendly name, preferred node) on serial32.

### 3.2 ANNOUNCE — battery → all

Identifier `0x1CE10000 | (serial32 & 0xFF)`. The low byte differs between
batteries, so simultaneous announcements from different batteries arbitrate
cleanly instead of colliding (identical identifiers with different payloads
are a CAN protocol violation). Announcement timing is additionally staggered
per battery.

| Byte | Content |
|---|---|
| 0 | bits 7..4 protocol version (`1`); bits 3..0 link state: 0 unassigned, 1 assigned |
| 1 | current node (0 if unassigned) |
| 2..5 | serial32 |
| 6..7 | current prefix (0 if unassigned) |

Sent every **1 s** while unassigned and every **5 s** while assigned (and once
shortly after any assignment change). Nothing else is transmitted while
unassigned.

### 3.3 ASSIGN — controller → one battery

Identifier `0x1CE10100`, DLC 8.

| Byte | Content |
|---|---|
| 0..3 | serial32 of the target battery (must match exactly) |
| 4..5 | prefix (`0..0x7FFF`, not `0x7384`) |
| 6 | node (`1..254`); `0` releases the battery back to unassigned |
| 7 | flags, see below |

Flags:

| Bit(s) | Name | Meaning |
|---|---|---|
| 0 | `STOP_ON_TIMEOUT` | If the controller goes quiet (section 4.2) the battery requests a graceful STOP. Default (0): keep the current run state. |
| 5..4 | `RATE_SHIFT` | Slow down non-essential telemetry by 2^n (n = 0..3). `STATUS`, `LIMITS` and `MEASUREMENTS` are never slowed. Use on large or shared buses. |
| others | reserved | send 0 |

Rules the battery applies:

* Wrong serial → silently ignored (it is for another battery).
* Prefix out of range, prefix `0x7384`, or node `0xFF` → rejected and logged.
* Node `0` → release: the battery returns to unassigned and announces.
* Same prefix/node as currently assigned → idempotent: liveness is refreshed,
  flags are updated, command tracking is kept (section 5.1).
* Otherwise → the battery adopts the new address immediately, resets its
  command tracking, and sends `STATUS`, `LIMITS`, `MEASUREMENTS` in the very
  next 20 ms tick, followed by all other frames over the next ~100 ms.

Send `ASSIGN` in response to an `ANNOUNCE` you want to act on. Do not send it
periodically; the battery keeps its assignment as long as it hears
`COMMAND`s. Re-sending the same assignment is harmless.

The battery's receive filter switches to the new address when it processes
the `ASSIGN` (within one 20 ms tick), so a `COMMAND` sent in the same
millisecond as an `ASSIGN` may be dropped. This is harmless because commands
are repeated (section 5), but if you want a deterministic handshake, wait for
the first `STATUS` before sending anything else.

## 4. Link lifecycle and liveness

```
                 ASSIGN (serial matches, valid)
  UNASSIGNED ───────────────────────────────────▶ ASSIGNED
  announce 1 s ◀─────────────────────────────── announce 5 s + telemetry
                 no controller frame for 5 s,
                 ASSIGN node 0, or address conflict
```

### 4.1 Battery restart

A rebooted battery starts unassigned and announces within about a second. The
controller sees `ANNOUNCE` with link state 0 from a serial it knows, and
re-assigns it (typically to the same node). Meanwhile the controller's own
battery timeout (below) has already flagged the battery as missing.

### 4.2 Controller restart / controller lost

The battery treats any `COMMAND` or `REQUEST` addressed to it (or to node
`0xFF`) on its prefix, and any `ASSIGN` for its serial, as proof of life. If
none arrives for **5 s** (`CUSTOM_CAN_CONTROLLER_TIMEOUT_MS`) the battery drops
its assignment, stops all telemetry, and announces at 1 s again.

A controller that restarts *quickly* simply sees announcements (state 1, with
the old prefix/node) or nothing yet, and can re-assign immediately — identical
re-assignment is accepted at any time. A controller that restarts *slowly*
finds every battery announcing as unassigned and re-discovers from scratch.
Either way no operator action is needed.

What happens to the contactors when the controller is lost is a policy
decision for the installer: by default the battery keeps whatever state it was
in (fail-static; the inverter will normally stop on its own because the
controller has also stopped talking to it). Set `STOP_ON_TIMEOUT` in `ASSIGN`
if you prefer the batteries to disconnect.

### 4.3 Battery lost (controller side)

`STATUS` is sent every **200 ms** regardless of anything else. Treat a battery
as lost after **2 s** without `STATUS` (`CUSTOM_CAN_BATTERY_TIMEOUT_MS`), and
stop counting its capacity and limits towards what you present to the
inverter.

### 4.4 Address conflicts

If a battery receives a battery-type frame (type ≥ `0x10`) carrying its own
prefix and node, another device is using its address (almost always a
controller bug assigning the same node twice). It logs a warning, drops its
assignment and re-announces. The controller will notice the battery vanish
and announce again; fix the assignment logic rather than the symptom.

## 5. Control

### 5.1 COMMAND — controller → battery (type `0x00`)

Identifier `prefix | 0x00<<8 | node` (or node `0xFF` for all). DLC ≥ 4.

| Byte | Content |
|---|---|
| 0 | desired state: 0 `NONE` (heartbeat only), 1 `STOP`, 2 `RUN` |
| 1 | reserved |
| 2..3 | sequence number, incremented every frame |
| 4..7 | reserved |

Send it to every assigned battery at least once per second; **500 ms** is
recommended. It is the controller's heartbeat, so send it even when you have
nothing to say (desired state `NONE`).

The desired state is a **level, repeated in every frame**, not a one-shot
command, so losing a frame never loses a command. The battery acts when the
value **changes** compared with the last non-`NONE` value it received (the
first value after (re)assignment always counts as a change). Consequences:

* A local operator can stop a battery from the HMI/CLI and the controller's
  steady stream of `RUN` will not fight it; the controller sees the discrepancy
  in `STATUS` (system state `INACTIVE` while commanded state is `RUN`).
* To re-assert a state, toggle it: send `STOP` for a frame or two, then `RUN`.
* Monitoring-only controllers send `NONE` forever and never influence the run
  state.

Internally the battery keeps a *pending* state until the system state machine
reaches it: `RUN` is satisfied by `OPERATING`; `STOP` by `INACTIVE` (or
`FAULT`, where contactors are already forced open). While pending, the battery
issues the corresponding system request whenever the system state machine is
able to accept it (e.g. `RUN` waits until initialisation has finished), at
most once per second. A pending `RUN` on a battery in `FAULT` stays pending
forever and is visible as such — the fault needs local attention.

### 5.2 Acknowledgement and the stop sequence

Every `COMMAND` (including heartbeats) is acknowledged by an immediate
`STATUS` frame echoing the sequence number, showing the *commanded state* the
battery has adopted and whether it is still *pending*.

A graceful stop looks like this:

1. Controller: reduce the inverter's charge/discharge to zero (the battery's
   `LIMITS` will drop to 0 A within a few ticks of the stop being accepted —
   under 250 ms including the frame period — but a cooperative controller
   gets there first).
2. Controller: send `COMMAND` with `STOP`.
3. Battery, next tick: `STATUS` with commanded `STOP`, `COMMAND_PENDING` set,
   sequence echoed.
4. Battery: system state → `INACTIVE`, contactor state → `AWAITING_OPEN`,
   `CURRENT_ENABLED` flag cleared, `LIMITS` = 0 A / 0 A.
5. Battery: contactors open once |current| < 1 A, or after 30 s if it is at
   least below 5 A (see `CONTACTORS_*_OPEN_MA`/`CONTACTORS_OPEN_TIMEOUT_MS` in
   `config/limits.h`). `STATUS` then shows contactor state `OPEN` and
   `COMMAND_PENDING` cleared.

Wait for step 5 before assuming the DC bus segment is dead. The battery never
opens contactors under significant load unless that timeout has passed, and
the controller cannot bypass this — there is deliberately no "force open"
command. Emergency disconnection is the job of the BMS's own protections and
of external hardware, not of the CAN link.

### 5.3 REQUEST — controller → battery (type `0x01`)

| Byte | Content |
|---|---|
| 0 | battery→controller message type to send now |
| 1 | page, for paged types (optional; `0xFF` = current page) |

Marks that frame as due immediately (it still goes through the priority
scheduler, so it arrives within a tick or two). One request yields one frame.
Use it for snapshots after connecting or for a UI "refresh" button — the
periodic schedule already covers steady-state needs, so polling with `REQUEST`
is unnecessary. It also counts as controller liveness.

## 6. Telemetry — battery → controller

All identifiers are `prefix | type<<8 | node`. Periods are the defaults;
non-essential ones are multiplied by 2^`RATE_SHIFT`. Field encodings: `u16`/
`i16`/`i32` big-endian.

| Type | Name | Period | Bytes |
|---|---|---|---|
| `0x10` | **STATUS** | 200 ms + on change | 0 system state · 1 contactor state · 2 flags · 3 bits 2..0 highest event level, bits 5..4 commanded state · 4..5 last COMMAND seq · 6 event type at highest level (`0xFF` none) · 7 res |
| `0x11` | **LIMITS** | 200 ms | 0..1 max pack V (i16, 0.1 V) · 2..3 min pack V (i16, 0.1 V) · 4..5 charge limit (u16, 0.1 A) · 6..7 discharge limit (u16, 0.1 A) |
| `0x12` | **MEASUREMENTS** | 200 ms | 0..1 pack V (u16, 0.01 V) · 2..5 current (i32, mA, + = charging) · 6..7 SoC for inverter (u16, 0.01 %) |
| `0x13` | ENERGY | 1 s | 0..1 SoH (u16, 0.01 %) · 2..3 remaining (u16, 0.1 Ah) · 4..5 full (u16, 0.1 Ah) · 6..7 charge used since full (i16, 0.1 Ah) |
| `0x14` | CELL_STATS | 1 s | 0..1 min · 2..3 max · 4..5 mean cell mV (i16) · 6 index of min · 7 index of max |
| `0x15` | TEMPERATURES | 1 s | 0..1 min · 2..3 max (i16, 0.1 °C) · 4 coldest index · 5 hottest index · 6..7 res |
| `0x16` | HV_VOLTAGES | 1 s | 0..1 output V (u16, 0.01 V) · 2..3 across +contactor · 4..5 across −contactor · 6..7 battery − output (i16, 0.1 V) |
| `0x17` | SUPPLY | 2 s | 3V3, 5V, 12V, contactor supply (u16 mV each) |
| `0x18` | BALANCING | 1 s | 0 state (0 idle/1 active) · 1 cells balancing · 2..3 longest remaining (i16, 1.28 s periods) · 4..5 pause counter · 6 flags (b0 active this cycle, b1 pause cycle, b2 auto-balancing enabled) · 7 res |
| `0x19` | EVENT | 250 ms | 0 event type (`0xFF` = nothing recorded) · 1 level · 2..3 count · 4..5 seconds since last (u16, sat.) · 6 number of event types in firmware · 7 number currently active |
| `0x1A` | ESTIMATORS | 2 s | voltage-based, basic-count, fancy-count, EKF SoC (u16, 0.01 % each, unscaled) |
| `0x1B` | CONFIG | 5 s | 0 protocol version · 1 chemistry (1 LFP, 2 NMC) · 2 cell count · 3 temperature sensor count · 4..5 nameplate · 6..7 working capacity (u16, 0.1 Ah) |
| `0x1C` | SERIAL | 5 s | 64-bit board id, byte 0 first |
| `0x1D` | SETTINGS | 5 s | 0..1 user charge limit · 2..3 user discharge limit (u16, 0.1 A, 0 = none) · 4..5 working-min · 6..7 working-max cell mV |
| `0x1E` | CELL_LIMITS | 5 s | soft min, soft max, hard min, hard max cell mV (u16) |
| `0x20` | MODULE_TEMPS | 250 ms/page | 0 first index · 1 total count · 2..7 three × i16 0.1 °C |
| `0x21` | CELL_VOLTAGES | 100 ms/page | 0 first index · 1 b2..0 "cell i balancing", b7 "disturbed by balancing" · 2..7 three × i16 mV |

`0x1F`, `0x22–0x2F` are reserved for future standard frames; `0x30–0x3F` for
experiments. Cell count for paging comes from `CONFIG`; indices beyond the
count carry `0x7FFF`.

### STATUS details

Flags (byte 2):

| Bit | Name | Meaning |
|---|---|---|
| 0 | `CURRENT_ENABLED` | Contactors closed, settled, current permitted. This — not "contactor state == CLOSED" — is the go/no-go for the inverter. |
| 1 | `BALANCING` | Balancing resistors active this BMB cycle |
| 2 | `ESTOP` | Local emergency-stop input active |
| 3 | `OPERATING` | Persisted "resume operating after power cycle" |
| 4 | `COMMAND_PENDING` | Commanded state not reached yet |
| 5 | `SLOW_MODE` | Cell data sampled infrequently (idle pack) |
| 6 | `PRECHARGE_CLOSED` | Precharge/bypass auxiliary contact |

System states (byte 0): 0 UNINITIALIZED, 1 INITIALIZING, 2 CALIBRATING,
3 INACTIVE, 4 OPERATING, 5 FAULT. Contactor states (byte 1) follow
`CONTACTORS_STATES` in `app/state_machines/contactors.h`; the useful ones are
0 OPEN, 7 PRECHARGING, 8 CLOSED, 9 AWAITING_OPEN, 16 PRECHARGE_FAILED,
17 TESTING_FAILED — treat others as "in transition".

Event levels: 0 NONE, 1 INFO, 2 WARNING, 3 CRITICAL (contactors will open soon
unless it clears), 4 FATAL (latched; contactors open; needs local reset).
An INFO level is normal (e.g. `INVERTER_DETECTED` is raised on assignment).

### Event types

The type byte in `STATUS`/`EVENT` is the index into `EVENT_TYPES` in
[`bms/sys/events/events.h`](../bms/sys/events/events.h). `EVENT` byte 6 tells
you how many types the firmware defines, so a mismatch with your table is
detectable. Current table:

| # | Event | # | Event |
|---|---|---|---|
| 0 | CONTACTOR_POS_STUCK_OPEN | 26 | BATTERY_VOLTAGE_VERY_HIGH |
| 1 | CONTACTOR_POS_STUCK_CLOSED | 27 | BATTERY_VOLTAGE_LOW |
| 2 | CONTACTOR_NEG_STUCK_OPEN | 28 | BATTERY_VOLTAGE_VERY_LOW |
| 3 | CONTACTOR_NEG_STUCK_CLOSED | 29 | CELL_VOLTAGE_HIGH |
| 4 | CONTACTOR_PRE_STUCK_OPEN | 30 | CELL_VOLTAGE_VERY_HIGH |
| 5 | CONTACTOR_PRE_STUCK_CLOSED | 31 | CELL_VOLTAGE_LOW |
| 6 | CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH | 32 | CELL_VOLTAGE_VERY_LOW |
| 7 | CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH | 33 | SOFT_CHARGE_BUFFER_EXCEEDED |
| 8 | CONTACTOR_POS_UNEXPECTED_OPEN | 34 | OVERCURRENT_CHARGING |
| 9 | CONTACTOR_NEG_UNEXPECTED_OPEN | 35 | OVERCURRENT_DISCHARGING |
| 10 | CONTACTOR_CLOSING_FAILED | 36 | BATTERY_TEMPERATURE_HIGH |
| 11 | SUPPLY_VOLTAGE_STALE | 37 | BATTERY_TEMPERATURE_VERY_HIGH |
| 12 | BATTERY_VOLTAGE_STALE | 38 | BATTERY_TEMPERATURE_LOW |
| 13 | BATTERY_TEMPERATURE_STALE | 39 | BATTERY_TEMPERATURE_VERY_LOW |
| 14 | CURRENT_STALE | 40 | VOLTAGE_MISMATCH |
| 15 | CELL_VOLTAGES_STALE | 41 | BMB_READ_ERROR |
| 16 | SUPPLY_VOLTAGE_3V3_LOW | 42 | BMB_CRC_MISMATCH |
| 17 | SUPPLY_VOLTAGE_3V3_HIGH | 43 | CELL_VOLTAGE_GLITCH |
| 18 | SUPPLY_VOLTAGE_5V_LOW | 44 | MODULE_TEMPERATURE_GLITCH |
| 19 | SUPPLY_VOLTAGE_5V_HIGH | 45 | INVERTER_DETECTED |
| 20 | SUPPLY_VOLTAGE_12V_LOW | 46 | ESTOP_PRESSED |
| 21 | SUPPLY_VOLTAGE_12V_HIGH | 47 | BOOT_NORMAL |
| 22 | SUPPLY_VOLTAGE_CONTACTOR_LOW | 48 | BOOT_WATCHDOG |
| 23 | SUPPLY_VOLTAGE_CONTACTOR_VERY_LOW | 49 | LOOP_OVERRUN |
| 24 | SUPPLY_VOLTAGE_CONTACTOR_HIGH | 50 | RESTARTING |
| 25 | BATTERY_VOLTAGE_HIGH | 51 | CAN_SLAVE_LOST (master mode) |
| | | 52 | CAN_SLAVE_FAULT (master mode) |

## 7. Scheduling and bus load

The battery runs one transmit scheduler per 20 ms tick. It walks the message
table in priority order and sends whatever is due, at most three frames per
tick, so the transmit queue can never overflow and the important frames are
never delayed by bulk data. Paged types advance one page per period. If the
queue is momentarily full the frame stays due and goes out on the next tick.

Steady-state load per battery with default rates is about **40 frames/s**
(≈ 5 kbit/s, ~1 % of a 500 kbit/s bus), about a quarter of which is cell
voltage pages. A 96-cell pack refreshes all cell voltages every 3.2 s. Sixteen
batteries at default rates use ~17 % of the bus; use `RATE_SHIFT` if that is
too much for your installation.

Controller-side load is negligible: one `COMMAND` per battery every 500 ms,
or a single broadcast to node `0xFF`.

## 8. Implementing a controller

A complete Python reference implementation lives in
[`tools/custom_can/`](../tools/custom_can/): `controller.py` is the controller
described below, `battery_sim.py` is a simulated battery that behaves like the
firmware, and `protocol.py` is the wire format. It needs only the standard
library for the in-process demo and tests; `pip install python-can` connects
it to a real adapter. From the repository root:

```bash
python -m tools.custom_can demo                                  # scripted scenario, no hardware
python -m tools.custom_can controller --bus slcan:COM3 --desired run   # drive real batteries
python -m tools.custom_can simulate --bus socketcan:can0 --batteries 2 # fake batteries for a real controller
python -m tools.custom_can sniff --bus socketcan:can0            # decode bus traffic
python -m unittest tools.custom_can.test_custom_can              # protocol/controller tests
```

Minimal, correct controller loop:

```text
on ANNOUNCE (id & 0x1FFFFF00 == 0x1CE10000):
    serial = u32(data[2..5]); state = data[0] & 0x0F
    if state == 0 or (serial known and node/prefix differ from ours):
        node = table.lookup_or_allocate(serial)      # persist serial -> node
        send ASSIGN(serial, PREFIX, node, flags)

every 500 ms:
    send COMMAND(desired_state, seq++) to node 0xFF   # or per node
    for each battery: if now - last_STATUS > 2 s: mark lost

on frame with id & 0x1FFFC000 == PREFIX << 14:
    type = (id >> 8) & 0x3F; node = id & 0xFF
    battery = table[node]           # ignore if unknown
    battery.last_STATUS = now if type == 0x10
    decode per section 6; ignore unknown types

aggregate for the inverter:
    voltage limits: min of max-limits, max of min-limits (parallel pack)
    current limits: sum over batteries with CURRENT_ENABLED and level < CRITICAL
    SoC/capacity: capacity-weighted over the same set
```

Common mistakes this protocol is designed to make obvious:

* **Sending `ASSIGN` periodically instead of on `ANNOUNCE`.** Works, but is
  pointless; use `COMMAND` as the heartbeat.
* **Sending a single `RUN` and stopping.** The battery drops the link after
  5 s of silence. Keep sending `COMMAND` — the desired state is a level.
* **Using contactor state `CLOSED` as "ready".** Use the `CURRENT_ENABLED`
  flag; the BMS enables current only after the contactors have settled.
* **Assuming `STOP` means "open now".** Watch for contactor state `OPEN`; the
  battery waits for current to fall (up to 30 s).
* **Assigning the same node twice.** Both batteries will drop off and
  re-announce (section 4.4).
* **Reading the event table with a stale mapping.** Check `EVENT` byte 6
  against the size of your table.

## 9. Master mode: a CellKeeper as the controller

A CellKeeper built with `-DCAN_MASTER=ON` controls its own battery *and* a
fleet of slave CellKeepers, and presents the whole parallel HV bus to the
inverter as one battery. Implementation:
[`bms/protocols/inverter/can_master.c`](../bms/protocols/inverter/can_master.c),
tests: [`tests/test_can_master.c`](../tests/test_can_master.c).

```
                 inverter
                    │ CAN1 (BYD protocol, unchanged)
   ┌────────────────┴──┐
   │  master CellKeeper│──── HV ────┬──────────────┬──── ... parallel HV bus
   └────────────────┬──┘            │              │
                    │ CAN2      ┌───┴────┐     ┌───┴────┐
                    └───────────┤ slave 1├──┬──┤ slave 2├── ... CAN2 fleet bus
                                └────────┘  │  └────────┘
```

| | Build | Bus use |
|---|---|---|
| Master | `cmake -DINVERTER_PROTOCOL=byd_can -DCAN_MASTER=ON ..` | CAN1 → inverter, CAN2 → slaves |
| Slave | `cmake -DINVERTER_PROTOCOL=custom_can ..` | CAN2 → master (CAN1 unused) |

The master is the controller of sections 3–5 with these policies:

* **Discovery**: slaves are assigned nodes `1..CAN_MASTER_MAX_SLAVES` (8) on
  `CAN_MASTER_PREFIX` (`0x0300`). After a master reboot, slaves that still
  hold an assignment on that prefix are adopted as they are, so they never
  drop their link. A slave offline for `CAN_MASTER_RETIRE_MS` (60 s) gives up
  its node number.
* **Run intent**: the heartbeat carries `RUN` while the master's persisted
  `operating` flag is set and `STOP` otherwise. So the operator's RUN/STOP on
  the master (HMI, CLI `toggle`) applies to the fleet, a power cycle restores
  it, and a FATAL fault on the master (which clears `operating`) stops the
  fleet. By default slaves are assigned without `STOP_ON_TIMEOUT`, i.e. they
  keep running if the master itself dies — the inverter loses its BMS at the
  same moment and stops on its own. Set `CAN_MASTER_ASSIGN_FLAGS` to change
  this.
* **Aggregation** into `model.fleet_outputs`, which `byd_can` sends instead of
  the local values. A member *contributes* when it is online, has
  `CURRENT_ENABLED` and no CRITICAL/FATAL event (the master's own battery is
  judged by the same rule). Over contributing members: current limits and
  capacities are **summed**, voltage limits are the **tightest** (min of the
  maxima, max of the minima), SoC is **capacity-weighted**. Over all online
  members: current is summed, temperatures are the extremes. Pack voltage is
  the master's own measurement (the fleet mean if unavailable). As for a
  single battery, SoC is forced to 0 %/100 % when the fleet cannot
  discharge/charge so SoC-driven inverters also stop.
* **Events on the master**: `CAN_SLAVE_LOST` (WARNING, data = serial) when an
  online slave stops sending `STATUS` for 2 s, cleared when every known slave
  is back; `CAN_SLAVE_FAULT` (WARNING) while any online slave reports FATAL.
  Neither escalates; the lost/faulted slave is simply subtracted from what the
  inverter is allowed to do.
* **Bus budget**: at most two frames per 20 ms tick on CAN2, all frame types
  other than `STATUS`/`LIMITS`/`MEASUREMENTS`/`ENERGY`/`TEMPERATURES` are
  ignored at interrupt level.

The CLI command `fleet` (and the periodic debug dump) prints the slave table:
serial, node, liveness, states, measurements, limits and command round-trip
time.

A stop from the master therefore looks like this on the HV bus: the operator
stops the master → `operating` clears → slaves receive `STOP` within 500 ms →
all members' limits go to zero → the inverter, told 0 A / 0 %, stops drawing
→ each battery's contactors open once its own current has fallen (section
5.2). At no point does any battery open under load unless its own 30 s
timeout has passed.

## 10. Versioning and extension rules

* The protocol version is carried in `ANNOUNCE` and `CONFIG`. It changes only
  when an existing field changes meaning or layout.
* Adding a message type, using a reserved byte or reserved flag bit, or
  appending bytes to a frame does **not** bump the version; receivers already
  ignore what they do not know.
* Type numbers, once published, are never reused for something else.
* Battery-side implementations must never depend on anything in the discovery
  block other than `ANNOUNCE`/`ASSIGN`; controller-side implementations must
  never hard-code a prefix or node other than the discovery block.
