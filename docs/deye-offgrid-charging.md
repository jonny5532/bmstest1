# CellKeeper BMS with a Deye / Sunsynk HV three-phase inverter off-grid

This note explains how the BMS's CAN values control charging on a Deye SUN-xxK-SG0xHP3 (50 kW
class, high-voltage battery) running off-grid. It covers which settings decide where charging
stops, and how to set them so the battery stays protected and the inverter doesn't trip F26 (DC
bus unbalance). It is based on the decompiled inverter firmware and on two live batteries, ck0
and ck1, at the Umauma site.

**Inverter firmware:** MCU1 (display/comms) 2001/C032 and MCU2 (DSP) V3103/1077, read from
RS485 registers 14/15/17/18. Behaviour may differ on other versions.

Deeper notes are in the `Deye-Firmware` repo (`50K/notes/`, local only): `BMS_Limits_and_SOC.md`,
`ControlArchitecture.md`, `F26_BusUnbalance.md`, `LoadDrop_Simulation.md` and
`RS485_RegisterMap.md`.

## The short version

- **The Deye regulates the battery voltage, not the current.** Off-grid it holds the battery at
  the **charge voltage limit (CVL) + 1.0 V**, as measured by the inverter, by curtailing PV.
- **The charge current limit (CCL) is not what ends a charge.** It sets how much current the
  battery converter may absorb in fast transients, such as when a load suddenly drops and PV
  surplus has to go somewhere.
- **So keep the CCL comfortably high near full**, at least the PV surplus a load drop can create.
  Use the CVL, through the BMS working max and the voltage offset, to decide where charging stops.
  A CCL of 0 (or very low) near full leaves the inverter nowhere to put a sudden surplus. The DC
  bus then rises, which makes an F26 trip more likely.
- **The SoC sent to the inverter doesn't affect charging.** It is for display and for the
  inverter's discharge settings.

## How the Deye uses each BMS value

| BMS → inverter value | What the Deye does with it (off-grid, BMS mode) |
|---|---|
| **Charge voltage limit (CVL)** | Battery-voltage PI loop (DSP `0xD500`) that holds the battery at **CVL + 1.0 V** (inverter-measured). Its output (`0xD512`) caps the PV current of each MPPT, so charging tapers off by itself. Active when off-grid (`F803 = 1`). |
| **Charge current limit (CCL)** | Fast transient sink limit for the battery DC-DC: `C750 = min(BMS CCL, inverter battery charge setting) × 100`. It limits how hard the battery converter may pull when the DC bus rises. It does not end the charge. |
| **CCL = 0** | The battery target is moved to *pack voltage − 3 V* and the battery sinks nothing. A load drop then has nowhere to go, the bus rises and the PV loop has to catch it alone. This is the F26 risk case. |
| **CVL + 20 V** | Hard backstop (`0xCF88`): forces charge current to 0 if the battery reaches CVL + 20 V. |
| **Discharge limits (DVL/DCL)** | Normal low-voltage cut-off and discharge current limit. |
| **SoC** | Stored (`D814 → C7FE`) and shown; used for the SoC-based discharge settings. **Not used in the charge path**, so a reported 100 % doesn't stop charging. |
| **F26 (BusUnbalance)** | Trips on \|V<sub>top half</sub> − V<sub>bottom half</sub>\| > ~243 V, with no debounce and a ~2–3 kHz filter, armed only while running. A single fast transient can trip it. |

The AC neutral sits on the DC-bus midpoint and the battery ports are **not isolated**. That is
why the BMS isolation readings only mean anything with the contactors open (see
`app/isolation.h`).

## How the BMS computes what it sends

**CVL (`model_calculate_inverter_voltage_limits`, `app/model.c`):**

```
CVL = cells × Cell Voltage Working Max
      − (highest cell − mean cell) × cells        (keeps the highest cell from overshooting)
      + Voltage Limit Offset Upper                (0.1 V units)
```

The lower limit works the same way, with Working Min, the lowest cell and Voltage Limit Offset
Lower.

**CCL:** the smallest of several limits, all visible in HA as `current_limit_*_charge`:

| Limit | Comes from |
|---|---|
| user | **Max Charge Current** setting |
| hard | profile maximum |
| cell voltage | tapers as the highest cell nears the soft max |
| working | open-circuit-voltage estimate against Working Max (`calculate_working_charge_current_limit`) |
| temperature | derates outside the charge temperature window |
| delta | cell-spread protection |

The result is published as `current_limit_effective_charge`.

**SoC sent to the inverter:**
- Scaled SoC = (SoC − **SoC Scaling Min**) / (**SoC Scaling Max** − SoC Scaling Min), clamped
  to 0–100 %.
- It is forced to 100 % if the CCL is 0 (and DCL > 0), and to 0 % if the DCL is 0 (and CCL > 0).

## Where charging actually stops

Off-grid the pack settles where the **inverter** measures CVL + 1.0 V:

```
final pack voltage (BMS)  ≈  CVL + 1.0 V − (inverter reading − BMS reading)
final cell voltage        ≈  final pack voltage / cells
```

**Inverter offset:** the inverter's battery voltage reading is not exactly the BMS's. Measure the
offset near zero current by comparing `sensor.sunsynk_battery_N_voltage` with the BMS
`pack_internal_voltage`.

**Choosing the setting for a target cell voltage V<sub>cell</sub>:**

```
Voltage Limit Offset Upper (dV) ≈ 10 × [ (V_cell − Working Max) × cells − 1.0 + (inverter − BMS) + cell spread ]
```

Here *cell spread* = (highest cell − mean) × cells, which the BMS subtracts from the CVL by
itself.

**Trade-off:**
- Putting the CV point right at Working Max lets the BMS's working-limit taper start cutting the
  CCL as the cells approach it. That shrinks the inverter's transient sink again.
- Setting the CV point a little below Working Max keeps the CCL high all the way to full. ck0 does
  this, as the example below shows. Set **SoC Scaling
Max** to the BMS SoC reached at that point, if you want the inverter to show 100 % there.

The BMS's own protection (cell-voltage taper, working limit, soft and hard limits) stays as the
backstop if the inverter overshoots. With the offset set this way, the inverter stops the charge
before those engage.

## Live example, 2026-10-12 ~13:00 HST (both batteries full, PV ~1.9 kW, load ~170 W)

The Deye has two independent battery ports: BAT1 is ck1, BAT2 is ck0.

| | **ck0** (NMC "Blue Tesla", 96 s) | **ck1** (LFP, 108 s) |
|---|---|---|
| Cell Voltage Working Max | 4100 mV (393.6 V) | 3450 mV (372.6 V) |
| Voltage Limit Offset Upper | −47 (−4.7 V) | −35 (−3.5 V) |
| CVL sent | **388.7 V** (393.6 − 0.2 spread − 4.7) | **369.0 V** (372.6 − 0.1 − 3.5) |
| Inverter holds (CVL + 1 V) | 389.7 V | 370.0 V |
| Inverter's reading of the battery | 389.25–389.6 V | 370.0 V |
| BMS pack voltage (cell sum) | 388.7 V | 370.3 V |
| Inverter − BMS reading | **+0.9 V** | **−0.3 V** |
| Cells | 4.044–4.051 V | 3.423–3.426 V |
| Charge current | 2.4–2.9 A, tapering (CV) | ~0 A (full) |
| CCL sent / Max Charge Current | 25 A / 25 A | 19.4 A (taper) / 50 A |
| SoC: BMS → sent | 88.97 % → **99.96 %** (Scaling Max 89) | 99.59 % → 99.59 % |

**Reading ck0:**
- The inverter is already at its CV point, so current is limited to ~3 A by voltage, not by the
  25 A CCL. That is the intended behaviour.
- The pack will settle around 389 V, i.e. **~4.05 V/cell**, which is below the 4.10 V working max.
  The BMS working-max taper never engages, because the inverter stops the pack first.
- SoC Scaling Max 89 makes this point read as 100 %.
- In the formula: (4.051 − 4.100) × 96 = −4.7 V, − 1.0, + 0.9, + 0.2 ≈ −4.6 V. That matches the −47
  setting.
- To charge ck0 all the way to 4.10 V/cell instead, the offset would be about
  0 − 1.0 + 0.9 + 0.2 ≈ **+0.1 V (+1)**. SoC Scaling Max would also need raising.
- The cost is that the working-limit taper then reduces the CCL near the top. Something like −20
  (≈4.08 V/cell) is a middle ground.

**Reading ck1:** the inverter holds 370.0 V and the pack sits at 370.3 V (3.43 V/cell) with
~0 A, i.e. full at its intended point.

## Gotchas seen on these batteries

- **Pack output voltage calibration.** ck0's "Pack Output Voltage" read 383.9 V while the cell
  sum and the internal channel read 388.7 V, with the contactors closed and ~0 V across them. The
  output channel is ~4.9 V low.
  - Charging is unaffected, because the CVL is derived from the cells.
  - It confuses comparisons, so recalibrate it with the BMS Calibrate function.
- **F58 after a BMS restart.** Older controller firmware didn't handle the inverter's
  re-identify requests (every 5 s), so the Deye stayed in F58 after a CAN interruption. The
  `bug-fix` branch firmware handles it. F58 cleared within seconds of updating ck1 (2026-10-12).
- **Raising the CCL is not a charge risk** with this inverter: the end of charge is set by the
  voltage loop. Keep the BMS soft and hard limits sensible as the backstop.
- **Opening one battery's contactors restarts the inverter** (or latches F56 without PV). The
  DSP treats a battery port below 152 V, or below its shutdown voltage/SoC, as a system-wide
  shutdown. To take one battery out without that, keep its contactors closed and send charge and
  discharge limits of 0. Details: `Deye-Firmware/50K/notes/BatteryLowShutdown.md`.
- **Contactor self-test failures while the inverter runs.** The Deye keeps an enabled battery
  port at about pack voltage, so the BMS's "contactor open" check reads a welded contactor
  (FATAL `CONTACTOR_POS_STUCK_CLOSED`). See [contactor-test-energized-port.md](contactor-test-energized-port.md).
- **Don't stop a full battery by dropping the CCL to 0.** It forces the reported SoC to 100 % and
  removes the inverter's transient sink (see the F26 notes). Let the CVL do it.
