# Contactor self-test fails while the Deye keeps its battery port energized

**Status:** fixed in `bug-fix` (2026-10-12). See "The fix" below.

- **What changed:** with both contactors commanded open, an output of at least 80 % of the battery
  voltage now records the info event `CONTACTOR_PORT_ENERGIZED` instead of a FATAL weld.
- **Hot-join:** for that start the BMS skips the weld test and precharges onto the energized port.
  Precharge must still succeed (≤ 16 V difference, ≤ 225 mA) before the main contactors close.
- **A dead output** still runs the full weld test.

A battery that already latched FAULT still needs a controller reboot. Run alone doesn't leave the
FAULT state.

## Symptom

When a battery is (re)started while the Deye inverter is running, the BMS contactor self-test
fails with **FATAL `CONTACTOR_POS_STUCK_CLOSED`** (plus WARNING `CONTACTOR_CLOSING_FAILED`). The
system goes to **FAULT**: the contactors stay open, and the battery needs a manual **Run** to try
again.

The contactors are not welded. The test passes as soon as the inverter's battery port is no longer
energized, for example right after an inverter restart, or while the inverter is faulted.

## Observed at Umauma (2026-10-12)

| Time (HST) | Battery | Sequence |
|---|---|---|
| 13:04:08–13:04:34 | ck1 (Rev0, BAT1) | Stopped → Run → `TST_NEG_OP` → **`TST_FAIL`** → FAULT. Events: `CONTACTOR_POS_STUCK_CLOSED` (FATAL), `CONTACTOR_CLOSING_FAILED`. The inverter showed F56 (see the Deye note below). |
| 14:42:06–14:42:21 | ck0 (Rev1, BAT2) | Controller reflashed → INIT → `TST_PRE_CL` → `TST_NEG_OP` → `TST_NEG_CL` → `TST_POS_OP` → **`TST_FAIL`** → FAULT. Same two events. The inverter showed F58 for about 20 s and the AC output dropped, then it ran on ck1. |
| 12:01, 12:54 | ck1, ck0 | Start-ups that **passed**. At those times the inverter was faulted or had just restarted, so the battery port was dead. |

**Readings during the 14:42 ck0 test:**
- With ck0's contactors open, its **output (the Deye BAT2 port) stayed at 379–385 V**. The pack
  was about 386–388 V.
- About 3 V across the negative contactor.
- Pack Output Voltage only fell to 0 V at 14:42:43, after the inverter had restarted.

## Why the test fails

The self-test (`bms/app/state_machines/contactors.c`) runs before precharge:

```
TESTING_PRE_CLOSED  → precharge relay aux contact must read closed
TESTING_NEG_OPEN    → all open:      confirm_contactor_pos_and_neg_seems_open()
TESTING_NEG_CLOSED  → neg closed:    confirm_contactor_neg_seems_closed()
TESTING_POS_OPEN    → all open:      confirm_contactor_pos_and_neg_seems_open()
TESTING_POS_CLOSED  → pos closed:    confirm_contactor_pos_seems_closed()
→ PRECHARGING
```

"Seems open" means **at least** `CONTACTORS_OPEN_VOLTAGE_THRESHOLD_MV` /
`CONTACTORS_POS_OPEN_VOLTAGE_THRESHOLD_MV` across the contactor (`config/limits.h`):

| Board | Threshold |
|---|---|
| Rev1 | **3 V** |
| Rev0 (positive contactor) | **10 V** |

The check assumes the output side is dead or floating when the contactors are open. On a Deye HV
inverter it isn't:
- **The Deye keeps the port energized.** While a battery input is enabled, it holds that port at
  about pack voltage from its DC bus.
- **That makes an open contactor look closed.** With Out+ ≈ Batt+ and Out− ≈ Batt−, the voltage
  across each open contactor is only a few volts, so it reads as stuck closed.
- **Only when the port goes dead does the test pass.** That happens when the inverter disables
  that input, for example after its low-battery shutdown and restart (see
  `Deye-Firmware/50K/notes/BatteryLowShutdown.md`), or when it is faulted.

The result depends on timing, which is why the same battery sometimes passes and sometimes fails.

## Workaround (current firmware)

- **After a FAULT from this,** press **Run** again once the inverter has restarted and the
  battery's *Pack Output Voltage* reads ~0 V. The test then passes.
- **Avoid opening a battery's contactors while the inverter is running.** Opening them also
  triggers the Deye's low-battery restart (see `deye-offgrid-charging.md`).

## The fix (as implemented)

**First attempt (9c3676b):** wait and retry the test. This doesn't work with a Deye: as long as
the BMS is on CAN, the inverter holds the port at about pack voltage − 3 V. That is its "CCL = 0"
target, see `BMS_Limits_and_SOC.md`. The port never goes dead, so the retries would run out and
latch the FATAL weld anyway.

**Current fix: hot-join** (`hot_join_energized_port()` in `contactors.c`)
- **Detecting an energized port:** in `TESTING_NEG_OPEN` / `TESTING_POS_OPEN`, both contactors are
  commanded open. If the output is ≥ `CONTACTORS_PORT_ENERGIZED_FRACTION` (0.8) × battery voltage,
  the port is energized from outside. A single welded contactor can't produce that.
- **What it does then:** count `CONTACTOR_PORT_ENERGIZED` (info, data = output mV), skip the
  remaining weld tests and go to `PRECHARGING_INIT`.
- **Precharge still guards the close:** the main contactors only close once battery − output
  ≤ 16 V and the precharge current is ≤ 225 mA.
- **Trade-off:** a double weld (both contactors stuck closed) would also look like an energized
  port and go unnoticed on that start. The next start with a dead port runs the full test again.
- **Tests (`tests/test_contactors.c`):**
  - `test_energized_port_skips_weld_test`: energized port goes to `PRECHARGING_INIT`, no weld
    event;
  - `test_dead_port_weld_still_detected`: a dead output still raises the FATAL weld.

## Field test, 2026-10-12 15:51 (ck0, bug-fix 3a31ee6)

ck0 was stopped from its screen, then **Run** was pressed 4 s later while the inverter was
running.

1. **15:51:07 Stop:** contactors open. The Deye kept BAT2 energized and the BMS read 381 V on the
   output.
2. **15:51:11 Run:** `TST_PRE_CL` → `TST_NEG_OP`, then
   `Output 381.3 V with contactors open: port energized externally, skipping weld test`, then
   `PRECHARGING`. The difference was 6–8 V, which includes ck0's ~5 V output-channel offset.
3. **15:51:14 `CLOSED`:** the contactors stayed closed for the rest of the capture (60 s+). The
   positive-contactor channel read −3.3 V while closed, and the "still closed" check tolerated it.
4. **The inverter didn't notice:** no fault, no restart, and the load stayed at 167–168 W. The
   port never dropped, so the Deye's low-battery check (`BatteryLowShutdown.md`) never fired.

So with this firmware, a quick Stop → Run on one battery is a hot restart that doesn't disturb
the inverter.

## Open questions

- How long the Deye keeps an enabled port energized after the battery disconnects. ck0's port
  stayed up for at least ~35 s until the inverter restart.
- Whether the Deye energizes the port before a battery is first connected, or only once it has
  been enabled.
