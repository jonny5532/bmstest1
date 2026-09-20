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




## Inverter integration

The BMS talks CAN to the outside world via one of two protocol
implementations, selected at build time with `-DINVERTER_PROTOCOL=<name>`:

- `byd_can` (default) emulates a BYD Battery-Box directly to an inverter.
- `custom_can` is a multi-battery protocol spoken on the inter-BMS port
  (CAN2, GPIO 4/5) to an upstream controller: batteries auto-announce on a
  fixed discovery ID, the controller assigns addresses, and the battery
  streams its complete internal state (limits, measurements, cell voltages,
  balancing, events, ...) while accepting RUN/STOP requests. See
  [docs/custom_can_protocol.md](docs/custom_can_protocol.md) for the
  specification and controller implementation guide, and
  [tools/custom_can/](tools/custom_can/) for a Python reference controller,
  battery simulator and demo (`python -m tools.custom_can demo`).

Several batteries on a parallel HV bus can be run by one CellKeeper in
**master mode** (`-DCAN_MASTER=ON` together with `byd_can`): it controls its
own battery and the `custom_can` slaves over CAN2, and presents the aggregate
(summed current limits and capacity, tightest voltage limits, weighted SoC)
to the inverter on CAN1. Enabling CAN2 in either role compiles out the
`isosnoop` ISOSPI debugging aid, which shares its PIO block.


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


