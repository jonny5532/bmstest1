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




### Cell voltage

There are two sets of cell voltage limits - hard limits and soft limits.

If the hard limits (the most extreme) are exceeded, a FATAL event is raised and
the contactors quickly open to prevent overcharge/overdischarge. This state will
require manual restoration of the battery before the BMS can be successfully
reset without immediately tripping again.

The soft limits allow for a more graceful recovery. In the soft-limit region,
the battery can still be charged (in the case of the lower limit) or discharged
(in the case of the upper limit) at a reduced current. If however the overcharge
or overdischarge continues beyond a certain threshold (measured in amp-hours),
or the voltage stays in the soft-limit region for more than an hour, a FATAL
event will be raised and the contactors will open.

This ensures that an unattended battery that overcharges or overdischarges will
be automatically disconnected whilst it is still in the soft-limit region, and
can then be safely discharged/charged back into normal service after manual
intervention.


### Response time

The main loop of the BMS runs every 20 milliseconds. It will therefore take at least 20ms for the BMS to react to a change, up to several multiples of 20ms if the signal propagates through the model in stages.

Most operations performed in the main loop should easily complete within the 20ms deadline. However the BMS performs occasional flash memory writes which can take much longer (the worst case block-erase time of the SPI flash chip is 2 seconds). The normal WARNING that occurs if a loop tick overruns is suppressed when flash memory operations are performed.

A system-wide watchdog timer is set up with a 5 second deadline, which gets reset every tick. If the main loop stalls for more than 5 seconds, the chip will reboot and raise a WARNING event.

