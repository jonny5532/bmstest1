"""Reference controller: the upstream ("Battery Emulator") side of the custom
multi-battery CAN protocol.

The controller

* discovers batteries from their ANNOUNCE frames and assigns them a node on
  its prefix (remembering serial -> node so a battery keeps its number across
  restarts of either side),
* sends the COMMAND heartbeat carrying the desired run state,
* decodes all telemetry into per-battery records with liveness tracking,
* aggregates the fleet into the numbers an inverter needs.

It is deliberately clock- and transport-agnostic: call ``tick(now)`` with a
monotonic time in seconds (real or simulated) and give it any object with
``send(can_id, data)`` / ``recv() -> Frame | None``.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set

from . import protocol as P

log = logging.getLogger("custom_can.controller")

# A battery that has been offline this long loses its node so the number can
# be reused; its serial -> node preference is kept.
NODE_RETIRE_S = 60.0


class NodeMap:
    """Persistent serial -> preferred node mapping (JSON file, optional)."""

    def __init__(self, path: Optional[Path] = None):
        self.path = path
        self.map: Dict[int, int] = {}
        if path and path.exists():
            try:
                self.map = {int(k, 16): int(v) for k, v in json.loads(path.read_text()).items()}
            except (OSError, ValueError) as e:
                log.warning("could not read node map %s: %s", path, e)

    def preferred(self, serial: int) -> Optional[int]:
        return self.map.get(serial)

    def remember(self, serial: int, node: int) -> None:
        if self.map.get(serial) == node:
            return
        self.map[serial] = node
        if self.path:
            try:
                self.path.write_text(json.dumps({f"{k:08X}": v for k, v in self.map.items()}, indent=2))
            except OSError as e:
                log.warning("could not write node map %s: %s", self.path, e)


@dataclass
class BatteryRecord:
    serial: int
    node: Optional[int] = None
    first_seen: float = 0.0
    last_announce: float = 0.0
    announce: Optional[P.Announce] = None
    assigned_at: Optional[float] = None
    assign_count: int = 0

    last_status_at: Optional[float] = None
    online: bool = False
    online_since: Optional[float] = None
    offline_since: Optional[float] = None

    # Latest decoded frames
    status: Optional[P.Status] = None
    limits: Optional[P.Limits] = None
    measurements: Optional[P.Measurements] = None
    energy: Optional[P.Energy] = None
    cell_stats: Optional[P.CellStats] = None
    temperatures: Optional[P.Temperatures] = None
    hv: Optional[P.HvVoltages] = None
    supply: Optional[P.Supply] = None
    balancing: Optional[P.Balancing] = None
    estimators: Optional[P.Estimators] = None
    config: Optional[P.Config] = None
    board_id: Optional[bytes] = None
    settings: Optional[P.Settings] = None
    cell_limits: Optional[P.CellLimits] = None
    module_temps: Dict[int, int] = field(default_factory=dict)
    cell_voltages: Dict[int, Optional[int]] = field(default_factory=dict)
    cells_balancing: Set[int] = field(default_factory=set)
    events: Dict[int, P.Event] = field(default_factory=dict)
    events_defined: int = 0
    events_active: int = 0
    frame_counts: Dict[int, int] = field(default_factory=dict)

    # Command tracking
    last_seq_sent: int = 0
    last_seq_sent_at: float = 0.0
    ack_seq: Optional[int] = None
    rtt: Optional[float] = None

    # ---- convenience -----------------------------------------------------

    @property
    def label(self) -> str:
        node = f"n{self.node}" if self.node is not None else "n?"
        return f"{self.serial:08X}/{node}"

    @property
    def enabled(self) -> bool:
        return bool(self.online and self.status and self.status.has(P.StatusFlag.CURRENT_ENABLED))

    @property
    def level(self) -> int:
        return self.status.level if self.status else P.EventLevel.NONE

    @property
    def contributing(self) -> bool:
        """Should this battery count towards what the inverter is allowed to do?"""
        return self.enabled and self.level < P.EventLevel.CRITICAL

    def active_events(self) -> List[P.Event]:
        return [e for e in self.events.values() if e.level > P.EventLevel.NONE]

    # ---- frame intake ----------------------------------------------------

    def on_frame(self, msg_type: int, data: bytes, now: float):
        frame = P.decode_battery_frame(msg_type, data)
        if frame is None:
            return None
        self.frame_counts[msg_type] = self.frame_counts.get(msg_type, 0) + 1

        if isinstance(frame, P.Status):
            self.status = frame
            self.last_status_at = now
            if frame.last_seq == self.last_seq_sent and self.ack_seq != frame.last_seq:
                self.ack_seq = frame.last_seq
                self.rtt = now - self.last_seq_sent_at
        elif isinstance(frame, P.Limits):
            self.limits = frame
        elif isinstance(frame, P.Measurements):
            self.measurements = frame
        elif isinstance(frame, P.Energy):
            self.energy = frame
        elif isinstance(frame, P.CellStats):
            self.cell_stats = frame
        elif isinstance(frame, P.Temperatures):
            self.temperatures = frame
        elif isinstance(frame, P.HvVoltages):
            self.hv = frame
        elif isinstance(frame, P.Supply):
            self.supply = frame
        elif isinstance(frame, P.Balancing):
            self.balancing = frame
        elif isinstance(frame, P.Event):
            self.events_defined = frame.types_defined
            self.events_active = frame.active
            if not frame.empty:
                self.events[frame.event_type] = frame
        elif isinstance(frame, P.Estimators):
            self.estimators = frame
        elif isinstance(frame, P.Config):
            self.config = frame
        elif isinstance(frame, P.Serial):
            self.board_id = frame.board_id
        elif isinstance(frame, P.Settings):
            self.settings = frame
        elif isinstance(frame, P.CellLimits):
            self.cell_limits = frame
        elif isinstance(frame, P.ModuleTempsPage):
            for idx, v in frame.items():
                self.module_temps[idx] = v
        elif isinstance(frame, P.CellVoltagesPage):
            n = self.config.num_cells if self.config else None
            for idx, mv, bal in frame.items():
                if n is not None and idx >= n:
                    continue
                if mv is None and n is None:
                    continue  # beyond the pack, presumably
                self.cell_voltages[idx] = mv
                if bal:
                    self.cells_balancing.add(idx)
                else:
                    self.cells_balancing.discard(idx)
        return frame

    def update_online(self, now: float) -> Optional[bool]:
        """Returns the new state if it changed, else None."""
        was = self.online
        self.online = self.last_status_at is not None and (now - self.last_status_at) <= P.BATTERY_TIMEOUT
        if self.online == was:
            return None
        if self.online:
            self.online_since, self.offline_since = now, None
        else:
            self.offline_since, self.online_since = now, None
        return self.online

    def describe(self) -> str:
        st = self.status
        parts = [f"{self.label:>16}", "ONLINE " if self.online else "offline"]
        if st:
            parts.append(f"{P._name(P.SystemState, st.system_state):<11} {P._name(P.ContactorState, st.contactor_state):<14}")
            parts.append("I:on " if st.has(P.StatusFlag.CURRENT_ENABLED) else "I:off")
            parts.append(f"lvl={P._name(P.EventLevel, st.level):<8}")
            parts.append(f"cmd={P._name(P.Desired, st.commanded):<4}{'*' if st.has(P.StatusFlag.COMMAND_PENDING) else ' '}")
        if self.measurements:
            parts.append(str(self.measurements))
        if self.limits:
            parts.append(str(self.limits))
        if self.cell_stats:
            parts.append(f"cells {self.cell_stats.min_mV}..{self.cell_stats.max_mV}mV")
        if self.temperatures:
            parts.append(f"T {self.temperatures.min_dC / 10:.1f}..{self.temperatures.max_dC / 10:.1f}C")
        if self.cells_balancing:
            parts.append(f"bal[{len(self.cells_balancing)}]")
        act = self.active_events()
        if act:
            parts.append("events: " + ", ".join(f"{P.event_name(e.event_type)}({P._name(P.EventLevel, e.level)})" for e in act))
        return " ".join(parts)


@dataclass
class Aggregate:
    """What the fleet looks like to an inverter."""
    n_batteries: int = 0
    n_online: int = 0
    n_contributing: int = 0
    voltage_V: Optional[float] = None
    current_A: float = 0.0
    soc_pct: Optional[float] = None
    charge_A: float = 0.0
    discharge_A: float = 0.0
    max_voltage_V: Optional[float] = None
    min_voltage_V: Optional[float] = None
    remaining_Ah: float = 0.0
    full_Ah: float = 0.0
    worst_level: int = P.EventLevel.NONE

    def __str__(self) -> str:
        v = f"{self.voltage_V:.1f}V" if self.voltage_V is not None else "?V"
        soc = f"{self.soc_pct:.1f}%" if self.soc_pct is not None else "?%"
        vl = (f"{self.min_voltage_V:.1f}..{self.max_voltage_V:.1f}V"
              if self.min_voltage_V is not None and self.max_voltage_V is not None else "?")
        return (f"fleet: {self.n_contributing}/{self.n_online}/{self.n_batteries} contributing/online/known | "
                f"{v} {self.current_A:+.1f}A SoC {soc} | limits chg {self.charge_A:.1f}A dis {self.discharge_A:.1f}A "
                f"V {vl} | {self.remaining_Ah:.0f}/{self.full_Ah:.0f}Ah | worst {P._name(P.EventLevel, self.worst_level)}")


class Controller:
    def __init__(self, bus, *, prefix: int = 0x0300, assign_flags: int = 0,
                 desired: P.Desired = P.Desired.NONE, heartbeat_s: float = P.COMMAND_RECOMMENDED_PERIOD,
                 node_map: Optional[NodeMap] = None, name: str = "controller"):
        if not (0 <= prefix <= P.PREFIX_MAX) or prefix == P.DISCOVERY_PREFIX:
            raise ValueError("invalid prefix")
        self.bus = bus
        self.prefix = prefix
        self.assign_flags = assign_flags
        self.desired = desired
        self.heartbeat_s = heartbeat_s
        self.node_map = node_map or NodeMap()
        self.name = name
        self.now = 0.0

        self.batteries: Dict[int, BatteryRecord] = {}
        self.nodes: Dict[int, int] = {}  # node -> serial
        self._seq = 0
        self._next_heartbeat = 0.0
        self._unknown_nodes: Dict[int, float] = {}
        self.frames_rx = 0

    # ---- public API ------------------------------------------------------

    def tick(self, now: float) -> None:
        self.now = now
        while True:
            f = self.bus.recv()
            if f is None:
                break
            self.frames_rx += 1
            self._on_frame(f.can_id, f.data, now)

        for b in self.batteries.values():
            changed = b.update_online(now)
            if changed is True:
                log.info("%s online", b.label)
            elif changed is False:
                log.warning("%s OFFLINE (no STATUS for %.1fs)", b.label, P.BATTERY_TIMEOUT)
            if (not b.online and b.node is not None and b.offline_since is not None
                    and now - b.offline_since > NODE_RETIRE_S):
                log.info("%s retired node %d after %.0fs offline", b.label, b.node, NODE_RETIRE_S)
                self.nodes.pop(b.node, None)
                b.node = None

        if now >= self._next_heartbeat:
            self._send_heartbeats(now)
            self._next_heartbeat = now + self.heartbeat_s

    def set_desired(self, desired: P.Desired) -> None:
        """Change the desired run state for the whole fleet (a level; it is
        repeated in every heartbeat from now on)."""
        if desired != self.desired:
            log.info("desired state -> %s", desired.name)
        self.desired = desired
        self._send_heartbeats(self.now)
        self._next_heartbeat = self.now + self.heartbeat_s

    def reassert_run(self) -> int:
        """Re-apply RUN to batteries that were stopped locally while we were
        commanding RUN. Sends STOP then RUN, but only to batteries that are
        not operating, so running batteries are never cycled. Returns count."""
        n = 0
        for b in self.batteries.values():
            if (self.desired == P.Desired.RUN and b.online and b.node is not None and b.status
                    and b.status.commanded == P.Desired.RUN and b.status.system_state != P.SystemState.OPERATING):
                self._send_command(b, P.Desired.STOP)
                self._send_command(b, P.Desired.RUN)
                log.info("%s re-asserted RUN", b.label)
                n += 1
        return n

    def release(self, serial: int) -> None:
        b = self.batteries.get(serial)
        if not b:
            return
        self.bus.send(P.ID_ASSIGN, P.Assign(serial, self.prefix, P.NODE_NONE, 0).encode())
        log.info("%s released", b.label)
        if b.node is not None:
            self.nodes.pop(b.node, None)
            b.node = None

    def request(self, serial: int, msg_type: int, page: int = 0xFF) -> bool:
        b = self.batteries.get(serial)
        if not b or b.node is None:
            return False
        self.bus.send(P.make_id(self.prefix, P.MsgType.REQUEST, b.node), P.Request(msg_type, page).encode())
        return True

    def aggregate(self) -> Aggregate:
        a = Aggregate(n_batteries=len(self.batteries))
        online = [b for b in self.batteries.values() if b.online]
        a.n_online = len(online)
        contributing = [b for b in online if b.contributing]
        a.n_contributing = len(contributing)

        volts = [b.measurements.voltage_cV / 100 for b in online if b.measurements]
        if volts:
            a.voltage_V = sum(volts) / len(volts)
        a.current_A = sum(b.measurements.current_mA / 1000 for b in online if b.measurements)

        for b in contributing:
            if b.limits:
                a.charge_A += b.limits.charge_dA / 10
                a.discharge_A += b.limits.discharge_dA / 10
            if b.energy:
                a.remaining_Ah += b.energy.remaining_dAh / 10
                a.full_Ah += b.energy.full_dAh / 10

        maxes = [b.limits.max_voltage_dV / 10 for b in online if b.limits]
        mins = [b.limits.min_voltage_dV / 10 for b in online if b.limits]
        if maxes:
            a.max_voltage_V = min(maxes)
            a.min_voltage_V = max(mins)

        weighted = [(b.measurements.soc_cpct / 100, b.energy.full_dAh) for b in contributing
                    if b.measurements and b.energy and b.energy.full_dAh > 0]
        if weighted:
            a.soc_pct = sum(s * w for s, w in weighted) / sum(w for _, w in weighted)
        else:
            socs = [b.measurements.soc_cpct / 100 for b in online if b.measurements]
            if socs:
                a.soc_pct = sum(socs) / len(socs)

        a.worst_level = max((b.level for b in online), default=P.EventLevel.NONE)
        return a

    def table(self) -> List[str]:
        lines = [str(self.aggregate())]
        for b in sorted(self.batteries.values(), key=lambda x: (x.node is None, x.node or 0, x.serial)):
            lines.append(b.describe())
        return lines

    # ---- internals -------------------------------------------------------

    def _on_frame(self, can_id: int, data: bytes, now: float) -> None:
        if P.is_announce_id(can_id):
            try:
                self._on_announce(P.Announce.decode(data), now)
            except ValueError as e:
                log.debug("bad announce: %s", e)
            return
        if P.id_prefix(can_id) != self.prefix:
            return
        msg_type = P.id_type(can_id)
        node = P.id_node(can_id)
        if P.is_controller_type(msg_type):
            log.warning("another controller is transmitting on prefix 0x%04X (id 0x%08X)", self.prefix, can_id)
            return
        serial = self.nodes.get(node)
        if serial is None:
            if node not in self._unknown_nodes:
                self._unknown_nodes[node] = now
                log.info("telemetry from unassigned node %d on our prefix; waiting for its ANNOUNCE", node)
            return
        self._unknown_nodes.pop(node, None)
        try:
            self.batteries[serial].on_frame(msg_type, data, now)
        except (ValueError, IndexError) as e:
            log.debug("bad frame type 0x%02X from node %d: %s", msg_type, node, e)

    def _on_announce(self, a: P.Announce, now: float) -> None:
        b = self.batteries.get(a.serial)
        if b is None:
            b = BatteryRecord(serial=a.serial, first_seen=now)
            self.batteries[a.serial] = b
            log.info("discovered battery %08X (protocol v%d, %s)", a.serial, a.version,
                     "unassigned" if a.link == P.LinkState.UNASSIGNED else f"assigned prefix 0x{a.prefix:04X} node {a.node}")
        b.last_announce = now
        b.announce = a

        if a.version != P.PROTOCOL_VERSION:
            log.warning("%08X speaks protocol v%d, we speak v%d", a.serial, a.version, P.PROTOCOL_VERSION)

        if a.link == P.LinkState.ASSIGNED and a.prefix == self.prefix and b.node is None:
            # Battery still holds an assignment (we probably restarted). Adopt
            # its node if nobody else has it; the re-ASSIGN below is idempotent.
            if self.nodes.get(a.node) in (None, a.serial) and P.NODE_MIN <= a.node <= P.NODE_MAX:
                log.info("%08X adopting existing assignment node %d", a.serial, a.node)
                self._bind(b, a.node, now)

        node = b.node if b.node is not None else self._allocate_node(a.serial)
        if node is None:
            log.error("no free node for %08X", a.serial)
            return

        consistent = a.link == P.LinkState.ASSIGNED and a.prefix == self.prefix and a.node == node
        if not consistent:
            self._assign(b, node, now)

    def _allocate_node(self, serial: int) -> Optional[int]:
        pref = self.node_map.preferred(serial)
        if pref is not None and P.NODE_MIN <= pref <= P.NODE_MAX and self.nodes.get(pref) in (None, serial):
            return pref
        for n in range(P.NODE_MIN, P.NODE_MAX + 1):
            if n not in self.nodes:
                return n
        return None

    def _bind(self, b: BatteryRecord, node: int, now: float) -> None:
        if b.node is not None and b.node != node:
            self.nodes.pop(b.node, None)
        b.node = node
        self.nodes[node] = b.serial
        self.node_map.remember(b.serial, node)
        if b.assigned_at is None:
            b.assigned_at = now

    def _assign(self, b: BatteryRecord, node: int, now: float) -> None:
        self._bind(b, node, now)
        b.assigned_at = now
        b.assign_count += 1
        self.bus.send(P.ID_ASSIGN, P.Assign(b.serial, self.prefix, node, self.assign_flags).encode())
        log.info("%08X -> ASSIGN prefix 0x%04X node %d flags 0x%02X", b.serial, self.prefix, node, self.assign_flags)

    def _send_command(self, b: BatteryRecord, desired: P.Desired) -> None:
        if b.node is None:
            return
        self._seq = (self._seq + 1) & 0xFFFF
        self.bus.send(P.make_id(self.prefix, P.MsgType.COMMAND, b.node), P.Command(int(desired), 0, self._seq).encode())
        b.last_seq_sent = self._seq
        b.last_seq_sent_at = self.now

    def _send_heartbeats(self, now: float) -> None:
        self.now = now
        for b in self.batteries.values():
            if b.node is not None:
                self._send_command(b, self.desired)
