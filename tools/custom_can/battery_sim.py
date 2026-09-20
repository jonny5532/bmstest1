"""Simulated battery speaking the custom multi-battery CAN protocol.

The protocol behaviour mirrors ``bms/protocols/inverter/custom_can.c``
(announce/assign, level-based commands with pending state, controller
timeout, conflict detection, priority scheduler with paging). The battery
itself is a toy: a simplified system/contactor state machine and a linear
pack model, just enough to demonstrate the exchanges a real controller must
handle, including the "won't open under load" wait.

Use it in-process against :class:`custom_can.controller.Controller` (see the
tests and the ``demo`` command), or put a few of them on a real CAN adapter to
exercise a controller such as Battery Emulator without any hardware battery.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

from . import protocol as P

log = logging.getLogger("custom_can.battery")

MAX_TX_PER_TICK = 3
REQUEST_RETRY_S = 1.0
ANNOUNCE_JITTER_TICKS = 25
TICK_S = 0.02

CHEMISTRY_PARAMS = {
    P.Chemistry.NMC: dict(working_min=3300, working_max=4050, soft_min=3000, soft_max=4200,
                          hard_min=2700, hard_max=4250, max_dA=300),
    P.Chemistry.LFP: dict(working_min=2900, working_max=3350, soft_min=2800, soft_max=3400,
                          hard_min=2500, hard_max=3650, max_dA=500),
}


@dataclass
class _Sched:
    msg_type: int
    period: float
    essential: bool
    build: Callable[[int], Optional[bytes]]
    pages: int = 1
    page: int = 0
    next_t: float = 0.0
    due_now: bool = False


class BatterySim:
    def __init__(self, bus, board_id: bytes, *, name: Optional[str] = None, n_cells: int = 96,
                 n_temps: int = 8, chemistry: P.Chemistry = P.Chemistry.NMC, capacity_ah: float = 147.0,
                 soc: float = 0.6, init_time: float = 2.0, precharge_time: float = 1.0,
                 open_timeout: float = 30.0, operating: bool = False, now: float = 0.0):
        if len(board_id) != 8:
            raise ValueError("board_id must be 8 bytes")
        self.bus = bus
        self.board_id = bytes(board_id)
        self.serial = P.serial32(self.board_id)
        self.name = name or f"{self.serial:08X}"
        self.n_cells = n_cells
        self.n_temps = n_temps
        self.chemistry = chemistry
        self.params = CHEMISTRY_PARAMS[chemistry]
        self.capacity_ah = capacity_ah
        self.init_time = init_time
        self.precharge_time = precharge_time
        self.open_timeout = open_timeout

        # Plant
        self.soc = soc
        self.load_a = 0.0        # what the "inverter" draws when contactors are closed (+ = charging)
        self.current_a = 0.0
        self.cells_mV: List[int] = [0] * n_cells
        self.temps_dC: List[int] = [250] * n_temps
        self.balancing_cells: set = set()
        self.events: Dict[int, List] = {}  # type -> [level, count, timestamp]

        # Persisted-across-reboot state
        self.operating = operating

        self.tx_count = 0
        self.conflicts = 0
        self._event_cursor = 0
        self.reboot(now)

    # ------------------------------------------------------------------ plant

    def set_load(self, amps: float) -> None:
        self.load_a = amps

    def raise_event(self, event_type: int, level: int, now: Optional[float] = None) -> None:
        slot = self.events.setdefault(event_type, [P.EventLevel.NONE, 0, 0.0])
        if slot[0] == P.EventLevel.NONE:
            slot[1] += 1
        slot[0] = max(slot[0], level)
        slot[2] = self._now if now is None else now

    def clear_event(self, event_type: int) -> None:
        if event_type in self.events and self.events[event_type][0] != P.EventLevel.FATAL:
            self.events[event_type][0] = P.EventLevel.NONE

    def inject_fault(self, event_type: int = 30) -> None:
        """Latch a FATAL event (default CELL_VOLTAGE_VERY_HIGH): contactors open, FAULT."""
        self.raise_event(event_type, P.EventLevel.FATAL)
        self.operating = False
        self._enter_system(P.SystemState.FAULT)

    @property
    def highest_level(self) -> int:
        return max((s[0] for s in self.events.values()), default=P.EventLevel.NONE)

    def reboot(self, now: float) -> None:
        """Power-cycle: everything volatile is lost, ``operating`` persists."""
        self._now = now
        self._last_tick = now
        self.link = P.LinkState.UNASSIGNED
        self.prefix = 0
        self.node = P.NODE_NONE
        self.assign_flags = 0
        self.rate_shift = 0
        self.last_controller_t = now
        self.last_seq = 0
        self.commanded = P.Desired.NONE
        self.pending = P.Desired.NONE
        self.next_request_t = now
        self.next_announce_t = now + (self.serial % ANNOUNCE_JITTER_TICKS) * TICK_S
        self.system_req = P.Desired.NONE
        self.system_state = P.SystemState.INITIALIZING
        self.system_since = now
        self.contactor_state = P.ContactorState.OPEN
        self.contactor_since = now
        self.enable_current = False
        self.current_a = 0.0
        self.events = {}
        self.raise_event(P.EVENT_NAMES.index("BOOT_NORMAL"), P.EventLevel.INFO, now)
        self._last_status_states = (-1, -1)
        self.schedule = self._make_schedule()
        log.info("[%s] booted, serial %08X", self.name, self.serial)

    # ------------------------------------------------------------------- tick

    def tick(self, now: float) -> None:
        dt = max(0.0, min(now - self._last_tick, 1.0))
        self._now = now
        self._last_tick = now

        self._process_rx(now)
        self._check_controller_timeout(now)
        self._apply_pending(now)
        self._system_tick(now)
        self._physics(dt)

        if self.link == P.LinkState.ASSIGNED and self._last_status_states != (self.system_state, self.contactor_state):
            self._expedite(P.MsgType.STATUS)

        self._announce_tick(now)
        if self.link == P.LinkState.ASSIGNED:
            self._schedule_tick(now)

    # ------------------------------------------------------------- receiving

    def _process_rx(self, now: float) -> None:
        while True:
            f = self.bus.recv()
            if f is None:
                return
            can_id = f.can_id & P.ID_MASK
            if can_id == P.ID_ASSIGN:
                self._handle_assign(f.data, now)
                continue
            if self.link != P.LinkState.ASSIGNED or P.id_prefix(can_id) != self.prefix:
                continue
            msg_type, node = P.id_type(can_id), P.id_node(can_id)
            if P.is_controller_type(msg_type):
                if node not in (self.node, P.NODE_ALL):
                    continue
                if msg_type == P.MsgType.COMMAND:
                    self._handle_command(f.data, now)
                elif msg_type == P.MsgType.REQUEST:
                    self._handle_request(f.data, now)
            elif node == self.node:
                self.conflicts += 1
                log.warning("[%s] another node transmits on our address (0x%08X)", self.name, can_id)
                self._unassign("address conflict", now)

    def _handle_assign(self, data: bytes, now: float) -> None:
        if len(data) != 8:
            return
        a = P.Assign.decode(data)
        if a.serial != self.serial:
            return
        if a.node == P.NODE_NONE:
            if self.link == P.LinkState.ASSIGNED:
                self._unassign("released by controller", now)
            return
        if a.prefix > P.PREFIX_MAX or a.prefix == P.DISCOVERY_PREFIX or a.node == P.NODE_ALL:
            log.warning("[%s] rejected invalid assignment prefix 0x%04X node %d", self.name, a.prefix, a.node)
            return
        if self.link == P.LinkState.ASSIGNED and (self.prefix, self.node) == (a.prefix, a.node):
            self.assign_flags = a.flags
            self.rate_shift = a.rate_shift
            self.last_controller_t = now
            self._expedite(P.MsgType.STATUS)
            self._expedite(P.MsgType.CONFIG)
            return
        self.link = P.LinkState.ASSIGNED
        self.prefix, self.node = a.prefix, a.node
        self.assign_flags, self.rate_shift = a.flags, a.rate_shift
        self.last_controller_t = now
        self.last_seq = 0
        self.commanded = P.Desired.NONE
        self.pending = P.Desired.NONE
        for e in self.schedule:
            e.page, e.next_t, e.due_now = 0, now, False
        self.next_announce_t = now + 0.5
        self.raise_event(P.EVENT_INVERTER_DETECTED, P.EventLevel.INFO, now)
        log.info("[%s] assigned prefix 0x%04X node %d flags 0x%02X", self.name, a.prefix, a.node, a.flags)

    def _unassign(self, why: str, now: float) -> None:
        log.info("[%s] unassigned (%s), was prefix 0x%04X node %d", self.name, why, self.prefix, self.node)
        if why == "controller timeout" and self.assign_flags & P.AssignFlag.STOP_ON_TIMEOUT:
            log.warning("[%s] controller lost, requesting STOP as configured", self.name)
            self.pending = P.Desired.STOP
            self.next_request_t = now
        else:
            self.pending = P.Desired.NONE
        self.link = P.LinkState.UNASSIGNED
        self.prefix, self.node = 0, P.NODE_NONE
        self.assign_flags = self.rate_shift = 0
        self.commanded = P.Desired.NONE
        self.last_seq = 0
        self.next_announce_t = now

    def _handle_command(self, data: bytes, now: float) -> None:
        if len(data) < 4:
            return
        c = P.Command.decode(data)
        self.last_controller_t = now
        self.last_seq = c.seq
        if c.desired not in (P.Desired.STOP, P.Desired.RUN):
            return
        if c.desired != self.commanded:
            log.info("[%s] controller requests %s (seq %d)", self.name, P.Desired(c.desired).name, c.seq)
            self.commanded = P.Desired(c.desired)
            self.pending = self.commanded
            self.next_request_t = now
            self._expedite(P.MsgType.STATUS)

    def _handle_request(self, data: bytes, now: float) -> None:
        if len(data) < 1:
            return
        self.last_controller_t = now
        r = P.Request.decode(data)
        for e in self.schedule:
            if e.msg_type == r.msg_type:
                if e.pages > 1 and r.page < e.pages:
                    e.page = r.page
                e.due_now = True

    def _check_controller_timeout(self, now: float) -> None:
        if self.link == P.LinkState.ASSIGNED and now - self.last_controller_t > P.CONTROLLER_TIMEOUT:
            log.warning("[%s] no controller frames for %.1fs", self.name, now - self.last_controller_t)
            self._unassign("controller timeout", now)

    # --------------------------------------------------- system state machine

    def _apply_pending(self, now: float) -> None:
        if self.pending == P.Desired.NONE:
            return
        sys = self.system_state
        satisfied = False
        request = P.Desired.NONE
        if self.pending == P.Desired.RUN:
            if sys == P.SystemState.OPERATING:
                satisfied = True
            elif sys == P.SystemState.INACTIVE:
                request = P.Desired.RUN
        else:
            if sys in (P.SystemState.INACTIVE, P.SystemState.FAULT):
                satisfied = True
            elif sys == P.SystemState.OPERATING:
                request = P.Desired.STOP
        if satisfied:
            self.pending = P.Desired.NONE
            self._expedite(P.MsgType.STATUS)
        elif request != P.Desired.NONE and self.system_req == P.Desired.NONE and now >= self.next_request_t:
            self.system_req = request
            self.next_request_t = now + REQUEST_RETRY_S

    def _enter_system(self, state: P.SystemState) -> None:
        if state != self.system_state:
            log.info("[%s] system %s -> %s", self.name, self.system_state.name, state.name)
        self.system_state = state
        self.system_since = self._now

    def _enter_contactor(self, state: P.ContactorState) -> None:
        if state != self.contactor_state:
            log.info("[%s] contactors %s -> %s", self.name, self.contactor_state.name, state.name)
        self.contactor_state = state
        self.contactor_since = self._now

    def local_stop(self) -> None:
        """What the HMI/CLI 'toggle' does on the real BMS."""
        if self.system_state == P.SystemState.OPERATING:
            self.system_req = P.Desired.STOP

    def local_run(self) -> None:
        if self.system_state == P.SystemState.INACTIVE:
            self.system_req = P.Desired.RUN

    def _system_tick(self, now: float) -> None:
        s = self.system_state
        if s == P.SystemState.INITIALIZING:
            if now - self.system_since >= self.init_time:
                self._enter_system(P.SystemState.INACTIVE)
                if self.operating:
                    self.system_req = P.Desired.RUN  # resume, as init.c does
        elif s == P.SystemState.INACTIVE:
            if self.system_req == P.Desired.RUN:
                self.system_req = P.Desired.NONE
                self.operating = True
                self._enter_system(P.SystemState.OPERATING)
            elif self.system_req == P.Desired.STOP:
                self.system_req = P.Desired.NONE
        elif s == P.SystemState.OPERATING:
            if self.system_req == P.Desired.STOP:
                self.system_req = P.Desired.NONE
                self.operating = False
                self._enter_system(P.SystemState.INACTIVE)
            elif self.system_req == P.Desired.RUN:
                self.system_req = P.Desired.NONE
        elif s == P.SystemState.FAULT:
            self.system_req = P.Desired.NONE

        want_closed = self.system_state == P.SystemState.OPERATING
        c = self.contactor_state
        self.enable_current = False
        if c == P.ContactorState.OPEN:
            if want_closed:
                self._enter_contactor(P.ContactorState.PRECHARGING)
        elif c == P.ContactorState.PRECHARGING:
            if not want_closed:
                self._enter_contactor(P.ContactorState.OPEN)
            elif now - self.contactor_since >= self.precharge_time:
                self._enter_contactor(P.ContactorState.CLOSED)
        elif c == P.ContactorState.CLOSED:
            if not want_closed:
                self._enter_contactor(P.ContactorState.AWAITING_OPEN)
            elif now - self.contactor_since >= 0.5:
                self.enable_current = True
        elif c == P.ContactorState.AWAITING_OPEN:
            # Never open under load unless the timeout has passed
            forced = self.system_state == P.SystemState.FAULT and now - self.contactor_since >= 2.0
            if abs(self.current_a) < 1.0 or now - self.contactor_since >= self.open_timeout or forced:
                self._enter_contactor(P.ContactorState.OPEN)

    # ---------------------------------------------------------------- physics

    def _physics(self, dt: float) -> None:
        closed = self.contactor_state in (P.ContactorState.CLOSED, P.ContactorState.AWAITING_OPEN)
        self.current_a = self.load_a if closed else 0.0
        self.soc = min(1.0, max(0.0, self.soc + self.current_a * dt / 3600.0 / self.capacity_ah))

        p = self.params
        ocv = p["working_min"] + self.soc * (p["working_max"] - p["working_min"])
        ir_mV = self.current_a * 0.6
        for i in range(self.n_cells):
            spread = ((i * 7919) % 11) - 5  # deterministic -5..+5 mV per cell
            self.cells_mV[i] = int(round(ocv + spread + ir_mV))
        for i in range(self.n_temps):
            target = 250 + int(abs(self.current_a) * 4) + (i % 3) * 5
            self.temps_dC[i] += int((target - self.temps_dC[i]) * min(1.0, dt * 0.05))

        if self.soc > 0.9:
            top = sorted(range(self.n_cells), key=lambda i: -self.cells_mV[i])[:3]
            self.balancing_cells = set(top)
        else:
            self.balancing_cells = set()

    # ---- derived values used by several frames

    @property
    def pack_V(self) -> float:
        return sum(self.cells_mV) / 1000.0

    def _current_limits_dA(self):
        if not self.enable_current:
            return 0, 0
        max_dA = self.params["max_dA"]
        charge = int(max_dA * min(1.0, max(0.0, (1.0 - self.soc) / 0.1)))
        discharge = int(max_dA * min(1.0, max(0.0, self.soc / 0.1)))
        return charge, discharge

    def _soc_cpct(self) -> int:
        return int(round(self.soc * 10000))

    # --------------------------------------------------------------- sending

    def _send(self, can_id: int, data: bytes) -> None:
        self.bus.send(can_id, data)
        self.tx_count += 1

    def _announce_tick(self, now: float) -> None:
        if now < self.next_announce_t:
            return
        assigned = self.link == P.LinkState.ASSIGNED
        self._send(P.announce_id(self.serial), P.Announce.build(
            self.serial, self.link, self.node if assigned else 0, self.prefix if assigned else 0).encode())
        self.next_announce_t = now + (P.ANNOUNCE_PERIOD_ASSIGNED if assigned else P.ANNOUNCE_PERIOD_UNASSIGNED)

    def _expedite(self, msg_type: int) -> None:
        for e in self.schedule:
            if e.msg_type == msg_type:
                e.due_now = True

    def _schedule_tick(self, now: float) -> None:
        sent = 0
        for e in self.schedule:
            if sent >= MAX_TX_PER_TICK:
                break
            if not e.due_now and now < e.next_t:
                continue
            period = e.period * ((1 << self.rate_shift) if not e.essential else 1)
            data = e.build(e.page)
            e.due_now = False
            e.next_t = now + period
            if data is None:
                continue
            self._send(P.make_id(self.prefix, e.msg_type, self.node), data)
            sent += 1
            if e.pages > 1:
                e.page = (e.page + 1) % e.pages

    def _make_schedule(self) -> List[_Sched]:
        pages = lambda n: (n + 2) // 3  # noqa: E731
        return [
            _Sched(P.MsgType.STATUS, P.STATUS_PERIOD, True, self._build_status),
            _Sched(P.MsgType.LIMITS, 0.2, True, self._build_limits),
            _Sched(P.MsgType.MEASUREMENTS, 0.2, True, self._build_measurements),
            _Sched(P.MsgType.ENERGY, 1.0, False, self._build_energy),
            _Sched(P.MsgType.CELL_STATS, 1.0, False, self._build_cell_stats),
            _Sched(P.MsgType.TEMPERATURES, 1.0, False, self._build_temperatures),
            _Sched(P.MsgType.HV_VOLTAGES, 1.0, False, self._build_hv),
            _Sched(P.MsgType.SUPPLY, 2.0, False, self._build_supply),
            _Sched(P.MsgType.BALANCING, 1.0, False, self._build_balancing),
            _Sched(P.MsgType.EVENT, 0.25, False, self._build_event),
            _Sched(P.MsgType.ESTIMATORS, 2.0, False, self._build_estimators),
            _Sched(P.MsgType.CONFIG, 5.0, False, self._build_config),
            _Sched(P.MsgType.SERIAL, 5.0, False, self._build_serial),
            _Sched(P.MsgType.SETTINGS, 5.0, False, self._build_settings),
            _Sched(P.MsgType.CELL_LIMITS, 5.0, False, self._build_cell_limits),
            _Sched(P.MsgType.MODULE_TEMPS, 0.25, False, self._build_module_temps, pages(self.n_temps)),
            _Sched(P.MsgType.CELL_VOLTAGES, 0.1, False, self._build_cell_voltages, pages(self.n_cells)),
        ]

    # ---- builders (mirror the C encoders)

    def _build_status(self, page: int) -> bytes:
        self._last_status_states = (self.system_state, self.contactor_state)
        flags = 0
        if self.enable_current:
            flags |= P.StatusFlag.CURRENT_ENABLED
        if self.balancing_cells:
            flags |= P.StatusFlag.BALANCING
        if self.operating:
            flags |= P.StatusFlag.OPERATING
        if self.pending != P.Desired.NONE:
            flags |= P.StatusFlag.COMMAND_PENDING
        if self.contactor_state in (P.ContactorState.CLOSED, P.ContactorState.AWAITING_OPEN):
            flags |= P.StatusFlag.PRECHARGE_CLOSED
        level = self.highest_level
        top = 0xFF
        if level > P.EventLevel.NONE:
            top = min(t for t, s in self.events.items() if s[0] == level)
        return P.Status.build(self.system_state, self.contactor_state, flags, level, self.commanded,
                              self.last_seq, top).encode()

    def _build_limits(self, page: int) -> bytes:
        charge, discharge = self._current_limits_dA()
        p = self.params
        return P.Limits(p["working_max"] * self.n_cells // 100, p["working_min"] * self.n_cells // 100,
                        charge, discharge).encode()

    def _build_measurements(self, page: int) -> bytes:
        return P.Measurements(int(self.pack_V * 100), int(self.current_a * 1000), self._soc_cpct()).encode()

    def _build_energy(self, page: int) -> bytes:
        full = int(self.capacity_ah * 10)
        return P.Energy(9900, int(full * self.soc), full, int(full * (1 - self.soc))).encode()

    def _build_cell_stats(self, page: int) -> bytes:
        lo = min(range(self.n_cells), key=lambda i: self.cells_mV[i])
        hi = max(range(self.n_cells), key=lambda i: self.cells_mV[i])
        return P.CellStats(self.cells_mV[lo], self.cells_mV[hi], sum(self.cells_mV) // self.n_cells, lo, hi).encode()

    def _build_temperatures(self, page: int) -> bytes:
        lo = min(range(self.n_temps), key=lambda i: self.temps_dC[i])
        hi = max(range(self.n_temps), key=lambda i: self.temps_dC[i])
        return P.Temperatures(self.temps_dC[lo], self.temps_dC[hi], lo, hi).encode()

    def _build_hv(self, page: int) -> bytes:
        closed = self.contactor_state in (P.ContactorState.CLOSED, P.ContactorState.AWAITING_OPEN)
        out = self.pack_V if closed else 0.0
        across = 0 if closed else int(self.pack_V * 10)
        return P.HvVoltages(int(out * 100), across, across, int((self.pack_V - out) * 10)).encode()

    def _build_supply(self, page: int) -> bytes:
        return P.Supply(3312, 5020, 12400, 12300).encode()

    def _build_balancing(self, page: int) -> bytes:
        active = bool(self.balancing_cells)
        return P.Balancing(1 if active else 0, len(self.balancing_cells), 40 if active else 0, 0,
                           (0x01 if active else 0) | 0x04).encode()

    def _build_event(self, page: int) -> bytes:
        n_types = len(P.EVENT_NAMES)
        active = sum(1 for s in self.events.values() if s[0] > P.EventLevel.NONE)
        recorded = sorted(self.events)
        if not recorded:
            return P.Event(0xFF, 0, 0, 0, n_types, 0).encode()
        nxt = next((t for t in recorded if t >= self._event_cursor), recorded[0])
        self._event_cursor = nxt + 1
        level, count, ts = self.events[nxt]
        age = min(65535, int(self._now - ts))
        return P.Event(nxt, level, min(count, 65535), age, n_types, active).encode()

    def _build_estimators(self, page: int) -> bytes:
        s = self._soc_cpct()
        return P.Estimators(max(0, s - 120), min(10000, s + 60), s, s).encode()

    def _build_config(self, page: int) -> bytes:
        return P.Config(P.PROTOCOL_VERSION, int(self.chemistry), self.n_cells, self.n_temps,
                        int(self.capacity_ah * 10), int(self.capacity_ah * 10 * 0.92)).encode()

    def _build_serial(self, page: int) -> bytes:
        return P.Serial(self.board_id).encode()

    def _build_settings(self, page: int) -> bytes:
        return P.Settings(0, 0, self.params["working_min"], self.params["working_max"]).encode()

    def _build_cell_limits(self, page: int) -> bytes:
        p = self.params
        return P.CellLimits(p["soft_min"], p["soft_max"], p["hard_min"], p["hard_max"]).encode()

    def _build_module_temps(self, page: int) -> bytes:
        first = page * 3
        vals = tuple(self.temps_dC[first + j] if first + j < self.n_temps else P.INT16_NA for j in range(3))
        return P.ModuleTempsPage(first, self.n_temps, vals).encode()  # type: ignore[arg-type]

    def _build_cell_voltages(self, page: int) -> bytes:
        first = page * 3
        flags = 0x80 if self.balancing_cells else 0
        vals = []
        for j in range(3):
            idx = first + j
            if idx < self.n_cells:
                vals.append(self.cells_mV[idx] or P.INT16_NA)
                if idx in self.balancing_cells:
                    flags |= 1 << j
            else:
                vals.append(P.INT16_NA)
        return P.CellVoltagesPage(first, flags, tuple(vals)).encode()  # type: ignore[arg-type]
