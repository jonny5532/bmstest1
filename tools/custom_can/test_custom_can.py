"""Tests for the Python reference controller and battery simulator.

Run from the repository root:

    python -m unittest tools.custom_can.test_custom_can -v

Everything runs on the in-process loopback bus with simulated time, so the
whole suite covering minutes of bus time finishes in a second or two.
"""

from __future__ import annotations

import json
import logging
import tempfile
import unittest
from pathlib import Path

from . import protocol as P
from .battery_sim import BatterySim, TICK_S
from .bus import LoopbackHub
from .controller import Controller, NodeMap

logging.getLogger("custom_can").setLevel(logging.CRITICAL)

BOARD_IDS = [bytes.fromhex(h) for h in ("E6614104032B9C27", "E661410403185B2A", "E6614104035D6D31")]


class World:
    """A controller and N simulated batteries sharing a loopback hub, driven
    in lock-step with a simulated 20 ms tick."""

    def __init__(self, n: int = 2, drop_rate: float = 0.0, **ctrl_kwargs):
        self.hub = LoopbackHub(drop_rate=drop_rate)
        self.now = 0.0
        self.sims = [BatterySim(self.hub.endpoint(f"bat{i}"), BOARD_IDS[i], name=f"bat{i}", now=0.0,
                                init_time=1.0, precharge_time=0.5) for i in range(n)]
        self.ctrl_bus = self.hub.endpoint("controller")
        self.ctrl = Controller(self.ctrl_bus, **ctrl_kwargs)
        self.ctrl_alive = True

    def run(self, seconds: float, until=None) -> bool:
        """Advance time; stop early (returning True) once ``until()`` holds."""
        ticks = int(round(seconds / TICK_S))
        for _ in range(ticks):
            self.now += TICK_S
            for s in self.sims:
                s.tick(self.now)
            if self.ctrl_alive:
                self.ctrl.tick(self.now)
            else:
                while self.ctrl_bus.recv() is not None:
                    pass
            if until is not None and until():
                return True
        return until is None or until()

    def restart_controller(self, **ctrl_kwargs) -> None:
        self.hub.remove(self.ctrl_bus)
        self.ctrl_bus = self.hub.endpoint("controller")
        self.ctrl = Controller(self.ctrl_bus, **ctrl_kwargs)
        self.ctrl_alive = True

    def record(self, sim: BatterySim):
        return self.ctrl.batteries.get(sim.serial)

    def all_assigned(self) -> bool:
        return all(s.link == P.LinkState.ASSIGNED for s in self.sims)

    def all_online(self) -> bool:
        return all(self.record(s) is not None and self.record(s).online for s in self.sims)

    def all_enabled(self) -> bool:
        return all(self.record(s) is not None and self.record(s).enabled for s in self.sims)


# ------------------------------------------------------------- pure protocol


class ProtocolTests(unittest.TestCase):
    def test_id_helpers(self):
        cid = P.make_id(0x7FFF, 0x3F, 0xFE)
        self.assertEqual(cid, 0x1FFFFFFE)
        self.assertEqual((P.id_prefix(cid), P.id_type(cid), P.id_node(cid)), (0x7FFF, 0x3F, 0xFE))
        self.assertEqual(P.id_prefix(P.ID_ANNOUNCE_BASE), P.DISCOVERY_PREFIX)
        self.assertEqual(P.id_type(P.ID_ASSIGN), 1)
        self.assertTrue(P.is_announce_id(P.ID_ANNOUNCE_BASE | 0xAB))
        self.assertFalse(P.is_announce_id(P.ID_ASSIGN))

    def test_serial32_is_fnv1a(self):
        # Standard FNV-1a 32-bit vectors
        self.assertEqual(P.serial32(b""), 0x811C9DC5)
        self.assertEqual(P.serial32(b"a"), 0xE40C292C)
        self.assertEqual(P.serial32(b"foobar"), 0xBF9CF968)

    def test_round_trips(self):
        samples = [
            P.Announce.build(0xDEADBEEF, P.LinkState.ASSIGNED, 7, 0x0300),
            P.Assign(0xDEADBEEF, 0x0300, 7, P.AssignFlag.STOP_ON_TIMEOUT | (2 << P.ASSIGN_RATE_SHIFT_POS)),
            P.Command(P.Desired.RUN, 0, 1234),
            P.Status.build(4, 8, 0x41, 2, P.Desired.RUN, 999, 29),
            P.Limits(3888, 3168, 300, 250),
            P.Measurements(37012, -12345, 5500),
            P.Energy(9900, 800, 1470, -33),
            P.CellStats(3690, 3712, 3700, 5, 90),
            P.Temperatures(245, 312, 0, 7),
            P.HvVoltages(37000, -2, 3, 12),
            P.Supply(3300, 5000, 12000, 12500),
            P.Balancing(1, 3, 40, 2, 5),
            P.Event(29, 2, 3, 120, 51, 1),
            P.Estimators(1, 2, 3, 4),
            P.Config(1, 2, 96, 8, 1470, 1350),
            P.Serial(BOARD_IDS[0]),
            P.Settings(0, 100, 3300, 4050),
            P.CellLimits(3000, 4200, 2700, 4250),
            P.ModuleTempsPage(6, 8, (250, 251, P.INT16_NA)),
            P.CellVoltagesPage(93, 0x82, (3700, 3701, 3702)),
        ]
        for frame in samples:
            data = frame.encode()
            self.assertEqual(len(data), 8, type(frame).__name__)
            self.assertEqual(type(frame).decode(data), frame, type(frame).__name__)

        st = samples[3]
        self.assertEqual((st.level, st.commanded), (2, P.Desired.RUN))
        self.assertTrue(st.has(P.StatusFlag.CURRENT_ENABLED))
        self.assertTrue(samples[1].rate_shift == 2)
        req = P.Request(P.MsgType.CELL_VOLTAGES, 3)
        self.assertEqual(P.Request.decode(req.encode()), req)
        self.assertEqual(P.Command.decode(bytes([2, 0, 0x04, 0xD2])).seq, 1234)  # DLC 4 is legal

    def test_cell_page_items(self):
        page = P.CellVoltagesPage(3, 0x82, (3700, P.INT16_NA, 3702))
        self.assertEqual(list(page.items()), [(3, 3700, False), (4, None, True), (5, 3702, False)])
        self.assertTrue(page.disturbed)

    def test_event_names_match_firmware(self):
        self.assertEqual(P.EVENT_NAMES[P.EVENT_INVERTER_DETECTED], "INVERTER_DETECTED")
        self.assertEqual(P.EVENT_NAMES[50], "RESTARTING")
        # The table parsed from events.h and the embedded fallback must agree
        self.assertEqual(P.EVENT_NAMES, P._FALLBACK_EVENT_NAMES)
        self.assertEqual(P.event_name(0xFF), "-")


# ------------------------------------------------------- controller + battery


class DiscoveryTests(unittest.TestCase):
    def test_discovery_assignment_and_telemetry(self):
        w = World(3, prefix=0x0300)
        self.assertTrue(w.run(3.0, w.all_online))
        nodes = sorted(w.record(s).node for s in w.sims)
        self.assertEqual(nodes, [1, 2, 3])
        for s in w.sims:
            self.assertEqual((s.prefix, s.node), (0x0300, w.record(s).node))

        w.run(6.0)  # let paged data complete
        for s in w.sims:
            r = w.record(s)
            self.assertEqual(r.config.num_cells, 96)
            self.assertEqual(r.board_id, s.board_id)
            self.assertEqual(len(r.cell_voltages), 96)
            self.assertEqual(len(r.module_temps), 8)
            self.assertIsNotNone(r.limits)
            self.assertIsNotNone(r.measurements)
            self.assertIsNotNone(r.rtt)
            self.assertLess(r.rtt, 0.1)
            self.assertIn(P.EVENT_INVERTER_DETECTED, r.events)
        self.assertEqual(w.ctrl.aggregate().n_online, 3)

    def test_invalid_prefix_rejected(self):
        hub = LoopbackHub()
        with self.assertRaises(ValueError):
            Controller(hub.endpoint(), prefix=P.DISCOVERY_PREFIX)

    def test_node_map_preference_and_persistence(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "nodes.json"
            nm = NodeMap(path)
            nm.remember(P.serial32(BOARD_IDS[1]), 42)
            w = World(2, prefix=0x0300, node_map=NodeMap(path))
            self.assertTrue(w.run(3.0, w.all_online))
            self.assertEqual(w.record(w.sims[1]).node, 42)
            self.assertEqual(w.record(w.sims[0]).node, 1)
            saved = json.loads(path.read_text())
            self.assertEqual(saved[f"{w.sims[0].serial:08X}"], 1)

    def test_tolerates_frame_loss(self):
        w = World(2, drop_rate=0.25, prefix=0x0300)
        self.assertTrue(w.run(10.0, w.all_online))
        w.ctrl.set_desired(P.Desired.RUN)
        self.assertTrue(w.run(10.0, w.all_enabled))
        w.run(5.0)
        self.assertTrue(w.all_online())  # heartbeats survive 25 % loss


class ControlTests(unittest.TestCase):
    def test_run_then_graceful_stop_waits_for_load(self):
        w = World(1, prefix=0x0300)
        s = w.sims[0]
        self.assertTrue(w.run(3.0, w.all_online))
        self.assertTrue(w.run(3.0, lambda: w.record(s).status.system_state == P.SystemState.INACTIVE))

        w.ctrl.set_desired(P.Desired.RUN)
        self.assertTrue(w.run(4.0, w.all_enabled))
        r = w.record(s)
        self.assertEqual(r.status.commanded, P.Desired.RUN)
        self.assertFalse(r.status.has(P.StatusFlag.COMMAND_PENDING))
        self.assertGreater(r.limits.discharge_dA, 0)

        s.set_load(-10.0)  # inverter discharging
        w.run(1.0)
        self.assertAlmostEqual(r.measurements.current_mA, -10000, delta=1)

        w.ctrl.set_desired(P.Desired.STOP)
        self.assertTrue(w.run(1.0, lambda: r.status.contactor_state == P.ContactorState.AWAITING_OPEN))
        self.assertEqual(r.status.commanded, P.Desired.STOP)
        self.assertFalse(r.enabled)
        self.assertEqual(w.ctrl.aggregate().discharge_A, 0.0)  # not enabled -> not contributing
        # LIMITS follows within its 200 ms period
        self.assertTrue(w.run(0.3, lambda: (r.limits.charge_dA, r.limits.discharge_dA) == (0, 0)))

        w.run(5.0)  # still under load: must not open
        self.assertEqual(s.contactor_state, P.ContactorState.AWAITING_OPEN)

        s.set_load(0.0)
        self.assertTrue(w.run(1.0, lambda: r.status.contactor_state == P.ContactorState.OPEN))
        self.assertEqual(r.status.system_state, P.SystemState.INACTIVE)

    def test_open_timeout_under_load(self):
        w = World(1, prefix=0x0300)
        s = w.sims[0]
        s.open_timeout = 3.0
        w.ctrl.set_desired(P.Desired.RUN)
        self.assertTrue(w.run(6.0, w.all_enabled))
        s.set_load(-4.0)
        w.run(0.5)
        w.ctrl.set_desired(P.Desired.STOP)
        self.assertTrue(w.run(1.0, lambda: s.contactor_state == P.ContactorState.AWAITING_OPEN))
        self.assertFalse(w.run(2.0, lambda: s.contactor_state == P.ContactorState.OPEN))
        self.assertTrue(w.run(2.0, lambda: s.contactor_state == P.ContactorState.OPEN))

    def test_local_stop_is_not_fought_and_reassert(self):
        w = World(1, prefix=0x0300, desired=P.Desired.RUN)
        s = w.sims[0]
        self.assertTrue(w.run(6.0, w.all_enabled))
        s.local_stop()
        self.assertTrue(w.run(2.0, lambda: s.system_state == P.SystemState.INACTIVE))
        w.run(3.0)  # many RUN heartbeats later...
        self.assertEqual(s.system_state, P.SystemState.INACTIVE)
        r = w.record(s)
        self.assertEqual(r.status.commanded, P.Desired.RUN)  # visible discrepancy

        self.assertEqual(w.ctrl.reassert_run(), 1)
        self.assertTrue(w.run(4.0, w.all_enabled))
        self.assertEqual(w.ctrl.reassert_run(), 0)  # nothing to do when operating

    def test_fault_excluded_from_aggregate(self):
        w = World(2, prefix=0x0300, desired=P.Desired.RUN)
        self.assertTrue(w.run(6.0, w.all_enabled))
        both = w.ctrl.aggregate()
        self.assertEqual(both.n_contributing, 2)
        self.assertGreater(both.charge_A, 0)

        w.sims[1].inject_fault()
        r1 = w.record(w.sims[1])
        self.assertTrue(w.run(1.0, lambda: r1.status.system_state == P.SystemState.FAULT))
        agg = w.ctrl.aggregate()
        self.assertEqual(agg.n_contributing, 1)
        self.assertEqual(agg.worst_level, P.EventLevel.FATAL)
        self.assertEqual(r1.status.level, P.EventLevel.FATAL)
        self.assertEqual(P.event_name(r1.status.top_event), "CELL_VOLTAGE_VERY_HIGH")
        # The RUN was already satisfied before the fault, so it is not pending;
        # the controller sees the discrepancy as commanded RUN vs state FAULT.
        self.assertEqual(r1.status.commanded, P.Desired.RUN)
        self.assertFalse(r1.status.has(P.StatusFlag.COMMAND_PENDING))
        self.assertTrue(w.run(2.0, lambda: r1.status.contactor_state == P.ContactorState.OPEN))

        # A RUN received while in FAULT stays pending (and visible) forever
        w.ctrl.set_desired(P.Desired.STOP)
        w.run(1.0)
        w.ctrl.set_desired(P.Desired.RUN)
        w.run(3.0)
        self.assertTrue(r1.status.has(P.StatusFlag.COMMAND_PENDING))
        self.assertEqual(r1.status.system_state, P.SystemState.FAULT)

    def test_request_expedites_frame(self):
        w = World(1, prefix=0x0300)
        self.assertTrue(w.run(3.0, w.all_online))
        w.run(1.0)
        r = w.record(w.sims[0])
        before = r.frame_counts.get(P.MsgType.CONFIG, 0)
        w.ctrl.request(w.sims[0].serial, P.MsgType.CONFIG)
        w.run(0.1)
        self.assertEqual(r.frame_counts[P.MsgType.CONFIG], before + 1)


class RestartTests(unittest.TestCase):
    def test_battery_reboot_is_rediscovered_and_resumes(self):
        w = World(2, prefix=0x0300, desired=P.Desired.RUN)
        self.assertTrue(w.run(6.0, w.all_enabled))
        s = w.sims[0]
        node = w.record(s).node

        s.reboot(w.now)
        r = w.record(s)
        assigns = r.assign_count
        self.assertEqual(s.link, P.LinkState.UNASSIGNED)
        # Re-discovery is quicker than the 2 s liveness timeout: the battery
        # is re-assigned (to the same node) before it is even marked offline.
        self.assertTrue(w.run(3.0, lambda: s.link == P.LinkState.ASSIGNED))
        self.assertTrue(r.online)
        self.assertEqual(r.assign_count, assigns + 1)
        self.assertEqual((r.node, s.node), (node, node))
        self.assertTrue(w.run(5.0, lambda: r.enabled))  # operating persisted + RUN level re-applied

    def test_quick_controller_restart_adopts_assignments(self):
        w = World(2, prefix=0x0300, desired=P.Desired.RUN)
        self.assertTrue(w.run(6.0, w.all_enabled))
        nodes = {s.serial: s.node for s in w.sims}

        w.restart_controller(prefix=0x0300, desired=P.Desired.RUN)
        links = []
        ok = w.run(7.0, lambda: (links.append([s.link for s in w.sims]) or w.all_online()))
        self.assertTrue(ok)
        # Batteries never dropped their assignment and kept their nodes
        self.assertTrue(all(all(l == P.LinkState.ASSIGNED for l in snapshot) for snapshot in links))
        self.assertEqual({s.serial: s.node for s in w.sims}, nodes)
        self.assertTrue(w.all_enabled())

    def test_slow_controller_restart_rediscovers(self):
        w = World(2, prefix=0x0300, desired=P.Desired.RUN)
        self.assertTrue(w.run(6.0, w.all_enabled))
        w.ctrl_alive = False
        self.assertTrue(w.run(7.0, lambda: all(s.link == P.LinkState.UNASSIGNED for s in w.sims)))
        # Without STOP_ON_TIMEOUT the batteries keep running
        self.assertTrue(all(s.enable_current for s in w.sims))

        w.restart_controller(prefix=0x0300, desired=P.Desired.RUN)
        self.assertTrue(w.run(5.0, w.all_enabled))

    def test_stop_on_timeout_flag(self):
        w = World(1, prefix=0x0300, desired=P.Desired.RUN, assign_flags=P.AssignFlag.STOP_ON_TIMEOUT)
        s = w.sims[0]
        self.assertTrue(w.run(6.0, w.all_enabled))
        w.ctrl_alive = False
        self.assertTrue(w.run(7.0, lambda: s.link == P.LinkState.UNASSIGNED))
        self.assertTrue(w.run(2.0, lambda: s.contactor_state == P.ContactorState.OPEN))
        self.assertEqual(s.system_state, P.SystemState.INACTIVE)

    def test_release_and_reassign(self):
        w = World(1, prefix=0x0300)
        s = w.sims[0]
        self.assertTrue(w.run(3.0, w.all_online))
        w.ctrl.release(s.serial)
        self.assertTrue(w.run(1.0, lambda: s.link == P.LinkState.UNASSIGNED))
        # It announces again and the controller re-assigns it
        self.assertTrue(w.run(3.0, w.all_online))

    def test_duplicate_node_conflict_self_heals(self):
        w = World(2, prefix=0x0300)
        self.assertTrue(w.run(3.0, w.all_online))
        a, b = w.sims
        # A buggy controller assigns b onto a's node
        w.ctrl_bus.send(P.ID_ASSIGN, P.Assign(b.serial, 0x0300, a.node, 0).encode())
        # Whoever hears the other's frames on its own address drops off and
        # re-announces; the (correct) controller then sorts it out again.
        self.assertTrue(w.run(1.0, lambda: a.conflicts + b.conflicts > 0))
        self.assertTrue(w.run(5.0, lambda: w.all_online() and a.node != b.node
                              and a.link == b.link == P.LinkState.ASSIGNED))
        settled = a.conflicts + b.conflicts
        w.run(3.0)
        self.assertEqual(a.conflicts + b.conflicts, settled)  # no further flapping
        self.assertTrue(w.all_online() and a.node != b.node)


if __name__ == "__main__":
    unittest.main()
