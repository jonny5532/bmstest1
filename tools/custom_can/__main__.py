"""Command line front end.

    python -m tools.custom_can demo        # controller + simulated batteries, no hardware
    python -m tools.custom_can controller --bus slcan:COM3 --desired run
    python -m tools.custom_can simulate   --bus socketcan:can0 --batteries 2
    python -m tools.custom_can sniff      --bus socketcan:can0

Bus specs are python-can ``interface:channel[:bitrate]`` strings.
"""

from __future__ import annotations

import argparse
import logging
import queue
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

from . import protocol as P
from .battery_sim import BatterySim, TICK_S
from .bus import LoopbackHub, PythonCanBus
from .controller import Controller, NodeMap

log = logging.getLogger("custom_can")

DEMO_BOARD_IDS = [
    bytes.fromhex("E6614104032B9C27"),
    bytes.fromhex("E661410403185B2A"),
    bytes.fromhex("E6614104035D6D31"),
    bytes.fromhex("E661410403A1C702"),
    bytes.fromhex("E661410403EF8815"),
    bytes.fromhex("E661410403C0FFEE"),
]


def _setup_logging(verbose: int) -> None:
    level = logging.WARNING if verbose == 0 else logging.INFO if verbose == 1 else logging.DEBUG
    logging.basicConfig(level=level, format="%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s", datefmt="%H:%M:%S")


def _print_table(lines: List[str], clear: bool) -> None:
    if clear and sys.stdout.isatty():
        sys.stdout.write("\033[2J\033[H")
    print("\n".join(lines))
    print(flush=True)


def _parse_desired(s: str) -> P.Desired:
    return {"none": P.Desired.NONE, "run": P.Desired.RUN, "stop": P.Desired.STOP}[s.lower()]


# ---------------------------------------------------------------------- demo


def cmd_demo(args: argparse.Namespace) -> int:
    """Scripted scenario on an in-process bus: discovery, RUN, load, a
    graceful STOP that has to wait for the load to go away, a battery power
    cycle, a controller restart and a battery fault."""
    hub = LoopbackHub(drop_rate=args.drop)
    now = 0.0
    n = min(args.batteries, len(DEMO_BOARD_IDS))
    sims = [BatterySim(hub.endpoint(f"bat{i}"), DEMO_BOARD_IDS[i], name=f"bat{i}", soc=0.5 + 0.12 * i,
                       open_timeout=args.open_timeout, now=now) for i in range(n)]
    ctrl_bus = hub.endpoint("controller")
    ctrl = Controller(ctrl_bus, prefix=args.prefix, name="controller")

    script = [
        (4.0, "controller: desired RUN", lambda: ctrl.set_desired(P.Desired.RUN)),
        (9.0, "inverter: discharging 10 A from every battery", lambda: [s.set_load(-10.0) for s in sims]),
        (14.0, "controller: desired STOP (load still flowing -> batteries wait before opening)",
         lambda: ctrl.set_desired(P.Desired.STOP)),
        (17.0, "inverter: load removed -> contactors open", lambda: [s.set_load(0.0) for s in sims]),
        (20.0, "controller: desired RUN again", lambda: ctrl.set_desired(P.Desired.RUN)),
        (23.0, "inverter: charging 5 A", lambda: [s.set_load(5.0) for s in sims]),
        (26.0, "bat0: power cycle (comes back with 'operating' persisted)", lambda: sims[0].reboot(now)),
        (34.0, "controller: restart (fresh instance, no node map) -> adopts existing assignments", "restart"),
        (42.0, "bat1: FATAL fault injected -> excluded from fleet limits", lambda: sims[1].inject_fault()),
        (46.0, "bat0: operator stops it locally; controller still says RUN (not fought)", lambda: sims[0].local_stop()),
        (47.0, "inverter: sees bat0 limits at 0 A and stops drawing from it -> contactors open", lambda: sims[0].set_load(0.0)),
        (50.0, "controller: reassert RUN (STOP+RUN to non-operating batteries only)", lambda: ctrl.reassert_run()),
    ]
    # The lambdas close over the *variables* ctrl/now/sims, so they follow the
    # controller restart below and always see the current bus time.
    end = max(args.duration, script[-1][0] + 4)
    next_table = 0.0
    step = 0
    wall = time.monotonic()
    print(f"demo: {n} simulated batteries, prefix 0x{args.prefix:04X}, {end:.0f} s of bus time"
          f"{' (as fast as possible)' if args.speed == 0 else f' at {args.speed}x'}, frame loss {args.drop:.0%}\n")

    while now <= end:
        while step < len(script) and now >= script[step][0]:
            t, text, action = script[step]
            print(f"\n=== t={t:5.1f}s  {text}\n")
            if action == "restart":
                # Same configuration as before (a real controller reloads it),
                # but no memory of which battery had which node.
                hub.remove(ctrl_bus)
                ctrl_bus = hub.endpoint("controller")
                ctrl = Controller(ctrl_bus, prefix=args.prefix, desired=ctrl.desired, name="controller2")
            else:
                action()
            step += 1

        for s in sims:
            s.tick(now)
        ctrl.tick(now)

        if now >= next_table:
            _print_table([f"t={now:6.1f}s  frames on bus: {hub.frames_sent} (dropped {hub.frames_dropped})"] + ctrl.table(),
                         clear=False)
            next_table = now + args.table_period

        now += TICK_S
        if args.speed > 0:
            target = wall + now / args.speed
            delay = target - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    print("\n=== final state")
    _print_table(ctrl.table(), clear=False)
    rate = hub.frames_sent / end
    print(f"average bus load: {rate:.0f} frames/s for {n} batteries (~{rate * 130 / 500000:.1%} of 500 kbit/s)")
    return 0


# ---------------------------------------------------------------- controller


def _stdin_reader(q: "queue.Queue[str]") -> None:
    for line in sys.stdin:
        q.put(line.strip())


def cmd_controller(args: argparse.Namespace) -> int:
    """Live controller on a real bus. Type commands on stdin:
    run | stop | none | reassert | release <serial> | request <serial> <type> | table | quit"""
    bus = PythonCanBus(args.bus)
    node_map = NodeMap(Path(args.map)) if args.map else None
    flags = (P.AssignFlag.STOP_ON_TIMEOUT if args.stop_on_timeout else 0) | ((args.rate_shift & 3) << P.ASSIGN_RATE_SHIFT_POS)
    ctrl = Controller(bus, prefix=args.prefix, assign_flags=flags, desired=_parse_desired(args.desired), node_map=node_map)

    cmds: "queue.Queue[str]" = queue.Queue()
    threading.Thread(target=_stdin_reader, args=(cmds,), daemon=True).start()
    print(f"controller on {args.bus}, prefix 0x{args.prefix:04X}, desired {ctrl.desired.name}. "
          "Commands: run stop none reassert release <serial> request <serial> <type> table quit")

    t0 = time.monotonic()
    next_table = 0.0
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            now = time.monotonic()
            ctrl.tick(now)
            try:
                line = cmds.get_nowait()
            except queue.Empty:
                line = None
            if line is not None:
                if not _handle_command(ctrl, line):
                    break
            if now >= next_table and args.table_period > 0:
                _print_table(ctrl.table(), clear=not args.no_clear)
                next_table = now + args.table_period
            time.sleep(TICK_S)
    finally:
        bus.close()
    return 0


def _handle_command(ctrl: Controller, line: str) -> bool:
    parts = line.split()
    if not parts:
        return True
    cmd, rest = parts[0].lower(), parts[1:]
    try:
        if cmd in ("run", "stop", "none"):
            ctrl.set_desired(_parse_desired(cmd))
        elif cmd == "reassert":
            print(f"re-asserted RUN on {ctrl.reassert_run()} batteries")
        elif cmd == "release":
            ctrl.release(int(rest[0], 16))
        elif cmd == "request":
            ok = ctrl.request(int(rest[0], 16), int(rest[1], 0), int(rest[2], 0) if len(rest) > 2 else 0xFF)
            print("requested" if ok else "unknown battery / not assigned")
        elif cmd == "table":
            _print_table(ctrl.table(), clear=False)
        elif cmd in ("quit", "exit", "q"):
            return False
        else:
            print("commands: run stop none reassert release <serial> request <serial> <type> [page] table quit")
    except (IndexError, ValueError, KeyError) as e:
        print(f"bad command: {e}")
    return True


# ------------------------------------------------------------------ simulate


def cmd_simulate(args: argparse.Namespace) -> int:
    """Put simulated batteries on a real bus (e.g. to test Battery Emulator)."""
    bus = PythonCanBus(args.bus)
    n = min(args.batteries, len(DEMO_BOARD_IDS))
    t0 = time.monotonic()
    sims = [BatterySim(bus, DEMO_BOARD_IDS[i], name=f"bat{i}", soc=args.soc, n_cells=args.cells,
                       open_timeout=args.open_timeout, now=0.0) for i in range(n)]
    for s in sims:
        s.set_load(args.load)
    print(f"{n} simulated batteries on {args.bus}: " + ", ".join(f"{s.name}={s.serial:08X}" for s in sims))
    print("Note: all simulated batteries share one receive queue here, so frames are dispatched to each in turn.")
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            now = time.monotonic() - t0
            # One physical socket: fan frames out to every simulated battery
            frames = []
            while True:
                f = bus.recv()
                if f is None:
                    break
                frames.append(f)
            for s in sims:
                s.bus = _Replay(frames, bus)
                s.tick(now)
            time.sleep(TICK_S)
    finally:
        bus.close()
    return 0


class _Replay:
    """Presents a captured list of frames as a bus to one simulated battery
    while forwarding its transmissions to the real bus."""

    def __init__(self, frames, real):
        self._frames = list(frames)
        self._real = real

    def recv(self):
        return self._frames.pop(0) if self._frames else None

    def send(self, can_id, data):
        self._real.send(can_id, data)


# --------------------------------------------------------------------- sniff


def cmd_sniff(args: argparse.Namespace) -> int:
    """Decode and print every protocol frame seen on the bus."""
    bus = PythonCanBus(args.bus)
    t0 = time.monotonic()
    try:
        while args.duration <= 0 or time.monotonic() - t0 < args.duration:
            f = bus.recv()
            if f is None:
                time.sleep(0.002)
                continue
            print(describe_frame(f.can_id, f.data))
    finally:
        bus.close()
    return 0


def describe_frame(can_id: int, data: bytes) -> str:
    can_id &= P.ID_MASK
    hexdata = data.hex(" ")
    if P.is_announce_id(can_id):
        return f"{can_id:08X} ANNOUNCE {P.Announce.decode(data)}  [{hexdata}]"
    if can_id == P.ID_ASSIGN:
        return f"{can_id:08X} ASSIGN {P.Assign.decode(data)}  [{hexdata}]"
    prefix, t, node = P.id_prefix(can_id), P.id_type(can_id), P.id_node(can_id)
    head = f"{can_id:08X} p=0x{prefix:04X} n={node:<3}"
    if t == P.MsgType.COMMAND:
        return f"{head} COMMAND {P.Command.decode(data)}"
    if t == P.MsgType.REQUEST:
        return f"{head} REQUEST {P.Request.decode(data)}"
    frame = P.decode_battery_frame(t, data) if len(data) >= 8 else None
    name = P.MsgType(t).name if t in P.MsgType._value2member_map_ else f"type 0x{t:02X}"
    return f"{head} {name} {frame if frame is not None else ''}  [{hexdata}]"


# ---------------------------------------------------------------------- main


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m tools.custom_can", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-v", "--verbose", action="count", default=1, help="-v info (default), -vv debug")
    ap.add_argument("-q", "--quiet", action="store_true", help="warnings only")
    sub = ap.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("demo", help=cmd_demo.__doc__.splitlines()[0])
    d.add_argument("--batteries", type=int, default=3)
    d.add_argument("--prefix", type=lambda s: int(s, 0), default=0x0300)
    d.add_argument("--speed", type=float, default=4.0, help="bus-time multiplier; 0 = as fast as possible")
    d.add_argument("--duration", type=float, default=54.0, help="bus seconds to simulate")
    d.add_argument("--drop", type=float, default=0.0, help="random frame loss fraction, e.g. 0.1")
    d.add_argument("--open-timeout", type=float, default=30.0, help="simulated contactor open timeout")
    d.add_argument("--table-period", type=float, default=2.0)
    d.set_defaults(func=cmd_demo)

    c = sub.add_parser("controller", help="run a live controller on a real bus")
    c.add_argument("--bus", required=True, help="python-can spec, e.g. socketcan:can0, slcan:COM3")
    c.add_argument("--prefix", type=lambda s: int(s, 0), default=0x0300)
    c.add_argument("--desired", default="none", choices=["none", "run", "stop"])
    c.add_argument("--map", help="JSON file remembering serial -> node")
    c.add_argument("--stop-on-timeout", action="store_true")
    c.add_argument("--rate-shift", type=int, default=0, choices=[0, 1, 2, 3])
    c.add_argument("--duration", type=float, default=0, help="seconds, 0 = until quit")
    c.add_argument("--table-period", type=float, default=1.0, help="0 = no periodic table")
    c.add_argument("--no-clear", action="store_true")
    c.set_defaults(func=cmd_controller)

    s = sub.add_parser("simulate", help="put simulated batteries on a real bus")
    s.add_argument("--bus", required=True)
    s.add_argument("--batteries", type=int, default=1)
    s.add_argument("--cells", type=int, default=96)
    s.add_argument("--soc", type=float, default=0.6)
    s.add_argument("--load", type=float, default=0.0, help="amps drawn once contactors close (+ charging)")
    s.add_argument("--open-timeout", type=float, default=30.0)
    s.add_argument("--duration", type=float, default=0)
    s.set_defaults(func=cmd_simulate)

    n = sub.add_parser("sniff", help="decode every protocol frame on the bus")
    n.add_argument("--bus", required=True)
    n.add_argument("--duration", type=float, default=0)
    n.set_defaults(func=cmd_sniff)

    args = ap.parse_args(argv)
    _setup_logging(0 if args.quiet else args.verbose)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
