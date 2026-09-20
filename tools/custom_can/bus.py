"""Bus abstraction: an in-process loopback hub for demos/tests, and a thin
wrapper around python-can for real hardware.

Both expose the same two calls, so the controller and the battery simulator
never know which one they are talking to:

    bus.send(can_id, data)      # 29-bit id, payload bytes
    bus.recv() -> Frame | None  # non-blocking
"""

from __future__ import annotations

import collections
import random
import threading
from dataclasses import dataclass, field
from typing import Deque, List, Optional


@dataclass
class Frame:
    can_id: int
    data: bytes
    timestamp: float = 0.0
    origin: str = ""


class LoopbackHub:
    """Connects any number of endpoints. A frame sent on one endpoint is
    delivered to every *other* endpoint, like a real bus (no self-reception).

    ``drop_rate`` randomly discards frames to exercise the protocol's
    tolerance of lost frames. Delivery is in-order and instantaneous; CAN
    arbitration is not modelled.
    """

    def __init__(self, drop_rate: float = 0.0, seed: int = 0):
        self.endpoints: List["LoopbackBus"] = []
        self.drop_rate = drop_rate
        self._rng = random.Random(seed)
        self.frames_sent = 0
        self.frames_dropped = 0
        self.log: Optional[List[Frame]] = None  # set to [] to record traffic

    def endpoint(self, name: str = "") -> "LoopbackBus":
        ep = LoopbackBus(self, name)
        self.endpoints.append(ep)
        return ep

    def remove(self, ep: "LoopbackBus") -> None:
        self.endpoints.remove(ep)

    def _broadcast(self, sender: "LoopbackBus", frame: Frame) -> None:
        self.frames_sent += 1
        if self.log is not None:
            self.log.append(frame)
        if self.drop_rate and self._rng.random() < self.drop_rate:
            self.frames_dropped += 1
            return
        for ep in self.endpoints:
            if ep is not sender:
                ep._queue.append(frame)


class LoopbackBus:
    def __init__(self, hub: LoopbackHub, name: str = ""):
        self.hub = hub
        self.name = name
        self.now = 0.0
        self._queue: Deque[Frame] = collections.deque()

    def send(self, can_id: int, data: bytes) -> None:
        self.hub._broadcast(self, Frame(can_id & 0x1FFFFFFF, bytes(data), self.now, self.name))

    def recv(self) -> Optional[Frame]:
        return self._queue.popleft() if self._queue else None

    def close(self) -> None:
        self.hub.remove(self)


class PythonCanBus:
    """python-can backed bus. Only 29-bit data frames are passed through.

    ``spec`` is ``interface:channel[:bitrate]``, e.g. ``socketcan:can0``,
    ``slcan:COM3``, ``slcan:/dev/ttyACM0:500000``, ``pcan:PCAN_USBBUS1``,
    ``virtual:demo`` (python-can's in-process virtual bus).
    """

    def __init__(self, spec: str, bitrate: int = 500000):
        try:
            import can  # type: ignore
        except ImportError as e:  # pragma: no cover - depends on environment
            raise SystemExit("python-can is required for real buses: pip install python-can") from e
        parts = spec.split(":")
        if len(parts) < 2:
            raise ValueError("bus spec must be interface:channel[:bitrate]")
        interface, channel = parts[0], parts[1]
        if len(parts) > 2:
            bitrate = int(parts[2])
        kwargs = {"interface": interface, "channel": channel}
        if interface not in ("virtual", "socketcan"):
            kwargs["bitrate"] = bitrate
        self._can = can
        self._bus = can.Bus(**kwargs)
        self._lock = threading.Lock()

    def send(self, can_id: int, data: bytes) -> None:
        msg = self._can.Message(arbitration_id=can_id & 0x1FFFFFFF, is_extended_id=True, data=bytes(data))
        with self._lock:
            try:
                self._bus.send(msg, timeout=0.05)
            except self._can.CanError as e:  # pragma: no cover - hardware dependent
                print(f"CAN send failed: {e}")

    def recv(self) -> Optional[Frame]:
        msg = self._bus.recv(timeout=0.0)
        while msg is not None:
            if msg.is_extended_id and not msg.is_remote_frame and not msg.is_error_frame:
                return Frame(msg.arbitration_id, bytes(msg.data), msg.timestamp)
            msg = self._bus.recv(timeout=0.0)
        return None

    def close(self) -> None:
        self._bus.shutdown()


def open_bus(spec: str, hub: Optional[LoopbackHub] = None, name: str = ""):
    """``loopback`` (needs a hub) or a python-can spec."""
    if spec == "loopback":
        if hub is None:
            raise ValueError("loopback bus needs a LoopbackHub")
        return hub.endpoint(name)
    return PythonCanBus(spec)
