"""Python reference implementation of the custom multi-battery CAN protocol.

Modules
-------
protocol      wire format (mirrors bms/protocols/inverter/custom_can.h)
bus           loopback hub for demos/tests, python-can wrapper for hardware
controller    the upstream controller ("Battery Emulator" side)
battery_sim   a simulated battery (the BMS side) for demos and tests

Run ``python -m tools.custom_can --help`` from the repository root for the
command line front end (demo, controller, simulate, sniff), and
``python -m unittest tools.custom_can.test_custom_can`` for the tests.

Only the standard library is needed for the loopback demo and the tests;
``pip install python-can`` to talk to a real CAN adapter.
"""

from . import protocol  # noqa: F401
from .bus import Frame, LoopbackBus, LoopbackHub, PythonCanBus, open_bus  # noqa: F401
from .controller import Aggregate, BatteryRecord, Controller, NodeMap  # noqa: F401
from .battery_sim import BatterySim  # noqa: F401
