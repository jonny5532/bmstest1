"""Wire format of the custom multi-battery CAN protocol.

This mirrors ``bms/protocols/inverter/custom_can.h`` (the authoritative
definition) and ``docs/custom_can_protocol.md``. Keep the three in sync.

Everything here is pure data: identifier helpers, the serial hash, and one
dataclass per frame with ``encode()``/``decode()``. Nothing in this module
touches a bus or a clock.
"""

from __future__ import annotations

import re
import struct
from dataclasses import dataclass, fields
from enum import IntEnum, IntFlag
from pathlib import Path
from typing import ClassVar, Dict, List, Optional, Tuple, Type

PROTOCOL_VERSION = 1

# --------------------------------------------------------------------------
# Identifier layout: prefix(15) | type(6) | node(8)
# --------------------------------------------------------------------------

ID_MASK = 0x1FFFFFFF
PREFIX_SHIFT = 14
PREFIX_MAX = 0x7FFF
TYPE_SHIFT = 8
TYPE_MASK = 0x3F
NODE_MASK = 0xFF

NODE_NONE = 0x00
NODE_ALL = 0xFF
NODE_MIN = 0x01
NODE_MAX = 0xFE

DISCOVERY_PREFIX = 0x7384
ID_ANNOUNCE_BASE = 0x1CE10000  # | (serial32 & 0xFF)
ID_ANNOUNCE_MASK = 0x1FFFFF00
ID_ASSIGN = 0x1CE10100

INT16_NA = 0x7FFF

# Timing (seconds)
ANNOUNCE_PERIOD_UNASSIGNED = 1.0
ANNOUNCE_PERIOD_ASSIGNED = 5.0
STATUS_PERIOD = 0.2
CONTROLLER_TIMEOUT = 5.0
COMMAND_MAX_PERIOD = 1.0
COMMAND_RECOMMENDED_PERIOD = 0.5
BATTERY_TIMEOUT = 2.0


def make_id(prefix: int, msg_type: int, node: int) -> int:
    return ((prefix & PREFIX_MAX) << PREFIX_SHIFT) | ((msg_type & TYPE_MASK) << TYPE_SHIFT) | (node & NODE_MASK)


def id_prefix(can_id: int) -> int:
    return (can_id & ID_MASK) >> PREFIX_SHIFT


def id_type(can_id: int) -> int:
    return (can_id >> TYPE_SHIFT) & TYPE_MASK


def id_node(can_id: int) -> int:
    return can_id & NODE_MASK


def is_announce_id(can_id: int) -> bool:
    return (can_id & ID_ANNOUNCE_MASK) == ID_ANNOUNCE_BASE


def announce_id(serial32: int) -> int:
    return ID_ANNOUNCE_BASE | (serial32 & 0xFF)


def serial32(board_id: bytes) -> int:
    """32-bit FNV-1a of the 64-bit board id, as in custom_can_serial32()."""
    h = 0x811C9DC5
    for b in board_id:
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


# --------------------------------------------------------------------------
# Enumerations
# --------------------------------------------------------------------------


class MsgType(IntEnum):
    # controller -> battery
    COMMAND = 0x00
    REQUEST = 0x01
    # battery -> controller
    STATUS = 0x10
    LIMITS = 0x11
    MEASUREMENTS = 0x12
    ENERGY = 0x13
    CELL_STATS = 0x14
    TEMPERATURES = 0x15
    HV_VOLTAGES = 0x16
    SUPPLY = 0x17
    BALANCING = 0x18
    EVENT = 0x19
    ESTIMATORS = 0x1A
    CONFIG = 0x1B
    SERIAL = 0x1C
    SETTINGS = 0x1D
    CELL_LIMITS = 0x1E
    MODULE_TEMPS = 0x20
    CELL_VOLTAGES = 0x21


CONTROLLER_TYPE_MAX = 0x0F


def is_controller_type(msg_type: int) -> bool:
    return msg_type <= CONTROLLER_TYPE_MAX


class LinkState(IntEnum):
    UNASSIGNED = 0
    ASSIGNED = 1


class Desired(IntEnum):
    NONE = 0
    STOP = 1
    RUN = 2


class AssignFlag(IntFlag):
    STOP_ON_TIMEOUT = 0x01


ASSIGN_RATE_SHIFT_POS = 4
ASSIGN_RATE_SHIFT_MASK = 0x30


class StatusFlag(IntFlag):
    CURRENT_ENABLED = 0x01
    BALANCING = 0x02
    ESTOP = 0x04
    OPERATING = 0x08
    COMMAND_PENDING = 0x10
    SLOW_MODE = 0x20
    PRECHARGE_CLOSED = 0x40


class SystemState(IntEnum):
    UNINITIALIZED = 0
    INITIALIZING = 1
    CALIBRATING = 2
    INACTIVE = 3
    OPERATING = 4
    FAULT = 5


class ContactorState(IntEnum):
    OPEN = 0
    TESTING_PRE_CLOSED = 1
    TESTING_NEG_OPEN = 2
    TESTING_NEG_CLOSED = 3
    TESTING_POS_OPEN = 4
    TESTING_POS_CLOSED = 5
    PRECHARGING_INIT = 6
    PRECHARGING = 7
    CLOSED = 8
    AWAITING_OPEN = 9
    CALIBRATING = 10
    CALIBRATING_PRECHARGE_INIT = 11
    CALIBRATING_PRECHARGE = 12
    CALIBRATING_CLOSED = 13
    CALIBRATING_ONLY_NEG_INIT = 14
    CALIBRATING_ONLY_NEG = 15
    PRECHARGE_FAILED = 16
    TESTING_FAILED = 17


class EventLevel(IntEnum):
    NONE = 0
    INFO = 1
    WARNING = 2
    CRITICAL = 3
    FATAL = 4


class Chemistry(IntEnum):
    LFP = 1
    NMC = 2


def _name(enum_cls, value: int) -> str:
    try:
        return enum_cls(value).name
    except ValueError:
        return f"{enum_cls.__name__}({value})"


# --------------------------------------------------------------------------
# Event names (index -> name), read from the firmware source when available
# --------------------------------------------------------------------------

_FALLBACK_EVENT_NAMES = [
    "CONTACTOR_POS_STUCK_OPEN", "CONTACTOR_POS_STUCK_CLOSED", "CONTACTOR_NEG_STUCK_OPEN",
    "CONTACTOR_NEG_STUCK_CLOSED", "CONTACTOR_PRE_STUCK_OPEN", "CONTACTOR_PRE_STUCK_CLOSED",
    "CONTACTOR_PRECHARGE_VOLTAGE_TOO_HIGH", "CONTACTOR_PRECHARGE_CURRENT_TOO_HIGH",
    "CONTACTOR_POS_UNEXPECTED_OPEN", "CONTACTOR_NEG_UNEXPECTED_OPEN", "CONTACTOR_CLOSING_FAILED",
    "SUPPLY_VOLTAGE_STALE", "BATTERY_VOLTAGE_STALE", "BATTERY_TEMPERATURE_STALE", "CURRENT_STALE",
    "CELL_VOLTAGES_STALE", "SUPPLY_VOLTAGE_3V3_LOW", "SUPPLY_VOLTAGE_3V3_HIGH", "SUPPLY_VOLTAGE_5V_LOW",
    "SUPPLY_VOLTAGE_5V_HIGH", "SUPPLY_VOLTAGE_12V_LOW", "SUPPLY_VOLTAGE_12V_HIGH",
    "SUPPLY_VOLTAGE_CONTACTOR_LOW", "SUPPLY_VOLTAGE_CONTACTOR_VERY_LOW", "SUPPLY_VOLTAGE_CONTACTOR_HIGH",
    "BATTERY_VOLTAGE_HIGH", "BATTERY_VOLTAGE_VERY_HIGH", "BATTERY_VOLTAGE_LOW", "BATTERY_VOLTAGE_VERY_LOW",
    "CELL_VOLTAGE_HIGH", "CELL_VOLTAGE_VERY_HIGH", "CELL_VOLTAGE_LOW", "CELL_VOLTAGE_VERY_LOW",
    "SOFT_CHARGE_BUFFER_EXCEEDED", "OVERCURRENT_CHARGING", "OVERCURRENT_DISCHARGING",
    "BATTERY_TEMPERATURE_HIGH", "BATTERY_TEMPERATURE_VERY_HIGH", "BATTERY_TEMPERATURE_LOW",
    "BATTERY_TEMPERATURE_VERY_LOW", "VOLTAGE_MISMATCH", "BMB_READ_ERROR", "BMB_CRC_MISMATCH",
    "CELL_VOLTAGE_GLITCH", "MODULE_TEMPERATURE_GLITCH", "INVERTER_DETECTED", "ESTOP_PRESSED",
    "BOOT_NORMAL", "BOOT_WATCHDOG", "LOOP_OVERRUN", "RESTARTING",
    "CAN_SLAVE_LOST", "CAN_SLAVE_FAULT",
]


def load_event_names(events_h: Optional[Path] = None) -> List[str]:
    """Parse EVENT_TYPES(X) from events.h; fall back to the embedded copy."""
    if events_h is None:
        events_h = Path(__file__).resolve().parents[2] / "bms" / "sys" / "events" / "events.h"
    try:
        text = events_h.read_text()
    except OSError:
        return list(_FALLBACK_EVENT_NAMES)
    m = re.search(r"#define EVENT_TYPES\(X\)(.*?)\n\s*\n", text, re.S)
    if not m:
        return list(_FALLBACK_EVENT_NAMES)
    names = re.findall(r"^\s*X\((\w+),", m.group(1), re.M)
    return names or list(_FALLBACK_EVENT_NAMES)


EVENT_NAMES: List[str] = load_event_names()
EVENT_INVERTER_DETECTED = EVENT_NAMES.index("INVERTER_DETECTED") if "INVERTER_DETECTED" in EVENT_NAMES else 45


def event_name(index: int) -> str:
    if index == 0xFF:
        return "-"
    return EVENT_NAMES[index] if index < len(EVENT_NAMES) else f"EVENT_{index}"


# --------------------------------------------------------------------------
# Frames
# --------------------------------------------------------------------------


def i16_or_none(v: int) -> Optional[int]:
    return None if v == INT16_NA else v


class _Struct:
    """Fixed-layout frame: dataclass fields map 1:1 onto a struct format."""

    FMT: ClassVar[str]
    TYPE: ClassVar[int]

    def encode(self) -> bytes:
        return struct.pack(self.FMT, *[getattr(self, f.name) for f in fields(self)])  # type: ignore[arg-type]

    @classmethod
    def decode(cls, data: bytes):
        size = struct.calcsize(cls.FMT)
        if len(data) < size:
            raise ValueError(f"{cls.__name__}: need {size} bytes, got {len(data)}")
        return cls(*struct.unpack(cls.FMT, bytes(data[:size])))


# --- discovery -----------------------------------------------------------


@dataclass
class Announce(_Struct):
    FMT: ClassVar[str] = ">BBIH"
    TYPE: ClassVar[int] = -1
    version_state: int
    node: int
    serial: int
    prefix: int

    @classmethod
    def build(cls, serial: int, link: LinkState, node: int = 0, prefix: int = 0) -> "Announce":
        return cls((PROTOCOL_VERSION << 4) | int(link), node, serial, prefix)

    @property
    def version(self) -> int:
        return self.version_state >> 4

    @property
    def link(self) -> LinkState:
        return LinkState.ASSIGNED if (self.version_state & 0x0F) == LinkState.ASSIGNED else LinkState.UNASSIGNED


@dataclass
class Assign(_Struct):
    FMT: ClassVar[str] = ">IHBB"
    TYPE: ClassVar[int] = -1
    serial: int
    prefix: int
    node: int
    flags: int = 0

    @property
    def rate_shift(self) -> int:
        return (self.flags & ASSIGN_RATE_SHIFT_MASK) >> ASSIGN_RATE_SHIFT_POS


# --- controller -> battery -----------------------------------------------


@dataclass
class Command(_Struct):
    FMT: ClassVar[str] = ">BBHI"
    TYPE: ClassVar[int] = MsgType.COMMAND
    desired: int
    flags: int = 0
    seq: int = 0
    reserved: int = 0

    @classmethod
    def decode(cls, data: bytes) -> "Command":
        # DLC >= 4 is legal; pad the reserved tail
        return super().decode(bytes(data) + b"\0" * 4)


@dataclass
class Request:
    TYPE: ClassVar[int] = MsgType.REQUEST
    msg_type: int
    page: int = 0xFF

    def encode(self) -> bytes:
        return bytes([self.msg_type, self.page])

    @classmethod
    def decode(cls, data: bytes) -> "Request":
        return cls(data[0], data[1] if len(data) > 1 else 0xFF)


# --- battery -> controller -----------------------------------------------


@dataclass
class Status(_Struct):
    FMT: ClassVar[str] = ">BBBBHBB"
    TYPE: ClassVar[int] = MsgType.STATUS
    system_state: int
    contactor_state: int
    flags: int
    level_cmd: int
    last_seq: int
    top_event: int = 0xFF
    reserved: int = 0

    @classmethod
    def build(cls, system_state: int, contactor_state: int, flags: int, level: int,
              commanded: int, last_seq: int, top_event: int = 0xFF) -> "Status":
        return cls(system_state, contactor_state, flags, (level & 0x07) | ((commanded & 0x03) << 4), last_seq, top_event)

    @property
    def level(self) -> int:
        return self.level_cmd & 0x07

    @property
    def commanded(self) -> int:
        return (self.level_cmd >> 4) & 0x03

    def has(self, flag: StatusFlag) -> bool:
        return bool(self.flags & flag)

    def __str__(self) -> str:
        flags = ",".join(f.name for f in StatusFlag if self.flags & f) or "-"  # type: ignore[union-attr]
        return (f"sys={_name(SystemState, self.system_state)} ctr={_name(ContactorState, self.contactor_state)} "
                f"flags={flags} level={_name(EventLevel, self.level)} cmd={_name(Desired, self.commanded)} "
                f"seq={self.last_seq} top={event_name(self.top_event)}")


@dataclass
class Limits(_Struct):
    FMT: ClassVar[str] = ">hhHH"
    TYPE: ClassVar[int] = MsgType.LIMITS
    max_voltage_dV: int
    min_voltage_dV: int
    charge_dA: int
    discharge_dA: int

    def __str__(self) -> str:
        return (f"V {self.min_voltage_dV / 10:.1f}..{self.max_voltage_dV / 10:.1f} "
                f"chg {self.charge_dA / 10:.1f}A dis {self.discharge_dA / 10:.1f}A")


@dataclass
class Measurements(_Struct):
    FMT: ClassVar[str] = ">HiH"
    TYPE: ClassVar[int] = MsgType.MEASUREMENTS
    voltage_cV: int
    current_mA: int
    soc_cpct: int

    def __str__(self) -> str:
        return f"{self.voltage_cV / 100:.2f}V {self.current_mA / 1000:+.2f}A SoC {self.soc_cpct / 100:.1f}%"


@dataclass
class Energy(_Struct):
    FMT: ClassVar[str] = ">HHHh"
    TYPE: ClassVar[int] = MsgType.ENERGY
    soh_cpct: int
    remaining_dAh: int
    full_dAh: int
    charge_used_dAh: int


@dataclass
class CellStats(_Struct):
    FMT: ClassVar[str] = ">hhhBB"
    TYPE: ClassVar[int] = MsgType.CELL_STATS
    min_mV: int
    max_mV: int
    mean_mV: int
    idx_min: int
    idx_max: int


@dataclass
class Temperatures(_Struct):
    FMT: ClassVar[str] = ">hhBBH"
    TYPE: ClassVar[int] = MsgType.TEMPERATURES
    min_dC: int
    max_dC: int
    idx_min: int
    idx_max: int
    reserved: int = 0


@dataclass
class HvVoltages(_Struct):
    FMT: ClassVar[str] = ">Hhhh"
    TYPE: ClassVar[int] = MsgType.HV_VOLTAGES
    output_cV: int
    pos_contactor_dV: int
    neg_contactor_dV: int
    delta_dV: int


@dataclass
class Supply(_Struct):
    FMT: ClassVar[str] = ">HHHH"
    TYPE: ClassVar[int] = MsgType.SUPPLY
    v3v3_mV: int
    v5_mV: int
    v12_mV: int
    contactor_mV: int


@dataclass
class Balancing(_Struct):
    FMT: ClassVar[str] = ">BBhHBB"
    TYPE: ClassVar[int] = MsgType.BALANCING
    state: int
    cells: int
    max_remaining_periods: int
    pause_counter: int
    flags: int = 0
    reserved: int = 0


@dataclass
class Event(_Struct):
    FMT: ClassVar[str] = ">BBHHBB"
    TYPE: ClassVar[int] = MsgType.EVENT
    event_type: int
    level: int
    count: int
    age_s: int
    types_defined: int
    active: int

    @property
    def empty(self) -> bool:
        return self.event_type == 0xFF

    def __str__(self) -> str:
        if self.empty:
            return f"no events recorded (firmware defines {self.types_defined})"
        return f"{event_name(self.event_type)} {_name(EventLevel, self.level)} x{self.count} {self.age_s}s ago"


@dataclass
class Estimators(_Struct):
    FMT: ClassVar[str] = ">HHHH"
    TYPE: ClassVar[int] = MsgType.ESTIMATORS
    voltage_based_cpct: int
    basic_count_cpct: int
    fancy_count_cpct: int
    ekf_cpct: int


@dataclass
class Config(_Struct):
    FMT: ClassVar[str] = ">BBBBHH"
    TYPE: ClassVar[int] = MsgType.CONFIG
    version: int
    chemistry: int
    num_cells: int
    num_temps: int
    nameplate_dAh: int
    working_dAh: int


@dataclass
class Serial(_Struct):
    FMT: ClassVar[str] = ">8s"
    TYPE: ClassVar[int] = MsgType.SERIAL
    board_id: bytes

    @property
    def serial32(self) -> int:
        return serial32(self.board_id)


@dataclass
class Settings(_Struct):
    FMT: ClassVar[str] = ">HHHH"
    TYPE: ClassVar[int] = MsgType.SETTINGS
    user_charge_dA: int
    user_discharge_dA: int
    working_min_mV: int
    working_max_mV: int


@dataclass
class CellLimits(_Struct):
    FMT: ClassVar[str] = ">HHHH"
    TYPE: ClassVar[int] = MsgType.CELL_LIMITS
    soft_min_mV: int
    soft_max_mV: int
    hard_min_mV: int
    hard_max_mV: int


@dataclass
class ModuleTempsPage:
    TYPE: ClassVar[int] = MsgType.MODULE_TEMPS
    first: int
    total: int
    values_dC: Tuple[int, int, int]

    def encode(self) -> bytes:
        return struct.pack(">BBhhh", self.first, self.total, *self.values_dC)

    @classmethod
    def decode(cls, data: bytes) -> "ModuleTempsPage":
        first, total, a, b, c = struct.unpack(">BBhhh", bytes(data[:8]))
        return cls(first, total, (a, b, c))

    def items(self):
        for i, v in enumerate(self.values_dC):
            if self.first + i < self.total and v != INT16_NA:
                yield self.first + i, v


@dataclass
class CellVoltagesPage:
    TYPE: ClassVar[int] = MsgType.CELL_VOLTAGES
    first: int
    flags: int
    values_mV: Tuple[int, int, int]

    FLAG_DISTURBED: ClassVar[int] = 0x80

    def encode(self) -> bytes:
        return struct.pack(">BBhhh", self.first, self.flags, *self.values_mV)

    @classmethod
    def decode(cls, data: bytes) -> "CellVoltagesPage":
        first, flags, a, b, c = struct.unpack(">BBhhh", bytes(data[:8]))
        return cls(first, flags, (a, b, c))

    @property
    def disturbed(self) -> bool:
        return bool(self.flags & self.FLAG_DISTURBED)

    def balancing(self, i: int) -> bool:
        return bool(self.flags & (1 << i))

    def items(self):
        """Yield (index, mV or None, balancing) for the three slots."""
        for i, v in enumerate(self.values_mV):
            yield self.first + i, i16_or_none(v), self.balancing(i)


BATTERY_FRAMES: Dict[int, Type] = {
    cls.TYPE: cls
    for cls in (Status, Limits, Measurements, Energy, CellStats, Temperatures, HvVoltages, Supply,
                Balancing, Event, Estimators, Config, Serial, Settings, CellLimits,
                ModuleTempsPage, CellVoltagesPage)
}


def decode_battery_frame(msg_type: int, data: bytes):
    """Decode a battery->controller payload, or return None for unknown types."""
    cls = BATTERY_FRAMES.get(msg_type)
    return cls.decode(data) if cls else None
