"""Radar wireless payload sample constraints used by Lab TX.

Wire lengths and field layouts come from ``protocol2027``. Existing numeric
range presets are retained for lab compatibility and are not certified as
2027 match limits until the rulebook values are reviewed.
"""

from __future__ import annotations

import random
from typing import Any, Iterable, Sequence

from .protocol2027 import (
    RADAR_BROADCAST_COMMAND_IDS,
    RADAR_COMMAND_DATA_LENGTHS,
    RADAR_MACRO_STATE_1_FIELDS,
    RADAR_MACRO_STATE_2_FIELDS,
    RADAR_HP_SLOT_NAMES,
    RADAR_BULLET_SLOT_NAMES,
    RADAR_POSITION_SLOT_NAMES,
)


POSITION_X_MAX_CM = 2800
POSITION_Y_MAX_CM = 1500

ROBOT_KEYS = tuple(name for name in RADAR_POSITION_SLOT_NAMES if not name.startswith("reserved_"))
HP_KEYS = RADAR_HP_SLOT_NAMES
BULLET_KEYS = RADAR_BULLET_SLOT_NAMES
BUFF_ROBOT_KEYS = (
    "opponent_heavy",
    "reserved_robot_2",
    "opponent_infantry_3",
    "opponent_infantry_4",
    "opponent_sentry",
)

# Legacy 2026 sample ranges below are retained for existing lab controls and
# are not certified 2027 limits. This migration updates wire layout only.
HP_MAX = {
    "opponent_heavy": 600,
    "reserved_robot_2": 0,
    "opponent_infantry_3": 400,
    "opponent_infantry_4": 400,
    "reserved_robot_6": 0,
    "opponent_sentry": 400,
}

# Theoretical per-robot maxima from the ammunition acquisition rules.  The
# infantry values include the fortress's up-to-500 reserve as specified by
# 0x0A03.  The sentry value includes 300 initial, 1000 team exchange and six
# 100-round supply-zone grants.
BULLET_MAX = {
    "opponent_heavy": 100,
    "opponent_infantry_3": 1500,
    "opponent_infantry_4": 1500,
    "reserved_robot_6": 0,
    "opponent_sentry": 1900,
}

# Field caps are legacy sample values; the reserved former-engineer block is
# zeroed to preserve the 2027 0x0A05 slot layout.
BUFF_FIELD_MAX = {
    "hp_recovery_percent": {
        key: 0 if key == "reserved_robot_2" else 25 for key in BUFF_ROBOT_KEYS
    },
    "shooting_heat_cooling": {
        "opponent_heavy": 200,
        "reserved_robot_2": 0,
        "opponent_infantry_3": 120,
        "opponent_infantry_4": 120,
        "opponent_sentry": 120,
    },
    "defense_percent": {
        key: 0 if key == "reserved_robot_2" else 99 for key in BUFF_ROBOT_KEYS
    },
    "negative_defense_percent": {
        key: 0 if key == "reserved_robot_2" else 100 for key in BUFF_ROBOT_KEYS
    },
    "attack_percent": {
        key: 0 if key == "reserved_robot_2" else 300 for key in BUFF_ROBOT_KEYS
    },
}

RADAR_BROADCAST_COMMANDS = tuple(
    f"0x{cmd_id:04X}" for cmd_id in RADAR_BROADCAST_COMMAND_IDS
)


def clamp_int(value: Any, low: int, high: int, default: int = 0) -> int:
    try:
        parsed = int(value, 0) if isinstance(value, str) else int(value)
    except (TypeError, ValueError):
        parsed = int(default)
    return min(max(parsed, int(low)), int(high))


def normalize_occupation_bits(value: Any) -> int:
    """Normalize the 2027 0x0A04 occupation_state_1 word."""
    raw = clamp_int(value, 0, 0xFFFFFFFF)
    normalized = 0
    for _name, offset, width, kind, maximum in RADAR_MACRO_STATE_1_FIELDS:
        field_max = 1 if kind == "bool" else maximum
        field_value = min((raw >> offset) & ((1 << width) - 1), field_max)
        normalized |= field_value << offset
    return normalized


def normalize_occupation_state_2(value: Any) -> int:
    """Normalize 2027 terrain-detection and control bits; clear reserved bits."""
    raw = clamp_int(value, 0, 0xFFFFFFFF)
    normalized = 0
    for _name, offset, width, kind, maximum in RADAR_MACRO_STATE_2_FIELDS:
        field_max = 1 if kind == "bool" else maximum
        field_value = min((raw >> offset) & ((1 << width) - 1), field_max)
        normalized |= field_value << offset
    return normalized


def random_occupation_bits(rng: Any = random) -> int:
    bits = 0
    for _name, offset, _width, kind, maximum in RADAR_MACRO_STATE_1_FIELDS:
        bits |= rng.randint(0, 1 if kind == "bool" else maximum) << offset
    return bits


def random_occupation_state_2(rng: Any = random) -> int:
    bits = 0
    for _name, offset, _width, kind, maximum in RADAR_MACRO_STATE_2_FIELDS:
        bits |= rng.randint(0, 1 if kind == "bool" else maximum) << offset
    return bits


def command_id_value(cmd_id: int | Sequence[int]) -> int:
    if isinstance(cmd_id, int):
        return int(cmd_id) & 0xFFFF
    values = [int(item) & 0xFF for item in cmd_id]
    if len(values) != 2:
        raise ValueError("cmd_id must contain exactly 2 bytes")
    # The flowgraph API represents 0x0A01 as [0x0A, 0x01].
    return (values[0] << 8) | values[1]


def _put_u16(value: int) -> list[int]:
    return [value & 0xFF, (value >> 8) & 0xFF]


def _put_u32(value: int) -> list[int]:
    return [value & 0xFF, (value >> 8) & 0xFF, (value >> 16) & 0xFF, (value >> 24) & 0xFF]


def random_payload_for_command(
    cmd_id: int | Sequence[int],
    size: int,
    rng: Any = random,
) -> list[int]:
    """Generate a schema-shaped sample using the currently configured limits."""
    command = command_id_value(cmd_id)
    expected_size = RADAR_COMMAND_DATA_LENGTHS.get(command)
    if expected_size is not None and int(size) != expected_size:
        raise ValueError(
            f"0x{command:04X} payload must be {expected_size} bytes, got {int(size)}"
        )

    out: list[int] = []
    if command == 0x0A01:
        for key in RADAR_POSITION_SLOT_NAMES:
            x = rng.randint(0, POSITION_X_MAX_CM) if not key.startswith("reserved_") else 0
            y = rng.randint(0, POSITION_Y_MAX_CM) if not key.startswith("reserved_") else 0
            out.extend(_put_u16(x))
            out.extend(_put_u16(y))
    elif command == 0x0A02:
        for key in HP_KEYS:
            out.extend(_put_u16(rng.randint(0, HP_MAX[key])))
    elif command == 0x0A03:
        for key in BULLET_KEYS:
            out.extend(_put_u16(rng.randint(0, BULLET_MAX[key])))
    elif command == 0x0A04:
        total = rng.randint(0, 0xFFFF)
        remaining = rng.randint(0, total)
        out.extend(_put_u16(remaining))
        out.extend(_put_u16(total))
        out.extend(_put_u32(random_occupation_bits(rng)))
        out.extend(_put_u32(random_occupation_state_2(rng)))
    elif command == 0x0A05:
        for key in BUFF_ROBOT_KEYS:
            out.append(rng.randint(0, BUFF_FIELD_MAX["hp_recovery_percent"][key]))
            out.extend(_put_u16(rng.randint(0, BUFF_FIELD_MAX["shooting_heat_cooling"][key])))
            out.append(rng.randint(0, BUFF_FIELD_MAX["defense_percent"][key]))
            out.append(rng.randint(0, BUFF_FIELD_MAX["negative_defense_percent"][key]))
            out.extend(_put_u16(rng.randint(0, BUFF_FIELD_MAX["attack_percent"][key])))
        out.append(rng.randint(1, 6))
        out.extend(
            0 if key == "reserved_robot_2" else rng.randint(0, 3)
            for key in BUFF_ROBOT_KEYS
        )
        out.extend(bytes(6))
    else:
        return [rng.getrandbits(8) for _ in range(max(0, int(size)))]

    if len(out) != int(size):
        raise AssertionError(f"0x{command:04X} generated {len(out)} bytes, expected {int(size)}")
    return out


def valid_password_bytes(values: Iterable[int]) -> bool:
    data = bytes(int(value) & 0xFF for value in values)
    return len(data) == 6 and all(
        ord("0") <= value <= ord("9")
        or ord("A") <= value <= ord("Z")
        or ord("a") <= value <= ord("z")
        for value in data
    )
