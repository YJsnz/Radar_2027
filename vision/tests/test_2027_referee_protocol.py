import struct

import pytest

from RM_serial_py.ser_api import (
    Radar_decision,
    build_data_decision,
    build_data_radar_all,
    is_valid_0305_payload,
)


@pytest.mark.parametrize(
    ("side", "first_prefix", "second_prefix"),
    (("R", "B", "R"), ("B", "R", "B")),
)
def test_vision_0305_builder_keeps_reserved_slots_zero(side, first_prefix, second_prefix):
    send_map = {
        f"{prefix}{role}": (
            (100 if prefix == "R" else 200) + int(role),
            (200 if prefix == "R" else 300) + int(role),
        )
        for prefix in ("R", "B")
        for role in ("1", "2", "3", "4", "6", "7")
    }

    payload = build_data_radar_all(send_map, side)
    values = struct.unpack("<24H", payload)

    assert len(payload) == 48

    def slot_values(prefix):
        return tuple(
            coordinate
            for role in ("1", None, "3", "4", None, "7")
            for coordinate in (send_map[f"{prefix}{role}"] if role is not None else (0, 0))
        )

    assert values == slot_values(first_prefix) + slot_values(second_prefix)
    assert all(values[offset:offset + 2] == (0, 0) for offset in (2, 8, 14, 20))
    assert is_valid_0305_payload(payload)

    malformed = bytearray(payload)
    malformed[4] = 1
    assert not is_valid_0305_payload(bytes(malformed))


def test_vision_0121_builder_uses_the_2027_13_byte_user_payload():
    frame_data = build_data_decision(
        2,
        "R",
        password_cmd=1,
        password=[65, 66, 67, 68, 69, 70],
        armor_fortify_cmd=3,
        fortify_center_x_mm=320,
        fortify_center_y_mm=410,
    )

    assert len(frame_data) == 6 + 13
    assert frame_data[:6] == bytes.fromhex("210109008080")
    assert frame_data[6:] == bytes((2, 1, 65, 66, 67, 68, 69, 70, 3)) + struct.pack(
        "<HH", 320, 410
    )


def test_vision_020e_parser_requires_eight_bytes_and_decodes_state_bits():
    assert Radar_decision(bytes((0x37, 0x0B, 1, 2, 3, 4, 5, 6))) == (3, 1, 2, 1)
    with pytest.raises(ValueError, match="8 字节"):
        Radar_decision(bytes(1))
    with pytest.raises(ValueError, match="8 字节"):
        Radar_decision(0x37)
