"""Canonical 2027 referee/radar protocol definitions used by the radio stack.

Values in this file are transcribed from the repository's official 2027
communication protocol PDF. Keep command lengths and field layouts here so
frame validation, decoding, and virtual air-link generation share one source.
This is the subset currently consumed by the radar station, not the full
competition protocol catalog.
"""

from __future__ import annotations


PROTOCOL_VERSION = "RoboMaster 2027 Communication Protocol V1.0.0 (2026-10-29)"

GAME_ROBOT_HP_LENGTH = 24
GAME_ROBOT_HP_FIELD_OFFSETS = {
    "ally_heavy_hp": 0,
    "reserved_robot_2_hp": 2,
    "ally_infantry_3_hp": 4,
    "ally_infantry_4_hp": 6,
    "damage_difference": 8,
    "ally_sentry_hp": 10,
    "red_outpost_hp": 12,
    "ally_base_hp": 14,
    "blue_outpost_hp": 16,
    "enemy_base_hp": 18,
    "ally_control_degree": 20,
    "enemy_control_degree": 22,
}

RADAR_DATA_LENGTHS = {
    0x0001: 11,
    0x0003: GAME_ROBOT_HP_LENGTH,
    0x0105: 3,
    0x0201: 17,
    0x020C: 2,
    0x020E: 8,
    0x0A01: 24,
    0x0A02: 12,
    0x0A03: 10,
    0x0A04: 12,
    0x0A05: 47,
    0x0A06: 6,
}

RADAR_COMMAND_NAMES = {
    0x0001: "game_status",
    0x0003: "game_robot_hp",
    0x0105: "dart_status",
    0x0201: "robot_status",
    0x020C: "radar_mark_progress",
    0x020E: "radar_decision_sync",
    0x0A01: "enemy_position",
    0x0A02: "enemy_hp",
    0x0A03: "enemy_bullet_allowance",
    0x0A04: "enemy_macro_status",
    0x0A05: "enemy_buff_status",
    0x0A06: "interference_password",
}

RADAR_BROADCAST_COMMAND_IDS = (0x0A01, 0x0A02, 0x0A03, 0x0A04, 0x0A05)
RADAR_AIR_DATA_LENGTHS = {
    cmd_id: RADAR_DATA_LENGTHS[cmd_id]
    for cmd_id in range(0x0A01, 0x0A07)
}
RADAR_COMMAND_DATA_LENGTHS = {
    cmd_id: RADAR_DATA_LENGTHS[cmd_id]
    for cmd_id in range(0x0A01, 0x0A07)
}

RADAR_MACRO_DATA_OFFSETS = {
    "remaining_coins": 0,
    "total_coins": 2,
    "occupation_state_1": 4,
    "occupation_state_2": 8,
}

# Each field is (name, bit offset, width, kind, maximum legal enum value).
# Reserved bits are intentionally absent and must remain zero in generated data.
RADAR_MACRO_STATE_1_FIELDS = (
    ("opponent_supply_zone_occupied", 0, 1, "bool", 1),
    ("red_fortress_buff_zone_status", 1, 3, "enum", 4),
    ("blue_fortress_buff_zone_status", 4, 3, "enum", 4),
    ("red_base_buff_zone_status", 7, 3, "enum", 4),
    ("blue_base_buff_zone_status", 10, 3, "enum", 4),
    ("opponent_highland_buff_zone_occupied", 13, 1, "bool", 1),
    ("red_central_highland_status", 14, 2, "enum", 3),
    ("blue_central_highland_status", 16, 2, "enum", 3),
    ("red_outpost_buff_zone_status", 18, 3, "enum", 4),
    ("blue_outpost_buff_zone_status", 21, 3, "enum", 4),
)

RADAR_MACRO_STATE_2_FIELDS = (
    ("opponent_central_highland_top", 0, 1, "bool", 1),
    ("ally_central_highland_top", 1, 1, "bool", 1),
    ("opponent_road_top", 2, 1, "bool", 1),
    ("ally_road_top", 3, 1, "bool", 1),
    ("opponent_tunnel_lower", 4, 1, "bool", 1),
    ("opponent_tunnel_middle", 5, 1, "bool", 1),
    ("opponent_tunnel_upper", 6, 1, "bool", 1),
    ("opponent_near_center_trapezoid_low", 7, 1, "bool", 1),
    ("opponent_near_center_trapezoid_middle", 8, 1, "bool", 1),
    ("opponent_near_center_trapezoid_high", 9, 1, "bool", 1),
    ("opponent_far_center_trapezoid_far", 10, 1, "bool", 1),
    ("opponent_far_center_trapezoid_middle", 11, 1, "bool", 1),
    ("opponent_far_center_trapezoid_near", 12, 1, "bool", 1),
    ("ally_tunnel_lower", 13, 1, "bool", 1),
    ("ally_tunnel_middle", 14, 1, "bool", 1),
    ("ally_tunnel_upper", 15, 1, "bool", 1),
    ("ally_near_center_trapezoid_low", 16, 1, "bool", 1),
    ("ally_near_center_trapezoid_middle", 17, 1, "bool", 1),
    ("ally_near_center_trapezoid_high", 18, 1, "bool", 1),
    ("ally_far_center_trapezoid_far", 19, 1, "bool", 1),
    ("ally_far_center_trapezoid_middle", 20, 1, "bool", 1),
    ("ally_far_center_trapezoid_near", 21, 1, "bool", 1),
    ("red_outpost_control_status", 22, 2, "enum", 2),
    ("blue_outpost_control_status", 24, 2, "enum", 2),
    ("red_base_control_status", 26, 2, "enum", 2),
    ("blue_base_control_status", 28, 2, "enum", 2),
)

RADAR_BUFF_ROBOT_SPECS = (
    ("opponent_heavy", 0),
    ("opponent_infantry_3", 14),
    ("opponent_infantry_4", 21),
    ("opponent_sentry", 28),
)

RADAR_MAIN_STATUS_ROBOT_SPECS = (
    ("opponent_heavy", 36),
    ("opponent_infantry_3", 38),
    ("opponent_infantry_4", 39),
    ("opponent_sentry", 40),
)
RADAR_RESERVED_BUFF_OFFSET = 7
RADAR_SENTRY_MODE_OFFSET = 35
RADAR_MAIN_STATUS_BASE_OFFSET = 36
RADAR_RESERVED_MAIN_STATUS_OFFSET = 37

RADAR_MAIN_STATUS_NAMES = {
    0: "alive",
    1: "destroyed",
    2: "invincible_not_weakened",
    3: "invincible_weakened",
}

RADAR_SENTRY_MODE_NAMES = {
    1: "offensive",
    2: "defensive",
    3: "mobile",
    4: "enhanced_offensive",
    5: "enhanced_defensive",
    6: "enhanced_mobile",
}

RADAR_FORTIFICATION_OFFSETS = {
    "center_x_mm": 41,
    "center_y_mm": 43,
    "radius_mm": 45,
}
