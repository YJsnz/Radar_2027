"""RMUC 2027 V1.0.0 radar payloads. RF/PHY parameters are not defined here."""
import struct

from .rm_protocol import (
    build_radar_cmd_payload as _legacy_password_fields,
    parse_radar_frame as _parse_unchanged,
)

AIR_LENGTHS = {0x0A01: 24, 0x0A02: 12, 0x0A03: 10, 0x0A04: 12, 0x0A05: 47, 0x0A06: 6}
SERIAL_LENGTHS = {0x0001: 11, 0x0003: 24, 0x0101: 8, 0x0105: 3, 0x0201: 17,
                  0x0202: 14, 0x0203: 12, 0x0204: 7, 0x0206: 7, 0x0209: 6,
                  0x020C: 2, 0x020E: 8, 0x0305: 48}
ROLES = ('1', '3', '4', '7')
NAMES = ('重装', '步兵3', '步兵4', '哨兵')
SLOTS = (0, 2, 3, 5)


def _select(values, slots=SLOTS):
    return {name: values[slot] for name, slot in zip(NAMES, slots)}


def _zone_fields(word, offsets):
    widths = (1, 3, 3, 3, 3, 1, 2, 2, 3, 3)
    names = ('补给区', 'R堡垒', 'B堡垒', 'R基地', 'B基地', '制高点', 'R中央高地',
             'B中央高地', 'R前哨', 'B前哨')
    return {name: (word >> offset) & ((1 << width) - 1)
            for name, offset, width in zip(names, offsets, widths)}


def decode_air(command, payload):
    if AIR_LENGTHS.get(command) != len(payload):
        raise ValueError('2027 radar command or data length mismatch')
    out = {'raw_hex': payload.hex(' ').upper()}
    if command == 0x0A01:
        xy = struct.unpack('<12H', payload)
        out['positions'] = [dict(name=name, x_cm=xy[slot*2], y_cm=xy[slot*2+1])
                            for name, slot in zip(NAMES, SLOTS)]
    elif command == 0x0A02:
        out['health'] = _select(struct.unpack('<6H', payload))
    elif command == 0x0A03:
        out['ammo'] = _select(struct.unpack('<5H', payload), (0, 1, 2, 4))
    elif command == 0x0A04:
        remaining, total, buffs, terrain = struct.unpack('<HHII', payload)
        out['economy'] = {'剩余金币': remaining, '累计总金币': total}
        out['zone_activation'] = _zone_fields(buffs, (0, 1, 4, 7, 10, 13, 14, 16, 18, 21))
        out['terrain_detection_bits'] = terrain & ((1 << 22) - 1)
        out['zone_control'] = {zone: (terrain >> shift) & 3 for zone, shift in
                               zip(('R前哨', 'B前哨', 'R基地', 'B基地'), (22, 24, 26, 28))}
    elif command == 0x0A05:
        fields = ('回血%', '冷却', '防御%', '负防御%', '攻击%')
        out['buffs'] = {name: dict(zip(fields, struct.unpack_from('<BHBBH', payload, offset)))
                        for name, offset in zip(NAMES, (0, 14, 21, 28))}
        out['sentry_mode'] = payload[35]
        out['robot_status'] = _select(payload[36:41], (0, 2, 3, 4))
        out['fortify_mm'] = struct.unpack_from('<HHH', payload, 41)
    else:
        key = payload.decode('ascii', errors='replace')
        if key.isascii() and key.isalnum():
            out['key'] = key
    return out


def build_radar_cmd_payload(radar_cmd=0, password_cmd=0, password=None,
                            *, fortify_cmd=0, center_x_mm=0, center_y_mm=0):
    """Serialize table 1-34; callers own the match counters and opportunity policy."""
    return (_legacy_password_fields(radar_cmd, password_cmd, password)
            + struct.pack('<BHH', fortify_cmd, center_x_mm, center_y_mm))


def map_payload(positions, own_side):
    if own_side not in ('red', 'blue'):
        raise ValueError('unknown side')
    ally = 'R' if own_side == 'red' else 'B'
    enemy = 'B' if ally == 'R' else 'R'
    words = [0] * 24
    for base, prefix in ((0, enemy), (12, ally)):
        for slot, role in zip(SLOTS, ROLES):
            point = positions.get(prefix + role)
            if point is not None and all(isinstance(v, int) and 0 <= v <= 65535 for v in point):
                x, y = point
                words[base + slot*2:base + slot*2 + 2] = (x, y)
    return struct.pack('<24H', *words) if any(words) else None


def parse_radar_frame(frame):
    command, data = frame.cmd_id, frame.data
    expected = SERIAL_LENGTHS.get(command)
    if expected is not None and len(data) != expected:
        return {'valid_radar_length': False, 'expected_bytes': expected, 'actual_bytes': len(data)}
    out = {'valid_radar_length': True}
    if command == 0x020E:
        info, fortify, x, y, radius = struct.unpack('<BBHHH', data)
        out['radar_decision_sync'] = {
            'double_vulnerability_count': info & 3,
            'is_double_vulnerability': bool(info & 4),
            'own_encryption_level': (info >> 3) & 3,
            'can_change_password': bool(info & 32),
            'fortify_active': bool(fortify & 1),
            'fortify_count': (fortify >> 1) & 7,
            'fortify_mm': [x, y, radius],
        }
    elif command == 0x0003:
        values = struct.unpack('<4Hh7H', data)
        out['game_robot_hp'] = dict(zip(
            ('ally_heavy', 'reserved', 'ally_3', 'ally_4', 'damage_difference', 'ally_7',
             'red_outpost', 'ally_base', 'blue_outpost', 'enemy_base',
             'ally_control_degree', 'enemy_control_degree'), values))
    elif command == 0x020C:
        word, = struct.unpack('<H', data)
        out['radar_mark_progress'] = {
            'enemy': {role: bool(word & (1 << slot)) for role, slot in zip(ROLES, SLOTS)},
            'ally': {role: bool(word & (1 << (slot + 6))) for role, slot in zip(ROLES, SLOTS)},
        }
    elif command == 0x0101:
        a, b = struct.unpack('<II', data)
        out['field_events'] = {'word0': a, 'word1': b}
    elif command == 0x0202:
        buffer_energy, heat17, heat42 = struct.unpack_from('<HHH', data, 8)
        out['power_heat'] = {'buffer_energy_j': buffer_energy, 'heat_17mm': heat17, 'heat_42mm': heat42}
    elif command == 0x0203:
        x, y, angle = struct.unpack('<fff', data)
        out['robot_position'] = {'x_m': x, 'y_m': y, 'angle_deg': angle}
    elif command == 0x0204:
        out['robot_buff'] = dict(zip(('recovery_percent', 'cooling', 'defense_percent',
                                     'vulnerability_percent', 'attack_percent'), struct.unpack('<BHBBH', data)))
    elif command == 0x0206:
        flags, sequence, x, y = struct.unpack('<BHhh', data)
        out['hurt'] = {'armor_id': flags & 15, 'reason': flags >> 4,
                       'hit_sequence': sequence, 'hit_position_mm': [x, y]}
    elif command == 0x0301:
        if len(data) < 6 or len(data) > 118:
            return {'valid_radar_length': False}
        sub, sender, receiver = struct.unpack_from('<HHH', data)
        out['robot_interaction'] = {'data_cmd_id': sub, 'sender_id': sender, 'receiver_id': receiver}
        if sub == 0x0121:
            if len(data) != 19:
                return {'valid_radar_length': False}
            double, password_cmd, key, fortify, x, y = struct.unpack_from('<BB6sBHH', data, 6)
            out['radar_cmd'] = {'radar_cmd': double, 'password_cmd': password_cmd,
                                'password': key.decode('ascii', errors='replace'),
                                'armor_fortify_cmd': fortify, 'center_mm': [x, y]}
    elif command in (0x0001, 0x0105, 0x0201):
        return _parse_unchanged(frame)
    else:
        out['payload_hex'] = data.hex()
    return out


def demo_information(period):
    """Repeatable lab data; reserved slots are zero."""
    d = period % 100 * 10
    dx, dy = ((d, 0) if d < 300 else (300, d - 300) if d < 500 else
              (800 - d, 200) if d < 800 else (0, 1000 - d))
    coords = [0] * 12
    for slot in SLOTS:
        coords[2*slot:2*slot+2] = (200 + slot*400 + dx, 200 + slot*150 + dy)
    return (
        (0x0A01, struct.pack('<12H', *coords)),
        (0x0A02, struct.pack('<6H', 500, 0, 400, 400, 0, 400)),
        (0x0A03, struct.pack('<5H', 100, 200, 200, 0, 300)),
        (0x0A04, struct.pack('<HHII', 500, 1000, 0, 0)),
        (0x0A05, bytes(35) + b'\x01' + bytes(11)),
    )
