import json
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
from std_msgs.msg import String
from rm_radio_ros.core.adaptive_armor import AdaptiveArmorDecision, CONFIG_SCHEMA
from rm_radio_ros.core.rm_protocol import RefereeFrameAssembler
from rm_radio_ros.nodes.radar_integration_node import RadarIntegrationNode
from rm_radio_ros.nodes.referee_serial_node import RefereeSerialNode
from test_adaptive_armor import heartbeat, populated, referee


def message(data):
    msg = String()
    msg.data = json.dumps(data)
    return msg


def test_ros_strategy_callbacks_preserve_pending_request_and_publish_status(monkeypatch):
    node = object.__new__(RadarIntegrationNode)
    node.armor = populated()
    node.armor.evaluate(300, referee(300), 'red')
    heartbeat(node.armor, 303)
    node.armor.evaluate(303, referee(303), 'red')
    request_id = node.armor.pending['request_id']
    node._handle_armor_config(message({'schema': CONFIG_SCHEMA, 'config': {
        **node.armor.config.to_dict(), 'mode': 'fixed_center',
    }}))
    assert node.armor.config.mode == 'fixed_center'
    assert node.armor.pending['request_id'] == request_id
    node._handle_referee_tx(message({'request_id': request_id, 'written': True}))
    assert node.armor.pending['written'] is True


def test_generated_semantic_request_passes_existing_serial_guards_and_encodes_13_bytes(monkeypatch):
    decision = populated()
    decision.evaluate(300, referee(300), 'red')
    heartbeat(decision, 303)
    command = decision.evaluate(303, referee(303), 'red')
    node = object.__new__(RefereeSerialNode)
    node.seq = 0
    node.last_radar_cmd_value = 0
    node.last_armor_fortify_cmd_value = 0
    node.last_radar_decision_sync = referee(303)['last_radar_decision_sync']
    node._effective_sender_id = lambda: 9
    node.get_parameter = lambda name: SimpleNamespace(value={
        'status_period_sec': 0.2, 'receiver_id': 0x8080,
    }[name])
    monkeypatch.setattr('rm_radio_ros.nodes.referee_serial_node.time.time', lambda: 303)
    payload = node._parse_algorithm_radar_cmd_message(json.dumps(command))
    frame, meta = node._build_radar_cmd_from_payload(payload)
    parsed = RefereeFrameAssembler().push_bytes(frame)[0].to_dict()['parsed']['radar_cmd']
    assert meta['user_data_length'] == 13
    assert parsed['armor_fortify_cmd'] == 1
    assert parsed['fortify_center_x_mm'] == command['fortify_center_x_mm']
    assert parsed['fortify_center_y_mm'] == command['fortify_center_y_mm']
    node.last_radar_decision_sync['remaining_fortify_opportunities'] = 0
    with pytest.raises(ValueError, match='no referee opportunity'):
        node._build_radar_cmd_from_payload(payload)


def test_ros_armor_timer_uses_existing_command_publisher_and_separate_status(monkeypatch):
    node = object.__new__(RadarIntegrationNode)
    node.armor = populated()
    node.armor.evaluate(300, referee(300), 'red')
    heartbeat(node.armor, 303)
    node.fusion = SimpleNamespace(own_side='red', snapshot=lambda _: {'side_mismatch': False})
    node._decision_referee_status = lambda _: referee(303)
    commands, statuses = [], []
    node.radar_cmd_pub = SimpleNamespace(publish=lambda msg: commands.append(json.loads(msg.data)))
    node.armor_status_pub = SimpleNamespace(publish=lambda msg: statuses.append(json.loads(msg.data)))
    monkeypatch.setattr('rm_radio_ros.nodes.radar_integration_node.time.time', lambda: 303)
    node._armor_tick()
    node._armor_tick()
    assert len(commands) == 1
    assert commands[0]['action'] == 'fortify_armor'
    assert len(statuses) == 2
    assert statuses[-1]['pending']['request_id'] == commands[0]['request_id']
