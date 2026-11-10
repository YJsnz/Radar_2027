import json
import os
from pathlib import Path
import threading
import time

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('PyQt5')


def test_qt_buttons_send_to_real_integration_and_restore_configuration(tmp_path, monkeypatch):
    # One isolated domain; no radio, serial writer, camera or production config.
    monkeypatch.setenv('ROS_DOMAIN_ID', '198')
    monkeypatch.setenv('QT_QPA_PLATFORM', 'offscreen')
    config_path = tmp_path / 'armor.json'
    original = {'mode': 'fixed_center', 'automatic_requests': True,
                'trusted_sources': ['qt-armor-simulator']}
    config_path.write_text(json.dumps(original))
    monkeypatch.setenv('RM_RADIO_ARMOR_CONFIG', str(config_path))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import String
    from PyQt5.QtWidgets import QApplication
    from rm_radio_ros.nodes.radar_integration_node import RadarIntegrationNode
    from rm_radio_ros.app_support.match_dashboard_node import RadioDashboardNode
    from tool.armor_simulator import SimulatorNode, SimulatorWindow

    rclpy.init(args=[])
    app = QApplication.instance() or QApplication([])
    radar = RadarIntegrationNode()
    monkeypatch.setattr(RadioDashboardNode, '_schedule_sdr_check', lambda self: None)
    web_root = Path(__file__).resolve().parents[2] / 'radio/apps/match_rx/web'
    dashboard = RadioDashboardNode(port=0, web_root=web_root)
    observer = rclpy.create_node('armor_simulator_test_observer')
    requests = []
    observer.create_subscription(String, '/rm_radar_algorithm/radar_cmd',
                                 lambda m: requests.append(m.data), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(radar); executor.add_node(observer); executor.add_node(dashboard)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    client = SimulatorNode()
    window = SimulatorWindow(client)
    window.show()
    def until(predicate, timeout=8):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            app.processEvents()
            if predicate(): return
            time.sleep(.01)
        raise AssertionError(f'Qt/ROS timeout: phase={window.phase}, status={client.latest}')
    try:
        until(lambda: window.start_button.isEnabled())
        saved = dict(client.latest['config'])
        window.start_button.click()
        until(lambda: window.phase == 'active')
        assert radar.armor.statistics.origin == 'synthetic'
        assert radar.armor.config.automatic_requests is False
        window.example_button.click()
        until(lambda: client.latest.get('own_received_hit_count') == 100
              and client.latest.get('proposal', {}).get('radius_mm') is not None)
        assert client.latest['own_scored_hit_count'] == 40
        assert client.latest['proposal']['center_x_mm'] < 40
        assert client.latest['proposal']['center_y_mm'] > 60
        assert window.canvas.status['proposal']['radius_mm'] > 0
        assert window.canvas.image.isNull() is False
        until(lambda: dashboard.state.adaptive_armor.get('data', {}).get('own_received_hit_count') == 100)
        dashboard_data = dashboard.state.adaptive_armor['data']
        assert dashboard_data['origin'] == 'synthetic'
        assert dashboard_data['proposal']['center_x_mm'] == client.latest['proposal']['center_x_mm']
        assert dashboard_data['proposal']['radius_mm'] == client.latest['proposal']['radius_mm']
        screenshot = os.environ.get('ARMOR_SIMULATOR_SCREENSHOT')
        if screenshot:
            assert window.grab().save(screenshot)
        window.hit_button.click()
        until(lambda: client.latest.get('own_received_hit_count') == 101)
        window.stop_button.click()
        until(lambda: window.phase == 'idle')
        assert not radar.armor.statistics.match_id
        assert client.latest['config'] == saved
        # Close also performs the acknowledged end/config restore sequence.
        until(lambda: window.start_button.isEnabled())
        window.mode.setCurrentIndex(1)
        window.start_button.click()
        until(lambda: window.phase == 'active')
        window.example_button.click()
        until(lambda: client.latest.get('own_received_hit_count') == 100
              and client.latest.get('proposal', {}).get('radius_mm') is not None)
        assert client.latest['proposal']['center_x_mm'] == 50
        assert client.latest['proposal']['center_y_mm'] == 50
        window.close()
        until(lambda: not window.isVisible())
        assert window.phase == 'idle'
        assert client.latest['config'] == saved
        assert not requests
        assert json.loads(config_path.read_text()) == original
    finally:
        window.timer.stop()
        window.match = None
        window.close()
        executor.shutdown()
        thread.join(timeout=3)
        for node in (client, radar, observer, dashboard): node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
