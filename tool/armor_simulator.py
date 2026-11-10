"""Qt 命中模拟发送端；算法计算在雷达站完成。"""
import argparse
from collections import deque
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time


def configure_qt():
    spec = importlib.util.find_spec('PyQt5')
    if spec and spec.submodule_search_locations:
        plugins = Path(next(iter(spec.submodule_search_locations))) / 'Qt5/plugins'
        if plugins.is_dir():
            os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = str(plugins)
    os.environ.pop('QT_PLUGIN_PATH', None)


configure_qt()
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFormLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QPushButton, QSpinBox, QTextEdit, QVBoxLayout, QWidget,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from rm_radio_ros.core.adaptive_armor import CONFIG_SCHEMA, CONFIG_TOPIC, EVENT_TOPIC, STATUS_TOPIC
from .armor_simulation import SimulatedMatch


class SimulatorNode(Node):
    def __init__(self):
        super().__init__('qt_armor_hit_simulator_' + str(os.getpid()))
        self.events = self.create_publisher(String, EVENT_TOPIC, 100)
        self.config = self.create_publisher(String, CONFIG_TOPIC, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, STATUS_TOPIC, self.receive, 10)
        self.latest = {}
        self.received_at = 0

    def receive(self, message):
        try:
            data = json.loads(message.data)
            if isinstance(data, dict) and data.get('schema') == 'shark.radar.adaptive_armor.v1':
                self.latest, self.received_at = data, time.monotonic()
        except (ValueError, TypeError):
            pass

    def publish_event(self, event):
        self.events.publish(String(data=json.dumps(event, ensure_ascii=False, allow_nan=False)))

    def publish_config(self, config):
        self.config.publish(String(data=json.dumps({'schema': CONFIG_SCHEMA, 'config': config})))

    def fresh(self):
        return bool(self.latest and time.monotonic()-self.received_at < 3)


class ArmorCanvas(QWidget):
    def __init__(self, selected):
        super().__init__()
        self.selected = selected
        self.width_mm, self.height_mm = 100, 100
        self.point = (15, 85)
        self.status = {}
        self.face = QRectF()
        image = Path(__file__).resolve().parents[1] / 'radio/apps/match_rx/web/assets/adaptive-armor-plate.png'
        self.image = QPixmap(str(image))
        self.setMinimumSize(360, 330)

    def position(self, x, y):
        return QPointF(self.face.left()+x/self.width_mm*self.face.width(),
                       self.face.bottom()-y/self.height_mm*self.face.height())

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor('#eef3f8'))
        aspect = self.width_mm/self.height_mm * (1197/990)/(1097/988.9)
        width = min(self.width()-50, (self.height()-50)*aspect)
        height = width/aspect
        art = QRectF((self.width()-width)/2, (self.height()-height)/2, width, height)
        self.face = QRectF(art.left()+103.6/1197*width, art.top()+103.6/1097*height,
                           990/1197*width, 988.9/1097*height)
        if not self.image.isNull():
            painter.drawPixmap(art, self.image, QRectF(self.image.rect()))
        painter.setPen(QPen(QColor('#5cb6a1'), 1, Qt.DashLine))
        painter.drawRect(self.face)
        painter.save()
        painter.setClipRect(self.face)
        painter.setPen(QPen(QColor('#fff0f0'), 1))
        painter.setBrush(QColor('#ee6b64'))
        for hit in self.status.get('hit_points', []):
            painter.drawEllipse(self.position(hit['x_mm'], hit['y_mm']), 3, 3)
        proposal = self.status.get('proposal') or {}
        x, y, radius = (proposal.get(k) for k in ('center_x_mm', 'center_y_mm', 'radius_mm'))
        if all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y, radius)):
            painter.setPen(QPen(QColor('#2689d8'), 2, Qt.DashLine))
            painter.setBrush(QColor(38, 137, 216, 60))
            center = self.position(x, y)
            painter.drawEllipse(center, radius/self.width_mm*self.face.width(),
                                radius/self.height_mm*self.face.height())
            painter.setBrush(QColor('#ffffff'))
            painter.drawEllipse(center, 4, 4)
        painter.setPen(QPen(QColor('#f4b541'), 2))
        p = self.position(*self.point)
        painter.drawLine(p-QPointF(7, 0), p+QPointF(7, 0))
        painter.drawLine(p-QPointF(0, 7), p+QPointF(0, 7))
        painter.restore()
        painter.setPen(QColor('#34465a'))
        painter.drawText(12, self.height()-10,
                         f'左下角 (0, 0) · {self.width_mm} × {self.height_mm} mm（模拟尺寸）')

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and self.face.contains(QPointF(event.pos())):
            x = round((event.x()-self.face.left())/self.face.width()*self.width_mm)
            y = round((self.face.bottom()-event.y())/self.face.height()*self.height_mm)
            self.selected(x, y)


class SimulatorWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.match = None
        self.phase = 'idle'
        self.original_config = None
        self.queue = deque()
        self.last_heartbeat = 0
        self.closing = False
        self.setWindowTitle('装甲命中模拟工具 · synthetic')
        self.resize(1130, 800)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        title = QLabel('装甲命中模拟工具')
        title.setStyleSheet('font-size: 22px; font-weight: 600')
        layout.addWidget(title)
        layout.addWidget(QLabel('发送模拟命中 → 雷达站计算 → 回传命中点与建议圆；模拟数据不会触发裁判加固。'))
        self.connection = QLabel('等待雷达站状态…')
        layout.addWidget(self.connection)
        body = QHBoxLayout()
        layout.addLayout(body, 1)
        controls = QWidget()
        controls.setMaximumWidth(350)
        left = QVBoxLayout(controls)
        left.setContentsMargins(0, 0, 10, 0)
        body.addWidget(controls)
        setup = QGroupBox('模拟会话')
        form = QFormLayout(setup)
        self.side = QComboBox()
        self.side.addItem('红方', 'red'); self.side.addItem('蓝方', 'blue')
        self.mode = QComboBox()
        self.mode.addItem('算法：最大覆盖', 'adaptive_coverage')
        self.mode.addItem('固定居中', 'fixed_center')
        self.width_input = self.spin(1, 5000, 100)
        self.height_input = self.spin(1, 5000, 100)
        for label, widget in [('己方', self.side), ('圆心策略', self.mode),
                              ('模拟宽度 mm', self.width_input), ('模拟高度 mm', self.height_input)]:
            form.addRow(label, widget)
        self.start_button = QPushButton('开始模拟')
        self.start_button.clicked.connect(self.start)
        self.stop_button = QPushButton('结束模拟并恢复配置')
        self.stop_button.clicked.connect(self.stop)
        form.addRow(self.start_button); form.addRow(self.stop_button)
        left.addWidget(setup)
        hits = QGroupBox('命中事件')
        hit_form = QFormLayout(hits)
        self.direction = QComboBox()
        self.direction.addItem('来袭命中', 'incoming')
        self.direction.addItem('己方攻击命中', 'outgoing')
        self.direction.setToolTip('来袭：对方击中己方，用于选点；己方攻击：己方击中对方，用于规则统计。')
        self.x_input = self.spin(0, 100, 15)
        self.y_input = self.spin(0, 100, 85)
        self.shots_input = self.spin(1, 1000, 5)
        self.spread = QDoubleSpinBox()
        self.spread.setRange(0.1, 500); self.spread.setValue(3); self.spread.setSuffix(' mm')
        for label, widget in [('方向', self.direction), ('命中 X / mm', self.x_input),
                              ('命中 Y / mm', self.y_input), ('单发对应发射数', self.shots_input),
                              ('示例热点散布', self.spread)]:
            hit_form.addRow(label, widget)
        self.hit_button = QPushButton('发送单发命中')
        self.hit_button.clicked.connect(self.send_hit)
        self.example_button = QPushButton('发送热点示例（含历史样本）')
        self.example_button.clicked.connect(self.send_example)
        hit_form.addRow(self.hit_button); hit_form.addRow(self.example_button)
        hint = QLabel('示例含己方攻击 40 发、来袭 100 发，并累计模拟发射数。\n橙色十字为待发送命中位置，点击板面只选命中坐标。')
        hint.setWordWrap(True)
        hit_form.addRow(hint)
        left.addWidget(hits)
        self.log = QTextEdit()
        self.log.setReadOnly(True); self.log.document().setMaximumBlockCount(100)
        left.addWidget(self.log, 1)
        right = QVBoxLayout()
        body.addLayout(right, 1)
        self.canvas = ArmorCanvas(self.select_hit)
        right.addWidget(self.canvas, 1)
        self.result = QLabel('等待雷达站回传计算结果')
        self.result.setWordWrap(True)
        self.result.setMinimumHeight(96)
        right.addWidget(self.result)
        right.addWidget(QLabel('红点：雷达站接收的来袭命中　蓝色虚线圆：雷达站算法建议（非裁判生效）'))
        for widget in (self.width_input, self.height_input, self.x_input, self.y_input):
            widget.valueChanged.connect(self.update_canvas)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(20)
        self.update_canvas()
        self.update_buttons()

    @staticmethod
    def spin(low, high, value):
        widget = QSpinBox(); widget.setRange(low, high); widget.setValue(value)
        return widget

    def note(self, text):
        self.log.append(time.strftime('%H:%M:%S') + '  ' + text)

    def select_hit(self, x, y):
        self.x_input.setValue(x); self.y_input.setValue(y)

    def update_canvas(self):
        self.canvas.width_mm, self.canvas.height_mm = self.width_input.value(), self.height_input.value()
        self.x_input.setMaximum(self.canvas.width_mm)
        self.y_input.setMaximum(self.canvas.height_mm)
        self.canvas.point = (self.x_input.value(), self.y_input.value())
        self.canvas.update()

    def start(self):
        if not self.node.fresh() or self.node.latest.get('match_id') or self.node.count_publishers(STATUS_TOPIC) != 1:
            self.note('请连接唯一且没有进行中装甲会话的雷达融合节点。')
            return
        self.original_config = dict(self.node.latest['config'])
        self.match = SimulatedMatch(self.width_input.value(), self.height_input.value(),
                                    self.side.currentData(), time.time())
        self.phase = 'starting'
        self.node.publish_event(self.match.session())
        self.note('已发送模拟会话，等待雷达站接纳。')
        self.update_buttons()

    def stop(self):
        if self.match and self.phase not in ('restoring', 'idle'):
            self.queue.clear()
            self.phase = 'ending'
            self.node.publish_event(self.match.event('end_session', time.time()))
            self.note('已请求结束自己的模拟会话，等待确认后恢复运行配置。')

    def send_hit(self):
        if self.phase == 'active':
            self.queue.append(self.match.hit(self.direction.currentData(), self.x_input.value(),
                                             self.y_input.value(), time.time(), self.shots_input.value()))
            self.note('已排队发送单发命中。')

    def send_example(self):
        if self.phase == 'active' and not self.queue:
            self.queue.extend(self.match.example(time.time(), self.x_input.value(),
                                                  self.y_input.value(), self.spread.value()))
            self.note('已排队发送 140 发示例命中；包含历史训练样本与近期验证样本。')

    def update_buttons(self):
        available = self.node.fresh() and self.node.count_publishers(STATUS_TOPIC) == 1
        self.start_button.setEnabled(available and self.phase == 'idle' and not self.node.latest.get('match_id'))
        self.stop_button.setEnabled(available and self.match is not None)
        for button in (self.hit_button, self.example_button):
            button.setEnabled(available and self.phase == 'active' and not self.queue)
        for widget in (self.width_input, self.height_input, self.side, self.mode):
            widget.setEnabled(self.phase == 'idle')

    def tick(self):
        for _ in range(4):
            rclpy.spin_once(self.node, timeout_sec=0)
        status = self.node.latest
        fresh = self.node.fresh()
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        phase_label = {'idle': '待机', 'starting': '等待接纳', 'configuring': '应用模拟配置',
                       'active': '模拟中', 'ending': '等待结束确认', 'restoring': '恢复配置'}[self.phase]
        self.connection.setText(f'ROS 域 {domain} · ' + ('已连接雷达站' if fresh else '等待雷达站 / 状态超时')
                                + f' · {phase_label} · 待发送 {len(self.queue)}')
        if self.match and fresh:
            own = status.get('match_id') == self.match.match_id and status.get('origin') == 'synthetic'
            if own and self.phase == 'starting':
                config = dict(self.original_config, mode=self.mode.currentData(), automatic_requests=False,
                              plate_width_mm=self.match.width, plate_height_mm=self.match.height,
                              plate_area_mm2=0, geometry_confirmed=False, boundary_policy='unconfirmed')
                self.simulation_config = config
                self.node.publish_config(config)
                self.phase = 'configuring'
            elif own and self.phase == 'configuring' and status.get('config') == self.simulation_config:
                self.phase = 'active'
                self.note('雷达站已接纳模拟会话和预览配置，可以发送命中。')
            elif not status.get('match_id') and self.phase in ('ending', 'active', 'configuring'):
                self.queue.clear()
                self.node.publish_config(self.original_config)
                self.phase = 'restoring'
            elif self.phase == 'restoring' and status.get('config') == self.original_config:
                self.match, self.phase = None, 'idle'
                self.canvas.status = {}
                self.result.setText('模拟已结束，运行配置已恢复。可以开始新的模拟会话。')
                self.note('模拟已结束，雷达站运行配置已恢复。')
                if self.closing:
                    self.close()
            elif status.get('match_id') and not own:
                self.queue.clear()
                self.note('雷达站绑定了其他会话，停止发送；未改写其他会话配置。')
                self.match, self.phase = None, 'idle'
            if own:
                self.canvas.status = status
                proposal = status.get('proposal') or {}
                radius = proposal.get('radius_mm')
                radius_label = f'{radius:.1f}' if isinstance(radius, (int, float)) else '待估计'
                self.result.setText(
                    f"雷达站接收：己方打中 {status.get('own_scored_hit_count')} 发 / 来袭 {status.get('own_received_hit_count')} 发\n"
                    f"建议圆心 ({proposal.get('center_x_mm')}, {proposal.get('center_y_mm')}) mm · 半径 {radius_label} mm\n"
                    f"训练 {proposal.get('train_count')} / 验证 {proposal.get('validation_count')} · {status.get('reason')}\n"
                    + (status.get('last_error') or 'synthetic · 圆由雷达站计算，仅作模拟预览'))
            if own and self.phase == 'active':
                for _ in range(min(5, len(self.queue))):
                    self.node.publish_event(self.queue.popleft())
                if not self.queue and time.monotonic()-self.last_heartbeat >= 0.5:
                    self.node.publish_event(self.match.heartbeat(time.time()))
                    self.last_heartbeat = time.monotonic()
        self.canvas.update()
        self.update_buttons()

    def closeEvent(self, event):
        if self.match and self.node.fresh():
            self.closing = True
            self.stop()
            event.ignore()
        else:
            self.timer.stop()
            event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, help='与雷达站一致的 ROS_DOMAIN_ID')
    args = parser.parse_args()
    if args.domain_id is not None:
        if not 0 <= args.domain_id <= 232:
            parser.error('ROS 域范围为 0～232')
        os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    rclpy.init(args=[])
    app = QApplication(sys.argv[:1])
    node = SimulatorNode()
    window = SimulatorWindow(node)
    window.show()
    try:
        return app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
