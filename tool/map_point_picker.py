#!/usr/bin/env python3
"""独立的 2027 地图两层选点工具，只保存地图像素坐标。"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys


spec = importlib.util.find_spec("PyQt5")
if spec and spec.submodule_search_locations:
    plugins = Path(next(iter(spec.submodule_search_locations))) / "Qt5/plugins"
    if plugins.is_dir():
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(plugins)
os.environ.pop("QT_PLUGIN_PATH", None)

from PyQt5.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QKeySequence, QPainter, QPen, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFileDialog, QGraphicsEllipseItem, QGraphicsItem,
    QGraphicsScene, QGraphicsSimpleTextItem, QGraphicsView, QHBoxLayout, QLabel,
    QMainWindow, QMessageBox, QPushButton, QRadioButton, QScrollArea, QShortcut,
    QSplitter, QVBoxLayout,
    QWidget,
)

ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = ROOT / "vision/images-2027/display-v1.0.0"
COLORS = ("#ffdc3b", "#2ddbf2")


class Marker(QGraphicsEllipseItem):
    def __init__(self, owner, layer, number, position):
        super().__init__(-7, -7, 14, 14)
        self.owner = owner
        self.ready = False
        self.setBrush(QColor(COLORS[layer]))
        self.setPen(QPen(Qt.black, 2))
        self.setFlags(QGraphicsItem.ItemIsMovable | QGraphicsItem.ItemIsSelectable |
                      QGraphicsItem.ItemIgnoresTransformations |
                      QGraphicsItem.ItemSendsGeometryChanges)
        self.setZValue(2)
        self.label = QGraphicsSimpleTextItem(self)
        self.label.setBrush(QColor(COLORS[layer]))
        self.label.setPen(QPen(Qt.black, .4))
        self.label.setFont(QFont("Sans Serif", 11, QFont.Bold))
        self.label.setPos(11, -25)
        self.renumber(layer, number)
        self.setPos(position)
        self.ready = True

    def renumber(self, layer, number):
        self.label.setText(f"L{layer + 1}-{number}")

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionChange:
            return QPointF(min(max(value.x(), 0), self.owner.map_width - 1),
                           min(max(value.y(), 0), self.owner.map_height - 1))
        if change == QGraphicsItem.ItemPositionHasChanged and self.ready:
            self.owner.mark_dirty()
        return super().itemChange(change, value)


class MapView(QGraphicsView):
    point_clicked = pyqtSignal(QPointF)
    zoom_changed = pyqtSignal(float)

    def __init__(self, scene):
        super().__init__(scene)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setBackgroundBrush(QColor("#25282d"))
        self.press_position = None
        self.pressed_marker = False
        self.auto_fit = True
        self.setMinimumSize(200, 200)

    def fit_map(self):
        self.auto_fit = True
        # 自动适应时隐藏滚动条，避免 fitInView 与滚动条显隐相互触发 resize。
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)
        self.centerOn(self.sceneRect().center())
        self.zoom_changed.emit(self.transform().m11() * 100)

    def set_zoom(self, percent, under_mouse=False):
        scale = min(max(percent / 100, .01), 16)
        center = self.mapToScene(self.viewport().rect().center())
        self.auto_fit = False
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setTransformationAnchor(self.AnchorUnderMouse if under_mouse else self.AnchorViewCenter)
        self.scale(scale / self.transform().m11(), scale / self.transform().m11())
        if not under_mouse:
            self.centerOn(center)
        self.zoom_changed.emit(self.transform().m11() * 100)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.auto_fit:
            QTimer.singleShot(0, self.fit_if_enabled)

    def fit_if_enabled(self):
        if self.auto_fit:
            self.fit_map()

    def mousePressEvent(self, event):
        self.press_position = event.pos()
        item = self.itemAt(event.pos())
        while item is not None and not isinstance(item, Marker):
            item = item.parentItem()
        self.pressed_marker = item is not None
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        if (event.button() == Qt.LeftButton and self.press_position is not None
                and (event.pos() - self.press_position).manhattanLength() <= 3
                and not self.pressed_marker):
            self.point_clicked.emit(self.mapToScene(event.pos()))
        self.press_position = None

    def wheelEvent(self, event):
        if not event.angleDelta().y():
            event.ignore()
            return
        factor = 1.2 if event.angleDelta().y() > 0 else 1 / 1.2
        self.set_zoom(self.transform().m11() * factor * 100, under_mouse=True)
        event.accept()


class PointPicker(QMainWindow):
    def __init__(self, side="B"):
        super().__init__()
        self.side = side
        self.layer = 0
        self.positions = {"R": [[], []], "B": [[], []]}
        self.dirty = {"R": False, "B": False}
        self.paths = {}
        self.markers = [[], []]
        self.setWindowTitle("2027 地图两层选点（独立工具）")
        available = QApplication.primaryScreen().availableGeometry()
        self.resize(min(1150, int(available.width() * .9)),
                    min(950, int(available.height() * .9)))
        self.setMinimumSize(560, 360)
        self.scene = QGraphicsScene(self)
        self.view = MapView(self.scene)
        self.view.point_clicked.connect(self.add_point)

        panel = QVBoxLayout()
        panel.addWidget(QLabel("地图视角"))
        self.side_box = QComboBox()
        self.side_box.addItem("蓝方在下", "B")
        self.side_box.addItem("红方在下", "R")
        self.side_box.setCurrentIndex(0 if side == "B" else 1)
        self.side_box.currentIndexChanged.connect(self.change_side)
        panel.addWidget(self.side_box)
        for layer, title in enumerate(("第一层 · 黄色", "第二层 · 青色")):
            button = QRadioButton(title)
            button.setStyleSheet(f"color: {COLORS[layer]}; font-weight: bold; background: #333; padding: 9px;")
            button.setChecked(layer == 0)
            button.toggled.connect(lambda checked, i=layer: self.set_layer(i) if checked else None)
            panel.addWidget(button)
        self.count_label = QLabel()
        panel.addWidget(self.count_label)
        zoom_label = QLabel("地图缩放")
        zoom_label.setToolTip("100% 显示原图大小")
        zoom_label.setWordWrap(True)
        panel.addWidget(zoom_label)
        zoom_row = QHBoxLayout()
        self.zoom_box = QDoubleSpinBox()
        self.zoom_box.setRange(1, 1600)
        self.zoom_box.setDecimals(1)
        self.zoom_box.setSingleStep(5)
        self.zoom_box.setSuffix(" %")
        self.zoom_box.setKeyboardTracking(False)
        self.zoom_box.valueChanged.connect(self.view.set_zoom)
        self.view.zoom_changed.connect(self.update_zoom)
        for title, factor in (("−", 1 / 1.2), ("+", 1.2)):
            button = QPushButton(title)
            button.setFixedWidth(self.fontMetrics().height() * 2)
            button.setToolTip("缩小地图" if factor < 1 else "放大地图")
            button.clicked.connect(lambda checked=False, f=factor:
                                   self.view.set_zoom(self.view.transform().m11() * f * 100))
            zoom_row.addWidget(button)
        zoom_row.insertWidget(1, self.zoom_box, 1)
        panel.addLayout(zoom_row)
        for title, action in (("删除选中点（Delete）", self.delete_selected),
                              ("撤销当前层末点", self.remove_last),
                              ("适应窗口", self.fit_map),
                              ("保存 JSON…", self.save_dialog)):
            button = QPushButton(title)
            button.setMinimumHeight(36)
            button.clicked.connect(action)
            panel.addWidget(button)
        help_label = QLabel("左键点击地图添加点\n拖动圆点调整位置\n拖动空白处平移地图\n滚轮缩放，便于精确选点\n\n两层自由选点，不限制数量\n红蓝视角分别保留选点\n只保存地图坐标，不计算矩阵")
        help_label.setWordWrap(True)
        panel.addWidget(help_label)
        panel.addStretch()
        controls = QWidget()
        controls.setLayout(panel)
        self.controls_scroll = QScrollArea()
        self.controls_scroll.setWidgetResizable(True)
        self.controls_scroll.setWidget(controls)
        self.controls_scroll.setMinimumWidth(max(
            240, controls.minimumSizeHint().width()
            + self.controls_scroll.verticalScrollBar().sizeHint().width()
            + self.controls_scroll.frameWidth() * 2))
        self.controls_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.splitter = QSplitter(Qt.Horizontal)
        self.splitter.setChildrenCollapsible(False)
        self.splitter.addWidget(self.controls_scroll)
        self.splitter.addWidget(self.view)
        self.splitter.setStretchFactor(0, 0)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([self.controls_scroll.minimumWidth(), 850])
        layout = QHBoxLayout()
        layout.setContentsMargins(6, 6, 6, 6)
        layout.addWidget(self.splitter)
        central = QWidget()
        central.setLayout(layout)
        self.setCentralWidget(central)
        QShortcut(QKeySequence(Qt.Key_Delete), self, self.delete_selected)
        self.load_map()
        QTimer.singleShot(0, self.fit_map)

    def set_layer(self, layer):
        self.layer = layer

    def mark_dirty(self):
        self.dirty[self.side] = True
        self.update_counts()

    def update_counts(self):
        counts = [len(items) for items in self.markers]
        suffix = "\n当前视角有未保存修改" if self.dirty[self.side] else ""
        self.count_label.setText(f"第一层：{counts[0]} 点\n第二层：{counts[1]} 点{suffix}")

    def current_points(self):
        return [[[round(item.pos().x(), 3), round(item.pos().y(), 3)]
                 for item in layer] for layer in self.markers]

    def change_side(self):
        self.positions[self.side] = self.current_points()
        self.side = self.side_box.currentData()
        self.load_map()

    def load_map(self):
        name = "red_bottom.png" if self.side == "R" else "blue_bottom.png"
        self.map_path = MAP_DIR / name
        pixmap = QPixmap(str(self.map_path))
        if pixmap.isNull():
            raise FileNotFoundError(f"无法读取地图：{self.map_path}")
        self.map_width, self.map_height = pixmap.width(), pixmap.height()
        self.scene.clear()
        self.markers = [[], []]
        self.scene.addPixmap(pixmap)
        self.scene.setSceneRect(0, 0, self.map_width, self.map_height)
        for layer, positions in enumerate(self.positions[self.side]):
            for position in positions:
                marker = Marker(self, layer, len(self.markers[layer]) + 1, QPointF(*position))
                self.scene.addItem(marker)
                self.markers[layer].append(marker)
        self.fit_map()
        self.update_counts()
        self.statusBar().showMessage(f"{name} · {self.map_width}×{self.map_height} · 坐标为原图像素，左上角为原点")

    def fit_map(self):
        self.view.fit_map()

    def update_zoom(self, percent):
        self.zoom_box.blockSignals(True)
        self.zoom_box.setValue(percent)
        self.zoom_box.blockSignals(False)

    def add_point(self, position):
        if not (0 <= position.x() < self.map_width and 0 <= position.y() < self.map_height):
            return
        marker = Marker(self, self.layer, len(self.markers[self.layer]) + 1, position)
        self.scene.clearSelection()
        self.scene.addItem(marker)
        self.markers[self.layer].append(marker)
        marker.setSelected(True)
        self.mark_dirty()

    def delete_selected(self):
        removed = False
        for layer, items in enumerate(self.markers):
            for item in items[:]:
                if item.isSelected():
                    items.remove(item)
                    self.scene.removeItem(item)
                    removed = True
            for number, item in enumerate(items, 1):
                item.renumber(layer, number)
        if removed:
            self.mark_dirty()

    def remove_last(self):
        if self.markers[self.layer]:
            self.scene.removeItem(self.markers[self.layer].pop())
            self.mark_dirty()

    def save_points(self, path):
        document = {
            "version": 1,
            "season": 2027,
            "map_profile": "rmuc2027-v1.0.0-display1",
            "side": self.side,
            "map_image": str(self.map_path.relative_to(ROOT)),
            "map_sha256": hashlib.sha256(self.map_path.read_bytes()).hexdigest(),
            "image_size": [self.map_width, self.map_height],
            "coordinate_system": "image_px_top_left_x_right_y_down",
            "layer_colors": list(COLORS),
            "map_points": self.current_points(),
        }
        Path(path).write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.paths[self.side] = str(path)
        self.dirty[self.side] = False
        self.update_counts()
        self.statusBar().showMessage(f"已保存：{path}")

    def save_dialog(self):
        default = self.paths.get(self.side, str(ROOT / "tool/data" / f"map2027_points_{self.side}.json"))
        Path(default).parent.mkdir(parents=True, exist_ok=True)
        path, _ = QFileDialog.getSaveFileName(self, "保存当前视角的两层点位", default, "JSON (*.json)")
        if path:
            try:
                self.save_points(path)
            except OSError as error:
                QMessageBox.critical(self, "保存失败", str(error))

    def closeEvent(self, event):
        if any(self.dirty.values()):
            answer = QMessageBox.question(self, "还有未保存的点位", "红方或蓝方有未保存修改，仍要退出吗？",
                                          QMessageBox.Discard | QMessageBox.Cancel, QMessageBox.Cancel)
            if answer != QMessageBox.Discard:
                event.ignore()
                return
        event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("R", "B"), default="B", help="初始地图视角，默认 B（蓝方在下）")
    args = parser.parse_args()
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv[:1])
    font = app.font()
    font.setPointSizeF(max(font.pointSizeF(), 10))
    app.setFont(font)
    window = PointPicker(args.side)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    sys.exit(main())
