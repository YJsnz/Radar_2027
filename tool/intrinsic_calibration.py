"""MV-SUA133GC-T 内参标定 Qt 工具；参数统一读取 YAML。"""
import argparse
from datetime import datetime, timezone
import importlib.util
import os
from pathlib import Path
import signal
import sys
import threading
import time


def configure_qt():
    spec = importlib.util.find_spec('PyQt5')
    if spec and spec.submodule_search_locations:
        plugins = Path(next(iter(spec.submodule_search_locations)))/'Qt5/plugins'
        if plugins.is_dir():
            os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = str(plugins)
    os.environ.pop('QT_PLUGIN_PATH', None)


configure_qt()
import cv2
import numpy as np
configure_qt()  # A non-headless cv2 import may replace Qt's plugin path.
from PyQt5.QtCore import Qt, QThread, QTimer, QUrl, pyqtSignal
from PyQt5.QtGui import QDesktopServices, QFontMetrics, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QFileDialog, QGroupBox, QHBoxLayout, QLabel, QListWidget,
    QMainWindow, QMessageBox, QProgressBar, QPushButton, QSplitter, QTextEdit, QVBoxLayout, QWidget,
)

from .intrinsic_core import Dataset, coverage, detect_board, load_settings, profile


class CameraWorker(QThread):
    error = pyqtSignal(str)

    def __init__(self, settings):
        super().__init__()
        self.settings = settings
        self.lock = threading.Lock()
        self.latest = None

    def run(self):
        from vision.mindvision_camera import MindVisionCamera
        config = self.settings.values['camera']
        try:
            with MindVisionCamera(config['exposure_us'], device_serial=config['serial'], require_single=True) as camera:
                while not self.isInterruptionRequested():
                    started = time.monotonic()
                    frame = camera.get_frame(config['frame_timeout_ms'])
                    received = time.monotonic()
                    if config['expected_width'] is not None and frame.shape[1::-1] != (config['expected_width'], config['expected_height']):
                        raise ValueError(f'实拍尺寸 {frame.shape[1]}×{frame.shape[0]} 与 YAML 预期不符')
                    metadata = camera.capture_metadata()
                    metadata.update(expected_model=config['expected_model'], acquired_at=datetime.now(timezone.utc).isoformat(),
                                    timestamp_basis='host_receive_after_buffer', image_size=[frame.shape[1], frame.shape[0]])
                    detection = detect_board(frame, self.settings)
                    with self.lock:
                        self.latest = (frame, detection, metadata, received)
                    remaining = 1/config['preview_fps']-(time.monotonic()-started)
                    if remaining > 0:
                        self.msleep(round(remaining*1000))
        except Exception as error:
            self.error.emit(f'相机采集已停止：{error}。如设备被占用，请先停止比赛/外参标定的取流。')

    def snapshot(self):
        with self.lock:
            return self.latest


class Job(QThread):
    completed = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, function):
        super().__init__()
        self.function = function

    def run(self):
        try:
            self.completed.emit(self.function())
        except Exception as error:
            self.failed.emit(str(error))


class IntrinsicWindow(QMainWindow):
    def __init__(self, config_path):
        super().__init__()
        self.config_path = Path(config_path).resolve()
        self.settings = load_settings(self.config_path)
        self.dataset = Dataset(self.settings)
        self.camera = None
        self.job = None
        self.bundle = None
        self.result = None
        self.closing = False
        self.next_auto = 0
        self.last_rejection = ''
        self.setWindowTitle('迈德威视 MV-SUA133GC-T · 相机内参标定')
        self.resize(1250, 880)
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)
        heading = QLabel('相机内参标定')
        heading.setStyleSheet('font-size: 23px; font-weight: 600')
        layout.addWidget(heading)
        layout.addWidget(QLabel('原始分辨率采样 → 棋盘格角点 → 内参与畸变 → 误差审阅；配置全部来自 YAML'))
        split = QSplitter(Qt.Horizontal)
        layout.addWidget(split, 1)
        panel = QWidget()
        left = QVBoxLayout(panel)
        left.setContentsMargins(0, 0, 8, 0)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        self.summary.setTextInteractionFlags(Qt.TextSelectableByMouse)
        left.addWidget(self.summary)
        self.edit_button = QPushButton('打开 YAML 配置文件')
        self.edit_button.clicked.connect(lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.config_path))))
        self.reload_button = QPushButton('重新加载 YAML / 新建数据集')
        self.reload_button.clicked.connect(self.reload)
        left.addWidget(self.edit_button); left.addWidget(self.reload_button)
        row = QHBoxLayout()
        self.open_button = QPushButton('打开相机')
        self.open_button.clicked.connect(self.open_camera)
        self.stop_button = QPushButton('停止采集')
        self.stop_button.clicked.connect(self.stop_camera)
        row.addWidget(self.open_button); row.addWidget(self.stop_button)
        left.addLayout(row)
        self.capture_button = QPushButton('采集当前棋盘格')
        self.capture_button.clicked.connect(self.capture)
        self.auto_button = QCheckBox('自动采样（按 YAML 间隔和质量条件）')
        left.addWidget(self.capture_button); left.addWidget(self.auto_button)
        self.import_button = QPushButton('导入原始图片（离线数据集）')
        self.import_button.clicked.connect(self.import_images)
        left.addWidget(self.import_button)
        progress = QGroupBox('采样覆盖提示 · X / Y / Size / Skew')
        progress_layout = QVBoxLayout(progress)
        self.progress = []
        for text in ('横向位置', '纵向位置', '棋盘大小', '透视变化'):
            bar = QProgressBar()
            bar.setFormat(text+' %p%')
            progress_layout.addWidget(bar)
            self.progress.append(bar)
        left.addWidget(progress)
        self.solve_button = QPushButton('计算内参')
        self.solve_button.clicked.connect(self.solve)
        self.undistort_button = QCheckBox('去畸变预览（保留 K，不裁剪）')
        self.save_button = QPushButton('保存内参 YAML 与误差报告')
        self.save_button.clicked.connect(self.export)
        for widget in (self.solve_button, self.undistort_button, self.save_button): left.addWidget(widget)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(120)
        left.addWidget(self.log, 1)
        split.addWidget(panel)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        self.preview = QLabel('尚未打开相机\n先在 YAML 填写棋盘格参数；也可导入原始图片')
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview.setMinimumSize(420, 310)
        self.preview.setStyleSheet('background: #17232d; color: #dce7ee; border-radius: 6px')
        right_layout.addWidget(self.preview, 3)
        self.camera_label = QLabel('相机信息与实际分辨率将在打开设备后显示')
        self.camera_label.setWordWrap(True)
        right_layout.addWidget(self.camera_label)
        self.samples = QListWidget()
        self.samples.setMaximumHeight(155)
        self.remove_button = QPushButton('移除选中样本（原始图片保留）')
        self.remove_button.clicked.connect(self.remove)
        right_layout.addWidget(self.samples)
        right_layout.addWidget(self.remove_button)
        self.result_label = QLabel('尚未计算内参；低重投影误差也不能替代实物几何核验。')
        self.result_label.setWordWrap(True)
        self.result_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        right_layout.addWidget(self.result_label)
        split.addWidget(right)
        split.setSizes([365, 885])
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)
        self.update_summary()
        self.refresh()

    def note(self, text):
        self.log.append(time.strftime('%H:%M:%S')+'  '+text)

    def update_summary(self):
        camera = self.settings.values['camera']
        board = self.settings.board
        output_label = QFontMetrics(self.summary.font()).elidedText(str(self.settings.output_root), Qt.ElideMiddle, 280)
        self.summary.setToolTip(f'配置：{self.config_path}\n输出：{self.settings.output_root}')
        self.summary.setText(
            f"目标：{camera['expected_model']}\n序列号：{camera['serial'] or '自动匹配唯一设备'}\n"
            f"棋盘：{f'{board[0]} 列 × {board[1]} 行内角点，{board[2]} mm / 格' if board else '未填写，请编辑 YAML'}\n"
            f"曝光：{camera['exposure_us']} μs · 镜头：{camera['lens'] or '未记录'}\n"
            f"输出：{output_label}\n配置：{self.config_path.name}")

    def reload(self):
        if self.camera is not None or self.job is not None:
            return
        try:
            settings = load_settings(self.config_path)
        except Exception as error:
            self.note(f'配置未加载：{error}')
            return
        self.settings = settings
        self.dataset = Dataset(settings)
        self.bundle = self.result = None
        self.auto_button.setChecked(False)
        self.samples.clear()
        self.result_label.setText('已新建数据集；此前采集的原始文件仍保留。')
        self.update_summary()
        self.note('已读取 YAML；所有后续样本使用本次配置快照。')

    def open_camera(self):
        if self.camera is not None or self.job is not None:
            return
        self.bundle = None
        self.camera = CameraWorker(self.settings)
        self.camera.error.connect(self.note)
        self.camera.finished.connect(self.camera_stopped)
        self.camera.start()
        self.note('正在打开迈德威视相机，不改写分辨率、ROI 或缩放模式。')

    def stop_camera(self):
        self.auto_button.setChecked(False)
        if self.camera is not None:
            self.camera.requestInterruption()

    def camera_stopped(self):
        self.camera.deleteLater()
        self.camera = None
        self.note('采集线程已释放相机。')
        if self.closing:
            self.close()

    def capture(self):
        if self.bundle is None or self.camera is None or self.job is not None:
            return
        frame, detection, metadata, received = self.bundle
        try:
            if time.monotonic()-received > self.settings.values['capture']['max_frame_age_sec']:
                raise ValueError('当前帧已过期，请等待新的相机画面')
            self.dataset.add(frame, detection, metadata)
        except (ValueError, OSError) as error:
            if str(error) != self.last_rejection:
                self.note(f'未采集：{error}')
                self.last_rejection = str(error)
            return
        self.result = None
        self.last_rejection = ''
        self.update_samples()
        self.note(f'已保存第 {len(self.dataset.views)} 张有效原始图像。')

    def update_samples(self):
        self.samples.clear()
        for index, view in enumerate(self.dataset.views):
            error = f" · RMS {self.result['per_view_rms_px'][index]:.3f}px" if self.result is not None else ''
            self.samples.addItem(f"{Path(view['file']).name} · 清晰度 {view['sharpness']:.0f} · 面积 {view['area_ratio']:.1%}{error}")
        self.result_label.setText('样本已更新，请重新计算内参。' if self.result is None else self.result_label.text())

    def remove(self):
        if self.job is not None:
            return
        index = self.samples.currentRow()
        if index >= 0:
            try:
                self.dataset.remove(index)
            except OSError as error:
                self.note(f'未移除样本，清单无法写入：{error}')
                return
            self.result = None
            self.update_samples()

    def start_job(self, function, done):
        if self.job is not None:
            return
        self.auto_button.setChecked(False)
        self.job = Job(function)
        self.job.completed.connect(done)
        self.job.failed.connect(lambda error: self.note('操作失败：'+error))
        self.job.finished.connect(self.job_finished)
        self.job.start()
        self.refresh()

    def job_finished(self):
        self.job.deleteLater()
        self.job = None
        if self.closing:
            self.close()

    def import_images(self):
        if self.job is not None or self.camera is not None:
            return
        paths, _ = QFileDialog.getOpenFileNames(self, '选择同一相机/模式的原始图像', '', 'Images (*.png *.jpg *.jpeg *.bmp *.tiff)')
        if not paths:
            return
        def load():
            messages = []
            last = None
            for path in paths:
                try:
                    frame = cv2.imread(path, cv2.IMREAD_COLOR)
                    if frame is None:
                        raise ValueError('无法读取原始图像')
                    detection = detect_board(frame, self.settings)
                    metadata = {'camera_mode': 'imported_images', 'device': {}, 'resolution': None,
                                'expected_model': self.settings.values['camera']['expected_model'],
                                'acquired_at': None, 'timestamp_basis': 'unknown_imported_image'}
                    self.dataset.add(frame, detection, metadata, source_file=str(Path(path).resolve()))
                    last = (frame, detection, metadata, 0)
                except (ValueError, OSError, cv2.error) as error:
                    messages.append(f'{Path(path).name}：{error}')
            return last, messages
        def done(result):
            last, messages = result
            if last is not None:
                self.bundle = last
                self.result = None
            self.update_samples()
            self.note(f'导入完成，累计有效样本 {len(self.dataset.views)} 张；未通过 {len(messages)} 张。')
            for message in messages: self.note(message)
        self.start_job(load, done)

    def solve(self):
        if self.job is not None:
            return
        self.result = None
        self.note('正在后台计算内参；逐张误差完成后列在样本列表，可手动移除并重新求解。')
        def done(result):
            self.result = result
            self.update_samples()
            matrix = result['matrix']
            warnings = '；'.join(result['warnings']) or '拟合与采样提示通过；仍需实物核验'
            self.result_label.setText(
                f"整体 RMS：{result['rms_px']:.4f} px · 最大逐张 RMS：{max(result['per_view_rms_px']):.4f} px\n"
                f"fx={matrix[0,0]:.3f}，fy={matrix[1,1]:.3f}，cx={matrix[0,2]:.3f}，cy={matrix[1,2]:.3f}\n"
                f"D={np.array2string(result['distortion'].reshape(-1), precision=5)}\n{warnings}")
            self.note('求解完成，参数尚未应用到比赛主程序。')
        self.start_job(self.dataset.solve, done)

    def export(self):
        if self.job is not None or self.result is None:
            return
        try:
            destination = self.dataset.export(self.result)
            self.note(f'已保存：{destination}')
        except (OSError, ValueError) as error:
            self.note(f'结果未保存：{error}')

    def refresh(self):
        busy = self.job is not None
        live = self.camera is not None
        if busy:
            for widget in (self.open_button, self.reload_button, self.import_button, self.capture_button,
                           self.auto_button, self.remove_button, self.solve_button, self.save_button):
                widget.setEnabled(False)
            return  # Import/solve owns the mutable dataset until completion.
        if live:
            latest = self.camera.snapshot()
            if latest is not None:
                self.bundle = latest
        self.open_button.setEnabled(not live and not busy and (not self.dataset.views or self.dataset.metadata['camera_mode'] == 'mindvision'))
        self.stop_button.setEnabled(live)
        self.reload_button.setEnabled(not live and not busy)
        self.import_button.setEnabled(not live and not busy and self.settings.board is not None
                                      and (not self.dataset.views or self.dataset.metadata['camera_mode'] == 'imported_images'))
        ready = live and not busy and self.settings.board is not None and self.bundle is not None
        self.capture_button.setEnabled(ready)
        self.auto_button.setEnabled(ready)
        self.remove_button.setEnabled(not busy and bool(self.dataset.views))
        self.solve_button.setEnabled(not busy and len(self.dataset.views) >= self.settings.values['capture']['min_samples'])
        self.save_button.setEnabled(not busy and self.result is not None)
        self.undistort_button.setEnabled(self.result is not None)
        for bar, value in zip(self.progress, coverage(self.dataset.views, self.settings)):
            bar.setValue(value)
        if self.auto_button.isChecked() and ready and time.monotonic() >= self.next_auto:
            self.next_auto = time.monotonic()+self.settings.values['capture']['auto_interval_sec']
            self.capture()
        if self.bundle is not None:
            frame, detection, metadata, _ = self.bundle
            display = frame.copy()
            config = self.settings.values['camera']
            current_profile = profile(dict(metadata, lens=config['lens'], focus=config['focus']))
            compatible = self.dataset.metadata is None or current_profile == profile(self.dataset.metadata)
            self.undistort_button.setEnabled(self.result is not None and compatible)
            if self.undistort_button.isChecked() and self.result is not None and compatible:
                display = cv2.undistort(display, self.result['matrix'], self.result['distortion'], None, self.result['matrix'])
            elif detection.get('corners') is not None:
                cv2.drawChessboardCorners(display, self.settings.board[:2], detection['corners'], True)
            rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
            image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
            self.preview.setPixmap(QPixmap.fromImage(image).scaled(self.preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
            device = metadata.get('device') or {}
            self.camera_label.setText(
                f"{'实时' if live else '最后一帧 / 离线'} · {frame.shape[1]}×{frame.shape[0]} · "
                f"SDK 产品：{device.get('product_name') or '未核对'} · 序列号：{device.get('serial') or '未核对'}\n"
                f"{detection.get('reason')} · 有效样本 {len(self.dataset.views)} / 最少 {self.settings.values['capture']['min_samples']}")

    def closeEvent(self, event):
        if self.camera is not None or self.job is not None:
            self.closing = True
            self.stop_camera()
            event.ignore()
        else:
            self.timer.stop()
            event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=Path(__file__).with_name('intrinsic_calibration.yaml'))
    args = parser.parse_args()
    app = QApplication(sys.argv[:1])
    try:
        window = IntrinsicWindow(args.config)
    except Exception as error:
        QMessageBox.critical(None, '无法读取内参标定 YAML', str(error))
        return 1
    window.show()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
