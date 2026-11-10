"""棋盘格内参数据集、采样检查与 OpenCV 求解；不改变比赛图像链路。"""
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid

import cv2
import numpy as np
import yaml


def numeric(value, name, low, high, integral=False):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high
            or (integral and not isinstance(value, int))):
        raise ValueError(f'{name} 应为 {low}～{high} 的' + ('整数' if integral else '数值'))
    return value


class ConfigurationLoader(yaml.SafeLoader):
    pass


def unique_mapping(loader, node):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if key in mapping:
            raise ValueError(f'YAML 字段重复：{key}')
        mapping[key] = loader.construct_object(value_node)
    return mapping


ConfigurationLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


@dataclass(frozen=True)
class Settings:
    path: Path
    text: str
    values: dict

    @property
    def board(self):
        board = self.values['board']
        if any(v is None for v in board.values()):
            return None
        return (board['columns'], board['rows'], board['square_size_mm'])

    @property
    def output_root(self):
        value = Path(self.values['output']['directory']).expanduser()
        return value.resolve() if value.is_absolute() else (self.path.parent/value).resolve()


def load_settings(path):
    path = Path(path).expanduser().resolve()
    text = path.read_text(encoding='utf-8')
    values = yaml.load(text, Loader=ConfigurationLoader)
    expected = {
        'camera': {'expected_model', 'serial', 'exposure_us', 'frame_timeout_ms', 'preview_fps',
                   'expected_width', 'expected_height', 'lens', 'focus'},
        'board': {'columns', 'rows', 'square_size_mm'},
        'capture': {'min_samples', 'max_samples', 'min_sharpness', 'min_board_area_ratio',
                    'min_pose_distance', 'auto_interval_sec', 'max_frame_age_sec'},
        'coverage': {'x_span', 'y_span', 'size_span', 'skew_span'},
        'calibration': {'distortion_model', 'max_rms_px'}, 'output': {'directory'},
    }
    if not isinstance(values, dict) or set(values) != set(expected) | {'schema'}:
        raise ValueError('YAML 顶层字段不完整或包含未知字段')
    if values['schema'] != 'shark.radar.intrinsic_tool.v1':
        raise ValueError('不支持的内参工具配置 schema')
    for section, keys in expected.items():
        if not isinstance(values[section], dict) or set(values[section]) != keys:
            raise ValueError(f'请核对 {section} 字段：{sorted(keys)}')
    camera, board, capture = (values[k] for k in ('camera', 'board', 'capture'))
    for key in ('expected_model', 'serial', 'lens', 'focus'):
        if not isinstance(camera[key], str):
            raise ValueError(f'camera.{key} 应为字符串')
    if not camera['expected_model'].strip():
        raise ValueError('请填写 camera.expected_model')
    numeric(camera['exposure_us'], 'camera.exposure_us', 4, 32400)
    numeric(camera['frame_timeout_ms'], 'camera.frame_timeout_ms', 100, 2000, True)
    numeric(camera['preview_fps'], 'camera.preview_fps', 1, 30)
    sizes = [camera[k] for k in ('expected_width', 'expected_height')]
    if (sizes[0] is None) != (sizes[1] is None):
        raise ValueError('expected_width 和 expected_height 应同时填写或同时留空')
    for value in sizes:
        if value is not None:
            numeric(value, 'camera.expected_width/height', 64, 16384, True)
    for key in ('columns', 'rows'):
        if board[key] is not None:
            numeric(board[key], f'board.{key}', 3, 30, True)
    if board['square_size_mm'] is not None:
        numeric(board['square_size_mm'], 'board.square_size_mm', 0.01, 1000)
    if board['columns'] and board['rows'] and board['columns']*board['rows'] > 600:
        raise ValueError('当前工具最多支持 600 个内角点')
    numeric(capture['min_samples'], 'capture.min_samples', 10, 200, True)
    numeric(capture['max_samples'], 'capture.max_samples', capture['min_samples'], 200, True)
    for key, low, high in (('min_sharpness', 0, 100000), ('min_board_area_ratio', .001, .8),
                           ('min_pose_distance', .001, .5), ('auto_interval_sec', .3, 60),
                           ('max_frame_age_sec', .1, 10)):
        numeric(capture[key], 'capture.'+key, low, high)
    for key, value in values['coverage'].items():
        numeric(value, 'coverage.'+key, .01, 1)
    if values['calibration']['distortion_model'] != 'plumb_bob':
        raise ValueError('当前只支持 plumb_bob 五参数针孔模型，不支持鱼眼模型')
    numeric(values['calibration']['max_rms_px'], 'calibration.max_rms_px', .01, 10)
    if not isinstance(values['output']['directory'], str) or not values['output']['directory'].strip():
        raise ValueError('output.directory 不能为空')
    return Settings(path, text, values)


def detect_board(image, settings):
    if settings.board is None:
        return {'corners': None, 'reason': '请在 YAML 填写内角点列数、行数和方格边长'}
    columns, rows, _ = settings.board
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2, 'findChessboardCornersSB'):
        found, corners = cv2.findChessboardCornersSB(gray, (columns, rows), cv2.CALIB_CB_NORMALIZE_IMAGE)
    else:
        found, corners = cv2.findChessboardCorners(gray, (columns, rows))
        if found:
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                                      (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
    if not found:
        return {'corners': None, 'reason': '未检测到完整棋盘格，请检查内角点行列数、曝光和摆放'}
    points = corners.reshape(rows, columns, 2)
    hull = np.float32([points[0, 0], points[0, -1], points[-1, -1], points[-1, 0]])
    area = abs(cv2.contourArea(hull))/(gray.shape[0]*gray.shape[1])
    x, y, width, height = cv2.boundingRect(hull)
    patch = gray[max(0, y):min(gray.shape[0], y+height), max(0, x):min(gray.shape[1], x+width)]
    sharpness = float(cv2.Laplacian(patch, cv2.CV_64F).var())
    horizontal = points[0, -1]-points[0, 0]
    vertical = points[-1, 0]-points[0, 0]
    sx, sy = np.linalg.norm(horizontal)/(columns-1), np.linalg.norm(vertical)/(rows-1)
    skew = max(1-min(sx, sy)/max(sx, sy),
               abs(float(horizontal@vertical))/(np.linalg.norm(horizontal)*np.linalg.norm(vertical)))
    descriptor = [float(corners[:, 0, 0].mean()/gray.shape[1]),
                  float(corners[:, 0, 1].mean()/gray.shape[0]), math.sqrt(area), float(skew)]
    return {'corners': corners.astype(np.float32), 'sharpness': sharpness, 'area_ratio': area,
            'descriptor': descriptor, 'reason': '检测到完整棋盘格'}


def coverage(views, settings):
    if len(views) < 2:
        return [0]*4
    descriptors = np.array([v['descriptor'] for v in views])
    targets = np.array([settings.values['coverage'][k] for k in ('x_span', 'y_span', 'size_span', 'skew_span')])
    return np.minimum(100, np.ptp(descriptors, axis=0)/targets*100).astype(int).tolist()


def write_json(path, data):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    temporary.replace(path)


def profile(metadata):
    return {k: metadata.get(k) for k in ('camera_mode', 'device', 'resolution', 'image_size', 'lens', 'focus')}


class Dataset:
    def __init__(self, settings):
        self.settings = settings
        self.path = None
        self.metadata = None
        self.views = []
        self.excluded = []
        self.counter = 0

    def add(self, frame, detection, metadata, source_file=None):
        if self.settings.board is None or detection.get('corners') is None:
            raise ValueError(detection.get('reason', '请先完整填写标定板参数'))
        cfg = self.settings.values['capture']
        if len(self.views) >= cfg['max_samples']:
            raise ValueError('已达到 YAML 设置的最大采样数')
        if detection['sharpness'] < cfg['min_sharpness']:
            raise ValueError(f"棋盘区域清晰度不足：{detection['sharpness']:.1f}")
        if detection['area_ratio'] < cfg['min_board_area_ratio']:
            raise ValueError('棋盘格太小，请靠近或调整拍摄距离')
        metadata = dict(metadata, image_size=[frame.shape[1], frame.shape[0]],
                        lens=self.settings.values['camera']['lens'], focus=self.settings.values['camera']['focus'])
        expected = self.settings.values['camera']
        if expected['expected_width'] is not None and metadata['image_size'] != [expected['expected_width'], expected['expected_height']]:
            raise ValueError('当前帧尺寸与 YAML 预期尺寸不匹配，不缩放图像')
        if self.metadata is not None and profile(metadata) != profile(self.metadata):
            raise ValueError('相机身份、ROI/缩放模式或帧尺寸变化，请新建数据集并重新采样')
        normalized = detection['corners'].reshape(-1, 2)/np.array(metadata['image_size'])
        for view in self.views:
            previous = view['corners'].reshape(-1, 2)/np.array(metadata['image_size'])
            distance = min(np.sqrt(np.mean(np.sum((normalized-order)**2, axis=1)))
                           for order in (previous, previous[::-1]))
            if distance < cfg['min_pose_distance']:
                raise ValueError('与已有样本姿态太接近，请改变位置、距离或倾斜角度')
        if self.path is None:
            name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
            self.path = self.settings.output_root/name
            (self.path/'images').mkdir(parents=True)
            (self.path/'settings.yaml').write_text(self.settings.text, encoding='utf-8')
        success, encoded = cv2.imencode('.png', frame)
        if not success:
            raise OSError('无法编码原始 PNG 图像')
        self.counter += 1
        filename = f'images/view_{self.counter:04d}.png'
        raw = encoded.tobytes()
        with (self.path/filename).open('xb') as stream:
            stream.write(raw)
        view = dict(detection, file=filename, sha256=hashlib.sha256(raw).hexdigest(),
                    acquired_at=metadata.get('acquired_at'), source_file=source_file,
                    capture_metadata=metadata)
        self.views.append(view)
        previous_metadata = self.metadata
        self.metadata = metadata
        try:
            self.persist()
        except OSError:
            self.views.pop()
            self.metadata = previous_metadata
            raise

    def persist(self):
        if self.path is not None:
            views = [dict(v, corners=v['corners'].reshape(-1, 2).tolist()) for v in self.views]
            write_json(self.path/'session.json', {
                'schema': 'shark.radar.intrinsic_dataset.v1', 'metadata': self.metadata,
                'config_sha256': hashlib.sha256(self.settings.text.encode()).hexdigest(),
                'views': views, 'excluded_images': self.excluded,
            })

    def remove(self, index):
        view = self.views.pop(index)
        self.excluded.append(view['file'])
        try:
            self.persist()  # Original images are retained for review.
        except OSError:
            self.views.insert(index, view)
            self.excluded.pop()
            raise

    def solve(self):
        if len(self.views) < self.settings.values['capture']['min_samples']:
            raise ValueError('有效样本数未达到 YAML 的 min_samples')
        columns, rows, square_mm = self.settings.board
        objects = np.zeros((rows*columns, 3), np.float32)
        objects[:, :2] = np.mgrid[:columns, :rows].T.reshape(-1, 2)*square_mm
        image_points = [v['corners'] for v in self.views]
        rms, matrix, distortion, rvecs, tvecs, std_k, _, errors = cv2.calibrateCameraExtended(
            [objects.copy() for _ in self.views], image_points,
            tuple(self.metadata['image_size']), None, None, flags=0)
        if (not all(np.isfinite(v).all() for v in (matrix, distortion, std_k, errors, np.array(rvecs), np.array(tvecs)))
                or not math.isfinite(rms) or min(matrix[0, 0], matrix[1, 1]) <= 0
                or distortion.size != 5):
            raise ValueError('内参解算不稳定或包含无效数值，请增加不同角度/距离的样本')
        warnings = []
        if rms > self.settings.values['calibration']['max_rms_px']:
            warnings.append('整体重投影 RMS 超过 YAML 审阅阈值')
        if min(coverage(self.views, self.settings)) < 100:
            warnings.append('X/Y/Size/Skew 采样覆盖尚未达到配置目标')
        if not self.metadata.get('lens') or not self.metadata.get('focus'):
            warnings.append('未完整记录镜头与对焦信息')
        if self.metadata.get('camera_mode') != 'mindvision':
            warnings.append('离线导入图像，未通过 SDK 核对相机身份与采集模式')
        else:
            if not self.metadata.get('resolution'):
                warnings.append('SDK 未提供完整 ROI/缩放模式信息')
            if (self.metadata.get('device') or {}).get('product_name') != self.metadata.get('expected_model'):
                warnings.append('SDK 实际产品名与 YAML 目标型号不一致，请核对实物及厂商命名')
        if not 0 <= matrix[0, 2] < self.metadata['image_size'][0] or not 0 <= matrix[1, 2] < self.metadata['image_size'][1]:
            warnings.append('主点位于图像范围之外，请核对 ROI 与采样质量')
        return {'matrix': matrix, 'distortion': distortion, 'rvecs': np.array(rvecs),
                'tvecs_mm': np.array(tvecs), 'rms_px': rms, 'std_intrinsics': std_k.reshape(-1).tolist(),
                'per_view_rms_px': errors.reshape(-1).tolist(), 'warnings': warnings}

    def export(self, result):
        name = 'solution-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'-'+uuid.uuid4().hex[:8]
        destination = self.path/name
        destination.mkdir()
        matrix, distortion = result['matrix'], result['distortion']
        def block(array):
            return dict(rows=array.shape[0], cols=array.shape[1], data=array.reshape(-1).tolist())
        device = self.metadata.get('device') or {}
        camera_info = {
            'image_width': self.metadata['image_size'][0], 'image_height': self.metadata['image_size'][1],
            'camera_name': 'mindvision_'+(device.get('serial') or 'imported'),
            'distortion_model': 'plumb_bob', 'camera_matrix': block(matrix),
            'distortion_coefficients': block(distortion.reshape(1, 5)),
            'rectification_matrix': block(np.eye(3)),
            'projection_matrix': block(np.column_stack((matrix, np.zeros(3)))),
        }
        (destination/'camera_info.yaml').write_text(yaml.safe_dump(camera_info, sort_keys=False), encoding='utf-8')
        report = {k: v for k, v in result.items() if k not in ('matrix', 'distortion', 'rvecs', 'tvecs_mm')}
        report.update(schema='shark.radar.camera_intrinsics.v1', metadata=self.metadata,
                      board=self.settings.values['board'], opencv_version=cv2.__version__, calibration_flags=0,
                      distortion_order=['k1', 'k2', 'p1', 'p2', 'k3'], camera_info=camera_info,
                      coverage_percent=coverage(self.views, self.settings), physical_validation_required=True,
                      views=[{'file': v['file'], 'sha256': v['sha256']} for v in self.views],
                      config_sha256=hashlib.sha256(self.settings.text.encode()).hexdigest())
        write_json(destination/'report.json', report)
        np.savez_compressed(destination/'observations.npz',
                            image_points=np.array([v['corners'] for v in self.views]),
                            camera_matrix=matrix, distortion=distortion, rvecs=result['rvecs'],
                            tvecs_mm=result['tvecs_mm'], image_size=self.metadata['image_size'])
        return destination
