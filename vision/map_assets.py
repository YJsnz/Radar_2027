"""标定、比赛与网页共用的版本化地图资源；不打开设备。"""
from copy import deepcopy
import hashlib
import json
import math
import shutil
from pathlib import Path

import yaml


VISION_ROOT = Path(__file__).resolve().parent
DEFAULT_RUNTIME_PATH = VISION_ROOT / 'map_runtime.yaml'


def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_profile(path):
    path = Path(path).resolve()
    profile = json.loads(path.read_text(encoding='utf-8'))
    if (profile.get('schema') != 'ysu.radar.map_profile.v1'
            or profile.get('season') != 2027
            or not profile.get('id')
            or profile.get('coordinate_system') != 'rmuc_reference_xy_cm'
            or profile.get('field_size_cm') != [2800, 1500]
            or profile.get('image_size') != [1500, 2800]
            or set(profile.get('sides', {})) != {'R', 'B'}):
        raise ValueError('不支持的地图档案／尺寸／阵营')
    entries = [profile['topdown']]
    for side in ('R', 'B'):
        entries.extend(profile['sides'][side][key] for key in ('image', 'mask', 'points'))
    for entry in entries:
        target = (path.parent / entry['file']).resolve()
        if not target.is_relative_to(path.parent) or not target.is_file():
            raise ValueError(f'地图资源缺失或路径越界：{entry["file"]}')
        if file_sha256(target) != entry['sha256']:
            raise ValueError(f'地图资源校验失败：{entry["file"]}')
    for side, entry in profile['sides'].items():
        points = json.loads((path.parent / entry['points']['file']).read_text(encoding='utf-8'))
        layers = points.get('map_points', [])
        if (points.get('side') != side or points.get('map_sha256') != entry['image']['sha256']
                or points.get('image_size') != profile['image_size']
                or len(layers) != 2 or any(len(layer) != 6 for layer in layers)
                or any(len(point) != 2 or any(not math.isfinite(value) or value < 0 or value >= limit
                                             for value, limit in zip(point, profile['image_size']))
                       for layer in layers for point in layer)):
            raise ValueError(f'{side} 方地图点的阵营／底图／点数／坐标无效')
    profile['_directory'] = str(path.parent)
    profile['_path'] = str(path)
    profile['_sha256'] = file_sha256(path)
    return profile


def profile_binding(profile, side):
    if side not in ('R', 'B'):
        raise ValueError(f'无效地图阵营：{side}')
    entry = profile['sides'][side]
    return {
        'profile_id': profile['id'], 'profile_sha256': profile['_sha256'],
        'side': side, 'coordinate_system': profile['coordinate_system'],
        'map_sha256': entry['image']['sha256'], 'mask_sha256': entry['mask']['sha256'],
        'points_sha256': entry['points']['sha256'],
        'topdown_sha256': profile['topdown']['sha256'],
    }


def apply_map_runtime(config, runtime_path=DEFAULT_RUNTIME_PATH):
    """在内存覆盖赛季素材；禁用时返回既有配置，不回写设备 YAML。"""
    config = deepcopy(config)
    runtime_path = Path(runtime_path)
    runtime = yaml.safe_load(runtime_path.read_text(encoding='utf-8'))
    if runtime.get('schema') != 'ysu.radar.map_runtime.v1':
        raise ValueError('无效地图运行配置')
    if not runtime.get('enabled', False):
        return config
    profile = load_profile(runtime_path.parent / runtime['profile'])
    directory = Path(profile['_directory'])
    config['_map_profile'] = profile
    config.setdefault('global', {})['map_size'] = profile['field_size_cm']
    config.setdefault('paths', {})['calibration'] = {
        'red': str(VISION_ROOT / 'calibration-2027/red.npy'),
        'blue': str(VISION_ROOT / 'calibration-2027/blue.npy'),
    }
    config['paths']['map_images'] = {
        'red': str(directory / profile['sides']['R']['image']['file']),
        'blue': str(directory / profile['sides']['B']['image']['file']),
        'backup': str(directory / profile['topdown']['file']),
    }
    for side in ('R', 'B'):
        config['paths']['map_images'][f'mask_{side}'] = str(directory / profile['sides'][side]['mask']['file'])
    config.setdefault('calibration', {}).update({
        'points_per_layer': 6,
        'presets_path': str(VISION_ROOT / 'calibration_presets_2027.json'),
    })
    # 新相机盲区尚未实测，阻断两套算法读取旧赛季固定预测点。
    config.setdefault('blind_zone', {}).update({'enabled': False, 'points': {}, 'roles': []})
    select_map_side(config, config['global']['state'])
    return config


def select_map_side(config, side):
    profile = config.get('_map_profile')
    if profile is not None:
        config['paths']['map_images']['mask'] = config['paths']['map_images'][f'mask_{side}']
        return profile_binding(profile, side)
    return None


def map_binding(config, side):
    profile = config.get('_map_profile')
    return profile_binding(profile, side) if profile is not None else None


def default_preset_name(side):
    return f'2027 V1.0.0 {"红方" if side == "R" else "蓝方"}候选六点/层'


def snapshot_map_assets(config, side, recording_directory):
    """归档原始素材及实际矩阵，避免后续重标覆盖录像的几何依据。"""
    from camera_calibration import calibration_points_path, validate_map_calibration

    profile = config.get('_map_profile')
    if profile is None:
        return None
    binding = profile_binding(profile, side)
    matrix = Path(config['paths']['calibration']['red' if side == 'R' else 'blue'])
    validate_map_calibration(matrix, binding)
    target = Path(recording_directory) / 'map_assets'
    target.mkdir(parents=True, exist_ok=True)
    source = Path(profile['_directory'])
    entries = [profile['topdown']]
    for entry in profile['sides'].values():
        entries.extend(entry[key] for key in ('image', 'mask', 'points'))
    for entry in entries:
        shutil.copy2(source / entry['file'], target / entry['file'])
    shutil.copy2(profile['_path'], target / 'profile.json')
    if profile_binding(load_profile(target / 'profile.json'), side) != binding:
        raise ValueError('录制开始时地图素材已变化，请重新加载并标定')
    shutil.copy2(matrix, target / matrix.name)
    shutil.copy2(calibration_points_path(matrix), target / calibration_points_path(matrix).name)
    validate_map_calibration(target / matrix.name, binding)
    return {'directory': 'map_assets', 'map_binding': binding,
            'calibration_file': matrix.name, 'calibration_sha256': file_sha256(target / matrix.name)}
