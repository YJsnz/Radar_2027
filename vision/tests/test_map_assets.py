"""使用真实资产与临时标定夹具检查赛季、阵营及图像域隔离；不创建实机结果。"""
import json
from pathlib import Path
import shutil
import os
import subprocess
import sys

import numpy as np
from PIL import Image
import pytest
import yaml

from calibration_presets import list_presets, load_presets, save_preset
from camera_calibration import validate_camera_calibration, validate_map_calibration
from map_assets import apply_map_runtime, file_sha256, load_profile, map_binding, profile_binding, snapshot_map_assets
from vision_telemetry import build_vision_telemetry


ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'radio/apps/match_rx/web/assets/map2027/profile.json'


def effective_config(side='B'):
    config = yaml.safe_load((ROOT / 'vision/config.yaml').read_text())
    config['global']['state'] = side
    return apply_map_runtime(config)


def write_fixture(directory, binding, intrinsics_id=''):
    """数值夹具仅位于 pytest 临时目录，不是相机标定。"""
    path = directory / 'synthetic.npy'
    matrices = np.array([np.eye(3), np.eye(3)])
    np.save(path, matrices)
    np.savez(path.with_name('synthetic_points.npz'), matrices=matrices,
             camera_mode=np.array('mindvision'), image_width=1280, image_height=1024,
             intrinsics_id=np.array(intrinsics_id),
             map_binding=np.array(json.dumps(binding)), matrix_sha256=np.array(file_sha256(path)))
    return path


@pytest.mark.parametrize('side', ['R', 'B'])
def test_real_assets_presets_and_side_specific_masks(side):
    config = effective_config(side)
    binding = map_binding(config, side)
    presets = list_presets(config['calibration']['presets_path'], side=side, map_binding=binding)
    assert len(presets) == 1 and presets[0]['points_per_layer'] == 6
    canonical = json.loads((ROOT / f'docs/2027-adaptation/map2027-selected-points-v1/points_{side}.json').read_text())
    assert presets[0]['map_points'] == canonical['map_points']
    source = 'red_bottom.png' if side == 'R' else 'blue_bottom.png'
    assert np.array_equal(np.asarray(Image.open(config['paths']['map_images']['red' if side == 'R' else 'blue'])),
                          np.asarray(Image.open(ROOT / 'vision/images-2027/display-v1.0.0' / source)))
    mask = np.asarray(Image.open(config['paths']['map_images']['mask']))
    assert mask.shape == (2800, 1500, 3)
    assert set(map(tuple, np.unique(mask.reshape(-1, 3), axis=0))) == {(0, 0, 0), (0, 255, 0)}
    assert not list_presets(ROOT / 'vision/calibration_presets.json', side=side, map_binding=binding)


def test_runtime_overlay_preserves_local_config_and_disables_old_predictions():
    base = yaml.safe_load((ROOT / 'vision/config.yaml').read_text())
    original = json.dumps(base, sort_keys=True)
    result = apply_map_runtime(base)
    assert json.dumps(base, sort_keys=True) == original
    for key in ['camera_params', 'recording', 'serial', 'referee']:
        assert result[key] == base[key]
    assert result['blind_zone']['enabled'] is False and result['blind_zone']['points'] == {}
    assert result['paths']['calibration']['blue'].endswith('calibration-2027/blue.npy')


def test_opt_out_keeps_legacy_config(tmp_path):
    runtime = tmp_path / 'runtime.yaml'
    runtime.write_text('schema: ysu.radar.map_runtime.v1\nenabled: false\n')
    base = {'global': {'state': 'B'}, 'blind_zone': {'enabled': True}}
    assert apply_map_runtime(base, runtime) == base


def test_corrupt_asset_and_cross_side_points_are_rejected(tmp_path):
    directory = tmp_path / 'assets'
    shutil.copytree(PROFILE.parent, directory)
    manifest = json.loads((directory / 'profile.json').read_text())
    (directory / manifest['sides']['B']['mask']['file']).write_bytes(b'broken PNG')
    with pytest.raises(ValueError, match='校验失败'):
        load_profile(directory / 'profile.json')
    shutil.copy2(PROFILE.parent / manifest['sides']['B']['mask']['file'], directory / manifest['sides']['B']['mask']['file'])
    manifest['sides']['R']['points'] = manifest['sides']['B']['points']
    (directory / 'profile.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='阵营'):
        load_profile(directory / 'profile.json')


def test_missing_and_old_calibration_are_rejected():
    binding = map_binding(effective_config(), 'B')
    with pytest.raises(ValueError, match='尚未完成 2027'):
        validate_map_calibration(ROOT / 'vision/calibration-2027/nonexistent.npy', binding)
    with pytest.raises(ValueError, match='档案校验失败'):
        validate_map_calibration(ROOT / 'vision/arrays_test_blue.npy', binding)


def test_bound_pair_accepts_correct_side_and_rejects_other_side_and_tampering(tmp_path):
    profile = load_profile(PROFILE)
    binding = profile_binding(profile, 'B')
    path = write_fixture(tmp_path, binding)
    validate_map_calibration(path, binding)
    validate_camera_calibration(path, 'mindvision', (1024, 1280, 3), map_binding=binding)
    with pytest.raises(ValueError, match='不匹配'):
        validate_map_calibration(path, profile_binding(profile, 'R'))
    np.save(path, np.array([np.eye(3) * 2, np.eye(3)]))
    with pytest.raises(ValueError, match='校验值'):
        validate_map_calibration(path, binding)


def test_disabling_intrinsics_and_changing_frame_size_require_recalibration(tmp_path):
    binding = map_binding(effective_config(), 'B')
    path = write_fixture(tmp_path, binding, 'real-domain-id-fixture')
    validate_camera_calibration(path, 'mindvision', (1024, 1280, 3), 'real-domain-id-fixture', binding)
    with pytest.raises(ValueError, match='相机内参'):
        validate_camera_calibration(path, 'mindvision', (1024, 1280, 3), None, binding)
    with pytest.raises(ValueError, match='相机不匹配'):
        validate_camera_calibration(path, 'mindvision', (720, 1280, 3), 'real-domain-id-fixture', binding)


def test_saved_custom_preset_keeps_binding(tmp_path):
    config = effective_config()
    binding = map_binding(config, 'B')
    preset = list_presets(config['calibration']['presets_path'], side='B', map_binding=binding)[0]
    path = tmp_path / 'presets.json'
    save_preset('custom', 'B', preset['map_points'], path=path, map_binding=binding)
    assert load_presets(path)[0]['map_binding'] == binding


def test_telemetry_carries_map_identity_without_changing_robot_coordinates():
    binding = map_binding(effective_config(), 'B')
    payload = build_vision_telemetry(side='B', send_map={'R1': (800, 600)}, valid_names={'R1'},
                                    guess_list={}, occlusion_names=set(), camera_ready=True,
                                    fps=20, inference_ms=10, filter_type='fixture', map_binding=binding)
    assert payload['vision']['map_binding'] == binding
    assert payload['robots']['R1']['position_cm'] == {'x': 800, 'y': 600}


def test_recording_snapshot_keeps_old_matrix_after_next_calibration(tmp_path):
    config = effective_config()
    binding = map_binding(config, 'B')
    matrix = write_fixture(tmp_path, binding)
    config['paths']['calibration']['blue'] = str(matrix)
    snapshot = snapshot_map_assets(config, 'B', tmp_path / 'recording')
    archive = tmp_path / 'recording' / snapshot['directory']
    validate_map_calibration(archive / matrix.name, binding)
    assert profile_binding(load_profile(archive / 'profile.json'), 'B') == binding
    np.save(matrix, np.array([np.eye(3) * 3, np.eye(3)]))
    validate_map_calibration(archive / matrix.name, binding)
    assert file_sha256(matrix) != snapshot['calibration_sha256']


def test_calibration_launcher_preset_stays_live_and_saves_bound_pair(tmp_path):
    script = r'''
import os, sys
from pathlib import Path
import numpy as np
import calibration as c
from camera_calibration import validate_camera_calibration
from map_assets import default_preset_name

before = Path('config.yaml').read_bytes()
c.update_config_state = lambda side: None
c.config['calibration']['run_after_save'] = False
c.camera_mode = 'mindvision'
c.camera_image = np.zeros((1024,1280,3), dtype=np.uint8)
c.camera_intrinsics_error = None
c.active_camera_intrinsics_id = ''
app = c.QApplication([])
for side in ('R', 'B'):
    c.state = side
    os.environ[c.PRESET_ENVIRONMENT_VARIABLE] = default_preset_name(side)
    ui = c.MyUI()
    app.processEvents()
    assert ui.capturing, 'Launcher default must not freeze the initial camera frame'
    assert ui.map_binding['side'] == side
    assert ui.map_counts == [6,6] and ui.image_counts == [0,0]
    ui.save_path = str(Path(sys.argv[1]) / (side + '.npy'))
    ui.image_points = [[(u*.5,v*.25) for u,v in layer] for layer in ui.map_points]
    ui.image_counts = [6,6]
    for repeat in range(2):
        ui.button4_clicked()
        validate_camera_calibration(ui.save_path, 'mindvision', (1024,1280,3), map_binding=ui.map_binding)
    ui.close()
assert Path('config.yaml').read_bytes() == before
'''
    result = subprocess.run([sys.executable, '-c', script, str(tmp_path)], cwd=ROOT / 'vision',
                            env={**os.environ, 'QT_QPA_PLATFORM': 'offscreen'},
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
