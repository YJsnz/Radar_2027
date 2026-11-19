import json

import pytest
import yaml

from camera_intrinsics import CameraIntrinsicsError, RuntimeCameraIntrinsics


def write_runtime_files(tmp_path, *, allow_candidate=True, product_name="MV-SUA133GC"):
    solution = tmp_path / "solution"
    solution.mkdir()
    serial = "048170110020"
    camera_info = {
        "image_width": 1280,
        "image_height": 1024,
        "camera_name": f"mindvision_{serial}",
        "distortion_model": "plumb_bob",
        "camera_matrix": {
            "rows": 3,
            "cols": 3,
            "data": [1000, 0, 640, 0, 1000, 512, 0, 0, 1],
        },
        "distortion_coefficients": {
            "rows": 1,
            "cols": 5,
            "data": [0, 0, 0, 0, 0],
        },
    }
    report = {
        "schema": "shark.radar.camera_intrinsics.v1",
        "rms_px": 0.475,
        "physical_validation_required": True,
        "camera_info": camera_info,
        "metadata": {
            "camera_mode": "mindvision",
            "device": {"product_name": product_name, "serial": serial},
            "expected_model": "MV-SUA133GC-T",  # 历史目标型号与 SDK 产品名不同
            "lens": "",
            "focus": "",
            "resolution": {"iWidth": 1280, "iHeight": 1024},
            "image_size": [1280, 1024],
        },
    }
    (solution / "camera_info.yaml").write_text(yaml.safe_dump(camera_info), encoding="utf-8")
    (solution / "report.json").write_text(json.dumps(report), encoding="utf-8")
    config = {
        "schema": "shark.radar.intrinsic_runtime.v1",
        "enabled": True,
        "physical_validation_confirmed": False,
        "allow_unverified_candidate": allow_candidate,
        "max_rms_px": 0.5,
        "solution": {"directory": str(solution)},
        "camera": {
            "model": "MV-SUA133GC",
            "serial": serial,
            "lens": "",
            "focus": "",
        },
    }
    config_path = tmp_path / "runtime.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    return config_path


def test_explicit_candidate_mode_loads_sdk_product_name_without_marketing_suffix(tmp_path):
    config_path = write_runtime_files(tmp_path)

    runtime = RuntimeCameraIntrinsics.load(config_path)

    assert runtime.camera["model"] == "MV-SUA133GC"
    assert runtime.report["metadata"]["expected_model"] == "MV-SUA133GC-T"


def test_unverified_solution_is_rejected_without_explicit_candidate_mode(tmp_path):
    config_path = write_runtime_files(tmp_path, allow_candidate=False)

    with pytest.raises(CameraIntrinsicsError, match="尚未通过实体棋盘"):
        RuntimeCameraIntrinsics.load(config_path)


def test_candidate_mode_still_rejects_wrong_sdk_product_name(tmp_path):
    config_path = write_runtime_files(tmp_path, product_name="MV-SUA133GC-T")

    with pytest.raises(CameraIntrinsicsError, match="SDK 产品型号"):
        RuntimeCameraIntrinsics.load(config_path)
