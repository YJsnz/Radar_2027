"""受身份与成像模式约束的相机内参加载和统一去畸变。"""
import hashlib
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np
import yaml


DEFAULT_CONFIG = Path(__file__).with_name("intrinsic_runtime.yaml")


class CameraIntrinsicsError(ValueError):
    """内参配置、标定资产或当前相机档案不匹配。"""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise CameraIntrinsicsError(f"YAML 字段重复：{key}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


def _canonical(value):
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _matrix(block, name, expected_shape):
    if not isinstance(block, dict):
        raise CameraIntrinsicsError(f"内参文件缺少 {name} 矩阵")
    rows, columns = expected_shape
    if block.get("rows") != rows or block.get("cols") != columns:
        raise CameraIntrinsicsError(f"{name} 矩阵尺寸应为 {rows}×{columns}")
    try:
        result = np.asarray(block.get("data"), dtype=np.float64).reshape(expected_shape)
    except (TypeError, ValueError) as error:
        raise CameraIntrinsicsError(f"{name} 矩阵数据格式无效") from error
    if not np.isfinite(result).all():
        raise CameraIntrinsicsError(f"{name} 矩阵包含非有限数值")
    return result


class BoundCameraIntrinsics:
    """已经与单台相机、当前 ROI 和帧尺寸绑定的 remap。"""

    def __init__(self, image_size, camera_matrix, distortion, profile):
        self.image_size = tuple(image_size)
        self.camera_matrix = camera_matrix
        self.distortion = distortion
        self.profile = profile
        self.intrinsics_id = hashlib.sha256(_canonical(profile).encode("utf-8")).hexdigest()
        self.map_x, self.map_y = cv2.initUndistortRectifyMap(
            camera_matrix,
            distortion,
            None,
            camera_matrix,
            self.image_size,
            cv2.CV_16SC2,
        )

    def apply(self, frame):
        if frame is None or frame.ndim < 2 or frame.shape[1::-1] != self.image_size:
            raise CameraIntrinsicsError("当前帧尺寸改变，拒绝继续使用已绑定的相机内参")
        return cv2.remap(
            frame,
            self.map_x,
            self.map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )


class RuntimeCameraIntrinsics:
    """读取已审阅的内参结果，并在相机打开时核验设备档案。"""

    def __init__(self, config_path, values, report, camera_info, camera_matrix, distortion):
        self.config_path = config_path
        self.values = values
        self.report = report
        self.camera_info = camera_info
        self.camera_matrix = camera_matrix
        self.distortion = distortion
        self.camera = values["camera"]
        self.serial = self.camera["serial"]

    @classmethod
    def load(cls, config_path=None):
        configured_path = config_path or os.environ.get("SHARK_CAMERA_INTRINSICS_CONFIG")
        path = Path(configured_path).expanduser() if configured_path else DEFAULT_CONFIG
        path = path.resolve()
        try:
            values = yaml.load(path.read_text(encoding="utf-8"), Loader=_UniqueLoader)
        except (OSError, yaml.YAMLError) as error:
            raise CameraIntrinsicsError(f"无法读取相机内参运行配置 {path}: {error}") from error

        expected = {"schema", "enabled", "physical_validation_confirmed", "max_rms_px", "solution", "camera"}
        if not isinstance(values, dict) or set(values) != expected:
            raise CameraIntrinsicsError(f"内参运行 YAML 顶层字段应为：{sorted(expected)}")
        if values["schema"] != "shark.radar.intrinsic_runtime.v1":
            raise CameraIntrinsicsError("不支持的相机内参运行配置 schema")
        if not isinstance(values["enabled"], bool) or not isinstance(values["physical_validation_confirmed"], bool):
            raise CameraIntrinsicsError("enabled 与 physical_validation_confirmed 必须填写 true/false")
        if isinstance(values["max_rms_px"], bool) or not isinstance(values["max_rms_px"], (int, float)):
            raise CameraIntrinsicsError("max_rms_px 必须是数值")
        if not math.isfinite(values["max_rms_px"]) or not 0.01 <= values["max_rms_px"] <= 10:
            raise CameraIntrinsicsError("max_rms_px 应在 0.01～10 像素范围内")
        if not isinstance(values["solution"], dict) or set(values["solution"]) != {"directory"}:
            raise CameraIntrinsicsError("solution 必须只包含 directory 字段")
        camera_fields = {"model", "serial", "lens", "focus"}
        if not isinstance(values["camera"], dict) or set(values["camera"]) != camera_fields:
            raise CameraIntrinsicsError(f"camera 字段应为：{sorted(camera_fields)}")
        if any(not isinstance(value, str) for value in values["camera"].values()):
            raise CameraIntrinsicsError("camera 下的 model/serial/lens/focus 均须为字符串")

        if not values["enabled"]:
            return None
        if not values["physical_validation_confirmed"]:
            raise CameraIntrinsicsError(
                "内参尚未通过实体棋盘和独立几何核验；完成核验后才可在 YAML 中确认 physical_validation_confirmed"
            )
        if any(not values["camera"][key].strip() for key in camera_fields):
            raise CameraIntrinsicsError("启用内参前必须填写并核对相机型号、序列号、镜头和对焦位置")
        solution_text = str(values["solution"]["directory"] or "").strip()
        if not solution_text:
            raise CameraIntrinsicsError("启用内参时必须填写 solution.directory")
        solution = Path(solution_text).expanduser()
        if not solution.is_absolute():
            solution = (path.parent / solution).resolve()
        report_path = solution / "report.json"
        camera_info_path = solution / "camera_info.yaml"
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
            camera_info = yaml.load(camera_info_path.read_text(encoding="utf-8"), Loader=_UniqueLoader)
        except (OSError, json.JSONDecodeError, yaml.YAMLError) as error:
            raise CameraIntrinsicsError(f"无法读取内参结果目录 {solution}: {error}") from error

        if not isinstance(report, dict) or report.get("schema") != "shark.radar.camera_intrinsics.v1":
            raise CameraIntrinsicsError("report.json 不是本项目内参工具导出的格式")
        if not isinstance(camera_info, dict):
            raise CameraIntrinsicsError("camera_info.yaml 顶层必须是映射对象")
        if _canonical(camera_info) != _canonical(report.get("camera_info")):
            raise CameraIntrinsicsError("camera_info.yaml 与 report.json 内嵌结果不一致")
        metadata = report.get("metadata")
        if not isinstance(metadata, dict) or metadata.get("camera_mode") != "mindvision":
            raise CameraIntrinsicsError("只接受通过 MindVision SDK 实机采集的内参结果，不接受普通导入图片")
        device = metadata.get("device")
        if not isinstance(device, dict):
            raise CameraIntrinsicsError("内参报告缺少 SDK 相机身份信息")
        expected_camera = values["camera"]
        profile_pairs = (
            (metadata.get("expected_model"), expected_camera["model"], "YAML 目标型号"),
            (device.get("product_name"), expected_camera["model"], "SDK 产品型号"),
            (device.get("serial"), expected_camera["serial"], "相机序列号"),
            (metadata.get("lens"), expected_camera["lens"], "镜头记录"),
            (metadata.get("focus"), expected_camera["focus"], "对焦记录"),
        )
        for actual, expected_value, label in profile_pairs:
            if not actual or str(actual).strip() != expected_value.strip():
                raise CameraIntrinsicsError(f"运行 YAML 的{label}与内参报告不一致或为空")
        resolution = metadata.get("resolution")
        image_size = metadata.get("image_size")
        if not isinstance(resolution, dict) or not resolution:
            raise CameraIntrinsicsError("内参报告缺少完整 SDK ROI/缩放模式，不能安全复用")
        if (not isinstance(image_size, list) or len(image_size) != 2
                or any(isinstance(value, bool) or not isinstance(value, int) or value <= 0 for value in image_size)):
            raise CameraIntrinsicsError("内参报告图像尺寸无效")
        if (camera_info.get("image_width"), camera_info.get("image_height")) != tuple(image_size):
            raise CameraIntrinsicsError("camera_info.yaml 与内参报告中的图像尺寸不一致")
        if camera_info.get("camera_name") != f"mindvision_{expected_camera['serial']}":
            raise CameraIntrinsicsError("camera_info.yaml 相机序列号与运行 YAML 不一致")
        if camera_info.get("distortion_model") != "plumb_bob":
            raise CameraIntrinsicsError("当前运行链路只支持 plumb_bob 模型")
        try:
            rms_px = float(report.get("rms_px"))
        except (TypeError, ValueError) as error:
            raise CameraIntrinsicsError("内参整体重投影 RMS 缺失或格式无效") from error
        if not math.isfinite(rms_px) or rms_px < 0 or rms_px > float(values["max_rms_px"]):
            raise CameraIntrinsicsError("内参整体重投影 RMS 缺失或超过运行 YAML 阈值")
        matrix = _matrix(camera_info.get("camera_matrix"), "camera_matrix", (3, 3))
        distortion_block = camera_info.get("distortion_coefficients")
        if not isinstance(distortion_block, dict) or distortion_block.get("rows") != 1 or distortion_block.get("cols") != 5:
            raise CameraIntrinsicsError("畸变系数必须是 plumb_bob 的五参数行向量")
        try:
            distortion = np.asarray(distortion_block.get("data"), dtype=np.float64).reshape(1, 5)
        except (TypeError, ValueError) as error:
            raise CameraIntrinsicsError("畸变系数格式无效") from error
        if not np.isfinite(distortion).all() or min(matrix[0, 0], matrix[1, 1]) <= 0:
            raise CameraIntrinsicsError("内参或畸变系数包含无效数值")

        return cls(path, values, report, camera_info, matrix, distortion)

    def bind(self, metadata, frame):
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise CameraIntrinsicsError("内参仅支持相机 SDK 输出的三通道 BGR 图像")
        width, height = frame.shape[1], frame.shape[0]
        expected_size = (int(self.camera_info["image_width"]), int(self.camera_info["image_height"]))
        if (width, height) != expected_size:
            raise CameraIntrinsicsError(
                f"实际帧尺寸 {width}×{height} 与内参尺寸 {expected_size[0]}×{expected_size[1]} 不符"
            )
        recorded = self.report["metadata"]
        device = metadata.get("device") if isinstance(metadata, dict) else None
        if metadata.get("camera_mode") != "mindvision" or metadata.get("output_format") != "BGR8":
            raise CameraIntrinsicsError("当前输入模式或像素格式与内参来源不匹配")
        if metadata.get("mono_sensor") is not False:
            raise CameraIntrinsicsError("相机传感器输出格式与内参来源不匹配")
        if not isinstance(device, dict):
            raise CameraIntrinsicsError("当前 SDK 未返回相机身份")
        if device.get("serial") != self.camera["serial"] or device.get("product_name") != self.camera["model"]:
            raise CameraIntrinsicsError("当前相机序列号或 SDK 产品型号与内参配置不匹配")
        if _canonical(metadata.get("resolution")) != _canonical(recorded.get("resolution")):
            raise CameraIntrinsicsError("当前相机 ROI/缩放/binning 配置与内参采集时不同")
        self.profile = {
            "schema": "shark.radar.camera_intrinsic_profile.v1",
            "camera_mode": "mindvision",
            "model": self.camera["model"],
            "serial": self.camera["serial"],
            "lens": self.camera["lens"],
            "focus": self.camera["focus"],
            "image_size": [width, height],
            "resolution": recorded["resolution"],
            "distortion_model": "plumb_bob",
            "camera_matrix": self.camera_matrix.tolist(),
            "distortion_coefficients": self.distortion.reshape(-1).tolist(),
        }
        return BoundCameraIntrinsics((width, height), self.camera_matrix, self.distortion, self.profile)


def load_runtime_intrinsics(config_path=None):
    """未启用时返回 None；启用时必须已有经确认的实机标定结果。"""
    return RuntimeCameraIntrinsics.load(config_path)
