"""标定矩阵来源和相机图像尺寸校验。"""
from pathlib import Path
from zipfile import BadZipFile
import hashlib
import json

import numpy as np


def calibration_points_path(matrix_path):
    matrix_path = Path(matrix_path)
    return matrix_path.with_name(f"{matrix_path.stem}_points.npz")


def validate_map_calibration(matrix_path, map_binding):
    """在打开设备前拒绝缺失、跨地图／阵营或矩阵与档案不配套的结果。"""
    if map_binding is None:
        return
    matrix_path = Path(matrix_path)
    points_path = calibration_points_path(matrix_path)
    if not matrix_path.is_file() or not points_path.is_file():
        raise ValueError(f'尚未完成 2027 {map_binding["side"]} 方实机标定，请先采集相机对应点：{matrix_path}')
    try:
        with np.load(points_path, allow_pickle=False) as points:
            binding = json.loads(str(np.asarray(points['map_binding']).item()))
            if binding != map_binding:
                raise ValueError('地图／掩码／点位版本或阵营与矩阵档案不匹配，必须重新标定')
            expected_hash = str(np.asarray(points['matrix_sha256']).item())
            if hashlib.sha256(matrix_path.read_bytes()).hexdigest() != expected_hash:
                raise ValueError('矩阵文件与标定点档案校验值不同')
            matrices = np.load(matrix_path, allow_pickle=False)
            if (matrices.shape != (2, 3, 3) or not np.isfinite(matrices).all()
                    or any(np.linalg.matrix_rank(matrix) != 3 for matrix in matrices)
                    or not np.array_equal(matrices, points['matrices'])):
                raise ValueError('两层矩阵无效或与标定点档案不一致')
    except (OSError, BadZipFile, KeyError, TypeError, ValueError) as error:
        raise ValueError(f'地图标定档案校验失败：{error}') from error


def validate_camera_calibration(matrix_path, camera_mode, frame_shape, intrinsics_id=None, map_binding=None):
    """拒绝在相机模式或分辨率变化后静默沿用旧图像标定。"""
    validate_map_calibration(matrix_path, map_binding)
    points_path = calibration_points_path(matrix_path)
    if not points_path.is_file():
        raise ValueError(f"缺少带相机来源信息的标定点文件：{points_path}")

    required_fields = {"camera_mode", "image_width", "image_height"}
    if intrinsics_id:
        required_fields.add("intrinsics_id")
    try:
        with np.load(points_path, allow_pickle=False) as points:
            missing = required_fields.difference(points.files)
            if missing:
                raise ValueError(f"标定点文件缺少字段：{', '.join(sorted(missing))}")
            saved_mode = str(np.asarray(points["camera_mode"]).item())
            saved_width = int(np.asarray(points["image_width"]).item())
            saved_height = int(np.asarray(points["image_height"]).item())
            saved_intrinsics_id = (
                str(np.asarray(points["intrinsics_id"]).item())
                if "intrinsics_id" in points.files
                else ""
            )
    except (OSError, BadZipFile, KeyError, TypeError, ValueError) as error:
        raise ValueError(f"无法读取标定点来源信息：{points_path}: {error}") from error

    frame_height, frame_width = frame_shape[:2]
    if saved_mode != camera_mode or (saved_width, saved_height) != (frame_width, frame_height):
        raise ValueError(
            "标定与当前相机不匹配："
            f"标定={saved_mode} {saved_width}×{saved_height}，"
            f"当前={camera_mode} {frame_width}×{frame_height}"
        )
    if saved_intrinsics_id != (intrinsics_id or ''):
        raise ValueError(
            "地图外参标定使用的相机内参与当前不一致："
            f"标定={saved_intrinsics_id or '未记录'}，当前={intrinsics_id}。"
            "启用、替换或关闭去畸变后都必须重新标定地图外参。"
        )
