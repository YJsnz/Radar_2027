"""标定矩阵来源和相机图像尺寸校验。"""
from pathlib import Path
from zipfile import BadZipFile

import numpy as np


def calibration_points_path(matrix_path):
    matrix_path = Path(matrix_path)
    return matrix_path.with_name(f"{matrix_path.stem}_points.npz")


def validate_camera_calibration(matrix_path, camera_mode, frame_shape, intrinsics_id=None):
    """拒绝在相机模式或分辨率变化后静默沿用旧图像标定。"""
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
    if intrinsics_id and saved_intrinsics_id != intrinsics_id:
        raise ValueError(
            "地图外参标定使用的相机内参与当前不一致："
            f"标定={saved_intrinsics_id or '未记录'}，当前={intrinsics_id}。"
            "启用、替换或关闭去畸变后都必须重新标定地图外参。"
        )
