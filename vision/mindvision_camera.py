"""迈德威视 mvsdk 图像采集封装。"""
import importlib

import cv2
import numpy as np


class MindVisionCameraError(RuntimeError):
    """迈德威视相机初始化或取流失败。"""


class MindVisionSDKUnavailable(MindVisionCameraError):
    """当前 Python 环境无法导入厂商 SDK。"""


class MindVisionNoCamera(MindVisionCameraError):
    """SDK 已加载，但没有发现相机。"""


class MindVisionCamera:
    """将 mvsdk 相机帧转换为 OpenCV 使用的 BGR ndarray。"""

    def __init__(self, exposure_time_us=11000, device_serial=None, require_single=False):
        try:
            self.sdk = importlib.import_module("mvsdk")
        except (ImportError, OSError) as error:
            raise MindVisionSDKUnavailable(
                "无法加载迈德威视 mvsdk。请安装 MindVision Linux SDK 和 Python 开发包，"
                "并确保 mvsdk.py 与 SDK 共享库对比赛启动器使用的 Python 可见。"
            ) from error

        required_api = (
            "CameraEnumerateDevice", "CameraInit", "CameraGetCapability",
            "CameraSetIspOutFormat", "CameraSetTriggerMode", "CameraSetAeState",
            "CameraSetExposureTime", "CameraAlignMalloc", "CameraPlay",
            "CameraGetImageBuffer", "CameraImageProcess", "CameraReleaseImageBuffer",
            "CameraUnInit", "CameraAlignFree", "c_ubyte",
        )
        missing_api = [name for name in required_api if not hasattr(self.sdk, name)]
        if missing_api:
            raise MindVisionSDKUnavailable(
                "当前 mvsdk Python 包不包含所需接口：" + ", ".join(missing_api)
            )

        self.handle = None
        self.frame_buffer = None
        self.is_mono = False
        self.device_serial = device_serial
        self.require_single = require_single
        self.device_info = {}
        self.exposure_time_us = int(round(float(exposure_time_us)))
        if not 4 <= self.exposure_time_us <= 32400:
            raise ValueError("MV-SUA133GC 曝光时间应在 4～32400 μs 范围内")

        try:
            self._open()
        except Exception:
            self.close()
            raise

    @staticmethod
    def _check_status(status, operation):
        status_value = getattr(status, "value", status)
        if isinstance(status_value, (int, np.integer)) and status_value != 0:
            raise MindVisionCameraError(f"{operation}失败，SDK 状态码={status_value}")

    def _open(self):
        devices = self.sdk.CameraEnumerateDevice()
        if not devices:
            raise MindVisionNoCamera("未发现迈德威视相机，等待设备连接")

        if self.device_serial:
            devices = [d for d in devices if d.GetSn() == self.device_serial]
            if len(devices) != 1:
                raise MindVisionNoCamera(f"未唯一匹配相机序列号：{self.device_serial}")
        elif self.require_single and len(devices) != 1:
            raise MindVisionCameraError("检测到多台相机，请在内参工具 YAML 中指定 camera.serial")

        device = devices[0]
        get_name = getattr(device, "GetFriendlyName", None)
        get_serial = getattr(device, "GetSn", None)
        name = get_name() if callable(get_name) else "MindVision"
        serial = get_serial() if callable(get_serial) else "未知"
        self.device_info = {"friendly_name": name, "serial": serial}
        for field, method in (("product_name", "GetProductName"), ("driver_version", "GetDriverVersion"),
                              ("sensor_type", "GetSensorType"), ("port_type", "GetPortType")):
            getter = getattr(device, method, None)
            self.device_info[field] = getter() if callable(getter) else None
        self.handle = self.sdk.CameraInit(device, -1, -1)

        capability = self.sdk.CameraGetCapability(self.handle)
        self.is_mono = bool(capability.sIspCapacity.bMonoSensor)
        output_format_name = "CAMERA_MEDIA_TYPE_MONO8" if self.is_mono else "CAMERA_MEDIA_TYPE_BGR8"
        output_format = getattr(self.sdk, output_format_name, None)
        if output_format is None:
            raise MindVisionCameraError(f"当前 mvsdk 缺少图像格式常量 {output_format_name}")

        self._check_status(
            self.sdk.CameraSetIspOutFormat(self.handle, output_format),
            "设置相机图像格式",
        )
        self._check_status(self.sdk.CameraSetTriggerMode(self.handle, 0), "设置连续采集")
        self._check_status(self.sdk.CameraSetAeState(self.handle, 0), "关闭自动曝光")
        self._check_status(
            self.sdk.CameraSetExposureTime(self.handle, self.exposure_time_us),
            "设置曝光时间",
        )

        width = int(capability.sResolutionRange.iWidthMax)
        height = int(capability.sResolutionRange.iHeightMax)
        bytes_per_pixel = 1 if self.is_mono else 3
        buffer_size = width * height * bytes_per_pixel
        if buffer_size <= 0:
            raise MindVisionCameraError("相机 SDK 返回了无效的最大图像尺寸")
        self.frame_buffer = self.sdk.CameraAlignMalloc(buffer_size, 16)
        if not self.frame_buffer:
            raise MindVisionCameraError(f"无法分配相机图像缓冲区（{buffer_size} 字节）")

        self._check_status(self.sdk.CameraPlay(self.handle), "启动相机采集")
        print(f"迈德威视相机已连接：{name}，序列号={serial}，曝光={self.exposure_time_us} μs")

    def capture_metadata(self):
        """Read the current SDK profile without changing ROI, zoom or binning."""
        if self.handle is None:
            raise MindVisionCameraError("相机尚未初始化")
        metadata = {"camera_mode": "mindvision", "device": dict(self.device_info),
                    "output_format": "BGR8", "mono_sensor": self.is_mono}
        for field, method, args in (
            ("exposure_us", "CameraGetExposureTime", (self.handle,)),
            ("analog_gain", "CameraGetAnalogGain", (self.handle,)),
            ("sdk_version", "CameraSdkGetVersionString", ()),
        ):
            getter = getattr(self.sdk, method, None)
            metadata[field] = getter(*args) if callable(getter) else None
        getter = getattr(self.sdk, "CameraGetImageResolution", None)
        resolution = getter(self.handle) if callable(getter) else None
        metadata["resolution"] = ({name: int(getattr(resolution, name))
                                    for name, _ in resolution._fields_ if name != "acDescription"}
                                   if resolution is not None else None)
        return metadata

    def get_frame(self, timeout_ms=1000):
        if self.handle is None or self.frame_buffer is None:
            raise MindVisionCameraError("相机尚未初始化")

        raw_data, frame_head = self.sdk.CameraGetImageBuffer(self.handle, int(timeout_ms))
        try:
            self._check_status(
                self.sdk.CameraImageProcess(self.handle, raw_data, self.frame_buffer, frame_head),
                "转换相机图像",
            )
            width = int(frame_head.iWidth)
            height = int(frame_head.iHeight)
            byte_count = int(frame_head.uBytes)
            if width <= 0 or height <= 0 or byte_count <= 0:
                raise MindVisionCameraError("相机 SDK 返回了无效的图像帧信息")

            channels = 1 if self.is_mono else 3
            required_bytes = width * height * channels
            if byte_count < required_bytes:
                raise MindVisionCameraError(
                    f"相机图像数据长度不足：收到 {byte_count}，至少需要 {required_bytes} 字节"
                )

            buffer_address = getattr(self.frame_buffer, "value", self.frame_buffer)
            raw_view = (self.sdk.c_ubyte * byte_count).from_address(int(buffer_address))
            image = np.frombuffer(raw_view, dtype=np.uint8, count=required_bytes)
            if self.is_mono:
                image = image.reshape(height, width)
                return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            return image.reshape(height, width, 3).copy()
        finally:
            self.sdk.CameraReleaseImageBuffer(self.handle, raw_data)

    def close(self):
        if self.handle is not None:
            try:
                self.sdk.CameraUnInit(self.handle)
            except Exception:
                pass
            self.handle = None
        if self.frame_buffer is not None:
            try:
                self.sdk.CameraAlignFree(self.frame_buffer)
            except Exception:
                pass
            self.frame_buffer = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
