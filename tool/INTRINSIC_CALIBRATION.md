# MV-SUA133GC-T 相机内参标定

## 1. 参考与范围

参考本机 `/home/geqingyu/Code/ysu-rm-vision-v3-dev` 的 `scripts/run_calibration.sh`、`src/rm_bringup/launch/camera_calibration.launch.py` 和 README 的标定流程：读取配置、棋盘格采样、X/Y/Size/Skew 提示、计算、审阅误差及导出 CameraInfo。本工具直接复用当前项目的 `vision/mindvision_camera.py` 与 MindVision SDK，不依赖参考项目的海康驱动或 ROS 标定节点。

参考项目的 1440×1080、棋盘行列数、方格边长和内参矩阵都不移植为本相机默认事实；其 README 的 0.03 m 示例与参数文件的 0.02 m 也不是本队标定板测量值。这里必须在 YAML 填写自己的实物参数。

解算依据 [OpenCV 相机标定说明](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)：在原始像素尺寸检测完整棋盘格，以多个姿态求解普通针孔模型的 K 和五项畸变。输出字段遵循 [ROS CameraInfo 的 K/D/R/P 约定](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg)。本工具支持 `plumb_bob`，不将普通针孔系数当作鱼眼模型使用。

## 2. 全部参数在 YAML 填写

默认文件：`tool/intrinsic_calibration.yaml`。不在 Qt 表单内维护另一套标定参数；修改后先停止采集，点击“重新加载 YAML / 新建数据集”。旧数据和结果保留，每个新数据集使用唯一目录。

首先填写：

```yaml
board:
  columns: null         # 改为实际内角点列数，整数
  rows: null            # 改为实际内角点行数，整数
  square_size_mm: null  # 改为实测方格边长，毫米
```

内角点数等于方格数减一。例如实物 12 列×9 行方格对应 11×8 内角点；这只是解释，工具不假定你使用该板。棋盘参数未填完整时允许打开相机看画面，但禁止采样与计算，不生成虚构内参。

| YAML 段 | 内容与约定 |
|---|---|
| camera | 目标型号、序列号、手动曝光 μs、取帧超时、处理/预览频率、可选预期宽高、镜头与对焦记录。多台设备必须填写 serial；未填写宽高就用首帧真实尺寸。 |
| board | 内角点列/行、实测方格边长 mm。不要填外部方格数，也不要把米直接填入毫米字段。 |
| capture | 最少/最多有效样本、清晰度、棋盘面积占比、重复姿态距离、自动采样间隔、最大帧年龄。工程初值可按现场条件修改。 |
| coverage | X/Y 中心位置跨度、棋盘大小跨度、透视变化跨度。仅启发式采样提示，不是标定精度保证。 |
| calibration | 当前为 plumb_bob；max_rms_px 为重投影拟合误差审阅阈值，不是官方或实物精度验收线。 |
| output | 输出根目录；相对路径按 YAML 所在目录解析，支持绝对路径和 `~`。默认在 tool/data/intrinsics。 |

采样和求解参数跟随数据集保存配置原文及 SHA256；文件存在重复字段、未知字段、非法类型或数值范围时明确报错。板面三个参数留空是有意的待填写状态。

## 3. 启动与采样

1. 固定要用于比赛的镜头、焦距和对焦位置，准备平整的棋盘格并实测单格边长。在 YAML 记录 lens/focus；换镜头、变焦、重新对焦后要重新评估并标定。
2. 停止正在占用这台相机的比赛主程序或外参标定取流。工具不会杀死其他进程、抢占相机或修改主程序配置。
3. 在项目根目录运行：

   ```bash
   ./tool/start_intrinsic_calibration.sh
   # 使用另一份配置：
   ./tool/start_intrinsic_calibration.sh --config /absolute/path/intrinsics.yaml
   ```

   启动脚本读取已有的 `~/.config/shark-radar/mindvision.env` 或 MINDVISION_ENV_FILE，发现 SDK Python 包和共享库，优先使用 vision/.venv。依赖项目已有的 PyQt5、OpenCV、NumPy、PyYAML 和 MindVision SDK；不需要 ROS 或 CUDA。Qt 和 OpenCV 按项目现有方式设置插件路径，图像展示不调用 cv2.imshow。

4. 点击“打开相机”，核对 SDK 实际产品名、序列号和帧尺寸。目标型号只是用户配置；若与 SDK 产品名不同，报告会要求核对实物/厂商命名，不自行认定别名相同。相机 ROI、binning、skip、软硬件缩放参数均只读，曝光沿用项目现有手动控制，不移植海康 dB 增益。
5. 在中心、四周和边缘，以不同距离、倾斜方向展示完整棋盘格。点击“采集当前棋盘格”，或勾选自动采样；过小、模糊、过期或近似重复的画面不收录。清晰度为棋盘区域的全分辨率 Laplacian 方差，重复性比较归一化角点位置，均是工程筛选规则。
6. 查看四项覆盖提示和采样数。默认至少 15 张，但数量与进度满格不等于准确。每张检测都使用实际原始分辨率，界面缩放只影响显示。

设备 SDK 的 serial、产品名、传感器/接口/驱动信息、SDK 版本、实际曝光和增益、ROI/缩放描述、图像宽高随采样保存。同一数据集中身份、ROI/缩放模式或帧尺寸变化会拒绝新样本。时间戳标注为主机取帧后的接收时间，不冒充传感器曝光时间。

## 4. 求解、审阅、保存

- 点击“计算内参”，后台调用 calibrateCameraExtended，求出 K、D、整体 RMS、逐张 RMS 和内参标准差。样本列表显示逐张误差；可以选择移除异常样本再计算，原始 PNG 不删除，不自动悄悄剔除高误差照片。
- K 的 fx/fy/cx/cy 单位为像素，D 顺序为 `[k1,k2,p1,p2,k3]`。RMS 为重投影拟合误差，不能证明真实场地精度；样本不足、多样性不足或标定板不平整仍可能产生错误参数。
- 勾选“去畸变预览”检查直线和边缘；采用原 K 作为输出矩阵，不裁剪或改变尺寸。新相机帧与当前解的身份/模式不一致时不套用旧解。
- 点击“保存内参 YAML 与误差报告”。每次导出创建新的 solution 目录，不覆盖旧解，不改写 vision/config.yaml，也不替换现有地图标定文件。

目录结构：

```text
tool/data/intrinsics/<UTC时间-唯一ID>/
  settings.yaml               # 采样使用的配置原文
  session.json                # 设备信息、角点、筛选指标、有效/移除样本及图像 SHA256
  images/view_0001.png        # SDK 输出的未去畸变全尺寸 BGR 图像，不是传感器 Bayer 原始流
  solution-<UTC时间-唯一ID>/
    camera_info.yaml          # 实际宽高、K、D、R=I、P=[K|0]
    report.json               # 逐张误差、标准差、配置摘要、采样覆盖与待核对项
    observations.npz          # 数值角点、K/D、各标定板姿态、图像尺寸
```

NPZ 中 rvecs/tvecs_mm 是每张棋盘相对相机的姿态，**不是雷达相机在场地中的外参**。图片及结果属于本机采集资产，默认输出目录已忽略，不会随代码提交。

## 5. 离线图片与主项目接入边界

停止相机后可导入同一设备、镜头、模式、尺寸的原始图片；每张重新检测和筛选，不与正在采集的 SDK 数据混用。导入源的相机身份/ROI 不能由普通图片证明，会明确标记 imported_images 并保留来源路径；报告不会把导入图片伪装成已核对的 MV-SUA133GC-T 实机数据。

当前项目已有 calibration.py 处理图像到地图的分层标定，主程序尚未消费本工具的 CameraInfo。后续接入去畸变时，**外参/地图标定和比赛推理必须使用同一图像坐标流程**；不能只在其中一边去畸变，或沿用未去畸变画面的旧地图矩阵。也不根据参考项目的分辨率比例缩放或拼凑内参。实体棋盘采集、误差/直线检查以及该消费链路仍需独立验收。

## 6. 本次开发记录

2026-11-10：完成独立 Qt 入口、YAML 参数、SDK 采样元数据、数据集保存、OpenCV 求解与结果导出实现；Python/启动脚本语法、启动帮助和缺参界面预览检查完成。未运行自动化测试，未采集实物棋盘格，未生成或宣称已经验证的本机真实 K/D；实体标定与主项目去畸变接入继续保持待办。
