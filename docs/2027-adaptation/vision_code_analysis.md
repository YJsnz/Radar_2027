# vision/ 子系统代码分析（2026 赛季基线 → 面向 2027 适配）

分析范围：仓库 `D:\Code\2027YSU-radar` 的 `vision/` 子模块（JNU-SHARK/shark-radar-vision，128 个文件）。只读分析，未运行 TensorRT / 相机 / ROS。行号均以当前工作区文件为准。

整体判断：vision 是"单目 + 双层仿射 + 目标级滤波"的雷达视觉端，代码质量较高（协议/配置/存储均有纯函数与测试分离），但**"2026 场地常量"渗透很深**：2800×1500、0cm/300cm 两层、"R1..R7 / B1..B7 且 5 号缺席"、盲区点、0x0305/0x0121 报文布局、裁判机器人 ID 9/109 等，全部是 2027 适配的高风险点。

---

## 1 启动与运行链路

### 1.1 三层启动

1. **仓库根 `start_match.sh`（一键双链路）**：定义 `RADIO_START=$ROOT_DIR/radio/apps/match_rx/start.sh`、`VISION_START=$ROOT_DIR/vision/scripts/start_launcher.sh`（均可用 `MATCH_RADIO_START`/`MATCH_VISION_START` 覆盖）；生成并导出统一的 `SHARK_MATCH_RUN_ID`（默认 `match_YYYYmmdd_HHMMSS_$$`，正则 `^[A-Za-z0-9][A-Za-z0-9_.-]*$`、≤88 字符，不合法直接 exit 2）；用 `setsid` 分别起两侧进程组，**单侧退出只重启故障侧**（`MATCH_RESTART_DELAY_SEC` 默认 2s），Ctrl+C 时按进程组 TERM→10s→KILL 收尾。
2. **`vision/scripts/start_launcher.sh`**：`set -eu`；先从 `~/.config/shark-radar/mindvision.env` 读取可选的 `MINDVISION_SDK_ROOT`（使桌面自启动也能找到 SDK），为该目录中的 `mvsdk.py` 与 MindVision `.so` 配置 Python/动态库路径；再临时加载 `/opt/ros/humble/setup.sh`（radio_ros 模式必需）并探测海康 MVS 运行库；Python 解释器优先 `$PROJECT_ROOT/.venv/bin/python`，否则 `python3`；`cd "$PROJECT_ROOT"` 后 `exec python -u launcher.py "$@"`。**注意 cwd 必须是 vision/**，因为程序按相对路径读 `config.yaml`。
3. **`vision/launcher.py`（Qt5 启动台）**：`MatchLauncher(QMainWindow)` 分比赛设置、裁判系统与设备、标定与测试、日志四块 UI；相机模式现为 mindvision/hik/video/test，MindVision 使用可配置曝光（4～32400 μs），海康序列号和 dB 增益控件仅适用于 Hik。启动台保存配置后拉起 `main.py` 或 `calibration.py`，二者继承启动器加载的 SDK 环境。

### 1.2 配置回写与子进程

`_save_configuration`（`vision/launcher.py:785`）把 20 个键交给 `config_editor.update_config_values`（`vision/config_editor.py:62`，按行改写、保留注释与缩进、yaml 复核、`os.replace` 原子替换）。写入键即"现场可改项"：`global.state / camera_mode / multi_car_recognition / use_serial / save_img / test.video_path`、`algorithm.mode`、`referee.transport`、`recording.selected_root`、`blind_zone.enabled`、`double_vulnerability.enabled(=false 恒定) / trigger_mode`、`calibration.points_per_layer / run_after_save`、`serial.port`、`filter.type`、`camera_params.{exposure_time,gain,device_serial}`。

`_start_process`（`vision/launcher.py:846`）：先 `_process_is_running()` 互斥（`:847`，防止同时跑标定和比赛），**强制先保存配置**（`:850`），`clear_runtime_status()`，然后 `QProcess.start(sys.executable, ["-u", script_name])`（`:870`），working dir = vision/；标定时额外注入 `SHARK_CALIBRATION_PRESET`（`calibration_presets.py:11`）。子进程输出合并进日志面板（`_read_process_output` `:880`），退出/启动失败走 `_process_finished`（`:887`）/`_process_error`（`:899`，`FailedToStart` 时清状态）。

### 1.3 主程序与操作流程

`main.py` 启动即：读 config → 校验 `algorithm.mode`/`filter.type`/阈值（`main.py:123-136`、`:193-225`）→ 按 `referee.transport` 建 `RefereeTransport`（`:95`）→ 若 legacy_serial 才开串口（`:140-155`）→ 加载标定数组与地图/掩码（`:511-532`）→ 加载两个引擎（`:1827-1859`）→ 起相机线程并等待首帧（`:1939`）→ 起发送线程（`:1884-1895`）→ 初始化录像（`:1976-2052`）→ 主循环（`:2175`）。`ESC/q` 或 SIGINT/SIGTERM（`:85-91`）退出，收尾在 `:2640-2646`。

**两人操作**：一人负责无线电侧（`radio/apps/match_rx/start.sh` + Dashboard 8765），一人负责视觉台（启动台里选阵营/相机/通信/录像目录 → 保存 → 启动比赛；换场先"开始标定"）。**单人操作**：`./start_match.sh` 同时拉起两侧，视觉台里标定完成且 `calibration.run_after_save=true` 时，`calibration.py:908-911` 用 `os.execv(sys.executable, [sys.executable,'-u','main.py'])` 直接把标定进程替换成比赛进程，一跳进入比赛。

### 1.4 相机迁移记录（2026-11-07）

视觉默认输入已切换为迈德威视 MV-SUA133GC-T，通过独立的 `mindvision_camera.py` 调用厂商 `mvsdk` Python 包，将帧统一转换为 OpenCV BGR。标定文件的 `_points.npz` 增加 `camera_mode`、`image_width`、`image_height`；MindVision 比赛启动时强制比较来源和实拍尺寸，不匹配就要求重新标定。海康、USB 与测试输入保留。当前环境未安装 MindVision SDK，也未接入实机，因此还未完成设备取流、曝光效果、稳定帧率和新标定点实测；详见根目录 README 的相机迁移 TODO。

---

## 2 检测与识别

### 2.1 车辆检测（第一级）

`YOLOv5Detector`（`vision/detect_function.py:36`）封装 YOLOv5 + TensorRT（`DetectMultiBackend`）。`.engine` 权重强制 `device='0'`，并显式检查 CUDA：`select_device('cpu')` 会把 `CUDA_VISIBLE_DEVICES` 设为 -1 导致 TensorRT 段错误，故 `.engine` + 无 GPU 直接抛 `RuntimeError('TensorRT 模型 (*.engine) 需要 CUDA...')`；`fp16/half` 恒关闭；`predict` 走 letterbox→CHW→RGB→/255→前向→`non_max_suppression(conf,iou,classes,agnostic,max_det)`→`scale_boxes`，返回 `[(class_name,[x,y,w,h],conf)]`（左上角+宽高）；`predict_batch` 仅后端支持动态 batch 时可用，否则抛 `当前装甲模型后端不支持动态 batch`。

主程序实例化（`main.py:1827-1832`）：`car.engine`，`img_size=(640,640)`，`conf_thres=vehicle_low_confidence=0.10`（低阈值留给第二遍关联），`iou_thres=0.5`，`max_det=14`（=7v7），`data='yaml/car.yaml'`。`vision/yaml/car.yaml` 定义 **5 类** `['car','armor','ignore','watcher','base']`，但代码只使用 `car` 类（`main.py:2230-2237`、`tracking/pipeline.py:165-168`），其余 4 类是历史遗留/未用。检测后按 `multi_car_recognition` 决定是否只保留置信度最高的一辆（`main.py:2238-2240`）。

### 2.2 装甲板识别（第二级，两条实现）

**legacy（比赛默认）**：对每辆车的 crop 加 10px padding 后排进 `armor_canvas_size`（`main.py:2246`，config 现为 640×640；超过画布的 ROI 直接 `continue` 跳过），逐画布调用静态 `armor.engine`（`img_size=(1280,1280)`、`conf=armor_conf_threshold=0.4`、`iou=0.2`、`max_det=10`，`ui=True`；`main.py:1854-1859`），再按框中心归属回各 ROI（`main.py:2294-2321`），每辆车只留 1 个框：`armor_selection.mode='center'`（离 ROI 中心最近，平局比 conf）或 `'confidence'`；坐标还原 `x0=x-rx+left, y0=y-ry+top`（`:2324-2327`）。

**hkust_tracker**：只有当 `algorithm.mode=='hkust_tracker'` 且 `models/armor_batch.engine` 存在时才加载动态 batch 引擎（`main.py:1834-1853`，`img_size=(320,320)`、`max_det=8`，`supports_dynamic_batch` 为假则抛 `engine 没有动态 batch profile`）。ROI 由轨迹的 `measurement_box` 扩张 `margin_ratio=0.08` 后 letterbox 到 320×320（`tracking/roi_batch.py:make_letterboxed_roi`），动态后端一次 `predict_batch`；静态后端则 `pack_tiles` 拼 1280 画布（4×4=16 格，`canvas_size % tile_size` 必须为 0，否则 `ValueError`），全画布推理后按中心落格回查 `RoiTransform`。每车保留 `top_k=2` 个装甲板（`tracking/pipeline.py:218-222`）。

> 与 README 的偏差：README 把"两级 ROI 批处理 + armor_batch 动态 batch"描述为主链路，实际 **legacy 比赛路径完全不使用 `armor_batch.engine`**，动态 batch 只服务 hkust_tracker。

类别来自 `vision/yaml/armor.yaml`：**12 类** `['B1'..'B5','B7','R1'..'R5','R7']`（缺 6、无 B6/R6，但含 5）。编号由类名后缀数字截取（`main.py:320` `re.search(r'(\d+)$')`）。

### 2.3 阵营（红/蓝）判定：颜色 + 几何双证据

核心 `analyze_armor_light_color`（`vision/vehicle_color.py:142`）：

- HSV 阈值：`S>=min_saturation(70)`、`V>=min_value(55)`；
- 红：`hue<=15 or hue>=165`；`yellow_as_red=true` 时把 `hue∈[yellow_hue_min(16), yellow_hue_max(40)]` 也算红（2026 赛季黄灯=红方）；蓝：`hue∈[90,140]`；
- **只取左右两侧**（`side_strip_ratio`，配置 0.4）：中间数字区被裁掉，避免数字颜色干扰（`:23-28`）；
- 灯条几何：`_find_light_bar_pair`（`vehicle_color.py:7`）对每侧做 3×1 垂直闭运算→外轮廓→`minAreaRect`，要求长边 ≥ `bar_min_height_ratio(0.12)×高度`、长宽比 ≥ `bar_min_aspect_ratio(1.3)`；配对要求高度相似度 ≥ `bar_min_height_similarity(0.4)`、y 偏移 ≤ `bar_max_y_offset_ratio(0.8)`、长轴夹角 ≤ `bar_max_angle_difference(35°)`（小装甲板豁免）；评分 `height_similarity*(1-y_offset)*(0.5+0.5*angle_score)`；
- 小目标兜底：装甲板高度 ≤ `compact_pair_max_armor_height(16)px` 时允许 `compact_pair`（置信度 ×`0.7`）；无配对且 `allow_single_light_bar` 时允许单灯条（×`0.55`）；
- 置信度：`purity*(0.5+0.5*support)*geometry_score`，`purity=主色像素/总像素`，`support=min(1, 主色像素/(3*min_pixels))`；`dominance_ratio=1.5` 决定主色必须压过对方 1.5 倍，否则返回空证据。

`VehicleColorMemory`（`vehicle_color.py:263`，比赛实例化于 `main.py:227-254`）是**按车辆框跟踪的颜色状态机**：

- 关联条件（`_find_track` `vehicle_color.py:326`）：同一编号、**必须上一帧可见**（`last_seen_frame` 差 1）、中心距 ≤120px、宽/高变化率 ≤0.5、IoU ≥0.3；过期轨迹不复用（注释明确"消失轨迹不得把颜色传给后来的车"）；
- 提交门槛：普通双灯 3 帧、切换 5 帧、单灯条 5 帧、compact_pair 5 帧、纯模型兜底 5 帧；
- 交叉验证 `require_model_agreement=true`：灯色与模型预测阵营冲突时灯证据作废；
- 灯不亮兜底：仅当**尚未提交**颜色、模型置信 ≥0.75 且（无灯 或 灯证据 ≤0.4）时允许用模型色初始化（`model_only`），且永不覆盖已提交灯色（`:426-440`）；
- 返回 `(label, used_hold, committed)`，`committed=false` 时主程序只画橙框"color pending"、**不产生坐标**（`main.py:2388-2400`）。

legacy 还有第二道门 `confirm_armor_detection`（`main.py:331`）：跨帧 track（中心距 ≤120、漏帧 ≤3）+ 连续 3 帧 + 编号白名单 `allowed_numbers=[1,2,3,4,7]`；随后按 `armor_duplicate.scope` 去重（`class`=同色同号，`number`=只按数字，`main.py:2426-2457`）。hkust 路径则用 `IdentityManager`（见 §4）。

---

## 3 标定与投影数学

### 3.1 标定流程（calibration.py，955 行）

- 固定 **2 层**：`height∈{0,1}` 对应 0cm 与 300cm（`button2_clicked` 切换 `self.height=(self.height+1)%2`，`calibration.py:666`）；每层 `points_per_layer`（config，默认 6）对点：左栏相机像素点、右栏地图点，成对点击（`left_top_clicked:612` / `right_top_clicked:633`，坐标按显示缩放比 `left_scale_x/right_scale_x` 还原），可拖动/撤回。
- 求解（`button4_clicked`，`calibration.py:859`）：
  - 检查两层点数齐；`points_per_layer==4` 用 `cv2.getPerspectiveTransform`（精确 4 点），否则 `cv2.findHomography(image_point, map_point, method=0)`（**method=0 是最小二乘，不是 RANSAC**）；返回 None 时提示"高度 i 多点拟合失败，请检查点位是否共线或顺序错误"；
  - 重投影误差 `projected = perspectiveTransform(image_point, M)`，`errors = ||projected-map_point||`，打印 `mean/max/all`（px）；
  - `update_config_state(self.state)`（`calibration.py:59`，保留注释原子改写 `global.state`）；
  - `np.save(save_path, self.T)` → `arrays_test_red.npy` / `arrays_test_blue.npy`，内容是 **`[M0, M300]` 两个 3×3**（顺序被 `main.py:523-524` 消费为 `M_ground`、`M_height_r`）；另存 `<name>_points.npz`（matrices/image_points/map_points）便于复现；
  - `calibration.run_after_save=true` 时 `os.execv` 直接进 `main.py`。
- 地图预制（`calibration_presets.py`，`calibration_presets.json`）：`{"version":2,"presets":[{name, side:"R"|"B", points_per_layer, map_points:[[layer0],[layer1]], updated_at}]}`，只存地图点；导入时点数不符直接拒绝、有现存地图点需确认、图像点仍需手标；启动台/标定窗口都可用 `SHARK_CALIBRATION_PRESET` 指定方案。当前库内已有 `red人`、`blue` 等 6 点方案。

### 3.2 双层仿射与坐标

`project_image_point_to_map(point_x, point_y)`（`main.py:411`，legacy 在 `:2478-2500` 有一份内联副本）：

1. 像素点先 clamp 到 `[0,img_x]×[0,img_y]`；
2. `cv2.perspectiveTransform` 用 `M_ground`（0cm 层）投影，结果 clamp 到 `[0,width]×[0,height]`（`width/height` 由掩码图尺寸决定，`main.py:527-529`）；
3. **用掩码图判层**：`mask_image[y_c, x_c]` 为纯黑(0,0,0) 视为地面/公路 → 采用 0cm 结果；否则视为中央高地 → 改用 `M_height_r` 再投一次。
   → 选层依据是"场地地图上的颜色分区"（`images-2026/map_mask26.jpg`），**不是几何高度**；2027 场地一旦改配色/改高地轮廓，必须重做 mask，否则落点直接偏一层。
4. 锚点（投影输入点）由 `projection.point_source` 决定：`car_bottom`（默认）= 车辆框 `(left+0.5w, top+0.92h)`；`armor_bottom` = 装甲板框 `(x0+0.5w, y0+1.5h)`（`main.py:2468-2476`）。

坐标有两套（`convert_projected_map_point`，`main.py:432`）：地图投影结果是"显示坐标"体系（2800×1500 画布），再转裁判坐标：

```
display = (2800 - y, x)        if side=='R'
display = (y, 1500 - x)        if side=='B'
referee = (display_x, 1500 - display_y)
```

即己方为红方时 `referee=(2800-y, 1500-x)`；蓝方镜像。收发两处都按同一公式（发送侧 `main.py:1172-1183`、`:2524`）。边界与异常：像素/地图二次 clamp；`(0,0)` 被保留为"无效"（telemetry 与 `send_map` 都用它表示缺测）；2800×1500 之外的坐标判无效（`vision_telemetry.py:19`、`tracking/types.py:59-63`）。

---

## 4 跟踪与滤波

### 4.1 两套并行实现

| 链路 | 位置 | 职责 |
| --- | --- | --- |
| legacy 滤波 | `main.py:677-893` | `MotionKalman2D`（每台一条 4 状态 CV）、`SlidingWindowFilter`、`HybridGatedKalmanFilter`、`create_filter_by_type` |
| hkust_tracker | `vision/tracking/`（5 模块） | 车级轨迹 + 全局身份分配 + 世界系卡尔曼 |

### 4.2 tracking/ 五模块

- `types.py`：`TrackState{tentative,confirmed,lost,removed}`、`TargetState{measured,occlusion_hold,blind_prediction,missing}`、`CarDetection`、`ArmorEvidence`、`TargetOutput`（`valid` = 有 position、非 missing、非 (0,0)、在 2800×1500 内）、`AlgorithmFrameResult`。
- `kalman.py`（165 行）：`LinearKalman`（P=10I、Joseph form 更新、`mahalanobis`）；`BoxKalman`：**8 状态 `[cx,cy,w,h,vx,vy,vw,vh]`**，`H` 取前 4，`R=diag(16,16,25,25)`，`dt` clamp `[1/120, 0.5]`，过程噪声由"加速度谱密度 `[4,4,2,2]`"按 `dt³/2、dt²` 建块，宽高下限 2px；`WorldKalman`：**4 状态 `[x,y,vx,vy]`（裁判坐标 cm）**，`R=36I`、`P=100I`、`q=40`，`update(point, ts, reset_distance=500)` 跳变超 5m 直接重置。
- `tracker.py`（371 行）：`VehicleTracker` 是 ByteTrack 式两遍关联：先用 `conf>=high_confidence(0.30)` 做匈牙利匹配（代价 `0.55*(1-IoU)+0.30*归一化中心距+0.15*尺寸变化`，门限 0.85），未匹配的 confirmed/lost 轨迹再用 `low_confidence(0.10)` 二遍回收（门限 0.70）；`confirm_hits=2` 转 confirmed，`remove_seconds=3.0` 后转 removed，tentative 一帧漏检即删；`measurement_box` 观测帧直接用检测框（避免卡尔曼显示滞后）。`IdentityManager`：每轨迹对 12 个类名维护衰减分数（`decay=0.85`，`add_evidence` 仅在 `color_committed` 时加分），用带"虚拟列"的匈牙利做全局唯一分配（代价 `-log(p)`，同标签 -0.8、同槽 -0.6、位置差 /500*0.5），得分概率需 ≥`min_confidence(0.60)` 且领先 `min_margin(0.15)`，连续 `confirm_hits(3)`/切换 `switch_hits(5)` 帧才确认；确认时若同一物理轨迹改判角色，会 `clear()` 旧槽避免继续广播旧角色（`tracker.py:356-367`）。
- `roi_batch.py`（131 行）：`expand_box`、`make_letterboxed_roi`、`pack_tiles`（1280/320=4×4）、`find_transform_for_box`、`RoiTransform.tile_box_to_image` 反变换。
- `pipeline.py`（447 行）：`TrackedRadarPipeline.process` 顺序 = 车检测 → 轨迹更新 → 投影（`car_bottom 0.92`）→ ROI 批量装甲识别 → 颜色证据 → 身份分配 → 目标状态机。目标状态：有观测 `MEASURED`；丢失但 `world_filter` 已初始化且 `age<=lost_target.predict_seconds(0.8)` → `OCCLUSION_HOLD`（纯预测）；更久则 `_blind_point` 取盲区点 → `BLIND_PREDICTION`，否则 `MISSING`。盲区选点打分 `0.6*exp(-d/400)+0.4*(cos+1)/2`，并对"上一次用的点"扣 0.25 防抖动；`base_time(3.0)+offset_time(1.7)=4.7s` 内保持同一吊射点。

### 4.3 legacy 的保持与盲区

- 遮挡保持 `occlusion_hold`（`main.py:289-292`、`:452-510`）：缓存 `{xy,last_seen_time,last_seen_frame,car_box,temporary_dead}`，超时 `max_hold_time=3`（0=永不释放）后删除；缺测时用缓存坐标补位并置 `guess_list=False`。
- 假死保持 `temporary_death_hold`（`:294-299`、`:394`、`:464`、`:2505-2521`）：装甲板未确认但车框与历史一致（中心 ≤80px、尺寸变化 ≤35%）时，沿用旧编号并用当前车框重新投影，标记 `temporary_dead`，车框丢失 3.0s 后放弃。
- 盲区预测（`blind_zone`）：`main.py:594-595` 的 `blind_zone_roles={'1','2','7'}` 是**硬编码**（未读 config 的 `roles`），`guess_table` 来自 `config.blind_zone.points`；发送侧 `send_point_guess`（`:1185`）在"标记进度 <120 且 4.7s 内无进度增长"时在两个预设点间交替（每 ID 独立 `guess_index`），进度一涨就重置计时。
- 滤波：`filter.type` 三选一（`sliding_window` 默认 / `hybrid_gated_kalman` / `hybrid`）；滑窗 `window_size=5, max_inactive_time=2.0, threshold=100000`；门控卡尔曼先做 3 点中值预滤，原始跳变 >120 且相对预测 >100 判为 outlier。

---

## 5 输出与协议

### 5.1 radio_ros（比赛默认，视觉不开串口）

`RadioRosTransport`（`vision/referee_transport.py:47`）起节点 `shark_vision_radar_transport`：以 20Hz（`referee.telemetry_rate_hz`，clamp 1~30）向 `/rm_radar_algorithm/telemetry` 发 `std_msgs/String`（紧凑 JSON），向 `/rm_radar_algorithm/radar_cmd` 发双倍易伤请求；订阅 `/rm_referee_serial_node/{referee_bridge,tx_frames,status}`，后台线程 spin，`snapshot()` 暴露 `connected/radio_online(status age≤2s)/pending_requests/last_ack/referee_status/strategy/last_error`。下行 ACK 必须 `source=='algorithm_radar_cmd_topic'` 且 `request_id` 匹配、`written is True` 才算成功（`referee_transport.py` 内 `_on_tx_frame`）。裁判 0x0305 由无线电侧融合后 **限速 4.8Hz** 发送——**这是跨子系统契约**。

遥测结构（`vision_telemetry.build_vision_telemetry`，`vision/vision_telemetry.py:19`）：

```
{schema:"shark.radar.telemetry.v1", source_time, side:"red"|"blue",
 vision:{camera_ready, model_ready:true, fps, processing_fps, camera_fps, inference_ms, filter_type},
 robots:{"R1":{"position_cm":{"x":int,"y":int},"valid":bool,"state":str,
               "confidence":float|null,"observed_at":ts|null}, ... } }
```

- 机器人键 = 每侧 `ROBOT_ROLE_IDS=('1','2','3','4','6','7')`（`vision_telemetry.py:6`，**5 号缺席**）；坐标 cm 整数；
- `state` 四值：`measured` / `occlusion_hold` / `blind_prediction` / `missing`；`valid` 要求 (x,y) 非 (0,0) 且在 `[0,2800]×[0,1500]`；`confidence` 只在 `measured` 且有效时给 1.0；`observed_at` 只在有效时给。

### 5.2 legacy_serial（直连串口）

`LegacySerialTransport` 本身是空壳（`referee_transport.py:31`），真正的收发在 `main.py` 的 `ser_send`/`ser_receive` 线程里，端口 `/dev/ttyUSB0`、115200、timeout 1s。

- **0x0305（雷达位置标记）**：`build_data_radar_all(send_map, state)`（`RM_serial_py/ser_api.py:108`）按角色顺序 `['1','2','3','4','6','7']`，**先敌方 6 台再己方 6 台**，每台两个 `uint16`（x,y，cm，小端）→ 48 字节；再 `build_send_packet(data, seq, [0x03,0x05])`（`ser_api.py:172`）加 `0xA5` 帧头、长度、seq、CRC8 与 CRC16。
- **0x0121（自主决策）**：`build_data_decision`（`ser_api.py:141`）= cmd_id `[0x01,0x21]` + 接收方 ID（红 9 / 蓝 109）+ 固定 `0x80,0x80` + `radar_cmd` + `password_cmd` + 6 字节密钥（ASCII）；`Radar_decision`（`ser_api.py:255`）解位域：bit0-1 双倍易伤机会、bit2 对方双倍状态、bit3-4 加密等级、bit5 密钥可改。密钥由 `generate_random_password()`（6 位字母数字）生成，`key_modifiable` 由 0→1 边沿触发重发（`main.py:1440-1453`）。
- **坐标约定**：视觉发出的就是**裁判坐标系 cm**（`send_map` 已由 `project_to_map_and_serial`/`send_point_*` 换算），但值域仍按 2800×1500 场地；`ser_y=1500-map_y`、`int(x)*10/10` 只是取整，不做单位换算。**没有"编码成裁判阵营/编号 ID"的步骤**——机器人身份完全由报文里的槽位顺序表��，帧内不出现 ID 字段。

### 5.3 视频/状态归档

`video_recorder.AsyncMatchVideoRecorder`（`vision/video_recorder.py:302`）后台线程写双路（相机+地图），码率/分辨率受 `camera_max_width=2560`、`map_max_width=1920` 限制，队列 4 帧、超 FPS 输入先限速再入队，满队列时"替换等待中的旧帧"而非阻塞推理；编码器顺序 `h264_nvenc → libx264 → mp4v → MJPG`（`:128-145`），PyAV 优先、OpenCV 兜底。录像目录由 `recording_storage.prepare_match_recording_directory`（`:263`）决定：走 `/proc/self/mountinfo` 找可写外置块设备（只读挂载、非块设备、USB 总线枚举一律忽略），优先外置盘 `SHARK-radar-recordings`，否则 `save_img/`，目录名带统一 `match_run_id`，并写 `recording_manifest.json`（schema `shark.radar.vision_recording.v1`，`main.py:1976-2052`）。运行状态写入 `.runtime/radar_status.json`（`runtime_status.py`），供启动台轮询。

---

## 6 配置项清单（config.yaml，224 行）

| 段 | 行 | 关键键 |
| --- | --- | --- |
| global | 1 | state、camera_mode、multi_car_recognition、use_serial、save_img、map_size:[2800,1500]、test.video_path/loop/start_frame/playback_speed |
| algorithm | 17 | mode(legacy/hkust_tracker)、vehicle_tracker(0.30/0.10/2/3.0s)、armor_roi(320/0.08/2/1280)、identity(0.85/0.60/0.15/3/5)、lost_target.predict_seconds 0.8 |
| calibration | 40 | points_per_layer、run_after_save |
| recording | 44 | fps 20、codec、camera_max_width 2560（MindVision 1280×1024 保持原尺寸）、map_max_width 1920、queue_size、max_catchup、prefer_external、selected_root |
| projection | 56 | point_source、car_x/y_ratio(0.5/0.92)、armor_x/y_ratio(0.5/1.5) |
| inference | 66 | car 640、armor 1280、armor_canvas 640、armor_conf 0.4 |
| armor_selection | 72 | mode(center/confidence) |
| vehicle_color_hold | 77 | 30+ 项（见 §2.3）含 `require_model_agreement`、`yellow_as_red`、各确认帧数 |
| armor_confirmation / duplicate | 113/120 | required_count 3、allowed_numbers[1,2,3,4,7]、scope(class/number) |
| occlusion_hold / temporary_death_hold | 124/128 | max_hold_time 3；80px/0.35/3.0s |
| referee_send / serial / referee | 135/138/143 | correct_only、115200、transport=radio_ros、telemetry_rate_hz 20 |
| filter / filter_compare | 148/163 | type、滑窗 5、门控卡尔曼 120/100/1e-5/1e-1 |
| paths | 169 | 标定 npy、2026 地图/掩码/备份图、三个引擎 |
| camera_params | 185 | exposure 11000、gain 17.9、device_serial `DA7873943` |
| blind_zone | 191 | enabled、roles[1,2,7]、base_time 3.0、offset_time 1.7、6 组各 2 个吊射点 |
| double_vulnerability | 204 | enabled:false、trigger_mode、启动窗 120s/1 次、距离 2.5m、运动阈值 |
| ui | 220 | 面板尺寸、show_armor_canvas |

**现场常改（= 启动台可写的 20 项，见 §1.2）**：阵营、相机模式/序列号、算法链路、通信方式与串口、录像开关与目录、曝光/增益、每层标定点数、滤波类型、盲区开关、易伤触发模式。其它（阈值、颜色、遮挡时长、230 个左右的细分参数）基本是开发期调定的。

---

## 7 测试与已知问题

**测试**：`vision/tests/` 11 个文件、约 70 个用例，全部只依赖纯逻辑（无需相机/相机 SDK/TensorRT），`pytest tests -v` 可跑。覆盖重点：`test_vehicle_color.py` 40+ 用例（颜色几何、保持/切换、模型兜底、过期不复用）、`test_referee_transport.py`（模式、请求去重、ACK、heartbeat、telemetry 角色集与状态词表）、`test_tracking_pipeline.py`（letterbox 往返、16 格切画布、两遍关联、身份唯一性、遮挡→盲区状态机）、`test_recording_storage.py`（外置盘发现、只读忽略、命名目录、掉盘回退、逃逸防护）、`test_video_recorder.py`（同步双路、限速、队列替换）、`test_calibration_presets.py`、`test_config_editor.py`、`test_runtime_status.py`、`test_frame_rate.py`、`test_mvs_runtime.py`、`test_camera_availability.py`。**未覆盖**：`main.py` 主循环与 legacy 发送逻辑、`calibration.py` UI、`launcher.py`、`hik_camera`。

**TODO/FIXME**：项目自有代码中**没有** TODO/FIXME/XXX；全部命中都在 vendored 的 `utils/`（YOLOv5 上游）与 `MvImport*/`（海康 SDK）。

**硬编码 / 不一致清单**（2027 适配前的技术债）：

1. 相机序列号 `config.yaml:185` 段内 `device_serial: DA7873943`（启动台可改，但配置文件里是写死的具体机器）；
2. 串口 `/dev/ttyUSB0`（`config.yaml:138`，启动台可改）；
3. 场地 `2800×1500` 散落在 `config.yaml:8`、`main.py:432`、`:2524`、`vision_telemetry.py:19`、`tracking/types.py:63`、`tracking/pipeline.py:327`；
4. 双层固定 0/300cm（`calibration.py:666` 两值取模；`M_height_r` 命名，`main.py:524`）；
5. 2026 地图/掩码文件名 `images-2026/*`（`config.yaml:169` 段）；
6. 编号白名单 `allowed_numbers=[1,2,3,4,7]`（`main.py:271-279`）、盲区角色 `blind_zone_roles={'1','2','7'}` 硬编码在 `main.py:594`、吊射点坐标 `config.yaml:191` 段；
7. `ROBOT_ROLE_IDS` 无 '5'（`vision_telemetry.py:6`）但 armor.yaml 有 R5/B5 类 —— 5 号不会被上报；
8. 0x0305 的 48 字节布局、角色顺序、0x0121 的接收方 ID 9/109、`0x80 0x80`、位域定义（`ser_api.py:108-167`、`:255-279`）；
9. 双倍易伤逻辑仍在（`main.py:577-583`、`:1560-1665`），但 `double_vulnerability.enabled:false` 且启动台强制写 false（`launcher.py:815`）；
10. README 与实现不一致：legacy 不使用 `armor_batch.engine`；`armor_canvas_size` 在 `main.py:205` 默认 1280、config 却是 640；`VehicleColorMemory` 的 `max_missed_frames` config=1、pipeline 里强制 `max(...,2)`（`tracking/pipeline.py:136`）；
11. 重复代码：`hik_camera_get`/`video_capture_get` 在 `main.py` 与 `calibration.py` 各一份；双层仿射在 `main.py:411` 与 `:2478` 各一份；颜色/身份在 legacy 与 tracking 两套实现。

---

## 8 2027 适配关注点（按风险排序）

| 风险 | 位置 | 说明 / 建议 |
| --- | --- | --- |
| 高 | `config.yaml:1`(map_size)、`vision_telemetry.py:6/19`、`tracking/types.py:59`、`main.py:432` | 场地尺寸与坐标系原点/朝向写死在多处；2027 若改场地（尺寸或镜像规则）需统一为配置常量，且要与 radio 侧融合节点同步改 |
| 高 | `config.yaml:169` 段、`main.py:411/2478`、`images-2026/map_mask26.jpg` | 选层靠地图掩码颜色；换场地必须重做 `mask`/`red.jpg`/`blue.jpg`/`backup`，且 mask 必须与标定图同尺寸，否则 clamp 与判层全错 |
| 高 | `RM_serial_py/ser_api.py:108-167`、`:255-279` | 0x0305 报文布局、0x0121 接收方 ID、位域定义直接对应裁判系统通信协议 V1.0.0(20261029)；2027 协议若有字段/命令字变化，这里与 `radio/` 需同步 |
| 高 | `armor.yaml` 12 类、`main.py:271-279,594`、`vision_telemetry.py:6` | 兵种/编号集合（无 5、无 6）与盲区角色、白名单耦合；2027 若恢复 5 号或调整被标记对象，须同时改模型类别、白名单、telemetry 角色集与 radio 侧解析 |
| 中 | `calibration.py:666`、`main.py:524` | 0cm/300cm 两层是硬编码概念（含变量名），2027 场地高地高度/层数变化时要改标定 UI、数据格式（npy 数组个数）与 `main.py` 消费顺序 |
| 中 | `.engine` 三个文件 | TensorRT/CUDA/GPU 版本绑定（README 锁定 TRT 10.13.3.9 / CUDA 12.8）；换机器或升级驱动必须用 `onnx2engine.py` / `scripts/build_armor_batch_engine.py` 重建，否则运行时段错误 |
| 中 | `main.py:1122-1558`（整段发送逻辑） | legacy 与 radio_ros 两套发送/盲区/易伤代码并存，`chances_flag`、`guess_value` 等全局状态多；2027 若彻底改 ROS 通信，建议删除 legacy 路径收敛到 telemetry |
| 中 | `config.yaml:191`（blind_zone.points） | 吊射点是 2026 场地像素坐标（按 calib 地图系），换场地全废；`base_time/offset_time=4.7s` 也是 2026 节奏调出来的 |
| 中 | `main.py:594` vs `config.yaml:198` | `blind_zone.roles` 配置存在但代码不用（硬编码 `{'1','2','7'}`），属"改了配置不生效"的陷阱 |
| 低 | `config.yaml:185-190` | 曝光 11000 μs 与海康增益 17.9 dB 来自旧相机配置；MindVision 当前只应用曝光且需现场复核，增益控件不跨厂商映射；标定来源与帧尺寸现由 `_points.npz` 记录 |
| 低 | `main.py:1976-2052`、`recording_storage.py` | 录像依赖 `/proc`、sysfs 与外置盘挂载，2027 若换主机/换发行版需回归测试（有单测覆盖） |
| 低 | `launcher.py:804-824` | 启动台写死的 20 个键若与新版 config 结构不一致会在 `update_config_values` 抛 `KeyError`（有校验，失败时不落盘） |

**结论**：视觉侧的算法与工程骨架（检测→ROI→颜色/身份→双层投影→滤波→遥测）可以直接复用；2027 的实际工作量的 80% 会落在"场地常量与标定资产重建（mask/地图/吊射点/层数）"、"编号与兵种集合对齐"、"裁判协议字段对齐（0x0305/0x0121/telemetry schema）"和"引擎重建"四件事上。建议第一步就把 §7 的 11 条硬编码收敛成 `config.yaml` 中显式的 `field:`/`protocol:` 段，并让 `main.py` 与 `tracking/` 共用一份投影/编号工具，避免两套实现继续分叉。
