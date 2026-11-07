# radio/ 子系统代码研读报告（RM2026 基线 → 2027 适配）

分析对象：`radio/`（JNU-SHARK/shark-radar-radio，ROS 2 Humble + GNU Radio 3.10.7 + gr-iio）。
只读分析，未修改仓库任何文件；未运行 ROS/GNU Radio 节点，结论均来自源码与 `radio/README.md`。

---

## 1. 架构总览

### 1.1 进程 / 节点拓扑

| 进程（console_script） | 代码入口 | 启动方式 | 职责 |
|---|---|---|---|
| `rm_gfsk_node` | `radio/src/rm_radio_ros/rm_radio_ros/nodes/gfsk_node.py`（125 B 薄封装）+ `nodes/flowgraph_node.py` | `launch/rx2.launch.py:10-218` | RX1 信息波接收（2 MS/s） |
| `rm_gfsk_interference_node` | 同上（同名类，不同参数） | 同上 | RX2 干扰波接收 |
| `rm_referee_serial_node` | `nodes/referee_serial_node.py` | `launch/referee_serial.launch.py:10-133` | 裁判系统串口收发（0x0305/0x0121/0x0234） |
| `rm_radar_integration` | `nodes/radar_integration_node.py`（349 行） | `apps/match_rx/start.sh:483-501` | 视觉+无线电双源融合、0x0305 下发、双倍易伤决策 |
| `rm_match_dashboard` | `app_support/match_dashboard_node.py`（1885 行） | `apps/match_rx/start.sh:503-508` | Web 面板 + HTTP API + 子进程管理 |
| `rm_match_recorder` | `nodes/match_recorder_node.py` | `apps/match_rx/start.sh:432-453` | 比赛事件 + IQ 录制 |
| `rm_auto_password_node` | `nodes/auto_password_node.py`（133 行） | `apps/match_rx/start.sh:477-481` | 自动申请/重置干扰密码 |
| `rm_ganraoyuan_node` | `nodes/ganraoyuan_node.py`（137 B）+ `flowgraphs/ganraoyuan/RM.py`（707 行） | `launch/ganraoyuan.launch.py:10-53` | 干扰波发射（复刻官方空口帧） |
| `rm_iq_replay_tx_node` | `nodes/iq_replay_tx_node.py` | 实验台 | 录制 IQ 回放发射，A/B 验证解调链路 |
| `rm_offline_iq_scan` | `core/offline_iq_scan.py`（~900 行） | CLI | 离线 IQ 盲扫解码（无硬件复盘） |
| `rm_find_usb_serial` / `rm_virtual_link_test` | `core/usb_serial_finder.py` / `core/virtual_link_test.py` | CLI | 串口定位 / 无裁判机联调 |

`setup.py:87-101` 注册 12 个 console_scripts；`setup.py:44-74` 把 4 个 launch、3 个 config、`masks/assets/double_vulnerability_trigger_mask.npz`、4 个 flowgraph 源文件装进 `share/`。

### 1.2 线程与调度模型

- **每个 RX flowgraph 节点**：rclpy 执行器 + GNU Radio `top_block`。流图启停走 4 个 Trigger 服务 `~/start`、`~/stop`、`~/reload`、`~/status_trigger`（`nodes/flowgraph_node.py:220-246`，处理函数 `nodes/flowgraph_node.py:2040-2062`）；状态发布由 `status_period_sec` 定时器驱动（`:246`）。流图模块由 `sys.path` 注入 + `spec_from_file_location` 动态导入（`:496-511`）。
- **ROS 参数 → 环境变量单向桥**：`nodes/flowgraph_node.py:445-494` 把参数写成 `RM_RADIO_*` 环境变量，流图在 `__init__` 里读 env（`flowgraphs/gfsk/RX.py:70-156`）。**这是唯一配置通道**，流图对象不暴露 rclpy 接口 → 新增 2027 参数必须同时改三处：节点声明、env 写入、流图读取。
- **referee 节点**：4 个定时器 —— 串口读 `read_period_sec=0.01`、状态 `1.0 s`、无敌目标 `rate×6`、帧看门狗（`nodes/referee_serial_node.py:375-386`）。
- **radar_integration**：2 个定时器，发送 `1/send_rate_hz`（4.8 Hz）、状态+决策 `1/state_rate_hz`（20 Hz）（`nodes/radar_integration_node.py:126-156`）。
- **面板**：HTTP 服务器线程 + 周期性聚合各节点 status（`app_support/match_dashboard_node.py:1707-1783`）。
- **算法进程**：`app_support/run_algorithm.py:173-190` 额外起 `rm-radio-0305-5hz` 线程，以 `radar_client_rate_hz` 独立补发 0x0305（与节点 timer 并行，双通道）。

### 1.3 话题与 QoS

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| RX 节点发布 | `~/status`、`~/frames`、`~/referee_bridge` | String(JSON) | depth 20, TRANSIENT_LOCAL（`flowgraph_node.py:220-246`） |
| RX 节点发布 | `~/spectrum`、`~/filtered_spectrum` | String(JSON) | depth 1, volatile（latest-only，`:220-246`） |
| RX 节点订阅 | `~/air_payload_hex`、`~/demod_bits_hex`、`~/setters_json`、`/rm_match_recorder/control` | String | depth 10（setters 为 TRANSIENT_LOCAL） |
| 串口节点发布 | `~/tx_frames`、`~/rx_frames`、`~/referee_bridge`、`~/raw_rx`、`~/status` | String | depth 20 |
| 串口节点发布 | `~/game_status`、干扰控制话题 | String | TRANSIENT_LOCAL（`referee_serial_node.py:346-373`） |
| 串口节点订阅 | `~/send_frame`、`~/send_interaction`、`~/send_radar_cmd`、3 路 `*/referee_bridge`、`/rm_radar_algorithm/radar_cmd` | String | depth 10 |
| 融合节点 | 发布 `/rm_referee_serial_node/send_frame`、`/rm_radar_algorithm/radar_cmd`、`~/status`；订阅 `/rm_radar_algorithm/telemetry`、3 路 bridge、串口 status/tx_frames | String | `radar_integration_node.py:126-156` |

话题命名统一用节点名限定（`/rm_gfsk_node/...`、`/rm_gfsk_interference_node/...`、`/rm_referee_serial_node/...`），跨进程只依赖字符串 JSON，无自定义 msg → 2027 换协议字段不改接口，只改 JSON schema。

### 1.4 端到端数据流（SDR → 裁判串口）

1. **射频收**：`iio.fmcomms2_source_fc32(ip:192.168.2.1, [True,True], 32768)`，B_BALANCED 端口，2 MS/s，`set_len_tag_key("")`（`flowgraphs/gfsk/RX.py:276-288`）。
2. **前端滤波**：信息波单级带通 5 k–260 kHz / 过渡带 50 k（`RX.py:290-302`、`RX.py:340-358`）；干扰波两级低通（16 k/50 k 级联）。
3. **解调**：`mm` 路径 `ContinuousGfskDemod`（`RX.py:304-321`）或 `legacy` 路径 `digital.gfsk_demod`；输出 uint8 0/1 比特流，经 `_DemodBitCallbackSink` 回调（`RX.py:169-191`）。
4. **比特 → 空口包**：`AccessCodeAirPacketExtractor` 以 8 字节 access code + 4 字节固定长度域 `00 0F 00 0F` 切出 15 字节载荷（`core/rm_protocol.py:364-473`，`AIR_LENGTH_FIELD` 在 `:48`）。
5. **帧重组**：15 字节切片依次 push 进自同步 `RefereeFrameAssembler`（`nodes/flowgraph_node.py:1147-1168` → `rm_protocol.py:287-291`），SOF=0xA5、crc8 头校验、crc16 整帧校验（`rm_protocol.py:299-361`）。
6. **解析**：`parse_radar_frame` 按 cmd_id 分派（`rm_protocol.py:716-846`），`RefereeBridge` 转成稳定的 JSON type（`core/referee_bridge.py:92-181`）。
7. **融合**：`RadarFusionState.update_radio_bridge` 只吃 `type=='RadarInfoToClient'`（`core/radar_integration.py:181-208`），与视觉遥测合成为 `shark.radar.fusion.v1` 快照。
8. **下发**：融合出 48 字节 0x0305 载荷（`core/radar_integration.py:59-71`）→ `/rm_referee_serial_node/send_frame` → 串口节点组 A5 帧、限速 5 Hz、写 `/dev/ttyUSB0`（`nodes/referee_serial_node.py:467-488`、`:908-926`）。
9. **决策回环**：`0x020E RadarDecisionSync`（可申请次数/can_change_password）→ 串口节点与 `AutoDoubleVulnerabilityDecision` 消费（`rm_protocol.py:568-583`、`core/double_vulnerability.py:138-163`）→ `0x0121` 雷达指令（`referee_serial_node.py:856-902`）。

---

## 2. 解调与帧同步链路

### 2.1 空口帧结构

```
| access code 8 B | length field 4 B (00 0F 00 0F) | payload 15 B |   = 216 bit
```
- access code：信息波 `2F 6F 4C 74 B9 14 49 2E`，干扰波 `16 E8 D3 77 15 1C 71 2D`（`core/rm_protocol.py:100-103`）。
- 关键事实：**空口载荷被切成固定 15 B 片段**（`rm_protocol.py:383-385`），而 0x0A01 完整帧是 5+2+24+2=33 B，0x0A05 是 50 B。因此帧是**跨多个空口包拼接**的：解调器每包只交 15 B 给 assembler 的滚动缓冲（`rm_protocol.py:293-297`）。**这意味着：2027 若 payload 长度或包结构变化，`packet_bits`、`payload_bits`、`push_air_payload` 的 15 字节硬约束必须同步改**，否则 assembler 永远拼不出帧。
- 允许反相：`allow_inverted=True` 时额外匹配 `1-bit` 变体（`rm_protocol.py:377-381`），应对 I/Q 交换或差分极性反转。

### 2.2 关键常量（2027 首要核对表）

| 参数 | 值 | 位置 |
|---|---|---|
| 流图采样率 | 2 000 000 Hz（硬编码，不读 env） | `flowgraphs/gfsk/RX.py:251` |
| 空中采样率 / 空中 SPS | 1 000 000 Hz / 47 | `core/radio_config.py:9-13` |
| 流图 input_sps | 94（= 47×2，`RM_RADIO_INPUT_SPS`） | `flowgraphs/gfsk/RX.py:149-156`、`:252` |
| 高斯 BT | 0.35 | `RX.py:253`、`continuous_gfsk_demod.py:34` |
| 信息波频点 | 红 433 200 000 / 蓝 433 920 000 | `core/radio_config.py:4-7`、`RX.py:29-32` |
| 信息波带宽 / BPF | 540 000 / 5 000–260 000（过渡 50 000） | `RX.py:34-37`、`radio_config.py:33-46` |
| 信息波低通 | 260 000 | `radio_config.py:33-46` |
| 干扰波频点（1/2/3 级） | 红 432.2/432.5/432.8 MHz，蓝 434.92/434.62/434.32 MHz | `radio_config.py:49-60` |
| 干扰波带宽 | 940 000 / 860 000 / 250 000 | 同上 |
| 干扰波低通 | 500 000 / 500 000 / 160 000 | `radio_config.py:63-140` |
| 灵敏度 sen_re | 信息波 1.5628/2；干扰波 2.8194/2、2.5681/2、0.6517/2 | `radio_config.py:20-31`、`RX.py:52-56` |
| RX 增益 | 默认 20.0 dB，硬边界 −1…73 dB | `radio_config.py:20-31`、`RX.py:58-67` |
| legacy gfsk_demod | gain_mu 0.175、mu 0.5、omega_relative_limit 0.005 | `RX.py:304-321` |
| mm 解调参数 | decimation 2、gaussian 60 taps、loop_bw 0.046、damping 1.0、ted_gain 0.85、max_deviation 1.5 | `continuous_gfsk_demod.py:31-46` |
| 解调内 AGC | complex AGC rate 1e-4 max_gain 4.0；float AGC rate 1e-4 max_gain 65536 | `continuous_gfsk_demod.py:71-92` |
| CRC8 / CRC16 | init 0xFF poly 0x8C 反射 / init 0xFFFF poly 0x8408 | `rm_protocol.py:126-148` |
| 空中包比特数 | 8×(8+4+15)=216 | `rm_protocol.py:383` |

空中频率与 oversampling 的设计意图（README.md:114-127）：空中 1 MS/s、SPS 47；接收端刻意 2 MS/s + SPS 94 过采样，再在解调器内 `rational_resampler` 抽 2 回到 1 MS/s / 47 SPS（`continuous_gfsk_demod.py:125-135` 的 `output_sps=input_sps/decimation`）。

### 2.3 帧同步与 CRC 位置

- **帧同步 = 硬匹配 access code**（汉明距离阈值 `max_access_hamming=3`）+ **长度域必须精确等于 `00 0F 00 0F`**（`max_length_hamming=0`），见 `rm_protocol.py:367-374`、`:418-423`。找到后直取 120 bit 载荷（`:425-429`）。
- 扫描是逐 bit 滑动（`_find_pattern`，`rm_protocol.py:448-463`），包内失配则 `del self._bits[0]` 重新对齐（`:421-423`），因此坏包不会卡死，但 WER 高时 CPU 消耗随窗口线性上升（`max_bits` 默认 16384，`flowgraph_node.py:57-143`）。
- **CRC 在字节流层，不在空口包层**：`RefereeFrameAssembler._drain`（`rm_protocol.py:299-361`）先校 crc8（覆盖帧头 4 B，`:311`），再算帧长 `5+2+len+2`（`:317-318`），最后校 crc16（覆盖除尾 2 B 外整帧，`:330-331`）。任一步失败只丢 1 字节重新找 SOF（`:314`、`:322`、`:334`），属于"自同步流式解析"，对丢包鲁棒但也意味着**单比特错误会被放大成整帧丢弃 + 重新同步开销**。
- **二级长度校验**：assembler 的 `allowed_cmd_lengths` 用 `RADAR_AIR_DATA_LENGTHS`（仅 0x0A01–0x0A06，`rm_protocol.py:25-29`、`flowgraph_node.py:157-161`），crc16 通过但长度不符也会被拒（`rm_protocol.py:341-352`）；`parse_radar_frame` 再用 `len(data)==N` 二次把关（`rm_protocol.py:718-721`、`:733/744/766/784/804/829`）。**2027 改帧长必须同时改这两层，否则表现为"CRC 正常但 valid_frames 不涨"**。

### 2.4 干扰波发射链路（ganraoyuan：RM.py + jiang.py）

发射侧是"复刻官方空口帧"的自研链路，用于对抗性验证与干扰压制：

- **比特源 `flowgraphs/ganraoyuan/jiang.py`**：自实现 `_crc8_rm`（init 0xFF，poly 0x8C 反射，`:50-60`）与 `_crc16_rm`（init 0xFFFF，poly 0x8408，`:63-78`），把 `cmd_id=[0x0A,0x06]`、`data=6`（干扰密码）、`SOF=0xA5` 组成完整裁判帧，再补 8 字节 access code + 4 字节长度域（`_AIR_PACKET_BYTES=8+4+6`，`:42-43`）；符号率默认 `1_000_000/47`（`:41`），与接收端 `OFFICIAL_GFSK_SAMPLE_RATE/OFFICIAL_GFSK_SPS` 严格一致（`core/radio_config.py:9-13`）。
- **调制与发射 `flowgraphs/ganraoyuan/RM.py`**：`digital.gfsk_mod`（`:430`，BT=0.35）→ `iio.fmcomms2_sink_fc32(interference_tx_uri, [True,True,False,False], tx_buffer_size)`（`:420`）；中心频点由 `_INTERFERENCE_TX_SETTINGS[side][level]` 决定（`:98-105`），带宽 `BW_ganrao` ∈ {940k, 860k, 250k}，`to_F_ganrao = BW_ganrao/2 − symbol_rate`、`sen_ganrao = 2π·to_F_ganrao/sample_rate`（`:181-203`、`:547-552`）。
- **功率控制**：`attend_gr`（衰减 dB）默认 12.5，UI 滑杆范围 10–89.75、步长 0.25（`:209-211`），是唯一可用的"干扰强度"旋钮；`set_sample_rate` 会同时改 sink 频率范围与 `sps`（`:502-512`），`set_symbol_rate` 联动 `jiang.set_symbol_rate`（`:534-537`）。
- **频点下拉** `_center_f_options` 是 6 个硬编码值（`:438`），与 `radio_config.py` 的干扰表重复，见 8.2。

### 2.5 增益控制与自动调优

- 增益模式：信息波 `manual`（默认 20 dB，上限 `broadcast_gain_max=73`，`RX.py:468-474`），干扰波 `fast_attack`（`radio_config.py:63-140`）。`start.sh:309-312` 强制 RX1 必须 manual，`:314-324` 校验 `-1 ≤ gain ≤ gain_max ≤ 73`。
- **IQ 辅助解码（第二套解调通道）**：同一条 IQ 流另存窗口（`flowgraph_node.py:1583-1611`），用 `core/rx_autotune.py` 盲扫：候选 SPS 由 `default_sps_values` 从 `sample_rate/(1e6/47)` 生成（`:21-27`），候选频偏由 FFT 峰值 + 固定偏移表生成（`:71-107`、`:132-159`），候选低通按 `profile/level/mode` 生成（`:43-68`），笛卡尔积上限 `max_frequency_candidates=8`、`max_candidates=24`（`:162-189`）。评分标准是"能解出期望命令"：broadcast 要 0x0A01–0x0A05、interference 要 0x0A06（`rx_autotune.py:11-14`）。
- **调优锁定**：`iq_decoder_autotune_lock_enabled=True`、`lock_max_miss=3`（`flowgraph_node.py:57-143`），锁定后不再漂移；`vote_repeated_frames=True` 对同载荷多帧投票（`flowgraph_node.py` 中 `_push_iq_decoded_payloads`，`:1861`），`core/offline_iq_scan.py:377-535` 提供 `_vote_consensus_payloads`/`_apply_soft_diversity` 两套离线算法。
- 采样率**在流图构造期硬编码**为 2 MS/s（`RX.py:251`），`RM_RADIO_SAMPLE_RATE` 只会进入节点参数，构造时不读；`RX.py:393-508` 虽有 `set_sample_rate` setter，但必须显式写进 `setters_json` 才会被 `_apply_setter_values` 调用（`flowgraph_node.py:513-579`）。如果 2027 规则书的空中速率变化（SPS 47 变了），这是**必须改的第一处**。

---

## 3. 0x0A01–0x0A06 解析（rm_protocol.py）

统一约定：全部小端（`le_u16`/`le_u32`/`le_u64`，`rm_protocol.py:151-183`）；`parse_radar_frame` 入口先算 `valid_radar_length`（`:716-721`）；每条命令同时输出"官方名"与"legacy 兼容名"两套键（`0A01/0A02/0A03`），下游可任选。

| cmd | 长度 | 结构 | 关键字段（偏移） | 位置 |
|---|---|---|---|---|
| 0x0A01 敌方位置 | 24 | 6×{u16 x, u16 y} | index×4 起；names=`RADAR_WIRE_ROBOT_NAMES`（对手 hero/engineer/infantry_3/infantry_4/aerial/sentry） | `:733-743`，名字表 `:51-58` |
| 0x0A02 敌方血量 | 12 | 6×u16 | 第 5 项 `reserved`（`:750`），其余 5 台真值 | `:744-765` |
| 0x0A03 敌方弹量 | 10 | 5×u16 | hero/infantry_3/infantry_4/aerial/sentry（无 engineer） | `:766-783` |
| 0x0A04 敌方宏观 | 8 | u16 remaining_coins + u16 total_coins + u32 bitfield | bit0 补给区占据、bit1-2 中央高地、bit3 梯形高地、bit4-5 堡垒增益、bit6-7 前哨站增益、bit8 基地增益、bit9-10 飞坡 RFID、bit11-12 己方飞坡、bit13-15 地形跨越 RFID | `:784-803` |
| 0x0A05 敌方增益 | 41 | 5×7 B buff + 1 B sentry_mode + 5 B main_status | 每台：hp_recovery(u8 %)、shooting_heat_cooling(u16)、defense(u8 %)、negative_defense(u8 %)、attack(u16 %)，偏移 0/7/14/21/28；sentry_mode@35；main_status@36..40 | `:804-828`，偏移表 `:64-70`，模式名 `:90-97`，状态名 `:82-87` |
| 0x0A06 干扰密码 | 6 | 6 字节 ASCII | 校验 `isalnum()`，非 ASCII 时 replace 解码并置 false | `:829-837` |

辅助命令（同一解析入口）：0x020E `radar_decision_sync` 1 B 位域（bit0-1 可申请次数、bit2 是否双倍易伤、bit3-4 己方加密等级/对手干扰难度、bit5 can_change_password、bit6-7 保留，`:568-583`）；0x0001 比赛状态 11 B（byte0 低 4 位 game_type、高 4 位 game_progress，`match_running = progress==4`，`:586-602`）；0x0003 血量 20 B（含 offset 8 的 int16 伤害差，`:605-622`）；0x0201 机器人状态 17 B（offset 12 float 弹速上限，`:654-679`）；0x020C 雷达标记进度 16 bit 位图（`:682-713`）；0x0301 机器人交互（data_cmd_id 0x0121 时再解析 8 B 雷达指令，`:840-844`）。

**长度校验三处**：帧长合法性 `frame_length<9`（`:319`）、命令长度白名单（`:341-352`）、解析前 `len==N`（`:733` 等）。字段越界读取由 `le_u16` 的 `struct.unpack_from` 保护（短 data 直接抛 `struct.error`，属于**未捕获异常点**，值得在 2027 加固）。

**2027 协议变化时"必须改"的坐标（本轮不深挖字段语义）**：

- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:9-22`（`RADAR_DATA_LENGTHS` 全部长度）
- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:25-29`（空中长度白名单）
- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:48`（`AIR_LENGTH_FIELD`）
- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:100-103`（access codes）
- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:51-70`（robot 名表与 buff 偏移）
- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:73-97`（main_status 表、sentry 模式表）
- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:733-844`（各命令字段偏移；位域图在 `:789-803`）
- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:568-583`（decision_sync 位域）
- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:528-545`（雷达指令 8 B：radar_cmd、password_cmd、6 字符密码）
- `radio/src/rm_radio_ros/rm_radio_ros/core/rm_protocol.py:116-119`（robot_id→阵营）+ `:106-113`（比赛阶段名）
- `radio/src/rm_radio_ros/rm_radio_ros/core/referee_bridge.py:10-23`（12 台顺序）、`:92-121`、`:145-181`（cmd→type 映射）
- `radio/src/rm_radio_ros/rm_radio_ros/core/radar_wireless_constraints.py:50-96`（血量/弹量/buff 上限，实验台随机载荷合法性）

---

## 4. 双源融合（radar_integration.py + radar_integration_node.py）

### 4.1 数据模型与坐标

- 场地系：2800 cm × 1500 cm；`valid_position` 要求 `0≤x≤2800`、`0≤y≤1500` 且 `(x,y)≠(0,0)`（`core/radar_integration.py:41-49`）。融合输出沿用该坐标系，**不做旋转/平移**——翻转只发生在双倍易伤 ROI 查表时（`core/double_vulnerability.py:60-77`：红方先 `x=2800-x, y=1500-y` 再映射像素行列）。
- 视觉输入 schema `shark.radar.telemetry.v1`（`:137` 校验），状态机 `measured / occlusion_hold / blind_prediction / missing`（`:10`），只有 `measured` 被当作实测可信。
- 无线电输入只认 `type=='RadarInfoToClient'`，取 `RadarSingleRobotInfo[0..5]` 的 `target_pos_x/y` 映射到"对手 6 台"（`:181-208`）；其它 type 原样塞进 `strategy` 字典（`:182-186`），供决策模块使用（`RadarEnemyBuffStatus` 就来自这里）。
- 0x0305 载荷：`struct.pack('<24H', ...)` 12 台 × (x,y)，**先对手 6 台、后己方 6 台**（`:59-71`）；全 0 返回 `None`。

### 4.2 时效与来源选择

| 阈值 | 值 | 位置 |
|---|---|---|
| `vision_timeout_sec` | 0.5 s | `core/radar_integration.py:77-96` |
| `radio_timeout_sec` | 1.0 s | 同上 |
| `radio_ghost_sec` | 2.0 s（丢失后保留幽灵位的时间） | 同上 |

来源优先级（`snapshot`，`core/radar_integration.py:216-311`，选择逻辑 `:250-259`）：

1. 该目标**不属于己方**且 radio 数据 fresh → `source='radio'`；
2. 否则 vision fresh（`measured`）→ `source='vision'`；
3. 否则 `missing`。

发送门（`:272-279`）：`payload is None` 或 `side_mismatch` 时 `send_allowed=False`，`block_reason ∈ {configured_side_mismatch, all_positions_zero_or_stale}`；面板据此报"视觉在线但坐标全无效"（`app_support/match_dashboard_node.py:1149`）。

阵营同步：`auto_sync_side=True` 时用裁判 0x0201 `robot_status` 反推己方红蓝，`set_referee_side` 热切换并清空 radio 缓存（`core/radar_integration.py:113-130`），避免把己方位置当敌方发出去。

### 4.3 节点接口

参数（`nodes/radar_integration_node.py:38-61`）：`radio_side`、`auto_sync_side`、`telemetry_topic=/rm_radar_algorithm/telemetry`、`bridge_topics`（RX1/RX2/串口三路）、`send_frame_topic=/rm_referee_serial_node/send_frame`、`send_rate_hz=4.8`、`state_rate_hz=20.0`、`vision/radio/ghost` 三个超时、`radar_cmd_topic=/rm_radar_algorithm/radar_cmd`、`auto_double_vulnerability_*`。
发布/订阅与定时器见 `:126-156`；`send_rate_hz` 硬上限 4.8、`state_rate_hz` 限 1–30（同处）。**双限速**：节点 timer 之外，`_send_tick` 再用 monotonic 二次限速（`:260-278`，保证不超 5 Hz），因为裁判协议对 0x0305 有 ≤5 Hz 的硬限制（串口节点 `:908-926` 是第三道限速）。`_status_tick`（`:280-338`）既发状态也驱动双倍易伤决策，决策异常被捕获而不影响 0x0305（`:301`）。

---

## 5. 裁判串口与决策

### 5.1 串口帧构造与防护

- 帧格式：`0xA5 | data_length(u16 LE) | seq(u8) | crc8 | cmd_id(u16 LE) | data | crc16(u16 LE)`（`core/rm_protocol.py:476-486`、`:299-361`）。seq 0–255 循环（`nodes/referee_serial_node.py:415-418`）。
- 0x0305 发送路径：`_handle_send_frame`（`:467-488`）→ 拒收原始 0x0301/0x0121（`:467-474`，防止绕过协议守卫）→ `_valid_0305_payload` 要求 **48 字节且非全 0**（`:79-80`、`:477-479`）→ `_allow_0305_send` 限 `radar_client_max_rate_hz=5.0`（`:253`、`:908-926`，超速计数并按 1/10/100 次降频打日志）→ `build_referee_frame` → `_send_bytes`（`:483`、`:433-455`）。
- 0x0121 雷达指令：`_build_radar_cmd_from_payload`（`:856-902`）组 `build_robot_interaction_frame(0x0121, sender_id=9, receiver_id=0x8080, user_data=8B)`；守卫三条——`radar_cmd` 只能递增或不变、不得下降（`:928-935`）；`password_cmd=1` 必须 `0x020E.can_change_password=True`（`:937-944`）；`password_cmd=2` 走密码校验并 10 s 冷却（`:794-830`、`password_verify_cooldown_sec=10.0`）。
- 语义指令通道：`_handle_algorithm_radar_cmd`（`:512-575`）接受 `shark.radar.command.v1` + `action='trigger_double_vulnerability'`，用 `request_id` 做 256 条去重缓存（`:565-575`），避免 ROS 重投递导致重复申请。
- 降级/健壮性：`dry_run=True` 时只发不写（`:433-455`）；`fcntl.flock` 独占锁 `/tmp/rm_radio_referee_serial_<dev>.lock`（`:107-137`），断线不释放锁以防双进程抢串口（`:195-198`）；写操作检查部分写（`:201-214`）；`require_serial_open_on_start` 决定串口打不开时是否直接失败（`:243-273`）；读循环 2 s 无有效帧即报 `frame_timeout`（`:971-1058`）。**注意：`dry_run` 默认值为 True（`:243-273`、`config/referee_serial.yaml`），生产必须由 `apps/match_rx/config.yaml` 的 `referee.dry_run: false` 与 start.sh 参数覆盖。**

### 5.2 无敌目标 0x0234 轮询

`_send_invincible`（`nodes/referee_serial_node.py:680-792`）：数据源为融合 `strategy['RadarEnemyBuffStatus'].robot_main_status`（状态 2=无敌未易伤、3=无敌已易伤，`core/invincible_targets.py:11-12`）；压缩成 1 字节 5 bit 掩码（`:88-126`）；**每帧只带 1 个接收者**，6 个接收者（1/3/4/5/6/7）轮询，3 Hz × 6 = 18 fps（`invincible_targets_send_rate_hz=3.0`，`:375-386`）；`data_cmd_id` 必须落在 0x0200–0x02FF（`:688`），默认 0x0234（`:9`）；数据新鲜度 `invincible_targets_freshness_sec=1.0`，过期不发。

### 5.3 双倍易伤自动申请（double_vulnerability.py）

- 配置（`core/double_vulnerability.py:30-37`）：`enabled=False`、`zone_dwell_sec=3.0`、`hero_hp_drop_amount=100`、`hero_hp_window_sec=3.0`、`endgame_trigger_sec=30.0`、`referee_data_timeout_sec=2.5`。生产值由 `apps/match_rx/config.yaml` 的 `radar_integration.auto_double_vulnerability` 段与 `start.sh:483-501` 的 `-p auto_double_vulnerability_*` 覆盖（enabled=true, dwell 3.0, hp_drop 100, window 3.0, endgame 30, timeout 2.5）。
- ROI 判定：加载 npz 掩码 `masks/assets/double_vulnerability_trigger_mask.npz`（`:40-92`），字段坐标→像素：红方先 180° 翻转，`row=x/2800*(h-1)`、`col=y/1500*(w-1)`（`:60-77`）。
- 三条件（`:316-447`），命中顺序固定：
  1. `hero_hp_rapid_drop`：己方 hero 血量在 `hero_hp_window_sec` 滚动窗口内的最大值 − 当前值 ≥ `hero_hp_drop_amount`（`:258-304`）；
  2. `enemy_in_trigger_zone_dwell`：敌方可达目标（角色 1/3/4/7，`:13-15`）在触发区连续停留 ≥ `zone_dwell_sec`（`:228-256`）；
  3. `endgame_unused_opportunity`：剩余时间 ≤ `endgame_trigger_sec` 且仍有可用次数。
- 前置闸门：比赛 running（0x0001 `game_progress==4`）、裁判数据新鲜（≤2.5 s）、`available>0`、无 pending（`:316-403`）。
- 信任过滤：只有 radio fresh 或 vision `measured` 的目标参与 ROI 停留（`:165-226`），避免幽灵位误触发。
- ACK 机制：申请后以 `0x020E` 的 `double_vulnerability_count` 下降视为成功（`:306-314`）；active 期间不重复消耗，endgame 场景可排队（`:379-403`）。
- 输出：`schema='shark.radar.command.v1'`、`action='trigger_double_vulnerability'`、带 `request_id`（`:409-416`），经 `/rm_radar_algorithm/radar_cmd` 进入串口节点。

### 5.4 自动密码与干扰等级联动

- `auto_password_node`：监听串口节点 status 的 `last_can_change_password`，上升沿时生成 6 位密码（字母表剔除 `0O1Il`，`nodes/auto_password_node.py:15-29`），发 `{"password_cmd":1,"password":...}`，最小间隔 5 s、同密码不重发（`:59-102`）。
- 串口节点另含：0x020E `own_encryption_level` 需连续确认 `referee_level_confirm_count=2` 次才生效（`:1148-1215`）；单 RX 时 level≥3 自动切 broadcast 解调（`:1217-1312`，`single_rx_auto_broadcast_after_level3`）；干扰等级可通过 `interference_control_topic`/`interference_rx_control_topic` 下发给发射/接收流图。

### 5.5 可观测性与故障定位

- 串口节点 `~/status` 是最大的 JSON（`nodes/referee_serial_node.py:1429-1593`）：`last_game_status`、`last_game_robot_hp`、`last_radar_decision_sync`、`last_robot_status`、`invincible_targets`（含每批接收者与掩码）、干扰等级协商状态、`tx_count`/`rx_count`、`blocked_0305_rate_count`、`radar_client_max_rate_hz`（回读时被 clamp 到 ≥0.001，`:1493-1497`）。
- RX 节点 `~/status` 分块（`nodes/flowgraph_node.py:980-1054`）：`decoder.stats`（字节/帧计数）、`frame_validation.primary/iq`（两套 assembler 的 crc8/crc16/长度失败计数，`rm_protocol.py:254-263`）、`reception_pipeline`（`core/reception_metrics.py:61-120` 用 CRC8 保护的 seq 缺口估算丢包率）、`iq_decode_diagnostics`（候选频偏/低通/SPS 命中情况）、`rx_gain`（回读的实际 hardwaregain）。
- 面板再聚合一层健康度：`tx_0305` 低于配置的一半 → warn；请求多但无成功串口写入 → bad；视觉在线但坐标全无效 → bad（`app_support/match_dashboard_node.py:1106-1150`）。这三条正好覆盖"融合没产出 → 串口没写 → 空口没到"的典型断点。

---

## 6. 配置与部署

### 6.1 配置分层（准确链路）

`radio/apps/match_rx/start.sh` 是无线电侧入口；仓库根目录的 `start_match.sh` 会同时监督无线电启动器和视觉启动台。`apps/match_rx` 的入口文件包括 `config.yaml`、`env.sh`、`start.sh`、`doctor.sh`、`optional_algorithm.sh`。无线电启动链路：

```
config.yaml ──┐
env.sh ───────┴→ start.sh ─┬→ ros2 launch rm_radio_ros rx2.launch.py  (RX1+RX2)
（env 覆盖 yaml）           ├→ ros2 run rm_radio_ros rm_match_recorder
                           ├→ ros2 launch rm_radio_ros referee_serial.launch.py
                           ├→ ros2 run rm_radio_ros rm_auto_password_node
                           ├→ ros2 run rm_radio_ros rm_radar_integration
                           ├→ ros2 run rm_radio_ros rm_match_dashboard
                           └→ optional_algorithm.sh
```

优先级（README.md:296-302）：`config.yaml` > `env.sh(RM_RADIO_*)` > 面板保存写回（`POST /api/config` 直接改 `config.yaml`，`match_dashboard_node.py:1752-1783`）。不支持隐式 `local.env`，需要时用 `MATCH_RX_ENV_FILE`。
`start.sh` 自带互斥（`flock /tmp/rm_radio_match_rx.lock`，PID `/tmp/rm_radio_match_rx.pid`，`:47-84`）、残留进程清理（`:98-114`、`env.sh:173-232`）、`trap stop_all`（`:305-307`）和守护循环（`:516-540`：裁判节点退出仅告警、其余节点退出则整体退出）。

### 6.2 config.yaml 全键（`radio/apps/match_rx/config.yaml`，155 行）

| 段 | 键（值） |
|---|---|
| `app` | host 127.0.0.1；port 8765；log_root；dry_run false；auto_open_browser true |
| `radio` | side red；interference_level 1；intelligent_mode true；rx1_uri 192.168.9.110；rx2_uri 192.168.9.111；sample_rate 2000000；rx_rf_port B_BALANCED；broadcast_gain_mode manual；broadcast_gain **32**；broadcast_gain_max 73；broadcast_low_pass_hz 260000；interference_gain_mode fast_attack；interference_gain 20；start_after_match_running false |
| `decoder.broadcast` | demod_mode mm；frontend_profile broadcast；input_sps 94；iq_snapshot_enabled false；iio_capture_enabled false；iio_capture_only false；capture_seconds 0.6；capture_period_sec 0.6；capture_start_delay_sec 1.0；iq_autotune_mode **off**；iq_low_pass_hz 260000；iq_sps_values "92,92.5,93,93.5,94,94.5,95"；iq_offset_step 8；iq_max_payloads 256；iq_max_frames 64 |
| `decoder.interference` | demod_mode legacy；frontend_profile interference；input_sps 94；iq_snapshot_enabled false；iq_autotune_mode quick；iq_low_pass_hz 0 |
| `spectrum` | period_sec 0.04；fft_size 1024；bin_count 256；native_gui false；native_spectrum_only true；qt_platform offscreen |
| `referee` | port `/dev/shark-referee`；baudrate 115200；bridge_topic 两路；radar_cmd_topic；dry_run false；require_serial_open_on_start false；frame_timeout_sec 2.0；auto_send_invincible_targets true；invincible_targets_data_cmd_id 564(0x0234)；invincible_targets_send_rate_hz 3.0；invincible_targets_freshness_sec 1.0 |
| `recording` | enabled true；auto_start true；root；control_topic；game_status_topic；start/stop_confirm_frames 2/2；stop_game_progress 5；post_roll_sec 10.0；max_duration_sec 900.0；finalize_timeout_sec 5.0；`iq.*`（root /media/shark/Resources/SHARK-radio-iq、mount_point、fallback_root、min_free_gb 15.0、output_sample_rate 1e6、segment_sec 30.0、queue_chunks 64、compression zlib、level 1）；`events.*`（min_free_gb 0.1、zstd、level 1、queue 4096、16 个 topic） |
| `radar_integration` | enabled；auto_sync_side；send_rate_hz 4.8；state_rate_hz 20.0；vision 0.5 / radio 1.0 / ghost 2.0；telemetry_topic；`auto_double_vulnerability.*`（enabled、mask_path、dwell 3.0、hp_drop 100、window 3.0、endgame 30.0、referee_timeout 2.5） |
| `algorithm` | enabled false；allow_failure true；dir ""；python "" |

注意 `radio.broadcast_gain` 在 yaml 里是 **32**、在 `env.sh:35-41` 是 **20**、在 `radio_config.py:20` 默认 **20** —— 三处不一致，`start.sh` 通过 env 生效，**这是排查"实际增益与配置不符"的第一嫌疑点**。

### 6.3 env.sh 关键环境变量（`radio/apps/match_rx/env.sh`，249 行）

`RM_RADIO_SIDE`、`RM_RADIO_INTERFERENCE_LEVEL`、`RM_RADIO_INTELLIGENT_MODE`、`RX1_URI`/`RX2_URI`（192.168.9.110/111）、`RM_RADIO_SAMPLE_RATE` 2000000、`RX_RF_PORT` B_BALANCED（`:28-34`）；`BROADCAST_RX_GAIN_MODE/GAIN/GAIN_MAX/LOW_PASS_HZ`、`INTERFERENCE_RX_GAIN_MODE/GAIN`（`:35-41`）；解码器组 `DEMOD_MODE/FRONTEND_PROFILE/INPUT_SPS/IQ_DECODER_ENABLED/IIO_CAPTURE_*/IQ_DECODER_AUTOTUNE_MODE/LOW_PASS_HZ/SPS_VALUES/OFFSET_STEP/MAX_PAYLOADS/MAX_FRAMES`（`:43-63`）；频谱 `SPECTRUM_PERIOD_SEC 0.04/FFT 1024/BIN 256`（`:64-69`）；录制全套（IQ root `/media/shark/Resources/SHARK-radio-iq`、min_free_gb 15、output 1e6、segment 30、queue 64、zlib，事件 zstd/level1/queue 4096）（`:89-135`）；`radar_integration` 与双倍易伤 6 个阈值（`:137-151`）；`RUN_ALGORITHM`、`ALLOW_ALGORITHM_FAILURE`、`RM_ALGO_DIR`、`RM_ALGO_PYTHON`（`:153-156`）；ROS 环境（`ROS_HOME=/tmp/ros_home`、source `/opt/ros/humble/setup.bash` + `install/setup.bash`、`PYTHONPATH`）（`:158-170`）。
`doctor.sh`（83 行）检查：config 是否被 git 跟踪/与提交一致（`:33-47`）、`ros2`/`python3`（`:48-49`）、录制栈与整数抽取比（`:50-66`）、GNU Radio 运行时版本（`:67`）、`iio_info` 探测两台 SDR（`:68-69`）、裁判串口设备（`:70-74`）、`rclpy/serial/numpy/scipy`（`:75-83`）。

### 6.4 实验台与硬件初始化

- `radio/apps/common/iio_init.sh`（14.5 KB）是启动前的 AD9361 预初始化与**硬件回读校验**脚本：`init_ad9361_rx <uri> <freq> <bw> <gain_mode> <gain> <rf_port> <rate>` / `init_ad9361_tx ...`，由 `apps/match_rx/start.sh:360-386` 调用（RX1 失败只 WARN，RX2 可被 `RM_RADIO_RX_SKIP_PREINIT=true` 跳过）；回读不匹配是"配置写了但硬件没吃"的唯一可靠发现手段。
- `radio/apps/lab_tx/`（实验台，不属于比赛链路）：`radio_tx_dashboard.py`（128 KB Web 面板）、`start.sh`/`start_dual_tx.sh`/`start_interference_tx.sh`/`start_iq_replay_tx.sh`/`start_dashboard.sh`、`env.sh`（11.6 KB）、`print_tx_plan.sh`、`reset_e310_rx.sh`、`run_dual_tx_raw_rx1_capture.sh`、`run_generated_link_test.sh`、`common/lab_tx_safety.sh`（发射安全联锁）。比赛日可完全不启动，但它是 2027 改完协议后**最先要跑的验证环境**：`run_generated_link_test.sh` 生成合法帧 → 双 TX 发射 → RX1 抓 IQ → `rm_offline_iq_scan` 解码，全链路不依赖裁判机。
- `apps/common/check_gnuradio_runtime.py` 校验 GNU Radio/gr-iio 版本与 ABI，`read_yaml.py` 给 shell 读 yaml，`browser_tabs.sh` 打开面板标签页。

---

## 7. 测试与质量

- 规模：`radio/src/rm_radio_ros/test/` 19 个 pytest 文件、**274 个 test 函数**。分布：`test_referee_serial_node.py` 46、`test_launch_and_script_safety.py` 36、`test_rm_protocol.py` 34、`test_dashboard_node.py` 32、`test_flowgraph_node.py` 26、`test_lab_tx_dashboard.py` 16、`test_offline_iq_scan.py` 13、`test_referee_bridge.py` 12、`test_jiang_protocol.py` 11、`test_double_vulnerability.py` 8、`test_radar_integration.py` 8、`test_rx_autotune.py` 7、`test_invincible_targets.py` 6、`test_continuous_gfsk_demod.py` 4、`test_match_recording.py` 4、`test_reception_metrics.py` 4、`test_virtual_link.py` 3、`test_auto_password_node.py` 2、`test_match_recorder_node.py` 2。
- 运行：`python3 -m pytest src/rm_radio_ros/test -v`（README.md:597-604）；另有 `test/ros_decode_smoke.py` + `run_ros_decode_smoke.sh` 做真实 ROS 解码冒烟（需要 rclpy 环境）。
- 运行注意：大部分用例只依赖 `numpy/scipy`（`setup.py:81`），可直接对源码目录跑；涉及 `rclpy` 的用例需先 `source /opt/ros/humble/setup.bash` 与 `install/setup.bash`。`setup.py:6-21` 特意处理了 colcon develop 的坑——Humble 自带 setuptools 被用户 site-packages 覆盖会丢失 `develop` 命令，所以 `COLCON=1` 且带 `--editable/--uninstall` 时以 `PYTHONNOUSERSITE=1` 重新执行；**在 2027 新机器上复现环境时，这一条比代码本身更容易踩**。
- 覆盖特征：协议层（长度/CRC/位域）、串口节点守卫（限速、密码冷却、0x0234 轮询）、launch 与 shell 脚本安全性（mock `iio_attr` 回读）、面板 API、离线 IQ 扫描算法均有对应用例；**没有** SDR 在环测试，也没有针对真实空口抓包的回归集（离线 IQ 只能手工喂）。
- 质量现状：`radio/` 下 `TODO/FIXME/XXX/HACK` **0 命中**（扫描 120 文件，13 个跳过：1 排除 + 12 非 UTF-8）。已知限制只能从实现与 README 推断，主要几条：
  1. 采样率 2 MS/s、SPS 47/94、15 字节空口包、access code 都是编译期常量，无配置热切换；
  2. 单 RX 降级要靠 `single_rx_after_level3` 与 `auto_rx_interference_level` 的启发式切换（`referee_serial_node.py:1217-1312`），无真正的双通道冗余；
  3. libiio 在以太网 SDR 掉线时会 abort 进程，只能靠 launch `respawn=True` 重建 IIO context（`launch/rx2.launch.py:166-169`）；
  4. 0x0305 三重限速与 5 Hz 协议上限绑定，算法线程与节点 timer 双通道容易互相抢占；
  5. `parse_robot_status` 等在短 data 下会抛 `struct.error`，依赖上游长度校验兜底；
  6. `dry_run` 默认 True、`broadcast_gain` 三处不一致（见 6.2），属于部署陷阱而非代码缺陷。

---

## 8. 2027 适配关注点

### 8.1 已是隔离层（改一处即可）。

| 变更类型 | 隔离层 | 说明 |
|---|---|---|
| 频率/带宽/低通/增益/灵敏度 | `core/radio_config.py:4-140` | 唯一"真源"（但有 3 份副本，见 8.2） |
| 裁判帧 CRC/组帧/解析 | `core/rm_protocol.py`（整文件） | 纯函数 + 数据类，无 ROS 依赖，可脱离节点单测 |
| cmd_id→语义 type | `core/referee_bridge.py:92-181` | 下游只吃 type 字符串 |
| 双源融合策略 | `core/radar_integration.py:74-311` | 超时、来源、发送门集中 |
| 双倍易伤判据 | `core/double_vulnerability.py:30-447` | 全部阈值集中在 `DoubleVulnerabilityConfig` |
| 无敌目标位图 | `core/invincible_targets.py:9-126` | data_cmd_id、接收者、状态集集中 |
| 解调数学 | `flowgraphs/gfsk/continuous_gfsk_demod.py:31-179` | 参数可热更新（`set_input_sps/set_input_sensitivity/set_bt`） |
| 流图↔ROS 桥 | `nodes/flowgraph_node.py:445-494` | 参数→env 单向，新增参数改这里 |

### 8.2 硬编码 / 需要联动修改的点

1. **频率表重复 4 份**（改一处必漏）：`core/radio_config.py:5,51,56,66-67,104-105`；`flowgraphs/gfsk/RX.py:30,41,46`；`flowgraphs/ganraoyuan/RM.py:98,103,438`；`apps/match_rx/start.sh:326,335,342`；实验台还有第 5 份 `apps/lab_tx/{env.sh:131,134,print_tx_plan.sh:17,reset_e310_rx.sh:15,run_dual_tx_raw_rx1_capture.sh:170,run_generated_link_test.sh:61,start_dual_tx.sh:31,start_iq_replay_tx.sh:48}`，加上 `flowgraphs/ganraoyuan/EGO.grc:372-429` 与 `config/tx_setters_interference_example.json:3` 的示例值。
2. **空中层硬约束**：`rm_protocol.py:48`（长度域）、`:383-385`（8/4/15 字节）、`:289`（`push_air_payload` 必须 15 B）、`:100-103`（access code）。2027 若换 access code 或包长，`flowgraph_node.py:1079-1094` 的 `len%15` 校验、`:269-283` 的 profile→access code 选择、`core/offline_iq_scan.py:22`（`ACCESS_BITS=8*8`）、`flowgraphs/ganraoyuan/jiang.py:41-43`（`_AIR_PACKET_BYTES=8+4+6`）要一起改。
3. **采样率与 SPS**：`flowgraphs/gfsk/RX.py:251`（2 MS/s 硬编码，**不读 env**）、`continuous_gfsk_demod.py:125-135`（`output_sps=input_sps/decimation`）、`core/rx_autotune.py:21-27`（nominal 由 `1e6/47` 推导）、`flowgraphs/ganraoyuan/RM.py:41`（TX 用官方 1 MS/s、SPS 47）。若 2027 空中速率变化，这 4 处 + 测试断言都要动。
4. **协议长度与字段偏移**：`rm_protocol.py:9-29`、`:733-844`、`:568-583`；`referee_serial_node.py:79-80`（0x0305 必须 48 B）；`radar_integration.py:59-71`（`'<24H'` 12 台）；`core/referee_bridge.py:10-23`（12 台顺序）；`apps/match_rx/start.sh:483-501` 若新增参数需同步。
5. **规则常量（非协议但随规则书变）**：`double_vulnerability.py:13-15`（场地 2800×1500、可申请角色）、`:30-37`（阈值）；`radar_wireless_constraints.py:50-96`（血量/弹量/buff 上限）；`invincible_targets.py:11-20`（状态集与 5 台映射）；`rm_protocol.py:82-97`（状态/哨兵模式枚举）、`:106-113`（比赛阶段）。
6. **ID 表**：`rm_protocol.py:116-119`（robot_id 红 1-11 / 蓝 101-111）、`:641-651`（红方雷达 sender_id=9、蓝方 109）、`referee_serial_node.py:243-273`（`receiver_id=0x8080` 默认）、`invincible_targets.py:22-37`。
7. **测试耦合**：`test_rm_protocol.py:232-257`、`test_virtual_link.py:35-36`、`test_flowgraph_node.py:197-202/355/393/442/582-686`、`test_referee_serial_node.py:991`、`test_launch_and_script_safety.py:352-541`、`test_match_recording.py:34`、`test_dashboard_node.py:206`、`test_lab_tx_dashboard.py:47` 都写死了频点/增益，改频点必须同批改测试，否则 CI 直接红。
8. **部署默认值陷阱**：`config/referee_serial.yaml`（`dry_run: true`）、`config/gfsk.yaml`（`iq_decoder_enabled: true`，生产 `rx2.launch.py` 覆盖为 RX1 false/RX2 true）、`config.yaml radio.broadcast_gain` 与 `env.sh` 不一致。2027 建议收敛为单一 yaml + 生成 env，从根上消除这份"四份频点、三份增益"的重复。

### 8.3 建议的适配顺序（先结构、后协议）

1. **收敛射频常量**：把 `core/radio_config.py` 定为唯一真源，让 `flowgraphs/gfsk/RX.py`、`flowgraphs/ganraoyuan/RM.py`、`apps/match_rx/start.sh` 从它派生（可在 start.sh 里调 `python -c "from rm_radio_ros.core.radio_config import ..."` 打印 JSON），删掉实验台第 5 份硬编码；顺便把 `radio.broadcast_gain` 的三处不一致统一。
2. **把采样率与 SPS 变成参数**：`flowgraphs/gfsk/RX.py:251` 构造期硬编码 2 MS/s、只能在启动后经 `setters_json` 调 `set_sample_rate` 补救（`flowgraph_node.py:513-579`）。改成构造期直接读 `RM_RADIO_SAMPLE_RATE`（env 与节点参数都已存在），再验证 `continuous_gfsk_demod` 的 `output_sps=input_sps/decimation` 在非 2 倍过采样下仍成立。
3. **扩展空中层**：把 15 字节 / 8+4 / access code 抽成 `AirLayerProfile` 数据类，`flowgraph_node.py:1079-1094`（`len%15`）、`rm_protocol.py:289`（`push_air_payload`）、`offline_iq_scan.py:22`、`jiang.py:42-43` 全部改为读 profile；2027 换包长时只改一处。
4. **协议字段落地**：以 `RADAR_DATA_LENGTHS` + `parse_radar_frame` 为边界逐条替换，先跑 `test_rm_protocol.py` 与新抓包对比，再改 `referee_bridge` 的 type 映射与 `double_vulnerability` 的 `strategy['RadarEnemyBuffStatus']` 依赖。
5. **规则侧**：双倍易伤阈值、无敌目标状态集、0x0305 十二台顺序分别集中在 `double_vulnerability.py:30-37`、`invincible_targets.py:11-22`、`radar_integration.py:59-71`，按 2027 规则书数值替换即可，不需要动数据流。
6. **回归**：每步都跑 `pytest src/rm_radio_ros/test`，并用 `apps/lab_tx/run_generated_link_test.sh` + `rm_offline_iq_scan` 做一次"生成帧→发射→抓 IQ→解码"的空口闭环；比赛前再用 `doctor.sh` 与面板 `tx_0305` 健康度做现场确认。

---

### 附：一句话结论

radio/ 的协议与规则逻辑隔离得较干净（rm_protocol / radar_integration / double_vulnerability / invincible_targets 都是无 ROS 依赖的纯 Python），2027 适配的主要成本不在算法，而在**四处重复的射频常量、空中包 15 字节硬约束、以及与之绑定的 274 个测试断言**；建议先做"射频常量单一真源 + 由 yaml 生成 env 与 launch 参数"的收敛改造，再动协议字段。
