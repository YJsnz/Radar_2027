# RX1 完整采用 RM2027 接收链路

日期：2026-11-16。实现版本：`rm2027-receiver-20261116`。
分支：`codex/rx1-rm2027-identical`，从 `cd60d2f` 创建。
范围为信息波 RX1；RX2 干扰波保持原有 legacy 接收实现。

## 相同部分与保留的外围

按用户要求，RX1 不再只模仿参考程序的参数，而是直接运行其源码：

- `receiver/phy.py`：完整 `GfskDemodulator` 和 `PhyDeframer`。
- `receiver/profiles.py`：原始台架频率、SPS、BT、接入码和长度域。
- `receiver/protocol.py`：原始 CRC、异常长度恢复、跨分片组帧与业务帧解析入口。
- `receiver/shark_core/wire2027.py` 及其 `rm_protocol.py` 依赖：参考业务解析。

文件存放于 `radio/src/rm_radio_ros/rm_radio_ros/vendor/rm2027/`；只把原来的顶层导入改为包内相对导入。每个原文件和引入文件的 SHA-256、改动范围记录在 `SOURCE.json`，MIT 归属保存在 `LICENSE.SHARK`。本次实际与用户目录逐文件比较，逆转这几处导入修改后逐字节一致；运行时不再依赖用户目录。

`flowgraphs/gfsk/rm2027_rx.py` 对齐参考 `Receiver.connect_device()`：
IIO 源通道 `[True, True, False, False]`、buffer 32768、2 MS/s，按同一顺序设置频率、Auto 滤波、manual 增益及三个 tracking 开关。RX1 跳过原项目的 IIO 预初始化，默认增益改为 10 dB。
频谱和瀑布图使用参考程序的 Qt sink 参数、连接到同一个原始 IQ 源。切红蓝前停止调度并等待结束，新建参考解调器和组帧器；切换失败恢复旧波源。只改增益时不重建解码器，只改 sensitivity 时调用参考 setter。
设备 URI 使用与参考 GUI 相同的 QLockFile 命名，避免两者同时打开同一 URI；同一设备的不同 URI 别名仍需操作者避免同时使用。

项目保留以下外围，因此不能把整个 ROS／Dashboard 应用称为与参考 GUI 逐字节相同：

1. GNU Radio 流线程内不调用项目的比特解码器、IQ Python 回调或 ROS 发布。ROS 定时器按参考 GUI 的 **250 ms** 周期调用 `take_frames()`，将已验证帧转为原有 `frames`／`referee_bridge` 输出，供融合、裁判与事件录制消费。
2. ROS `frames` 增加 `rm2027_decoded`，直接保存参考程序的业务解析字典；原有 `parsed` 和 bridge 结构由适配层保留，保证订阅者兼容。适配层重建原始帧字节供日志使用，不参与空口分片或 CRC 接收决策。
3. 为保留网页频谱，同一 IQ 源额外连接一个 **C++ latest-only probe**，由 ROS 每 250 ms 读取快照。它不在取流回调中执行 Python，不是第二个 Pluto 读者，不提供连续 IQ 记录。
4. 状态新增 `reference_decoder`，显示与参考程序相同的累计分片／头部拒绝／CRC8／CRC16／有效帧计数。原项目主组帧器和 10 秒候选流水线已旁路，其 `frame_validation.primary`／`reception_pipeline` 显式置 `null`。Dashboard 使用参考计数，不把空的旧计数当作实时结果，不显示没有相应观测依据的总丢包估计。

事件录制仍保留完整 JSON 帧／状态，包含新增参考解析和计数。旧事件回放仍走原有 JSON schema，Dashboard 同时兼容旧的 10 秒诊断与新参考计数。

## 启动与限制

- `apps/match_rx/config.yaml` 的 `decoder.broadcast.demod_mode` 设为 `rm2027`，信息波增益 10 dB。
- 比赛启动、通用 `gfsk.yaml`、双 RX launch、Dashboard 的 RX 重启均选择参考链路；通用显式 `mm`／`legacy` 配置仍可用于历史诊断。RX2 的 launch 仍显式选择 `legacy`。
- 参考模式只支持 Pluto 的明确 `ip:`／`usb:` URI。RX1 若要换回 RTL，需要显式选择旧接收模式；本分支不把 RTL 当作等价参考后端。
- 参考模式拒绝同时启用原 IIO 采集解码器或 IQ 快照解码器，避免第二个硬件读者或混入另一条解析链。
- **RX1 连续 IQ 比赛录制暂不支持**：参考路径没有原项目的连续滤波后录制回调，状态明确报告 `enabled=false`。没有把低频 probe 快照冒充连续录制，也没有降低磁盘空闲保护。RX2 的原录制路径未变。比赛事件录制不受此限制。
- 未采集本次实机故障时段 IQ；本轮重放的是已有干净实录。台架 PHY 仍非已确认的 2027 官方 RF 参数。

## 验证

1. `radio/src/rm_radio_ros/test` 与 `vision/tests` 全量 **486 项通过**。新增测试覆盖正常／反相比特、1／22／256／20000 比特分块、异常长度恢复、CRC 拒绝、参考解析到 ROS 帧的转换；mock 硬件核对源初始化参数与写入顺序、切换恢复和 URI lease 释放；ROS 层确认不挂载旧比特／IQ 回调。
2. `colcon build --packages-select rm_radio_ros --symlink-install` 通过；Python 编译、JS 语法、shell 语法及 `git diff --check` 通过。
3. 生产 `RM2027InformationRx` 完整流图，仅把硬件源替换为原始 IQ 文件：输入 4000000 个 complex64 采样，得到 **186 分片、92 有效帧、CRC8／CRC16 错误均为 0**。92 帧的完整字节哈希及顺序与原 RM2027 一致；ROS 外壳和 `rm2027_decoded` 均存在。见[完整流图记录](pluto_rx1_bench_20261116/rm2027_branch_offline.json)。
4. 进一步运行真实 `FlowgraphNode`、ROS 执行器和新参考流图，仍只替换硬件输入：250 ms 轮询实际发布 **92 帧和 92 个桥接结果**；旧比特回调批次、IQ 快照和 IIO 第二解码计数均为 0，新状态正确停用旧丢包卡片和 RX1 连续 IQ 录制。见[ROS 接入记录](pluto_rx1_bench_20261116/rm2027_branch_ros_offline.json)。

本轮 USB 列表未发现 Pluto，主机也没有 Pluto USB 网络接口，IIO 连接探测未成功。因此没有运行新代码的硬件取流、超过一分钟的实机长稳、双 Pluto 或真实裁判串口验收。这些 TODO 继续未勾选，离线通过不能替代实机质量结论。
