# 双 USB Pluto 接收配置

配置版本：`dual-pluto-v1`，2026-11-16。按用户要求，两路均改用 Pluto；目前先接第一台，第二台稍后接入。此前 RTL＋Pluto 的实收结果是历史记录，不能作为双 Pluto 验收依据。

## 分工与地址

| 角色 | 配置入口 | 接收内容 | 验证状态 |
|---|---|---|---|
| RX1 | `ip:192.168.2.1` | 信息波，参考 RM2027 接收程序更新滤波和同步 | 已识别本次接入的序列号 `03df62ca26572c053ad76ac22e4f341d22`，信息波实收记录见下文。 |
| RX2 | `ip:192.168.3.1` | 干扰波，沿用 legacy 连续解调 | 第二台的部署预设，尚未确认或写入实机。 |

`ip:` 可以通过 Pluto 的 USB 网络接口通信。两台必须具有不同的设备地址和对应主机接口地址，不能把两个 URI 指向同一台设备。如果两台均使用 `192.168.2.1`，先分开连接并配置第二台的网络，再一起接入核对序列号；仅修改项目 YAML 不会修改设备网络。

首次扫描未发现设备，重新接入后 `lsusb` 与 `iio_info` 均已识别一台 Pluto Rev.B，固件 `v0.39-dirty`。USB 网络接口为 `enx00e022a4acb6`，主机地址 `192.168.2.10/24`。本次序列号与 2026-11-13 的干扰波 Pluto 不同，不能沿用旧设备的身份记录。

## 项目设置

- 配置入口为 `radio/apps/match_rx/config.yaml`，RX1／RX2 分别绑定上述 URI。
- 两路 RX 口使用 `A_BALANCED`，采样率均为 2 MS/s；Pluto 直接使用 IIO 源，不经过 RTL 的 125/128 重采样。
- 清空部署配置的 `rtl_driver_library_dir`，双 Pluto 启动不再依赖本机 RTL 厂家库；RTL 后端代码继续保留。
- RX1 使用 manual 32 dB，软件增益上限为 73 dB。本机实际报告 `[-1 1 73]`，原 RTL 的 32.8 dB 写入后只能回读 32 dB，会使严格的预初始化验证失败，因此按硬件步进修正配置和面板值。RX2 保留 fast_attack。
- 频率、接入码和调制沿用原台架参数，不代表已确认的 2027 官方 RF。红方信息波台架频点为 433.2 MHz，红方 1 级干扰波为 432.2 MHz；面板阵营／等级必须与测试源一致。

## 仅连接第一台时

默认比赛启动会创建两路节点；第二台未连接时，RX2 应显示离线。不要为了消除离线提示把 RX2 改成 RX1 的地址。仅做 RX1 实收可在停止整套接收进程后，从仓库根目录单独运行：

```bash
source radio/apps/match_rx/env.sh
source radio/apps/common/iio_init.sh
init_ad9361_rx "$RX1_URI" 433200000 540000 manual "$BROADCAST_RX_GAIN" "$RX_RF_PORT" defer
ros2 run rm_radio_ros rm_gfsk_node --ros-args \
  -p flowgraph_name:=gfsk -p flowgraph_module:=RX -p flowgraph_class:=RadioRx \
  -p rx_uri:="$RX1_URI" -p rx_profile:=broadcast -p frontend_profile:=broadcast \
  -p radio_side:=red -p demod_mode:=mm -p input_sps:=94.0 \
  -p air_extractor_access_code_bits:=40 \
  -p air_extractor_max_access_hamming:=6 \
  -p air_extractor_max_length_hamming:=2 \
  -p sample_rate:=2000000.0 -p iq_decoder_enabled:=false \
  -p recording_enabled:=false -p disable_gui_sinks:=true
```

该示例固定为红方信息波，预初始化失败时应先解决设备连接。不要另开 `iio_readdev` 或其他接收程序抢占正在取流的设备。

## 待验收

- [x] 第一台枚举、IIO 访问、序列号与信息波首轮台架实收。
- [ ] 第二台独立地址／主机 USB 网络、序列号和干扰波实收。
- [ ] 两路同时接收、红蓝／等级切换、CRC 质量与长时稳定性。
- [ ] 热插拔恢复和真实裁判提交回读。

## 本轮软件验证

相关启动与脚本、Dashboard、RTL／IIO 后端回归共 **73 项通过**；`env.sh` 实际加载出的两路 URI、空 RTL 库目录、A_BALANCED 和 73 dB 软件上限已核对，启动脚本语法与 `git diff --check` 通过。没有修改解调代码，本轮未重复 ROS 构建或全量测试。

实际 Dashboard API 已显示 RX1／RX2 的双 Pluto 地址和 RX1 的 −1～73 dB 软件控制范围。首次插入后旧 RX1 进程仍报 `Unable to create context`；手动重启 RX1 后恢复实际取流，不能据此宣称自动热插拔恢复已完成。

## 首台 Pluto 信息波实收

用户确认独立设备正在发送红方信息波 433.2 MHz。RX1 重启后，IIO 实际回读为 433200000 Hz、2 MS/s、A_BALANCED、manual 32 dB；ROS 与 Dashboard 收到 0x0A01～0x0A05 以及对应五类桥接输出，没有注入模拟包。硬件访问及信息波接收已通过首轮台架验证，完整实机验收继续待完成。

- 32 dB 的 **30.012 秒**观察内，新增 **352 帧有效包、352 个桥接结果**，同时新增 128 次 CRC8、322 次 CRC16 失败，接收质量不稳定。原始计数、硬件回读和五类解析样例见 [32 dB 实收记录](pluto_rx1_bench_20261116/pluto_rx1_information_observation.json)。
- 在同一接收进程上临时比较手动增益：50 dB 下 **10.015 秒 462 帧**，新增 CRC8／CRC16 失败为 1／3；60 dB 下 **10.014 秒 466 帧**，两类失败均未增加。两段结束时 IQ 诊断未见削顶。见 [增益对比](pluto_rx1_bench_20261116/pluto_rx1_gain_comparison.json)。测试顺序与环境未受严格控制，不能把短时比较写成通用最优增益或长期稳定结论。
- 比较脚本结束时选择了临时 60 dB，随后面板值已变为 15 dB，未再次覆盖。部署初值保留可准确回读的 32 dB；现场需根据信号质量调整，不把本次 60 dB 写成通用默认值。
- 第二台尚未接入，RX2 离线符合当前接线；没有验证双路同时接收，也没有真实裁判串口发送／回读。

## 信息波接收链路参考适配（2026-11-16）

参考程序位于用户提供的 `RM2027/receiver/`。适配限于信息波 RX1：在现有带通后增加 260 kHz 低通、复用抽取前的 470 kHz 抗混叠滤波，带通窗改为 Blackman-Harris；接入码同步改用官方空口接入码末 40 位，最多 6 位接入码差错、2 位长度字段差错。CRC8、CRC16、命令长度表和业务解析仍由本项目处理且保持严格；RX2 干扰波保留原 64 位、3 位接入码差错、0 位长度差错策略。`radio/apps/match_rx` 的比赛启动和 Dashboard 重启 RX1 均应用这组信息波参数，通用 ROS 节点默认保持旧的严格同步设置。

参考源文件 SHA-256：`receiver/phy.py` 为 `c868fc75ff1562d34ada2b4ffc791507cf485d3aa89b6c6da8cd9fcab8fa174b`，`receiver/profiles.py` 为 `bda92715680211bc38bf29c20fcc4d59a1611d21eb643215dbd35ad0604f1fd5`。参考程序与其 MIT 许可归属见该目录；本项目没有运行时依赖它。

### 对照结果与限制

- 真实 Pluto RX1 在 433.2 MHz、2 MS/s、A_BALANCED、10 dB 下录制的 2 秒 IQ（4,000,000 个复数采样，SHA-256 `edd6797d79028ffa743778dd3bd49ac5a43e9614f9fb3255495f1a90699db687`）离线回放时，新链路与参考链路判决比特完全一致，均解出 92 个 CRC 有效帧，覆盖 0x0A01～0x0A05；旧链路也解出相同 92 帧。因此干净单信号实录只证明兼容性，不能证明解帧率提升。IQ 文件保存在本机 `/tmp/pluto_info_reference_comparison.c64`，没有提交到仓库。
- 对这段真实 IQ 做可控数字叠加：将目标信号衰减 20 dB 后，叠加位于 −400 kHz、相对目标约 +60 dB 的合成 GFSK 邻道。新链路与参考链路各解出 92 个相同的 CRC 有效帧，CRC8／CRC16 失败均为 0；旧链路解出 0 个有效帧，即使使用 40 位接入码和相同的宽松同步阈值也没有有效帧。该项说明新增滤波在此合成邻道条件下有效，不代表两路真实发射设备同时测试或射频灵敏度验收。
- 当前正在运行的比赛接收界面确认 RX1 已加载新设置，实机状态为 2 MS/s、manual 32 dB、节点 `started=true`。短时状态采样中，10 秒新增 113 个 CRC 有效帧；同时 CRC8 失败由 29 增至 146、CRC16 失败由 98 增至 354。说明实机可持续出帧，但该时段仍有较多校验失败；没有已知发射端总包数作为分母，不能计算丢包率。RX2 因第二台 Pluto 未接入而报告 `Unable to create context`，符合当前单机接线。
- 修改后验证：`radio/src/rm_radio_ros/test` 与 `vision/tests` 全量 474 项通过；`colcon build --packages-select rm_radio_ros --symlink-install` 通过。实际运行的 Dashboard 显示 RX1 同步参数为 40／6／2，ROS 有效帧与桥接输出计数一致。双机、红蓝两侧、频点／等级切换、长时稳定性和真实裁判串口仍待验收。
