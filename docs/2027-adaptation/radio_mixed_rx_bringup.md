# RTL-SDR＋PlutoSDR 混合接收台架记录

日期：2026-11-13。用途：用户现有的一台 RTL-SDR 接收信息波、一台 USB PlutoSDR 接收干扰波，后续再替换成双 Pluto。这里只记录台架实收与软件适配，不代表 2027 官方 RF 或真实裁判链路验收。

## 1. 当前分工与设备

| 角色 | 设备与身份 | 入口 | 接收设置 |
|---|---|---|---|
| RX1 信息波 | RTL-SDR Blog V4 / R828D；首轮序列号 `00000001`，USB `0bda:2838` | `rtlsdr:00000001`，SoapyRTLSDR | 红方台架频点 433.2 MHz；manual 32.8 dB，硬件范围 0～49.6 dB。 |
| RX2 干扰波 | PlutoSDR Rev.B / AD9363A；序列号 `03df6d86b5871b332dd7658ebd9f032b35`；固件 `v0.39-dirty` | `ip:192.168.2.1`，IIO | 红方 1 级台架频点 432.2 MHz，A_BALANCED、fast_attack。 |

Pluto 通过 USB 连接，本机出现 USB 网络接口、地址 `192.168.2.10/24`，以上 `ip:` 仍走这条 USB 链路。本机直接 USB IIO 扫描未发现可用 context，设备节点权限为 root:root；没有为此修改系统权限或固件，使用已实测可达的 USB 网络接口。

频率、接入码与调制继续使用项目原有台架参数，不将其标记成已确认的 2027 官方物理层。用户独立测试设备发射过红方信息波，后切换干扰波，提供校验密钥 `fvXVKC`；发射频点最终确认是 432.2 MHz，先前的 433.2 MHz 回复由用户更正为笔误；尚未取得发射程序和完整参数记录。

## 2. 接收源适配

`flowgraphs/gfsk/RX.py` 按 URI 选择硬件：

- `rtlsdr:<序列号>` 创建 GNU Radio Soapy RTL-SDR 源，明确指定物理序列号，不自动抢第一只 USB 设备。
- 原 `ip:`／`usb:` IIO 路径继续使用 fmcomms2 源。
- 当前 SoapyRTLSDR 仅公布 2.048 MS/s 等离散档位，拒绝 2 MS/s；因此 RTL 原始 2.048 MS/s 经过 **125/128** 有理重采样后，统一为原链路的 2 MS/s。信息波 M&M、干扰波 legacy、SPS=94、IQ 回调和录像均沿用这个 DSP 采样域。RTL 入口拒绝只改采样率而不改重采样比例。
- RTL 频率、带宽和增益调用 Soapy 接口，跳过 AD9361 端口、FIR、DC／正交跟踪等控制；其自动增益仅映射为 Soapy 的 automatic，不宣称与 AD9361 fast_attack 算法相同。诊断输出后端、URI、硬件／DSP 采样率及实际读回的 RTL 增益。
- 启动脚本对 RTL 跳过 IIO 预初始化；Dashboard 根据同 URI 的新鲜接收节点状态检查 RTL，避免第二个进程打开正在取流的 USB 设备。

ROS 状态增加 `rx_uri`，原始事件录制会随状态一并保存；其余命令、位置、视觉元数据和融合格式不变。没有新增射频发射路径。

## 3. 本机运行依赖

GNU Radio 实际加载 3.10.7.0，gr-iio 与 runtime ABI 一致；本机 libiio 0.23，Pluto 服务端 libiio 0.26。SoapySDR 0.8.1 与 SoapyRTLSDR 0.3.2 已安装。

系统 librtlsdr 0.6.0-4 不能正确驱动 Blog V4。参考 [厂家 V4 说明](https://www.rtl-sdr.com/v4/)，在用户目录编译厂家驱动 [rtl-sdr-blog](https://github.com/rtlsdrblog/rtl-sdr-blog)，固定本次源码提交 `aed0ea19f3a273370a13c9009b96313c75d54c7b`。安装前缀为 `~/.local/share/shark-radio/rtl-sdr-blog`，使用 `DETACH_KERNEL_DRIVER=ON`，没有卸载系统包或改写 EEPROM。

`radio/apps/match_rx/config.yaml` 明示 `radio.rtl_driver_library_dir`，`env.sh` 仅在配置含 RTL 接收器时将该目录加入当前应用的 `LD_LIBRARY_PATH`；路径缺失则报错。部署其他电脑需先安装对应驱动，不能只复制此 YAML。

缺少的 Ubuntu `libiio-utils=0.23-2` 已下载、解包在 `~/.local/share/shark-radio/libiio-utils-0.23/`，`~/.local/bin/` 链接了 `iio_info`、`iio_attr`、`iio_readdev`。这两项用户目录依赖不在 Git 内。

## 4. 实际结果与未完成项

1. **信息波首轮通过**：直接运行修改后的真实接收图，约 **8.15 秒收到 373 帧**，0x0A01～0x0A05 依次为 74／75／75／75／74 帧；746 个空口载荷、约 1600 万个 DSP IQ 样本。对收到的帧，CRC8／CRC16／命令长度检查均无失败。发射端总帧数未记录，因此没有宣称零丢包。原始计数、四个解析样例和来源见 [首轮实收 JSON](radio_mixed_rx_bench_20261113/rtl_information_first_pass.json)。
2. **双路 ROS 与网页曾正常取流**：同一台机器上两个真实接收节点均 `started=true`，无加载错误；Dashboard 可见两路频谱与真实硬件状态。调试使用 `ROS_DOMAIN_ID=199`，未启动裁判串口、自动密码或录制服务。接收不到当前位置时不注入模拟点。
3. **Pluto 干扰波首轮通过**：在用户完成接线／源调整并确认频点后，432.2 MHz、A_BALANCED、2 MS/s 实收约 **8.04 秒、79 帧有效 0x0A06**，密码逐字核对为 `fvXVKC`。CRC16／命令长度失败均为 0，但同步过程中有 **32 次 CRC8 失败**，仍需稳定性与原因检查。首轮原始计数和四个帧样例见 [Pluto 实收 JSON](radio_mixed_rx_bench_20261113/pluto_interference_first_pass.json)。此前无包的记录和临时 433.2 MHz 测试不作为通过依据，默认频率表没有因误填数值而修改。
4. **重连仍有限制**：Pluto 曾从 USB 列表消失；重插后 IIO 工具已可达，但当时 RX2 进程持续重试失败，手动重启该节点才恢复。完整热拔插自动恢复尚未验收。
5. **RTL 通信异常及恢复**：停止接收后出现 USB 控制传输超时 `-7`，一次设备复位未恢复；用户确认同一只重插后枚举成 `0bda:2832`、Generic RTL2832U、序列号 `77771111153705700`，仍超时。没有把这个异常身份写回原 `00000001` 绑定，没有修改 EEPROM。后续换接到 USB `1-9` 后，设备恢复为 `0bda:2838`、RTLSDRBlog / Blog V4、原序列号 `00000001`，原 RX1 节点自行启动并实收。这里只确认换接后的恢复现象，未确定此前异常的根因。
6. **Pluto 的 ROS／网页闭环通过首轮**：重新启动 RX2 后，网页持续显示真实 `fvXVKC`。保存快照时 251 帧有效 0x0A06 对应 251 个桥接结果；原始帧与 `RadarCommand0121/password_cmd=2` 桥接值一致，见 [ROS／Dashboard 快照](radio_mixed_rx_bench_20261113/pluto_ros_dashboard_snapshot.json)。未运行裁判串口，所以这只验证接收与软件桥接，没有实际提交密钥。RTL 异常期间 RX1 暂停。
7. **RTL 恢复后的 60 秒观察**：2026-11-13 19:34～19:35，在用户开启红方信息波后，只读取正在运行的 RX1 ROS 状态和 Dashboard API，没有额外打开 RTL。433.2 MHz、manual 32.8 dB 下，约 **60.017 秒新增 2800 帧有效信息波、2800 个桥接结果**，约 46.7 帧／秒；观察区间内 CRC8／CRC16／长度检查失败增量均为 0。快照保留 0x0A01～0x0A05 的实际解析及五类桥接样例。节点早期累计已有 1 次 CRC8、11 次 CRC16 失败，不能把一分钟的结果解释成全程无错误或零丢包。详见 [恢复观察 JSON](radio_mixed_rx_bench_20261113/rtl_recovery_observation.json)。裁判串口未打开、实际发送数为 0；测试源坐标不是真实机器人观测。该观察开始时 Pluto 不在 USB 列表中，结束快照 RX2 仍离线，不作为双路同时接收验收。
8. **本次 Pluto 重接后的节点恢复**：上述一分钟观察后，Pluto 再次出现在 USB 列表中，`iio_info` 可读出原设备序列号，但旧 RX2 节点仍报 `Unable to create context`。只终止该 RX2 进程，由原 ROS launch 拉起新进程后，RX2 恢复取流，两路均 `started=true`、无加载错误，并有新鲜 IQ 诊断；RX1 进程保持运行。恢复后的状态附在同一 JSON 的 `post_observation_rx2_recovery`。该快照时面板已被操作为蓝方 3 级，不将此前红方实收结果作为该设置的验收；没有验证双路持续同时解包，也没有实现自动恢复修复。

## 5. 复现入口

在依赖与设备恢复后，以仅接收的台架方式启动：

```bash
ROS_DOMAIN_ID=199 RUN_REFEREE=false RUN_AUTO_PASSWORD=false RUN_RECORDER=false \
  RM_RADIO_AUTO_OPEN_BROWSER=false bash radio/apps/match_rx/start.sh --no-browser
```

打开 `http://127.0.0.1:8765/` 的“无线电监控”。按上表提供相应测试信号，审阅每路 `decoder.stats`、CRC／长度计数、帧内容及桥接结果；未匹配信号源时有频谱不等于有有效帧。当前固定的是首轮确认的 RTL 身份，异常枚举未恢复前不要自动替换绑定。

RTL 暂停期间可独立启动面板和 RX2。先用 `RUN_RX=false RUN_REFEREE=false RUN_AUTO_PASSWORD=false RUN_RECORDER=false RUN_RADAR_INTEGRATION=false` 启动同一脚本；面板启动完成后，在另一个终端进入仓库根目录运行：

```bash
source radio/apps/match_rx/env.sh
export ROS_DOMAIN_ID=199
ros2 run rm_radio_ros rm_gfsk_node --ros-args -r __node:=rm_gfsk_interference_node \
  -p flowgraph_module:=RX -p flowgraph_class:=RadioRx -p flowgraph_name:=gfsk \
  -p rx_uri:=ip:192.168.2.1 -p rx_profile:=interference \
  -p frontend_profile:=interference -p demod_mode:=legacy -p input_sps:=94.0 \
  -p radio_side:=red -p interference_level:=1 -p sample_rate:=2000000.0 \
  -p iq_decoder_enabled:=false -p recording_enabled:=false -p disable_gui_sinks:=true
```

须显式指定模块／类；名为 `rm_gfsk_node` 的 YAML 配置节不会自动套用到重命名的 `rm_gfsk_interference_node`。当前调试中相同参数已通过参数服务设置并生效。重新启动整套脚本前先停止这个独立 RX2，避免重复占用设备。

未来换成第二台 Pluto 时，将 RX1 URI 指向实际设备、核对两台独立地址／序列号与 A_BALANCED 端口，取消不再需要的 RTL 库目录；再按真实硬件增益范围更新上限。不能让两路 URI 指向同一台设备，也不能保留首轮台架结果作为双 Pluto 验收记录。

## 6. 软件验证

`vision/tests` 与 `radio/src/rm_radio_ros/test` 全量 **469 项通过**；新增测试用 mock 硬件构建真实 GNU Radio 图，检查 125/128 实际重采样输出、两类硬件控制分离、拒绝错误采样域；网页测试检查 RTL 探测不额外抢占 USB、错 URI／过期状态不能冒充运行正常，以及 RTL 增益控件不提供负增益。`colcon build --packages-select rm_radio_ros --symlink-install`、Python 编译、启动脚本语法及 `git diff --check` 通过。浏览器实际打开监控页确认密钥显示，控制台无 warning/error。

当前通过的是信息波与干扰波各自首轮台架实收，以及 RTL 换接恢复后的一分钟接收观察。整体无线电实机、2027 RF、红蓝／等级切换、密钥向真实裁判提交与回读、长期稳定性和多轮重新接线后的复测继续保持 TODO。
