# 双 USB Pluto 接收配置

配置版本：`dual-pluto-v1`，2026-11-16。按用户要求，两路均改用 Pluto；目前先接第一台，第二台稍后接入。此前 RTL＋Pluto 的实收结果是历史记录，不能作为双 Pluto 验收依据。

## 分工与地址

| 角色 | 配置入口 | 接收内容 | 验证状态 |
|---|---|---|---|
| RX1 | `ip:192.168.2.1` | 信息波，沿用 Gaussian＋M&M 连续解调 | 沿用此前 Pluto 的 USB 网络地址；本轮仍需识别设备、核对序列号并实收。 |
| RX2 | `ip:192.168.3.1` | 干扰波，沿用 legacy 连续解调 | 第二台的部署预设，尚未确认或写入实机。 |

`ip:` 可以通过 Pluto 的 USB 网络接口通信。两台必须具有不同的设备地址和对应主机接口地址，不能把两个 URI 指向同一台设备。如果两台均使用 `192.168.2.1`，先分开连接并配置第二台的网络，再一起接入核对序列号；仅修改项目 YAML 不会修改设备网络。

当前 `lsusb` 与 `iio_info -s` 尚未发现新接入的 Pluto，不据此声称设备可用。优先检查 USB 数据口、数据线和电脑端口；灯亮只证明上电，仍需 USB 枚举和 IIO 可访问。

## 项目设置

- 配置入口为 `radio/apps/match_rx/config.yaml`，RX1／RX2 分别绑定上述 URI。
- 两路 RX 口使用 `A_BALANCED`，采样率均为 2 MS/s；Pluto 直接使用 IIO 源，不经过 RTL 的 125/128 重采样。
- 清空部署配置的 `rtl_driver_library_dir`，双 Pluto 启动不再依赖本机 RTL 厂家库；RTL 后端代码继续保留。
- RX1 保留 manual 32.8 dB，软件增益上限恢复为既有 IIO 的 73 dB；实际设置与可用范围以硬件回读为准。RX2 保留 fast_attack。
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
  -p sample_rate:=2000000.0 -p iq_decoder_enabled:=false \
  -p recording_enabled:=false -p disable_gui_sinks:=true
```

该示例固定为红方信息波，预初始化失败时应先解决设备连接。不要另开 `iio_readdev` 或其他接收程序抢占正在取流的设备。

## 待验收

- [ ] 第一台枚举、IIO 访问、序列号与信息波实收。
- [ ] 第二台独立地址／主机 USB 网络、序列号和干扰波实收。
- [ ] 两路同时接收、红蓝／等级切换、CRC 质量与长时稳定性。
- [ ] 热插拔恢复和真实裁判提交回读。

## 本轮软件验证

相关启动与脚本、Dashboard、RTL／IIO 后端回归共 **73 项通过**；`env.sh` 实际加载出的两路 URI、空 RTL 库目录、A_BALANCED 和 73 dB 软件上限已核对，启动脚本语法与 `git diff --check` 通过。没有修改解调代码，本轮未重复 ROS 构建或全量测试，也未取得 Pluto 实收结果。

重启当前无线电服务后，实际 Dashboard API 已显示 RX1／RX2 的双 Pluto 地址和 RX1 的 −1～73 dB 软件控制范围；硬件增益未读回，不将配置显示视为设备已连接。
