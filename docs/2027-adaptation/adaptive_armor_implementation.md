# 装甲双模式实现与内部事件接口

日期：2026-11-10。研究依据与未确认规则见 [圆心策略调研](adaptive_armor_center_research.md)。以下 JSON 是项目内部适配器协议，不是官方新增命令号或无线报文格式。

## 1. 当前可使用的功能

正常启动项目后，在 `http://127.0.0.1:8765/#armor` 的“圆心策略”选择：

- **固定居中（默认）**：毫米圆心为检测面几何中心，输出按协议取整；宽高未知时仅显示 50%/50% 相对中心。
- **算法选点 · 最大覆盖**：按近期来袭命中、时间衰减和装甲区域倍率施加前的伤害权重，搜索给定半径下的高覆盖候选；近似同分时优先保持当前中心。

点击“保存策略”持久化配置并通知雷达融合节点。它不会直接发送裁判命令。页面蓝色虚线圆表示策略建议，紫色表示裁判确认圆；红色小点表示最近来袭受击位置。圆心仅由固定居中或算法选点策略确定，装甲图片作为只读展示，不提供点击、拖动或本机候选保存入口。

“检测面尺寸标定”支持毫米宽高、可选板面面积、物理几何确认与边界策略。未知宽高保持空值，未知边界保持“规则边界尚未核实”。输入尺寸而不确认几何时，仅作为预览比例使用；不会解除真实请求门控。面积留空时暂按矩形宽高估计，其是否适用于官方板面面积仍需核实。

“满足条件时自动加固”默认关闭。即使勾选，事件源未确认、比赛状态不新鲜、数据不完整、几何不明确、收益不足或机会不足都会阻止请求。真实适配器尚未接入，因此默认部署会等待数据。

## 2. 配置与运行位置

- 用户配置：`~/.config/shark-radar/adaptive-armor.json`；可用 `RM_RADIO_ARMOR_CONFIG` 指定另一文件，比赛台与融合节点必须使用同一路径。
- 默认配置示例：`radio/apps/match_rx/adaptive_armor.example.json`。示例中的时间窗口、样本数、置信水平与收益余量都是工程初值，不是官方常量或已完成实战调优的参数。
- `radar_integration_node` 内部的独立 2 Hz 定时器执行策略；复用既有裁判状态和 `radar_cmd_pub`，无需新增相机进程或另开裁判串口。
- 配置 JSON 缺失时使用固定居中、无几何、无可信来源、自动请求关闭的默认值。非法配置不启用自动请求，并报告错误。
- 网页只可修改模式、自动请求开关和几何相关字段；不能通过网页把输入源加入 `trusted_sources`。真实适配器接口、统计完整性和队内链路确认后，开发者在配置文件中登记来源，重启节点或保存策略以重新加载。该名单是本地适配器配置约束，不是 ROS 消息的身份认证。

核心位于 `radio/src/rm_radio_ros/rm_radio_ros/core/adaptive_armor.py`，运行状态为 `shark.radar.adaptive_armor.v1`。节点从来源事件累计统计，不读取网页模拟数，也不从 0x0A05 圆心/半径反推命中分布。

## 3. ROS 与 HTTP 通道

| 通道 | 结构与用途 |
|---|---|
| `/rm_radar_algorithm/armor_events` | `std_msgs/String`，以下内部事件；等待后续官方/队内适配器提供。 |
| `/rm_radar_algorithm/adaptive_armor_config` | `{"schema":"shark.radar.armor_config.v1","config":{...}}`，Transient Local，深度 1；比赛台发布、融合节点订阅。 |
| `/rm_radar_algorithm/adaptive_armor` | `std_msgs/String`，统计、建议、等待原因及请求状态；Dashboard 订阅并经原有 state/SSE 转发。 |
| `/rm_radar_algorithm/radar_cmd` | 复用 `shark.radar.command.v1` / `fortify_armor`；融合节点仅在门控全部通过后发布。 |
| `/rm_referee_serial_node/tx_frames` | 接收请求 ID 对应的写入/拒绝/演练回报；写入成功后继续等待裁判确认。 |
| `POST /api/adaptive-armor/config` | 配置部分更新；原子落盘成功后才发布，不包含任何直接选点发令操作。 |

模式或几何变更会重置候选稳定观察，保留比赛统计和未核对请求，不能通过切换模式绕过超时锁定。

## 4. 内部事件约定

示例时间戳只用于说明。线上使用同一 UNIX 秒时基；所有事件的接收时间由节点本机提供，源时间不得超前超过 0.5 s，命中时间不得早于会话开始。来源适配器需要保留原始官方帧以便审计。

### 4.1 开始会话

```json
{
  "schema": "shark.radar.armor_event.v1",
  "kind": "session",
  "timestamp": 100,
  "coordinate_frame": "armor_surface_lower_left_mm",
  "match_id": "match-example",
  "source": "example-adapter",
  "origin": "synthetic",
  "weight_basis": "count",
  "side": "red",
  "complete_from_start": true
}
```

`origin` 可为 `live`、`replay`、`synthetic`；后两者永远不能产生请求。线上必须确认 `live`、来源名单、正确阵营与完整当局数据。会话开始后不能通过重复 `session` 清空统计；当前会话在新鲜裁判状态从进行中转为非进行中时释放，再允许开始下一局。中途加入且没有全量补齐时，`complete_from_start` 必须为 false。`weight_basis` 默认 count，整局每发等权；仅当每发伤害均可靠时选择 baseline_damage，防止未知伤害与已知伤害混权。

模拟工具另可发送 `kind: "end_session"`：保留相同 schema、coordinate_frame、match_id、source 和当前 timestamp，不带命中载荷。仅匹配来源的 synthetic/replay 会话、无未核对请求、时间合法时才清空统计；live 会话仍必须等待裁判比赛结束，不能被工具复位。Qt 的正常关闭会先等待结束确认，再恢复原运行配置。工具入口和操作见 [tool/README.md](../../tool/README.md)。

### 4.2 逐发有效命中

```json
{
  "schema": "shark.radar.armor_event.v1",
  "kind": "hit",
  "timestamp": 101,
  "coordinate_frame": "armor_surface_lower_left_mm",
  "match_id": "match-example",
  "source": "example-adapter",
  "direction": "incoming",
  "event_id": "R1:armor0:boot0:sequence65536",
  "confirmed": true,
  "x_mm": 50,
  "y_mm": 60,
  "baseline_damage": 1
}
```

- `outgoing` 表示己方打中对方；用于己方命中数门槛与己方 η/σ。`incoming` 表示己方装甲被对方击中；用于选点与对方 η/σ。状态兼容字段 `own_hit_count` 明确等于 `own_scored_hit_count`，另有 `own_received_hit_count`。
- `event_id` 必须由适配器组合源机器人、模块、启动代号与展开后的命中序号，保证同一方向、同一比赛内唯一。不能仅用原始 uint16 序号；跨模块、回绕和重启不能互相去重。
- X/Y 为非负整数 mm，输入保持官方原始左下角坐标，不转成地图坐标或未经确认的板面比例。已确认几何下的来袭点超界会被拒绝。当前实现仅支持一种已确认的公共矩形检测面；多种模块几何尚未实现，适配器不得直接混入不同物理坐标约定的样本。
- `confirmed` 表示上游已完成弹丸事件类型和最终有效性确认；不能将撞击或未核对的本地扣血直接置为 true。官方载荷的具体解析仍未实现。
- count 会话每发权重统一为 1，不因某发额外提供伤害而混权；baseline_damage 会话要求每发都提供该字段。其值应为装甲区域伤害倍率施加前的伤害，不能把已乘 0.5/1.5 的最终扣血值作为权重。每发等权时的预计伤害倍率是等伤害假设下的估计，不是混合弹丸伤害的保证。真实离散命中不因聚类判为“噪点”而删除。

### 4.3 当局累计实际发射统计与心跳

```json
{
  "schema": "shark.radar.armor_event.v1",
  "kind": "shots",
  "timestamp": 102,
  "coordinate_frame": "armor_surface_lower_left_mm",
  "match_id": "match-example",
  "source": "example-adapter",
  "own_shots": 100,
  "opponent_shots": 100,
  "data_complete": true
}
```

这是实际发射数，不是允许发弹量。计数不能倒退；心跳必须带最新源时间。默认超过 3 s 未收到新的完整源状态即停止自动请求、标记面积估计过期。发现丢失命中或分母不可重建时必须设置 `data_complete:false`；完整性一旦丢失，本局不再自行恢复，需下一会话或后续全量恢复机制，不能仅恢复心跳就宣称统计完整。

## 5. 选点、收益与请求生命周期

双方全局 η/σ 从当局完整数据计算。每方至少 10 个有效样本、非零散布和有效实际发射分母才估计规则面积；不以 epsilon 代替零散布。当前已生效时优先使用新鲜 0x020E 实际半径；未生效时使用可核对的面积估计，指令仅发送圆心，不发送半径。

近期预测窗口分为较早训练部分和最新验证部分。算法在训练样本上搜索，覆盖率与下界在验证样本上计算；较近样本按指数衰减加权。搜索限制网格规模并分级细化，最终用整数毫米坐标重新评分，是近似候选搜索，不承诺连续空间全局最优。

下界采用独立时间块假设下的加权 Hoeffding 界；连射不当作大量独立命中。它不能保证对手策略不变，仍需真实回放调参。首次请求要求覆盖下界大于 `0.5 + first_margin`；改点要求在同一当前半径下，相对现中心的覆盖改善下界超过 `change_margin`。都需满足样本数、稳定时间、真实输入、完整统计、己方攻击命中 ≥10、新鲜比赛和 0x020E、剩余机会、阵营与几何约定。

状态包括 `waiting`、`preview`、`pending`、`timeout`、`failed`、`dry_run`。串口 written 不直接表示生效；需在后续新鲜 0x020E 中观察到预期圆心、生效标记和机会下降，记录 `last_result.phase=confirmed`。机会下降检查可能受同时发放新机会影响，无法唯一确认时保守超时。未提供 request_id 的裁判回报只能作为组合证据，不能声称具有完整请求级关联。

超时、写失败或演练后保留请求锁，本局不自动重试。超时后如果迟到的有效写入/裁判证据补齐，仍可确认；否则需要人工核对实际状态后重启决策或等下一局，不能通过网页切换模式重新发令。重启前需确认当前生效区、串口计数与完整事件源；软件重启本身不保证能恢复当局全量统计。

## 6. 离线回放与录制

不依赖 ROS 的命令（仓库根目录运行）：

```bash
PYTHONPATH=radio/src/rm_radio_ros python3 -m rm_radio_ros.core.adaptive_armor_replay \
  --input /absolute/path/events.jsonl \
  --config /absolute/path/adaptive-armor.json \
  --output /absolute/path/armor-decisions.jsonl
```

支持 JSONL、JSONL.gz 与 stdin。对于现有 zstd 比赛日志，可先用 `zstd -dc /absolute/path/events.jsonl.zst` 解压，再将输出管道传给 `--input -`，并显式指定保存的配置。安装包重建后也可使用 `rm_adaptive_armor_replay` 入口。

独立 JSONL 每行为 `{"kind":"event","timestamp":接收时间,"data":上面的事件}`；还支持 `kind=config/referee/tick`。`referee` 的 data 为现有串口节点状态，含 `last_game_status` 和 `last_radar_decision_sync`，各自保留原始时间戳。必须保存原始到达顺序；回放接收时间不能倒退。

工具也识别现有 recorder 的 `kind=topic`、`topic`、`data` 结构。配置可来自配置话题或状态中 config，但**不能从 Dashboard 截断的 hit_points 重建全局统计**。即使日志中原先为 live，回放也强制转换为 replay，不创建 ROS 节点或发令。

recorder 的默认话题与比赛配置已加入事件、配置和状态；开录时另存 `adaptive_armor_config_snapshot.json`，回放文件同目录存在该快照时会自动读取。自动录像启动前的会话/命中不会被凭空补齐：用于完整统计复算的日志必须从 session 和第一发开始收集；若缺失 session，回放明确报错。实战录制早于事件会话的验收仍需真实适配器接入后完成。

五种候选方法的独立未来窗口对比使用 `rm_adaptive_armor_benchmark`，输入格式相同，输出逐会话 JSON 报告。方法、评分规则、合成样例及限制见 [离线对比说明](adaptive_armor_benchmark.md)。它比较候选覆盖，不模拟实际机会消耗或替代线上首次/改点门控。

## 7. 验证与待完成项

- 核心/回放测试覆盖方向计数、去重、非法坐标/时间、全局公式、零散布、偏心分布、块置信界、首次/改点收益、资格/阵营/过期门控、请求锁、裁判确认与比赛重置。
- 软件链路测试覆盖配置落盘失败不发布、刷新/SSE 配置、ROS 回调保留 pending、建议请求通过既有串口门控并生成 13 字节载荷；不打开真实裁判串口。
- 浏览器在独立 ROS 域 199、独立 8766 端口和临时配置上验证两种模式切换、毫米居中、算法圆/受击点叠加与刷新持久化。样本明确标记 synthetic，未发送真实裁判请求。
- 真实逐发输入与发射分母、实体检测面/边界、跨模块映射、实战参数调优、裁判实机确认及完整比赛录制尚未完成；自适应装甲大功能继续保持未勾选。
