# WP-SENTRY-RX-01：哨兵侧收发包（v1.0-draft）

> 工作包：**WP-SENTRY-RX-01**｜版本：**v1.0-draft**｜日期：2026-10-31
> 共享字节布局以 [`ICD.md`](ICD.md) 为准；本包不重复定义字段。
> 依据：2027 通信协议 V1.0.0（20261029）、2027 规则 V1.0.0（20261015）

## 1. 目标

在哨兵机载 ROS 上：

1. 解析雷达 0x0301/0x0210 suggestion；
2. 用 `action_seq` 去重，把动作映射到行为树，把目标注入 `ApproachTarget`；
3. 目标可见时用自身视觉/自瞄，不可见时按雷达最后位置接近，失效时安全回退；
4. 1 Hz 回传 `0x0220`（含 ACK），让雷达闭环。

**不做**：哨兵本地 RL、雷达坐标最终瞄准、姿态/小陀螺/买弹/复活/开火的雷达远程命令。

## 2. 范围与职责边界

| 雷达负责 | 哨兵负责 |
|---|---|
| “去哪 / 打谁”：`action_id` + 目标 ID/位置/age/quality | “怎么打”：姿态、小陀螺、买弹、复活、何时开火 |
| 1 Hz 动作、10 Hz 目标位置 | 行为树执行、视觉/自瞄、安全回退 |

- 雷达只发**建议 + 目标情报**；哨兵行为树保留否决/回退权。
- 0x0210 不含姿态/开火命令；`0x0220.posture` 只是状态回传，不是命令。

## 3. 交付物

**第一阶段（本目录 `code/`，不先改源仓库）**

| 交付 | 说明 |
|---|---|
| `code/radar_link.py` | 0x0210/0x0211/0x0220 编解码 + 校验；纯 Python 参考实现 |
| `code/sentry_rx.py` | 0x0301 帧解析、`action_seq` 去重、输出 suggestion 事件 |
| `code/sentry_state.py` | 1 Hz 0x0220 状态/ACK 编码 |
| `code/test_radar_link.py` | 黄金帧、坏帧、截断、CRC、ID、action/target 越界 |
| `code/test_sentry_rx.py` | 动作去重、目标可见/不可见/失效、链路断开 |

**第二阶段（源仓库集成，需另行确认）**

| 交付 | 计划路径 |
|---|---|
| ROS 消息 | `rm_decision_interfaces/msg/RadarSuggestion.msg`、`RadarLinkStatus.msg`、`SentryTeamState.msg` |
| RX 节点 | `sentry_radar_rx`：0x0301 → `/radar/suggestion`、`/radar/link` |
| BT 适配 | `radar_decision_adapter`：`action_id` 1–8 → 子树；目标注入 |
| 目标执行 | `ApproachTarget` 子树 |
| 状态发送 | `sentry_state_tx`：1 Hz 0x0220 |
| 0x020D 最小解析 | `sentry_info`：姿态/强化/6 个计时/复活/兑换状态 |

> `sentry_radar_rx`、`radar_decision_adapter`、`ApproachTarget`、`sentry_state_tx` 是哨兵侧；雷达侧节点叫 `radar_sentry_tx` / `radar_state_rx`，不要混名。

## 4. 任务

| ID | 任务 | 产出 |
|---|---|---|
| S1 | 确认 0x0301 串口/驱动入口（现有驱动还是新增解析） | 接口确认 |
| S2 | 实现 0x0301 帧 + 0x0210 校验/解码 | `radar_link.py` |
| S3 | `action_seq` 去重：同 seq 只更新目标 | `sentry_rx.py` |
| S4 | 发布 `/radar/suggestion`（action/seq/target/age/quality） | ROS 消息 |
| S5 | BT 适配：`action_id` 1–8 → 子树；1–7 先映射现有子树，8 占位 | `radar_decision_adapter` |
| S6 | `ApproachTarget`：可见→视觉/自瞄；不可见且位置新鲜→去最后位置；位置失效→停止接近/搜索 | 子树 |
| S7 | 失效回退：300 ms / 1 s / 2 s 规则；链路断开视觉-only | 安全逻辑 |
| S8 | 1 Hz 0x0220：位置、姿态、6 计时、HP/弹量、队友位置、ACK | `sentry_state.py` |
| S9 | 0x020D 最小解析：姿态/强化/计时（0x0220 依赖） | `sentry_info` |
| S10 | 测试：黄金帧、坏帧、去重、目标失效、断链、30 min soak | 报告 |

## 5. 动作与目标执行

### 5.1 动作映射

- `action_id` 1–8 由动作表定义；**通信层不解释语义**。
- 当前暂定：1–7 映射现有 7 个子树；8 暂留占位，等动作表定稿。
- `action_seq` 未变：只刷新目标位置/age；BT 不重启，避免 10 Hz 抖动。
- `action_seq` 变：按新 `action_id` 切子树；带目标时写 `target_id`/`target_pose`。
- **回退恢复**：进入 `HOLD_OUTPOST` 时清 `last_action_seq` 并置 `fallback_latched=true`；恢复后第一条有效 suggestion 即使同 seq 也重新应用动作，避免卡死。

### 5.2 目标状态机（ENGAGE）

| 条件 | 行为 |
|---|---|
| 目标可见（本地视觉/自瞄） | 用自身视觉打；雷达坐标只作后备；重置丢失计时 |
| 不可见，`target_valid && position_valid && age≤300 ms` | 去雷达最后位置接近；重置丢失计时 |
| 不可见，`age>300 ms`，雷达链路仍正常 | 停止接近；原地搜索（保持当前点）；最多等 2 s 新位置/重新可见 |
| 链路正常但 2 s 无新位置且始终不可见 | 清目标锁，回 `HOLD_OUTPOST` |
| 雷达链路断开（>1 s 无 suggestion 或 status=2）且 action 是 `ENGAGE` | **视觉-only**：可见继续打；不可见回 `HOLD_OUTPOST`；不等待 2 s |
| 链路断开且 action 不是 `ENGAGE` | 不继续执行旧 suggestion；按本地安全策略回 `HOLD_OUTPOST` |
| `target_alive=0` 或 `target_valid=0` | 清目标；有其它可见敌则最近可见，否则 `HOLD_OUTPOST` |
| 无盲射 | 目标不可见时不得仅凭雷达坐标开火 |

**阈值权威定义在 ICD §8.3**；本表是其执行版。

回退恢复：进入 HOLD 时清 `last_action_seq`；下一条有效包（即使同 seq）重新应用 action 并退出 HOLD。

### 5.3 姿态命令（不在 0x0301 内）

- `0x0220.posture/enhanced/timers` 来自 **0x020D**；由 `sentry_info` 最小解析器提供。
- 姿态命令走 **0x0120**，由串口驱动发送；转换只允许发生在一个地方。
- **不要**把 `RobotState.state` 直接透传给 0x0120：
  - `RobotState.msg`：1=移动，2=防御，3=进攻，**4=待机**；
  - 协议 0x0120/0x020D：1=进攻，2=防御，3=移动，**4=强化进攻，5=强化防御，6=强化移动**。
  - 4 的含义完全不同，直传会把“待机”变成“强化进攻”。
- 推荐：新增 `SentryPosture.msg`，直接使用协议 1–6；BT 姿态动作发布该消息，串口驱动只做一次 0x0120 编码。
- 若复用 `RobotState` 并加 `enhanced` 字段，映射必须唯一放在串口驱动：
  `(1,0)→3, (1,1)→6, (2,0)→2, (2,1)→5, (3,0)→1, (3,1)→4, (4,*)→不发命令`。
- `sentry_state_tx` **不参与**姿态命令转换，只读 0x020D 生成 0x0220，避免翻两次。

### 5.4 0x0220 状态

- 1 Hz；字段见 ICD §6。
- `ammo` 取 0x0208 的 `projectile_allowance_17mm`。
- `sentry_x/y`、3 台队友位置用官方帧；队友顺序固定重装/3/4。
- ACK：`last_action_seq`、`rx_count`、`crc_errors`。
- 雷达 >2 s 未收到 0x0220 时，雷达侧团队状态置无效（雷达侧处理）。

## 6. 验收

- 10 Hz suggestion 解析+发布 P99 < 2 ms；动作切换在 **1 个 suggestion 周期内**完成，无 10 Hz BT 抖动。
- 黄金帧逐字节一致；坏 CRC/错 ID/action_id 越界/target_id 非白名单/截断/NaN 全部拒绝且不崩溃。
- 同一 `action_seq` 10 包只应用一次；`action_seq` 变化 1 包内切子树。
- 目标可见→视觉/自瞄；不可见且 `age≤300 ms`→去最后位置；`age>300 ms` 停止接近；链路断开 >1 s：`ENGAGE` 视觉-only，非 `ENGAGE` 回 HOLD；链路正常但 2 s 无新位置且不可见→`HOLD_OUTPOST`。
- 断链检测：从最后一包起 **≤1.2 s** 报 `link down`（阈值 1.0 s，留 200 ms 调度余量）。
- 1 Hz 0x0220 正常；ACK 与哨兵实际应用一致；30 min soak 无崩溃/无内存增长。
- 雷达恢复后 2 包内重新有效；回退 HOLD 后即使 `action_seq` 未变，第一条有效 suggestion 也重新应用动作（不卡死）；旧 `action_seq` 不覆盖新动作。

## 7. 依赖与风险

| 项 | 说明 |
|---|---|
| 0x0301 串口驱动 | 现有 `wildwolf_serial_nav2` 没有源码；需确认谁提供 0x0301 解析/透传 |
| 0x020D 解析 | 0x0220 姿态/计时依赖；本包只做最小字段，不扩展完整观测 |
| 0x020B/0x0003/0x0101 | 完整 RL 观测另立 `WP-SENTRY-OBS-01`；本包不依赖它们做雷达通信 |
| 姿态命令 | `RobotState` 4=待机 vs 协议 4=强化进攻；必须单点转换 |
| 坐标 | 官方帧/`map` 帧分离；4 点标定 <0.2 m |
| 动作表 | 8 宏语义未定；通信层只传 `action_id` |

## 8. 测试清单

1. 0x0210 黄金帧：逐字节编码/解码一致。
2. 坏帧：CRC、长度、sender/receiver、content ID、version、action_id=0/9、target_id 非白名单、NaN、截断。
3. 去重：同 `action_seq` 10 包 → BT 只切一次；旧 seq 不覆盖新动作。
4. 目标状态：可见；不可见+age 100 ms；不可见+age 400 ms；不可见+age 2.1 s；target_alive=0。
5. 链路：1.0 s 无包 → link down ≤1.2 s；恢复 2 包内有效；断链但目标可见仍打；回退 HOLD 后同 seq 恢复能重新应用。
6. 0x0220：1 Hz；ACK 字段；30 min soak。
