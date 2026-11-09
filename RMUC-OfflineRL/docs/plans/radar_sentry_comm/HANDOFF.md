# HANDOFF — 雷达—哨兵通信新会话记忆

> 你是一个新会话。先读 `AGENTS.md`，再读 `ICD.md`、`WP-SENTRY-RX-01.md`、`WP-RADAR-TX-01.md`。
> 不要先改代码；先把“待确认”问清楚。

## 1. 任务背景

RMUC 2027 哨兵全自动。原计划 RL 在哨兵上，但哨兵进程/算力不够，改为：

- **RL 决策跑在雷达上**。
- 雷达通过 **0x0301** 给哨兵发**单条 suggestion**（动作 + 目标情报）。
- 哨兵行为树执行：雷达管“去哪/打谁”，哨兵管“怎么打/什么姿态/何时开火”。
- 目标可见→哨兵自身视觉/自瞄；不可见→去雷达最后位置；不盲射。
- 动作暂定 **8 宏**；通信层只透传 `action_id`，不解释语义。

## 2. 当前版本

- **v1.0-draft**（2026-10-31 重写）。
- 两个包：`WP-SENTRY-RX-01`、`WP-RADAR-TX-01`；共享 `ICD.md`。
- 代码：`code/` 参考实现；两个可安装工程在 `../radar_sentry_projects/`；37 项 code 测试 + 两个工程各 5 项测试通过；未接 ROS / 源仓库。
- 文档：本目录 + 源目录 `/home/w/pd2026git/RMUC-OfflineRL/docs/plans/radar_sentry_comm/`。

## 3. 已定 ICD

| 内容 | 方向 | 内容长度 | 整帧 | 频率 |
|---|---|---:|---:|---:|
| `0x0210` Suggestion | 雷达→哨兵 | 21 B | 36 B | 10 Hz |
| `0x0211` RadarStatus | 雷达→哨兵 | 12 B | 27 B | 1 Hz |
| `0x0220` SentryState+Ack | 哨兵→雷达 | 59 B | 74 B | 1 Hz |

关键规则：

- 动作 **1 Hz** 才变；目标位置 **10 Hz** 刷新。
- `action_seq` 只在 `action_id` 或 `target_id` 变化时 +1；同一 seq 只刷目标，不重入 BT。
- `target_alive`/`target_valid`/`position_valid` 变化不依赖 `action_seq`，每个包都检查。
- 哨兵只接受 `sender_id=己方雷达`、`receiver_id=己方哨兵` 的 0x0301。
- 0x0301 `user_data` ≤112 B、整包 ≤127 B、≤30 Hz；哨兵额度 5120 B/s × λ。
- 哨兵 ID：红 7 / 蓝 107；雷达：红 9 / 蓝 109。
- `0x0220.ammo` = 0x0208 的 `projectile_allowance_17mm`。
- 目标是敌方地面机器人；坐标只用于接近，不作为最终瞄准点。

## 4. 失效状态机（权威在 ICD §8.3）

| 条件 | 行为 |
|---|---|
| `target_age_ms > 300` | 位置无效；停止按最后位置接近；可见→视觉打，不可见→原地搜索（保持当前点） |
| 连续 `>1000 ms` 无 0x0210 或 `status=2` | 雷达链路断开；`ENGAGE` 视觉-only：可见继续打，不可见→`HOLD_OUTPOST`；非 `ENGAGE` 不继续旧 suggestion，回 `HOLD_OUTPOST` |
| 链路正常但 `target_age_ms > 2000` 且不可见 | 清目标锁，回 `HOLD_OUTPOST` |
| 连续 `>2000 ms` 无 0x0220（雷达侧） | 团队状态置无效；RL 用掩码，不阻塞 suggestion |

- 300 ms–2 s：位置失效，但目标可见仍打；不可见则原地搜索（保持当前点），等新位置。
- 链路 1 s 就断时，不等待 2 s；ENGAGE 直接视觉-only，非 ENGAGE 回 HOLD。
- 断链检测：哨兵 **≤1.2 s**；雷达侧 0x0220 **≤2.2 s**。
- 回退 HOLD 后清 `last_action_seq`；恢复时第一条有效 suggestion 即使同 seq 也重新应用，避免卡死。

## 5. 协议事实（别搞混）

- 哨兵多机通信：可发给所有己方，**只能收雷达**；雷达反之只能收哨兵。
- 0x020B 是**己方**地面位置（重装/3/4），服务器→哨兵；雷达收不到。
- 0x020D 是哨兵姿态/强化/复活/兑换，服务器→哨兵；雷达收不到。
- 0x0003 给结构 HP + 累计控制度；0x0101 给控制点/增益状态；**没有**占点 progress、没有基地 armor_open。
- 雷达无线 0x0A01–0x0A06 是 10 Hz；本包消费 0x0A01–0x0A05；**0x0A06 是干扰波密钥（6 B ASCII），雷达自用，不转发**。
- 0x0A01 坐标单位 **cm**，发送前 /100 并转官方帧。
- 哨兵和雷达之间**没有直连串口**；两边各用电源管理模块 User 串口 115200 8N1，经裁判服务器转发 0x0301。

## 6. 代码/文件现状

- 源仓库：`/home/w/pd2026git/RMUC-OfflineRL`
  - `sentry_rl/`：mock + Arena-1 训练工作区；`./scripts/launch.sh` 可跑。
  - `behaviortree_27/.../rmul_2027_7v7.xml`：7 子树 + DecisionSwitch；**install 仍是旧的，未重编译**。
- 本目录：`/home/w/pd2027/radar_sentry_comm/`（交接副本）。
- `/home/w/pd2027` 还有：
  - `UTF-8__RoboMaster 2027 ... 通信协议 ...pdf`
  - `RM2027_雷达与哨兵通信协议解读.md`
  - `behaviortree_27/`（原始 BT 工作区；install 里有旧 symlink）
- 本目录 `code/`：参考实现，37 项测试通过。
- `../radar_sentry_projects/radar_side`：可安装雷达工程，5 项测试通过。
- `../radar_sentry_projects/sentry_side`：可安装哨兵工程，5 项测试通过。
- 现有 checkpoints 是 mock 数据训的，`obs` 是 Arena-1 口径，**不能直接上车**。

## 7. 下一步

1. 确认“待确认”5 项，尤其 8 动作语义、哨兵自身位置、雷达算力。
2. 复核 `code/` 与 ICD 的字段/阈值；`make test` 必须保持全绿。
3. 哨兵侧 ROS：`sentry_radar_rx` + `radar_decision_adapter` + `ApproachTarget` + `sentry_state_tx`。
4. 雷达侧 ROS：`radar_sentry_tx` + `radar_state_rx` + `radar_obs`。
5. 在实机安装 `radar_sentry_projects/radar_side` 和 `sentry_side`，先 `--demo`，再接 User 串口。
6. 接 0x0301 串口驱动/0x0A 接收；做 loopback、动作去重、断链恢复、30 min soak。
7. 最后改 Arena-1 虚拟雷达观察者，训练/评估。

## 8. 节点命名（避免混名）

| 侧 | 节点 |
|---|---|
| 哨兵 | `sentry_radar_rx`（收）、`radar_decision_adapter`（BT 适配）、`sentry_state_tx`（1 Hz 0x0220） |
| 雷达 | `radar_sentry_tx`（发 0x0210/0x0211）、`radar_state_rx`（收 0x0220） |

## 9. 姿态

- 0x0220 `posture`：1=进攻/2=防御/3=移动；`enhanced` 0/1；6 个计时。
- 数据来自 0x020D；WP-SENTRY 的 `sentry_info` 做最小解析。
- 姿态命令走 0x0120，转换只能在**串口驱动**一处：
  - `RobotState.msg`：1=移动/2=防御/3=进攻/**4=待机**；
  - 协议：1=进攻/2=防御/3=移动/**4=强化进攻/5=强化防御/6=强化移动**。
  - 推荐新增 `SentryPosture.msg` 直接 1–6；若复用 `RobotState` + `enhanced`，映射为 `(1,0)→3,(1,1)→6,(2,0)→2,(2,1)→5,(3,0)→1,(3,1)→4,(4,*)→不发`。
- `sentry_state_tx` 不参与姿态命令转换，避免翻两次。

## 10. 待确认

1. 8 个动作的具体语义（1–7 是否沿用现有子树，8 是什么）。
2. 哨兵自身位置是否保留在 0x0220（ICD 按必填 59 B 处理；否决要升版本改布局）。
3. 雷达算力测完后，1 Hz 推理进程放主控还是伴飞电脑。
4. `0x0210/0x0211/0x0220` 是否与雷达现有自定义子内容 ID 冲突。
5. 0x0301 串口驱动由谁提供（现有 `wildwolf_serial_nav2` 还是新增节点）。

## 11. 不要做的事

- 不要把雷达坐标直接当最终瞄准点。
- 不要每 10 Hz 包都重启行为树。
- 不要把 `0x020B` 当成敌方位置。
- 不要把 `0x0A06` 转发给哨兵。
- 不要用现有 mock checkpoint 作为真实策略效果。
- 不要在未重编译 BT 的情况下声称 `DecisionSwitch` 已支持 8 个动作。
- 不要先改源仓库/`behaviortree_27`；先在 `code/` 实现，集成前问用户。
