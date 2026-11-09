# 雷达—哨兵通信工作包（v1.0-draft）

> 工作包编号：**WP-RADAR-SENTRY-COMM-01**｜版本：**v1.0-draft**｜日期：2026-10-31
> 依据：2027 通信协议 V1.0.0（20261029）、2027 规则 V1.0.0（20261015）
> 共享字节布局：[`ICD.md`](ICD.md)（唯一来源）

## 两个包

| 包 | 宿主 | 文件 |
|---|---|---|
| 哨兵侧收发包 **WP-SENTRY-RX-01** | 哨兵机载 ROS | [`WP-SENTRY-RX-01.md`](WP-SENTRY-RX-01.md) |
| 雷达侧收发包 **WP-RADAR-TX-01** | 雷达机载 | [`WP-RADAR-TX-01.md`](WP-RADAR-TX-01.md) |
| 共享接口 | 双方 | [`ICD.md`](ICD.md) |

## 代码状态

`code/` 已有纯 Python 参考实现（未接 ROS / 源仓库）：

- `radar_link.py`：0x0301 / 0x0210 / 0x0211 / 0x0220 编解码、CRC、流式解析
- `radar_parse.py`：0x0A01–0x0A05 解析，0x0A06 仅本地 key
- `radar_tx.py`：1 Hz 动作 + 10 Hz 位置、`action_seq`、发送器
- `radar_state_rx.py`：0x0220 接收/ACK/2 s 超时
- `sentry_state.py`：1 Hz 0x0220 构造
- `sentry_info.py`：0x020D 姿态/强化/兑换/复活 + 0x0208 允许发弹量
- `sentry_rx.py`：`action_seq` 去重、ENGAGE 状态机、回退/恢复

测试：

```bash
cd /home/w/pd2027/radar_sentry_comm
make test
```

当前 **37 项通过**；ROS 消息/节点、BT 适配和源仓库集成还没做。

## 可安装工程

两个独立工程已放到 `../radar_sentry_projects/`：

| 工程 | 作用 |
|---|---|
| `radar_sentry_projects/radar_side` | 雷达：发 0x0210，收 0x0220 |
| `radar_sentry_projects/sentry_side` | 哨兵：收 0x0210，发 0x0220 |

各自带 `pyproject.toml`、`setup.py`、`config.example.json`、README 和测试；可以 `pip install -e .` 安装。

## 已定决策

| 编号 | 决策 |
|---|---|
| D1 | RL 决策跑在**雷达**上；哨兵进程不支持 RL |
| D2 | 雷达→哨兵只发**单条 suggestion**，取消“4 台目标批”下行 |
| D3 | **动作 1 Hz + 目标位置 10 Hz，单包 10 Hz** |
| D4 | 雷达管“去哪/打谁”；哨兵管“怎么打/什么姿态/何时开火” |
| D5 | 目标可见→哨兵自身视觉/自瞄；不可见→去雷达最后位置；不盲射 |
| D6 | 动作暂定 **8 宏**，后续可能改；通信层只透传 `action_id`，不解释语义 |
| D7 | 雷达侧推理负载待测；1 Hz 推理 + 看门狗，不阻塞 0x0A 解析 |

## ICD 摘要

| 内容 | 方向 | 内容长度 | 整帧 | 频率 | 带宽 |
|---|---|---:|---:|---:|---:|
| `0x0210` Suggestion | 雷达→哨兵 | 21 B | 36 B | 10 Hz | 360 B/s |
| `0x0211` RadarStatus | 雷达→哨兵 | 12 B | 27 B | 1 Hz | 27 B/s |
| `0x0220` SentryState+Ack | 哨兵→雷达 | 59 B | 74 B | 1 Hz | 74 B/s |

- 0x0301 `user_data` ≤112 B、整包 ≤127 B、≤30 Hz；哨兵接收额度 5120 B/s × λ。
- 哨兵/雷达 ID：红 7/9，蓝 107/109。
- 动作 1 Hz 才变；`action_seq` 只在 `action_id`/`target_id` 变时 +1；同 seq 只刷目标。
- 目标位置 `age>300 ms` 停止接近；链路 >1 s：`ENGAGE` 视觉-only，非 `ENGAGE` 回 HOLD；链路正常但 2 s 无新位置且不可见→`HOLD_OUTPOST`。
- `0x0220.ammo` = 0x0208 的 `projectile_allowance_17mm`，不是 0x020D 的兑换累计。

## 联调与验收

| 步骤 | 内容 | 通过标准 |
|---|---|---|
| 1 | ICD 冻结（v1.0-draft） | 双方确认字节布局/ID/频率/阈值 |
| 2 | 离线 loopback：雷达编码→哨兵解析 | 10,000 帧零错位 |
| 3 | 动作去重：同 `action_seq` 10 包 | BT 只切一次 |
| 4 | 目标状态：可见/不可见/age 300 ms/2 s | 视觉/最后位置/原地搜索/HOLD_OUTPOST 符合 ICD §8.3 |
| 5 | 0x0220 状态/ACK | 1 Hz；ACK 与实际应用一致 |
| 6 | 链路断开与恢复 | 哨兵 ≤1.2 s 报 suggestion lost；雷达 ≤2.2 s 报 state lost；恢复 2 包内有效，且回退 HOLD 后同 seq 能重新应用 |
| 7 | 30 min soak | 无崩溃、无内存增长、带宽达标 |

## 计划

| 阶段 | 内容 | 工期 |
|---|---|---|
| W1 | ICD 冻结、`code/radar_link.py` + 黄金帧、0x0A 解析 | 5 天 |
| W2 | 哨兵 RX/BT 适配、雷达 TX、0x0220、loopback | 5 天 |
| W3 | 半实物/全场 soak、坐标标定、报告 | 5 天 |

## 角色

| 角色 | 职责 |
|---|---|
| 雷达软件 | 0x0A 解析、RL 观测、suggestion 编码、0x0220 接收 |
| 哨兵 ROS | 0x0301 解析、BT 适配、`ApproachTarget`、0x0220、0x020D 最小解析 |
| RL | 动作表（1–8）、1 Hz 决策、目标选择、奖励/评估 |
| 测试 | 黄金帧、去重、目标状态、断链、30 min soak |

## 待确认

1. 8 个动作的具体语义（1–7 是否沿用现有子树，8 是什么）。
2. 哨兵自身位置是否保留在 0x0220（ICD 已按必填 59 B 处理；否决要升版本改布局）。
3. 雷达算力测完后，1 Hz 推理进程放主控还是伴飞电脑。
4. 0x0210/0x0211/0x0220 是否与雷达现有自定义子内容 ID 冲突。
5. 0x0301 串口驱动由谁提供（现有 `wildwolf_serial_nav2` 还是新增节点）。

## 交接

新会话先读 [`HANDOFF.md`](HANDOFF.md) 和 [`AGENTS.md`](AGENTS.md)。代码先落在本目录 `code/`，不先改源仓库。
