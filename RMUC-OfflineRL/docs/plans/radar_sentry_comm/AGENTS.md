# AGENTS.md — 雷达—哨兵通信交接目录

本目录是 2027 雷达—哨兵通信工作包（v1.0-draft）。新会话先读：

1. `HANDOFF.md`（记忆/现状/下一步）
2. `ICD.md`（唯一字节布局）
3. `WP-SENTRY-RX-01.md`、`WP-RADAR-TX-01.md`

## 规则

- 通信层只透传 `action_id`（1–8）；8 个动作的语义未定，不要写死进编解码。
- 任何字段/ID/频率/坐标/阈值/语义变更，先改 `ICD.md` 并升版本，再改代码。
- 雷达坐标只用于接近/导航；可见时哨兵用自身视觉/自瞄，不把雷达坐标当最终瞄准点。
- 0x020B 是己方位置，不是敌方；敌方只能靠雷达 0x0301 转发。
- 0x0A01–0x0A05 进观测；**0x0A06 是干扰波密钥，雷达自用，不转发**。
- 哨兵与雷达没有直连串口：两边各用电源管理模块 User 串口 115200 8N1，经裁判服务器转发。
- `0x0220.ammo` 取 0x0208 的 `projectile_allowance_17mm`；不是 0x020D 的已兑换累计。
- 时间阈值以 ICD §8.3 为准：位置 age>300 ms 停止接近；链路 >1 s：ENGAGE 视觉-only，非 ENGAGE 回 HOLD；链路正常但 2 s 无新位置且不可见→HOLD_OUTPOST。
- 回退 HOLD 后清 `last_action_seq`；恢复时同 seq 的有效 suggestion 也要重新应用，避免卡死。
- 姿态命令转换只在串口驱动一处：`RobotState` 4=待机，协议 4=强化进攻，不能直传。
- 先在本目录 `/home/w/pd2027/radar_sentry_comm/code/` 实现；要改源仓库 `sentry_rl/` 或 `/home/w/pd2027/behaviortree_27`，先问用户。
- `code/` 已有参考实现；先跑 `make test`，37 项应全绿。ROS/BT/源仓库还没接。
- 中文、直接、先结论后细节；不确定的事实先查协议/文件，不要猜。
