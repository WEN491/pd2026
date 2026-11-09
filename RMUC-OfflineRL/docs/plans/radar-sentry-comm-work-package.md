# 雷达—哨兵通信工作包（v1.0-draft，已拆分）

> 本文件是旧 v0.2 的单文件版本，已拆分为两个工作包并重写为 **v1.0-draft**。
> 新位置：[`radar_sentry_comm/README.md`](radar_sentry_comm/README.md)

| 文件 | 内容 |
|---|---|
| [`radar_sentry_comm/ICD.md`](radar_sentry_comm/ICD.md) | 共享字节布局、频率、阈值（唯一来源） |
| [`radar_sentry_comm/WP-SENTRY-RX-01.md`](radar_sentry_comm/WP-SENTRY-RX-01.md) | 哨兵侧收发包 |
| [`radar_sentry_comm/WP-RADAR-TX-01.md`](radar_sentry_comm/WP-RADAR-TX-01.md) | 雷达侧收发包 |
| [`radar_sentry_comm/HANDOFF.md`](radar_sentry_comm/HANDOFF.md) | 新会话记忆 |
| [`radar_sentry_comm/AGENTS.md`](radar_sentry_comm/AGENTS.md) | 交接目录规则 |

核心变化：单条 suggestion（取消 4 台目标批）、动作 1 Hz + 位置 10 Hz、0x0220 回传、
`0x0A06` 雷达自用不转发、`ammo` 取 0x0208、失效阈值统一为 300 ms / 1 s / 2 s。
