# Radar—sentry communication reference code

纯 Python 标准库实现，对应 `ICD.md` v1.0-draft；不依赖 ROS，不直接改源仓库。

## 模块

| 文件 | 作用 |
|---|---|
| `radar_link.py` | 0x0301 帧 / 0x0210 / 0x0211 / 0x0220 编解码、CRC、校验 |
| `radar_parse.py` | 0x0A01–0x0A05 解析（cm→m）；0x0A06 只保留为雷达本地 key |
| `radar_tx.py` | 1 Hz 动作 + 10 Hz 目标位置、`action_seq`、发送器 |
| `radar_state_rx.py` | 雷达侧 0x0220 接收/ACK/2 s 超时 |
| `sentry_state.py` | 哨兵侧 1 Hz 0x0220 构造 |
| `sentry_info.py` | 0x020D 姿态/强化/兑换/复活；0x0208 允许发弹量 |
| `sentry_rx.py` | 哨兵侧 `action_seq` 去重、ENGAGE 状态机、回退/恢复 |

> `sentry_rx.py` 的 `engage_action_id=6` 只是现有 7 宏的参考默认值。通信层不解释动作语义；动作表定稿后由调用方传入。
>
> 频率由调用方调度：suggestion 10 Hz、0x0211/0x0220 1 Hz；`SuggestionBuilder` 只强制“动作 1 Hz 才变”，不负责 10 Hz 定时。

## 运行测试

```bash
cd /home/w/pd2027/radar_sentry_comm
python3 -m unittest discover -s code -p 'test_*.py' -v
```

或：

```bash
cd /home/w/pd2027/radar_sentry_comm
make test
```

## 端到端 demo

```bash
cd /home/w/pd2027/radar_sentry_comm
python3 code/demo.py
```

输出会打印 0x0210 的 36 B 帧、哨兵 mode、0x0220 的 74 B 帧和雷达 ACK。

## 帧长

| 内容 | 内容长度 | 整帧 |
|---|---:|---:|
| 0x0210 Suggestion | 21 B | 36 B |
| 0x0211 RadarStatus | 12 B | 27 B |
| 0x0220 SentryState+Ack | 59 B | 74 B |

## 不做

- 不解析/转发 0x0A06 密钥；
- 不生成姿态/开火命令；
- 不替代 ROS 节点或行为树；
- 不把雷达坐标当最终瞄准点。
