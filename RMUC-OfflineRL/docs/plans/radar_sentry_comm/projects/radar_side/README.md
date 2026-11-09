# radar-side — 雷达侧通信工程

发送 **0x0210 Suggestion**，接收 **0x0220 SentryState+Ack**。

- 0x0210：10 Hz，单条 suggestion；动作 1 Hz 才变，目标位置 10 Hz 刷新。
- 0x0220：1 Hz，解析哨兵位置/姿态/弹量/ACK。
- 0x0A06 干扰波密钥雷达自用，不进入 0x0210。

## 安装

```bash
cd /home/w/pd2027/radar_sentry_projects/radar_side
python3 -m venv .venv
. .venv/bin/activate
pip install -e .
```

无第三方依赖（Linux `termios` + `select`）。

> 如果 `python3 -m venv` 报 `ensurepip` 缺失，安装 `python3-venv`，或直接
> `python3 -m pip install --user -e .`。

## 连接

雷达和哨兵之间**没有直连串口**。两边都用**电源管理模块 User 串口**：

- 115200 bps，8 数据位，1 停止位，无校验，无硬件流控；
- 0x0301 由裁判系统服务器转发；
- 雷达 ID：红 9 / 蓝 109；哨兵 ID：红 7 / 蓝 107。

设备名以现场 udev 为准，例如 `/dev/ttyUSB0` 或 `/dev/serial/by-id/...`。

## 运行

```bash
# 先不接硬件，验证帧
radar-sentry-radar --demo --duration 2

# 接雷达 User 串口
radar-sentry-radar \
  --device /dev/ttyUSB0 \
  --sender 9 --receiver 7
```

或：

```bash
python3 -m radar_sentry_radar.app --config config.example.json
```

`--demo` 使用内存 transport，不发真实串口；日志会打印 36 B 的 0x0210 帧和发送计数。

## 接入 RL / 感知

`radar_sentry_radar/source.py` 的 `SuggestionSource` 是唯一接入点：

```python
class MyRlSource:
    def poll(self, now_ms, sentry_state):
        # 1 Hz 动作 + 10 Hz 目标位置
        return SuggestionInput(
            action_id=6, target_id=101,
            target_x=1.0, target_y=2.0,
            target_age_ms=50, target_quality=2, target_alive=True,
        )
```

- 动作 1–8 语义由动作表维护，通信层只透传 `action_id`。
- `action_id` 变化必须间隔 ≥1 s；`SuggestionBuilder` 会拒绝过快变化。
- 0x0A01–0x0A05 的解析已提供（`radar_parse.py`），0x0A06 仅本地 key。
- 雷达坐标只用于接近/导航；哨兵可见时由哨兵自瞄。

## 帧与校验

| 帧 | 内容 | 整帧 |
|---|---:|---:|
| 0x0210 | 21 B | 36 B |
| 0x0211 | 12 B | 27 B |
| 0x0220 | 59 B | 74 B |

CRC8/CRC16 由 `radar_link.py` 实现；流式拆包用 `FrameStreamParser`。

## 测试

```bash
cd /home/w/pd2027/radar_sentry_projects/radar_side
python3 -m unittest discover -s tests -v
```

## 限制

- 这是纯 Python 服务，不依赖 ROS；接 ROS 时在 `SuggestionSource` 外面做桥接。
- `timestamp_ms` 使用启动后的毫秒数，协议上限 420000；正式比赛请在开赛时启动，或把 `now_ms` 换成比赛时间。
- 未实现 0x0A 无线接收硬件驱动；`radar_parse.py` 只做字节解析。
