# 雷达—哨兵通信工程（两个独立项目）

对应工作包 **v1.0-draft**；一个装雷达，一个装哨兵。

```text
radar_sentry_projects/
├── radar_side/     # 雷达：发 0x0210 Suggestion，收 0x0220 SentryState+Ack
└── sentry_side/    # 哨兵：收 0x0210 Suggestion，发 0x0220 SentryState+Ack
```

## 数据流

```text
雷达：RL/感知 → 0x0210 Suggestion（10 Hz） → 哨兵
哨兵：行为树执行 → 0x0220 SentryState+Ack（1 Hz） → 雷达
```

- **0x0210**：动作 1 Hz 才变；目标位置 10 Hz 刷新；`action_seq` 只在动作/目标 ID 变化时 +1。
- **0x0220**：哨兵位置、姿态、6 个姿态计时、HP/弹量、3 台队友位置、ACK。
- 0x0301 的 `user_data` ≤112 B、整包 ≤127 B、≤30 Hz；两条流合计远低于哨兵 5120 B/s × λ。
- 0x0A06 干扰波密钥雷达自用，**不转发**。

## 安装

两边都是纯 Python（≥3.10，无第三方依赖，Linux `termios`）：

```bash
# 雷达
cd /home/w/pd2027/radar_sentry_projects/radar_side
python3 -m venv .venv && . .venv/bin/activate
pip install -e .

# 哨兵
cd /home/w/pd2027/radar_sentry_projects/sentry_side
python3 -m venv .venv && . .venv/bin/activate
pip install -e .
```

> 如果 `python3 -m venv` 报 `ensurepip` 缺失，安装 `python3-venv`，或直接
> `python3 -m pip install --user -e .`。

> 两个工程各自内置了一份 `radar_link.py`（ICD v1.0-draft）。改协议时先改
> `/home/w/pd2027/radar_sentry_comm/ICD.md`，再同步两份副本并升版本。

## 运行

```bash
# 不接硬件：内存 transport，只打印帧
radar-sentry-radar --demo --duration 2
radar-sentry-sentry --demo --duration 2

# 接电源管理模块 User 串口
radar-sentry-radar  --device /dev/ttyUSB0 --sender 9 --receiver 7
radar-sentry-sentry --device /dev/ttyUSB0 --sender 7 --receiver 9
```

ID：红雷达 9 / 红哨兵 7；蓝雷达 109 / 蓝哨兵 107。

## 接线

**没有“雷达—哨兵”直连串口**。两边各接自己的**电源管理模块 User 串口**：

- 115200 bps，8 数据位，1 停止位，无校验，无硬件流控；
- 0x0301 由裁判系统服务器转发；
- 设备名以现场 udev 为准，例如 `/dev/ttyUSB0` 或 `/dev/serial/by-id/...`；
- 用户需要在 `dialout` 组，或用 udev 规则固定权限。

## 配置

每个工程有 `config.example.json`，也可用命令行覆盖：

```bash
radar-sentry-radar  --config config.example.json
radar-sentry-sentry --config config.example.json
```

## 接入点

| 侧 | 接入 | 说明 |
|---|---|---|
| 雷达 | `SuggestionSource` | 返回 `SuggestionInput`：`action_id` + 目标 ID/位置/age/quality |
| 哨兵 | `StateProvider` | 返回 `StateSample`：位置/HP/弹量/姿态/计时/队友/可见目标 |

- 动作 1–8 语义未定；通信层只透传 `action_id`。
- 哨兵可见目标时用自身视觉/自瞄；不可见才用雷达最后位置接近。
- 0x0220 的 `ammo` 取 0x0208 的 `projectile_allowance_17mm`。
- 姿态命令走 0x0120，转换只在串口驱动一处；`RobotState` 4=待机，协议 4=强化进攻，不能直传。

## 测试

```bash
# 雷达
cd /home/w/pd2027/radar_sentry_projects/radar_side
python3 -m unittest discover -s tests -v

# 哨兵
cd /home/w/pd2027/radar_sentry_projects/sentry_side
python3 -m unittest discover -s tests -v
```

当前：雷达 5 项、哨兵 5 项，全部通过。

## 限制

- 纯 Python 服务，不直接依赖 ROS；ROS/BT 在接入点外面桥接。
- 0x0A 无线接收硬件驱动不在本工程；`radar_parse.py` 只做字节解析。
- 0x020D/0x020B/0x0201/0x0208 的串口驱动不在本工程；哨兵侧 `sentry_info.py` 只做字节解析。
- `--demo` 不验证真实串口时序、权限和丢包。
