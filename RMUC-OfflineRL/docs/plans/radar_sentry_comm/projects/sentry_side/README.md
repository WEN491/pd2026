# sentry-side — 哨兵侧通信工程

接收 **0x0210 Suggestion**，发送 **0x0220 SentryState+Ack**。

- 0x0210：10 Hz 接收；`action_seq` 去重，同 seq 不重入行为树。
- 0x0220：1 Hz 发送；哨兵位置、姿态、6 个姿态计时、HP/弹量、3 台队友位置、ACK。
- 动作 1–8 语义由动作表维护；`engage_action_id=6` 只是参考默认值。

## 安装

```bash
cd /home/w/pd2027/radar_sentry_projects/sentry_side
python3 -m venv .venv
. .venv/bin/activate
pip install -e .
```

无第三方依赖（Linux `termios` + `select`）。

> 如果 `python3 -m venv` 报 `ensurepip` 缺失，安装 `python3-venv`，或直接
> `python3 -m pip install --user -e .`。

## 连接

哨兵和雷达之间**没有直连串口**。两边都用**电源管理模块 User 串口**：

- 115200 bps，8 数据位，1 停止位，无校验，无硬件流控；
- 0x0301 由裁判系统服务器转发；
- 哨兵 ID：红 7 / 蓝 107；雷达 ID：红 9 / 蓝 109。

## 运行

```bash
# 先不接硬件，验证帧
radar-sentry-sentry --demo --duration 2

# 接哨兵 User 串口
radar-sentry-sentry \
  --device /dev/ttyUSB0 \
  --sender 7 --receiver 9
```

或：

```bash
python3 -m radar_sentry_sentry.app --config config.example.json
```

`--demo` 使用内存 transport，不发真实串口；日志会打印 74 B 的 0x0220 帧和当前 mode。

## 接入哨兵状态

`radar_sentry_sentry/state_provider.py` 的 `StateProvider` 是状态输入点：

```python
class MyStateProvider:
    def sample(self, now_ms):
        # 从 0x0201/0x0208/0x020B/0x020D 组装
        return StateSample(...)

    def visible_target_id(self):
        return 101  # 本地视觉看到的目标；None 表示不可见
```

- `0x0220.ammo` 必须是 0x0208 的 `projectile_allowance_17mm`。
- `posture` 是协议口径：1=进攻，2=防御，3=移动；`enhanced` 单独 0/1。
- 姿态命令走 0x0120，且只在串口驱动做一次转换：
  `RobotState` 4=待机，协议 4=强化进攻，不能直传。
- 动作语义未定；`engage_action_id` 由动作表传参，不要写死到通信层。

## 行为树接入

`SentryApp` 当前把 decision 打成日志。接行为树时替换 `_log_decision`/增加发布：

- `mode=approach`：把 `target_id`/`target_pose` 写黑板，调 `ApproachTarget`；
- `mode=vision`：走自身视觉/自瞄；
- `mode=fallback`：清目标，回 `HOLD_OUTPOST`；
- 回退后清 `last_action_seq`，恢复时同 seq 也能重新应用。

## 帧与校验

| 帧 | 内容 | 整帧 |
|---|---:|---:|
| 0x0210 | 21 B | 36 B |
| 0x0220 | 59 B | 74 B |

## 测试

```bash
cd /home/w/pd2027/radar_sentry_projects/sentry_side
python3 -m unittest discover -s tests -v
```

## 限制

- 纯 Python 服务，不依赖 ROS；ROS/BT 通过 `StateProvider` 和 decision 回调接。
- `0x020D`/`0x020B`/`0x0201`/`0x0208` 的串口驱动未包含；`sentry_info.py` 只解析 0x020D/0x0208 字节。
- `--demo` 不验证真实串口时序。
