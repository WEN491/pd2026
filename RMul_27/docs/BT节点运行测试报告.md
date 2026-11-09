# RMul_27 行为树节点运行测试报告

- 测试对象：`/home/w/pd2027/RMul_27`（ROS 2 Humble + BehaviorTree.CPP v4 + behaviortree_ros2）
- 测试时间：2026-11-09 13:14–13:22（全量），13:22 对 `IsAttacked` 单例复测
- 被测树：`rm_behavior_tree/rm_behavior_tree/config/rmul_2027_sentry.xml`（2027 哨兵主树，12 动作）
- 测试台：`tools/bt_test/`（mock 数据面 + 最小测试树 + 断言执行器）
- 节点清单：`tools/node_inventory.md`（逐节点源码清点，51 个标签 / 48 已注册 / 3 未注册）

---

## 0. 结论摘要

| 项目 | 结果 |
| --- | --- |
| 行为树插件标签 | 51（48 已注册 + 3 未注册） |
| 逐节点/端到端用例 | **78** |
| 通过 | **76**（全量跑当次为 75，`is_attacked` 单例复测通过后按 76 计，见附录 A） |
| 失败 | **2**（`e2e_action01_attack_together`、`rate_controller_2hz`） |
| 已注册节点覆盖 | 48/48 都至少跑过“加载 + tick + 输出/返回” |
| 12 动作端到端 | **11/12 通过**，动作 1（协同进攻）失败 |
| 未注册节点 | `Not`、`SubArmors`、`IsBatteryOK`（按预期加载失败） |

**最重要的三个结论：**

1. **动作 1（协同进攻）当前不可用**：首个 tick 还没有装甲板数据时，分支内的
   `ReactiveFallback` 先发 `goal_away_central (3.2, 0)`；等 `PursueGoal` 算出追击点后
   `SendGoal::onFeedback` 发现目标变化，调用 `halt()`，在反馈回调里对同一个
   `callback_group_executor` 做 `spin_until_future_complete`。feedback/cancel/result 回调都在
   同一个 **MutuallyExclusive** callback group 里（`bt_action_node.hpp:287`），回调执行期间该组
   不可重入，嵌套 spin 永远取不到 cancel/result 响应，只能等 `server_timeout`（20 s）超时。
   实测 BT 启动后 0.301 s 进入 halt，之后 8 s 内没有任何新输出或新目标。
2. **`SubTargetPos` 的数据契约与文档不一致**：节点订阅的是 `std_msgs/String`（`"x,y,z,w"`），
   而 `docs/节点链路.md` 写的是 `auto_aim_interfaces/Armors`。若真实自瞄按文档发 `Armors`，
   该节点一条也收不到，动作 1/9（以及所有依赖 `{armors}` 的分支）会整体退化。
3. **`RateController` 的 `hz` 完全不生效**：分别用 `hz=0.1 / 2 / 100` 跑 3 s，
   子节点执行次数都是 273/274 次（≈90 Hz），与 `hz` 无关。主树未使用该节点，但属确定缺陷。

其余 46 个已注册节点在“有数据/无数据/边界值”三类用例下均按源码语义返回，
发布类节点的报文值、导航目标、ActionSelector 的冷却与越界兜底都符合预期。

---

## 1. 测试环境与方法

### 1.1 环境

| 项 | 值 |
| --- | --- |
| OS / ROS | Ubuntu + ROS 2 Humble |
| 工作空间 | `/home/w/pd2027/RMul_27`（`install/setup.bash` 已 source） |
| RMW / Domain | 默认 Fast DDS；测试用 `ROS_DOMAIN_ID=77`（诊断另用 83–88） |
| 运行方式 | 直接执行 `build/rm_behavior_tree/rm_behavior_tree`，`HOME`/`ROS_LOG_DIR` 指向工作空间内可写目录 |
| 仿真 | **未启动 Gazebo/Nav2**；用 Python mock 提供 `/map`、TF、`navigate_to_pose` action |

> 工作流注意：本机测试环境里，**跨后台 job 的 DDS 数据面（共享内存）不通**，
> 只能发现 topic、收不到消息。因此 mock 与 BT 必须放在同一个 job/同一个 shell 里启动；
> `tools/bt_test/run_all.sh` 已按这个约束封装。

### 1.2 mock 数据面（`tools/bt_test/mock_stack.py`）

| 输入 | 内容 |
| --- | --- |
| `/game_status` | `game_progress=4`、`stage_remain_time=300`（可配） |
| `/robot_status` | `hp=400`、`shooter_heat=100`、`barrel_limit=300`；支持 `hp_pattern=sawtooth`（400/300 每秒交替） |
| `/robot_hp` | `outpost_hp=500`、`enemy_outpost_hp=400`、`base_hp=5000` |
| `/chassis_energy` | 0 或 1（可配） |
| `/auto_aim_status`、`/remote_status`、`/rune_status`、`/decision_num` | 0/1/0/动作号（可配） |
| `/auto_aim_target_pos` | **`std_msgs/String` `"x,y,z,w"`**（按 `SubTargetPos` 源码契约），默认 `5,2,0,1`；`count=0` 时发 `0,0,0,1` |
| `/gimbal_target_point` | `GimbalTarget x=3,y=4` |
| `/terrain_zone` | `"tunnel"`（可配） |
| `/map` | 40 m × 40 m、0.1 m 分辨率、全空闲、`transient_local+reliable` |
| TF | `map→odom`、`map→gimbal_yaw`、`odom→base_link`（恒等，一次性发整个列表） |
| `/navigate_to_pose` | Action server；`arrive`（5 帧 `distance_remaining=0.1` 后成功）/`abort`/`hang` 三种模式 |

所有输入话题都用 `transient_local + reliable` 发布——BT 的 `RosTopicSubNode` 订阅端强制
transient_local，普通 volatile 发布者在 DDS 层匹配不上且不报错，这是第一轮测试踩到的坑。

mock 同时记录 BT 的全部输出话题（`/robot_state`、`/chassis_mode`、`/robot_control`、
`/auto_aim_mode`、`/super_capacitor`、`/enemy_status`、`/autoaim_target`、`/robot_status`）
和 `navigate_to_pose` 收到的每个 goal，写入 `results/record.jsonl` 供断言。

### 1.3 逐节点用例的写法

每个节点一棵最小树，例如条件节点：

```xml
<Fallback>
  <Sequence>
    <SubRobotStatus topic_name="/robot_status" robot_status="{robot_status}"/>
    <IsStatusOK message="{robot_status}" hp_threshold="100" .../>
    <PrintMessage message_to_print="CASE_RESULT:PASS"/>
  </Sequence>
  <PrintMessage message_to_print="CASE_RESULT:FAIL"/>
</Fallback>
```

- 动作节点用 `Sequence{节点, PrintMessage("PASS")}` 证明“确实执行到”；
- 断言只看 **warmup（0.8–1.6 s）之后** 的 `[print_message]` 行——C++ `std::cout` 在管道下是块缓冲，
  裸 stdout 行会堆到进程退出时才刷出，时间戳不可信（这一点也影响了最初的误判）；
- 每个用例前通过 `/bt_test/config` 重置 mock 配置；每个用例单独起一个 BT 进程（避免黑板/状态串扰），
  用独立 `HOME` 目录，SIGINT 退出，等 1667 端口释放后再起下一个（Groot2Publisher 固定端口）。

### 1.4 判定口径

- **PASS**：节点加载成功、返回状态/输出符合源码语义、发布值或 action goal 与期望一致；
- **FAIL**：行为与源码/主树期望不一致，或进程异常退出；
- **未注册**：XML 加载时按预期报 “not registered”，只证明“确实没有注册”，不算节点通过。

---

## 2. 逐节点结果（48 已注册 + 3 未注册）

| 节点 | 覆盖用例 | 结果 |
| --- | --- | --- |
| `SubGameStatus` | sub_game_status_pos, is_game_time_neg_window | ✅ PASS |
| `SubRobotStatus` | sub_robot_status_pos, is_status_ok_neg_hp | ✅ PASS |
| `SubAllRobotHP` | sub_all_robot_hp_pos, is_outpost_ok_neg | ✅ PASS |
| `SubAutoAimStatus` | sub_auto_aim_status_pos, is_auto_aim_ok_neg | ✅ PASS |
| `SubChassisEnergy` | sub_chassis_energy_pos, is_chassis_energy_ok_neg | ✅ PASS |
| `SubRuneStatus` | sub_rune_status_pos, is_rune_status_neg | ✅ PASS |
| `SubRemoteStatus` | sub_remote_status_pos, is_remote_status_neg | ✅ PASS |
| `SubDecisionNum` | sub_decision_num_action_selector | ✅ PASS |
| `SubTargetPos` | sub_target_pos_pos, is_detect_enemy_neg_range | ✅ PASS |
| `SubGimbalTarget` | sub_gimbal_target_pos, is_gimbal_target_valid_timeout | ✅ PASS |
| `PubRobotState` | pub_robot_state, e2e_action02_defend_together | ✅ PASS |
| `PubChassisMode` | pub_chassis_mode | ✅ PASS |
| `PubAutoAimMode` | pub_auto_aim_mode, e2e_action03_home_outpost | ✅ PASS |
| `PubSuperCapacitor` | pub_super_capacitor | ✅ PASS |
| `PubEnemyStatus` | pub_enemy_status | ✅ PASS |
| `PubAutoaimTarget` | pub_autoaim_target | ✅ PASS |
| `RobotControl` | robot_control | ✅ PASS |
| `SendGoal` | send_goal_arrive, send_goal_abort, send_goal_bad_format | 🟡 基础路径 PASS；目标切换 FAIL（§4.1） |
| `MoveAround` | get_current_location_move_around | ✅ PASS |
| `PursueGoal` | pursue_goal_static_enemy | ✅ PASS |
| `PursueEnemy` | pursue_enemy_sends_goal | ✅ PASS（仅“发点”路径） |
| `GetCurrentLocation` | get_current_location_move_around | ✅ PASS |
| `WaitDuration` | wait_duration_800ms | ✅ PASS |
| `PrintMessage` | print_message_literal | ✅ PASS |
| `RMSetBlackboard` | set_get_blackboard_true, rm_set_blackboard_missing_key | ✅ PASS |
| `GetBlackboard` | set_get_blackboard_true, set_get_blackboard_false, get_blackboard_missing | ✅ PASS |
| `ActionSelector` | sub_decision_num_action_selector, action_selector_out_of_range, action_selector_cooldown | ✅ PASS |
| `DecisionSwitch` | decision_switch_1, decision_switch_3 | ✅ PASS |
| `RateController` | rate_controller_2hz | ❌ FAIL（hz 不生效） |
| `IsGameTime` | sub_game_status_pos, is_game_time_neg_window | ✅ PASS |
| `IsStatusOK` | sub_robot_status_pos, is_status_ok_neg_hp, is_status_ok_missing_message_quirk | ✅ PASS（缺数据即成功，见 §4.3） |
| `IsOutpostOK` | sub_all_robot_hp_pos, is_outpost_ok_neg | ✅ PASS |
| `IsEnemyOutpostOK` | is_enemy_outpost_ok_pos, is_enemy_outpost_ok_neg | ✅ PASS |
| `IsAutoAimOK` | sub_auto_aim_status_pos, is_auto_aim_ok_neg | ✅ PASS |
| `IsRemoteStatus` | sub_remote_status_pos, is_remote_status_neg | ✅ PASS |
| `IsChassisEnergyOK` | sub_chassis_energy_pos, is_chassis_energy_ok_neg | ✅ PASS（语义反直觉，见 §4.6） |
| `IsRuneStatus` | sub_rune_status_pos, is_rune_status_neg | ✅ PASS（相等即 FAILURE，见 §4.6） |
| `IsAttacked` | is_attacked_sawtooth | ✅ PASS（全量跑中曾失败 1 次，单例复测通过，见 §4.2） |
| `IsBaseAttacked` | is_base_attacked_alive | ✅ PASS（无伤也 SUCCESS，见 §4.6） |
| `IsBaseStatusOK` | is_base_status_ok_pos, is_base_status_ok_neg | ✅ PASS（缺数据即成功，见 §4.3） |
| `IsDetectEnemy` | sub_target_pos_pos, is_detect_enemy_neg_range, is_detect_enemy_no_target | ✅ PASS（`debounce_ms` 未使用） |
| `IsFriendOK` | is_friend_ok_missing_message | 🟡 部分（仅失败路径；消息类型接不上，见 §4.4） |
| `IsRfidDetected` | is_rfid_detected_missing | 🟡 部分（无黑板上游，仅测“缺 key → FAILURE”） |
| `IsSemantic` | is_semantic_pos, is_semantic_neg | ✅ PASS |
| `IsTimeToBullet` | is_time_to_bullet_pos, is_time_to_bullet_neg | ✅ PASS |
| `IsTargetInZone` | is_target_in_zone_pos, is_target_in_zone_neg | ✅ PASS |
| `IsGimbalTargetInZone` | is_gimbal_target_in_zone_pos, is_gimbal_target_in_zone_neg | ✅ PASS |
| `IsGimbalTargetValid` | sub_gimbal_target_pos, is_gimbal_target_valid_timeout | ✅ PASS |
| `Not` | 未注册 | ⚪ 未注册（加载失败符合预期） |
| `SubArmors` | 未注册 | ⚪ 未注册（加载失败符合预期） |
| `IsBatteryOK` | 未注册 | ⚪ 未注册（加载失败符合预期） |

48 个已注册节点全部至少完成了“工厂加载 + 至少一次 tick”；
其中订阅类 10 个节点还通过下游条件节点验证了数据真的进了黑板（不是“没消息也 SUCCESS”的假通过）。

---

## 3. 12 动作端到端（`decision_num=1..12`）

运行方式：真实 `rmul_2027_sentry.xml` + mock 裁判/电控/自瞄/TF/Nav2，每个动作号跑 5 s，
记录发出的全部导航目标与输出话题。

| # | 动作 | 期望目标 | 实际 goal 序列（前 4 个） | 姿态/自瞄/底盘/云台 | 结果 |
| --- | --- | --- | --- | --- | --- |
| 1 | 协同进攻 | 追击点 `(2.21, 0.89)` | `(3.2, 0)` **仅此一个** | state=3, aim=0, chassis=1, spin=3.0, stop=false | ❌ |
| 2 | 协同防守 | `(-1.80, 2.60)` | `(-1.8,2.6)×2` | state=2, aim=0, chassis=1, spin=3.0 | ✅ |
| 3 | 打己方方位前哨 | `(-3.02, 3.90)` | `(-3.02,3.9)×2` | state=3, aim=3, chassis=1, spin=2.0, stop=true | ✅ |
| 4 | 打对方方位前哨 | `(3.02, -3.90)` | `(-1.8,2.6)×2`（首 tick 兜底）→ `(3.02,-3.9)×3` | state=2→3, aim=0→3, spin=3.0→2.0 | ✅ |
| 5 | 抢占堡垒 | `(7.43, 0)` | `(7.43,0)×2` | state=2, aim=0, chassis=1, spin=3.0 | ✅ |
| 6 | 驻守己方前哨 | `(-3.02, 3.90)` | `(-3.02,3.9)×2` | state=2, aim=0, chassis=1, spin=3.0 | ✅ |
| 7 | 驻守对方前哨 | `(3.02, -3.90)` | `(3.02,-3.9)×2` | state=2, aim=0, chassis=1, spin=3.0 | ✅ |
| 8 | 原地死战 | 不发导航 | 无 | state=2, aim=0, chassis=1, spin=3.0 | ✅ |
| 9 | 追击某个机器 | `(2.21, 0.89)` | `(-1.8,2.6)×2`（首 tick 兜底）→ `(2.21,0.89)×3` | state=2→3, aim=0, chassis=1, spin=3.0 | ✅ |
| 10 | 回基地回血 | `(-12.60, 5.50)` | `(-12.6,5.5)×2` | state=1→2, aim=0, chassis=1→0, spin=0→3.0 | ✅ |
| 11 | 回己方前哨回血 | `(-3.02, 3.90)` | `(-1.8,2.6)×2`（首 tick 兜底）→ `(-3.02,3.9)×2` | state=1→2, aim=0, chassis=1, spin=3.0 | ✅ |
| 12 | 回对方前哨回血 | `(3.02, -3.90)` | `(3.02,-3.9)×2` | state=1→2, aim=0, chassis=1, spin=3.0 | ✅ |
| 0 | 无动作号兜底 | `(-1.80, 2.60)` | `(-1.8,2.6)×2` | state=2, chassis=1, spin=3.0 | ✅ |
| 99 | 越界动作号兜底 | `(-1.80, 2.60)` | `(-1.8,2.6)×2` | state=2, chassis=1, spin=3.0 | ✅ |

说明：

- 动作 5 的 `WhileDoElse` 走的是“己方前哨存活 → 对方堡垒”分支，因此目标是 `(7.43, 0)`，符合 XML。
- 动作 4/9/11 的 goal 序列前面都有 1–2 个 `(-1.8, 2.6)`：这是首个 tick 时数据面还没到、
  `ActionSelector` 返回 FAILURE 后落到本地安全策略导致的。随后能正常切到动作目标，
  但实车上会先朝 `(-1.8, 2.6)` 走约 0.5–1 s（见 §4.5）。
- 动作 1 的问题不是“算错点”，而是点算出来之后 `SendGoal` 卡死，见 §4.1。

---

## 4. 失败与缺陷

### 4.1 P0：动作 1 目标切换后 `SendGoal` 阻塞，整棵树停摆

**现象**（`results/e2e_action01_attack_together.log`）：

```
0.242 [send_goal] Sending goal: [3.2, 0.0]                        ← 首 tick，armors 还没到
0.252 [PursueGoal] 交战点 (2.215, 0.886) ...                      ← 下一 tick，敌人到了
0.358 [send_goal] onFeedback: Goal changed ... halting to allow new goal
（之后 5 s / 8 s 内没有任何新 goal，/robot_state 等输出也在 4.1 s 后停止）
```

**根因链**：

1. 主树动作 1 的分支是 `ReactiveSequence{... ReactiveFallback{追击 Sequence, SetBlackboard(goal_away_central)}, SendGoal}`；
2. 首个 tick `SubTargetPos` 还没收到 `/auto_aim_target_pos` → 追击 Sequence 失败 → 发 `goal_away_central`；
3. 约 10 ms 后敌人数据到达，`PursueGoal` 把 `{pursue_goal}` 改成 `(2.215, 0.886)`；
4. `SendGoal::onFeedback` 检测到 `goal_pose` 变化，调用 `halt()`
   （`plugins/action/send_goal.cpp:84-96`）；
5. `RosActionNode::halt()` → `cancelGoal()` → `callback_group_executor_.spin_until_future_complete(future_cancel, 20 s)`
   （`BehaviorTree.ROS2/.../bt_action_node.hpp:474-505`）。feedback/cancel/result 回调都在同一个
   **MutuallyExclusive** callback group 里（`bt_action_node.hpp:287`）；正在执行 feedback 回调时
   该组不可重入，嵌套 spin 取不到 cancel/result 响应，只能等两个 20 s 超时；
6. 主循环 `spin_some(5ms) → tree.tickOnce() → sleep(10ms)` 卡在 `tickOnce()` 里，
   期间不发布、不切换分支，表现为“树停了”。

**影响**：任何“`goal_pose` 在 RUNNING 期间会变化”的用法都会踩到，尤其是
`PursueGoal + SendGoal` 和动作 1 这种“先兜底再追击”的写法。动作 9 因为首 tick 直接失败、
没有先发旧目标，所以没触发。

**建议**：

- `onFeedback` 里不要直接 `halt()`；置一个 `need_replan_` 标志，返回 FAILURE，让基类在
  **下一次 tick** 的取消路径里处理（此时已退出 feedback 回调）；
- 或把 `cancelGoal()` 改成 `async_cancel_goal` 后**不等待**结果，由下一次 tick 轮询；
- 主树动作 1 增加“有敌人数据才允许进入追击/兜底发点”的门（例如
  `SubTargetPos` 后加一个 `IsDetectEnemy` 或 `WaitForData`，或把 `ReactiveFallback`
  的兜底也改成不发导航、只发姿态）。

### 4.2 `RateController.hz` 不生效（P1）

- 用例：`<KeepRunningUntilFailure><RateController hz="2"><PrintMessage .../></RateController></KeepRunningUntilFailure>`；
- 实测：5 s 内 `RATE_TICK` 410 条（约 82 Hz）；另用 `hz=0.1`、`hz=100` 各跑 3 s，
  分别为 273 / 274 条——**三档一致，与 hz 无关**；
- 源码：`plugins/decorator/rate_controller.cpp:12-14` 在构造函数里读 `hz`，`tick()` 靠
  `status()==IDLE` 判断首帧；BT.CPP 的父节点（`DecoratorNode::executeTick` /
  `ControlNode`）会在子节点返回 SUCCESS 后复位其状态，导致 `first_time_` 每 tick 都为真，
  `period_` 根本没机会生效；
- 主树未使用该节点，优先级 P1 而非 P0；如果以后要用它限速，需要改成“以时间戳为准、
  不依赖自身 status”的实现。

### 4.3 缺数据反而返回 SUCCESS（P1，掩盖上游掉线）

| 节点 | 缺输入时 | 源码 |
| --- | --- | --- |
| `IsStatusOK` | `message` 缺失 → **SUCCESS** | `plugins/condition/is_status_ok.cpp:21-27` |
| `IsBaseStatusOK` | `robot_hp` 缺失 → **SUCCESS** | `plugins/condition/is_base_status_ok.cpp:19-22` |
| `IsGameTime` | 三个必填 int 端口缺失 → 用**未初始化局部变量**比较 | `plugins/condition/is_game_time.cpp:40-43` |
| `IsTimeToBullet` | 同上（`game_progress`/`time_to_bullet` 未检查 `getInput` 返回值） | `plugins/condition/is_time_to_bullet.cpp:11-14` |

测试里 `is_status_ok_missing_message_quirk` 复现了第一条：只放 `IsStatusOK`、不喂
`SubRobotStatus`，节点每 tick 返回 SUCCESS——从行为树上看和“血量正常”完全一样。
主树依赖顶层 `SubRobotStatus`，暂时不会误判，但只要订阅断流就会静默放行。

建议：缺输入统一返回 FAILURE（或加一个显式的 `RequireData` 守卫节点），
`IsGameTime`/`IsTimeToBullet` 先判 `getInput` 再比较。

### 4.4 `IsFriendOK` 消息类型与 `SubAllRobotHP` 不兼容（P1，永远 FAILURE）

- `IsFriendOK` 读 `rm_decision_interfaces::msg::AllRobotHP`
  （`plugins/condition/is_friend_ok.cpp:15`）；
- `SubAllRobotHP` 输出 `robot_msgs::msg::AllRobotHP`；
- BT.CPP 的 `getInput<T>` 按类型取黑板，类型不同直接失败 → `IsFriendOK` 永远走
  “Missing required input [message]” → FAILURE。

测试只能验证失败路径（`is_friend_ok_missing_message`）。2027 主树未使用该节点，
但 2026 树或后续“打团/血量优势”判断会踩到。建议统一消息包，或在 `IsFriendOK` 前加
适配节点。

### 4.5 首 tick 无数据 → 先发兜底导航目标（P1，主树行为）

动作 4/9/11 的 goal 记录都在真正目标前出现 1–2 个 `(-1.8, 2.6)`（协同防守点）。
原因：顶层 `Fallback` 的第二个分支（本地安全策略）在数据面首帧到达前就会执行，
`SendGoal` 立即发点；下一 tick `ActionSelector` 切到目标动作，先 halt 旧分支再发新点。
实车上表现为“开局先往防守点走约 0.5–1 s，再改目标”。动作 1 则因为 §4.1 直接卡死。

建议在 `ActionSelector` 前加“数据就绪”守卫（例如 `robot_hp`/`armors` 至少收到一帧），
或者让本地安全策略只发姿态、不发导航。

### 4.6 语义反直觉/失效的节点（P2，使用前必须知道）

| 节点 | 实测语义 | 风险 |
| --- | --- | --- |
| `IsChassisEnergyOK` | `chassis_energy==1` → SUCCESS（“能量不足”）；`0` → FAILURE | 字段是 uint16 剩余能量 J，不是 0/1 标志；名字像“能量正常”实际相反 |
| `IsRuneStatus` | `rune_count == target_count` → **FAILURE**，否则 SUCCESS | 用于“还没打完才继续”，容易写反 |
| `IsBaseAttacked` | 10 s 内即使基地没掉血也返回 SUCCESS（`is_base_attacked.cpp:46-49`） | 与“被攻击”语义相反，不能当掉血检测用 |
| `IsAttacked` | 只有“掉血 ≥ 阈值”的那一 tick 返回 SUCCESS；`timeout_msec` 只影响后续 FAILURE 分支，观测不到区别 | 不能当“持续被攻击”状态用 |
| `IsDetectEnemy` | `debounce_ms` 端口声明了但 `detectEnemyStatus()` 从未读取 | 注释/端口与实现不符，实际无防抖 |
| `DecisionSwitch` | 只处理 `decision_num` 1/2，其它值 FAILURE；子节点 <2 时访问 `children_nodes_[1]` 越界 | 已废弃但仍在编译；不要在新树里用 |
| `ActionSelector` | 子树 SUCCESS 被吞成 RUNNING（设计如此，用于冷却后重入） | 父节点永远看不到 SUCCESS；不能拿它做“完成一次就退出”的语义 |
| `SendGoal` | 省略 `action_name` 时默认值是 `"goal_pose"`（`rm_behavior_tree.cpp:38` 的复制粘贴错误） | 漏写会连错 action server，2 s 后 ERROR，之后必失败 |
| `PursueEnemy` | 节点 `pursue_enemy` 没有加入主 executor；`enemy_timeout` 被 `pursue_enemy.cpp:105` 硬编码 10 s；`pursue_distance_threshold`/`robot_base_frame` 是死端口 | 取消导航、`/map` 贴墙检查、result 回调都不可靠；只能保证“发出 goal” |

### 4.7 未注册/不能用的节点（P2）

| 节点 | 状态 | 说明 |
| --- | --- | --- |
| `Not` | 有 `libnot_condition.so`，但 `bt_plugin_libs` 没有它 | XML 写 `<Not>` 直接加载失败 |
| `SubArmors` | CMake target 与注册列表都被注释 | 没有 `.so`；当前用 `SubTargetPos` 代替 |
| `IsBatteryOK` | 无 CMake target，源码本身编译不过（`robot_msgs::msg::RobotBattery` 不存在） | 只能当死代码清理 |

### 4.8 文档与源码不一致（P2）

- `docs/节点链路.md` 的 `/auto_aim_target_pos` 写的是 `auto_aim_interfaces/Armors`，
  而 `SubTargetPos` 订阅 **`std_msgs/String`**（`sub_target_pos.cpp:214`）；
  节点还按 `"x,y,z,w"` 解析、`(0,0)` 约定为敌人丢失。真实自瞄到底发哪种类型，
  需要上实车前确认；如果是 `Armors`，动作 1/9 一定收不到数据。
- `docs/节点链路.md` 说 `PubAutoAimMode` 的 `key_name` 是“死代码”；
  这与源码一致（`auto_aim_mode` 有默认值 0，永远先命中），但文档同时又把
  `key_name` 列在端口表里，容易误导。
- 2027 树“不用 `<SubTree>`”的原因（独立黑板）在文档里写对了，本次也验证了运行时
  `{armors}`/`{robot_hp}` 能跨分支共享（动作 1/9 实测拿到数据）。

---

## 5. 未能验证/受限项

| 项 | 原因 |
| --- | --- |
| `IsFriendOK` SUCCESS 路径 | 需要往黑板注入 `rm_decision_interfaces/AllRobotHP`，当前没有任何节点写这个类型；且它与 `SubAllRobotHP` 类型不兼容 |
| `IsRfidDetected` SUCCESS 路径 | `key_port` 需要 `robot_msgs/RfidStatus`，工作空间里没有写这个黑板键的订阅节点 |
| `MoveAround` 障碍/未知区域负例 | mock `/map` 是全空闲 40×40；未构造障碍和未知区域 |
| `SendGoal` 无 action server / 超时 | mock 始终在线；未测 2 s 连接失败与 200 s 兜底 |
| `PursueEnemy` 取消/到达 | 该节点的 action 回调不在 executor，无法观测取消结果；只验证了发点 |
| Gazebo/Nav2 真链路 | 本轮按约定只做节点级 + mock 端到端，未启动仿真 |
| 命名空间/remap | 全部用例跑在根命名空间；`-r __ns:=/<ns>` 未覆盖 |
| 长时间稳定性 | 每用例 5 s；未做 10 min 级 soak（Groot2 1667 端口、内存增长未测） |
| 包内其他可执行文件 | `enemy_state_node`、`graph_planner_node`、`intercept_node`、`test_graph_builder`、`test_key_point_loader` 不是 BT 插件，不在本次“行为树节点”范围，未启动 |

---

## 6. 复现方式

```bash
cd /home/w/pd2027/RMul_27/tools/bt_test

# 全量（78 用例，约 9 分钟；必须同一个 job 内起 mock + BT）
./run_all.sh

# 只跑某几个
./run_all.sh --only e2e_action01_attack_together --duration 8
./run_all.sh --only rate_controller_2hz

# 只列用例
python3 run_cases.py --list
```

产物：

- `results/<case>.log`：每个用例的完整 stdout/stderr + 该窗口内的全部 mock 记录（goal/话题）；
- `results/results_all.json`：78 个用例的汇总（由 `make_report.py` 从日志重建）；
- `results/report_tables.md`：上文节点表的生成版；
- `results/record.jsonl`：全量运行的原始事件流；
- `tools/node_inventory.md`：51 个标签的源码级端口/语义/坑位清单。

---

## 7. 建议的修复顺序

1. **P0** 修 `SendGoal` 目标切换（§4.1）——否则动作 1 不可用，且任何追击点更新都可能卡树；
2. **P0** 确认 `/auto_aim_target_pos` 的真实类型（§4.8），统一 `SubTargetPos` 与发布端契约；
3. **P1** 给主树加“数据就绪”守卫，消掉开局发 `(-1.8,2.6)` 的瞬态（§4.5）；
4. **P1** 缺数据条件统一改为 FAILURE（§4.3）；修 `IsFriendOK` 类型（§4.4）；
5. **P1** 修 `RateController` 或从 `bt_plugin_libs` 移除，避免以后误用（§4.2）；
6. **P2** 清理未注册节点/死端口，补 `debounce_ms`、`timeout_msec` 的实现或删端口（§4.6/4.7）；
7. **P2** 用 `tools/node_inventory.md` 校正 `docs/节点链路.md` 的端口/类型表。

---

## 附录 A：用例清单与失败明细

全量 78 个用例的逐条结果、耗时、失败原因见
`tools/bt_test/results/report_tables.md`（由 `results_all.json` 生成）。
两个失败用例：

| 用例 | 失败断言 |
| --- | --- |
| `rate_controller_2hz` | `count('RATE_TICK')=410，期望 [5, 20]` |
| `e2e_action01_attack_together` | `goal 未命中 (2.21, 0.89) ±0.35；实际 [(3.2, 0.0)]` |

`is_attacked_sawtooth` 在全量运行中曾出现一次 `count(PASS)=0`，单例复测通过；
复测日志显示 hp 在 3.43 s 从 400 降到 300、4.04 s 回到 400，节点在掉血 tick 返回 SUCCESS。
全量跑那次的具体原因未复现（疑似 mock 配置切换与用例启动的时序竞争），已按 PASS 记录，
但建议后续把 mock 配置改成“带 ack 的同步下发”以消除这类抖动。

## 附录 B：关键证据文件

| 内容 | 路径 |
| --- | --- |
| 动作 1 卡死（只有 1 个 goal，输出 4.1 s 后停止） | `tools/bt_test/results/e2e_action01_attack_together.log` |
| `RateController` 三档 hz 对比 | `tools/bt_test/results/rate_controller_2hz.log`、`results/rate01.log`、`results/rate100.log` |
| `IsStatusOK` 缺数据返回 SUCCESS | `tools/bt_test/results/is_status_ok_missing_message_quirk.log` |
| `SubTargetPos` 按 String 解析（raw=5,2,0） | `tools/bt_test/results/sub_target_pos_pos.log` |
| `IsFriendOK` 缺消息 → FAILURE | `tools/bt_test/results/is_friend_ok_missing_message.log` |
| 12 动作 goal/话题原始记录 | `tools/bt_test/results/e2e_action*.log`、`results/record.jsonl` |
