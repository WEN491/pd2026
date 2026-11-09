# BT 插件节点清单（源码静态清点）

> 工作空间：`/home/w/pd2027/RMul_27`（ROS 2 Humble + BehaviorTree.CPP v4 + behaviortree_ros2）
> 清点方式：只读源码，未编译、未运行、未修改任何源码。所有结论均给出源文件位置；凡与
> `rm_behavior_tree/rm_behavior_tree/docs/行为树动作节点清单.md` 不一致处，条目内标注 **“文档说 X，源码是 Y”**。
>
> ⚠ 任务里写的 `docs/行为树动作节点清单.md` 在仓库根目录不存在；实际文件在
> `rm_behavior_tree/rm_behavior_tree/docs/行为树动作节点清单.md`（且它只覆盖动作节点，条件/控制/装饰节点没有表）。

## 0. 清点口径与总览

范围（4 个插件目录 + 对应头文件 + 注册入口）：

- `rm_behavior_tree/rm_behavior_tree/plugins/{action,condition,control,decorator}/*.cpp`
- `rm_behavior_tree/rm_behavior_tree/include/rm_behavior_tree/plugins/**/*.hpp`
- `rm_behavior_tree/rm_behavior_tree/src/rm_behavior_tree.cpp`（注册入口）
- 辅助事实来源：`rm_behavior_tree/rm_behavior_tree/CMakeLists.txt`（决定哪些 cpp 会编译成插件库）、
  `rm_behavior_tree/BehaviorTree.ROS2/behaviortree_ros2/include/behaviortree_ros2/{bt_topic_sub_node,bt_topic_pub_node,bt_action_node}.hpp`
  （基类行为）、`behaviortree_cpp/src/xml_parsing.cpp:818-836`（端口默认值如何进入 `NodeConfig`）。

统计：

| 项 | 数量 |
| --- | --- |
| 范围内 `.cpp` 文件 | 52（action 27 + condition 22 + control 2 + decorator 1） |
| 其中空文件（0 字节，无节点） | 1（`plugins/action/posture_change.cpp`） |
| 有 XML 标签的节点 | **51** |
| 已在 `src/rm_behavior_tree.cpp` 注册 | **48** |
| 未注册 | **3**（`SubArmors`、`IsBatteryOK`、`Not`） |
| 注册函数 | `RegisterRosNode` 34 个 / `registerFromPlugin`（`BT_REGISTER_NODES`）14 个 |

两条注册路径的源码事实：

1. `CreateRosNodePlugin(类, "Tag")`（cpp 末尾）+ `RegisterRosNode(factory, getOSName("库名"), params)`：
   **库名必须是 CMake target 名**，XML 标签来自 `CreateRosNodePlugin` 第二个参数。共 34 个。
2. `BT_REGISTER_NODES(factory){ factory.registerNodeType<T>("Tag"); }` + `factory.registerFromPlugin(getOSName("库名"))`
   （`bt_plugin_libs` 列表）。共 14 个。

`msg_update_plugin_libs`（10 个：8 个 `RosTopicSubNode<T>` + `SubTargetPos`/`SubGimbalTarget` 两个 `SyncActionNode`）
与 `bt_plugin_libs`（14 个）用的是
`RegisterRosNode + params_update_msg` / `registerFromPlugin`，**都没有默认值**：
`params_update_msg` 只有 `nh`（`rm_behavior_tree.cpp:25-26`），所以这些节点的 `topic_name` 必须由 XML 显式给出。

---

# 一、订阅类动作节点（`BT::RosTopicSubNode<T>` ×8 + 自实现订阅 ×2）

§1–§8 是 8 个 `BT::RosTopicSubNode<T>`；§9 `SubTargetPos`、§10 `SubGimbalTarget` 是**自实现订阅的 `SyncActionNode`**，
公共事实不同（分别标注）。

**§1–§8 这 8 个节点的公共事实**（源码：`bt_topic_sub_node.hpp`）：

- 基类是 **`BT::ConditionNode`**（不是 ActionNode），XML 里写 `<Action ID="...">` 还是 `<Condition ID="...">`
  都能被加载（BT.CPP 按 `providedPorts` 与注册类型实例化，不看标签名——但 Groot2 面板会按 `<TreeNodesModel>` 显示）。
- **`topic_name` 端口没有默认值**（如 `sub_game_status.hpp:19` 只有 `BT::InputPort<std::string>("topic_name")`），
  而 `params_update_msg.default_port_value` 为空 → XML 漏写 `topic_name` 时构造函数直接
  `throw std::logic_error("Both [topic_name] in the InputPort and the RosNodeParams are empty.")`
  → `createTreeFromFile` 抛异常，整棵树加载失败。**这是最容易踩的第一个坑。**
- 订阅 QoS 是 `KeepLast(1) + transient_local + reliable`（`bt_topic_sub_node.hpp:74-78`，安装副本
  `install/behaviortree_ros2/include/.../bt_topic_sub_node.hpp:72-78` 一致）。DDS 兼容性要求
  publisher 的 durability ≥ subscriber 的 durability，所以**测试发布端必须 `transient_local`（+reliable）**，
  `ros2 topic pub` 默认 volatile 会连不上（不报错，只是收不到）。
- 每个实例有独立 `CallbackGroup`（`automatically_add_to_executor_with_node=false`）+ 自带
  `SingleThreadedExecutor`，`tick()` 内做 `sub_instance_->callback_group_executor.spin_some()`
  → **这类节点不依赖主程序 `executor.add_node`，也不需要外部 spin**。
- `tick()` 强制 `onTick()` 只能返回 `SUCCESS`/`FAILURE`（返回 RUNNING 会 `throw std::logic_error`）。
- 语义：`last_msg` 是“自上次 tick 以来的新消息”，无新消息为 `nullptr`；**无消息也返回 SUCCESS**（只有
  `IsSemantic` 是 FAILURE），且无消息时不写输出端口（输出保持上一次的值）。

---

## 1. SubAllRobotHP

- 文件：`plugins/action/sub_all_robot_hp.cpp`、`include/.../action/sub_all_robot_hp.hpp`
- C++ 类 / 基类：`rm_behavior_tree::SubAllRobotHPAction` : `BT::RosTopicSubNode<robot_msgs::msg::AllRobotHP>`
- 注册：✅ `RegisterRosNode`（在 `msg_update_plugin_libs` 里，库 `sub_all_robot_hp`，`params_update_msg`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填**（声明无默认，params 也无默认 → 漏写抛 logic_error） |
| `robot_hp` | out | `robot_msgs::msg::AllRobotHP` | 无（`setOutput` 未给输出端口时静默丢弃） |

运行期行为：`onTick` 有 `last_msg` 才 `setOutput("robot_hp", *last_msg)`；**始终返回 SUCCESS**。

测试要点：
- 正例：`ros2 topic pub -r 10 --qos-durability transient_local --qos-reliability reliable /robot_hp robot_msgs/msg/AllRobotHP "{red_1_robot_hp: 200, ...}"`，之后同 tick 内下游 `IsOutpostOK` 等能读到；`robot_hp` 端口要写 `robot_hp="{robot_hp}"` 才会出现在黑板。
- 负例：不发消息 → 仍 SUCCESS，但 `{robot_hp}` 黑板键**根本不存在**（不是零值），下游 `getInput` 失败 → `IsOutpostOK/IsEnemyOutpostOK/IsBaseAttacked` 返回 FAILURE、`IsBaseStatusOK` 返回 SUCCESS（缺输入即 SUCCESS）。
- 订阅在**构造期**建立（`topic_name` 是字面量时）；若把 `topic_name` 写成 `{key}`，则首次 tick 才建订阅，且 `createSubscriber` 二次调用会 `throw RuntimeError`。

坑：
- `robot_msgs::msg::AllRobotHP` 与 `rm_decision_interfaces::msg::AllRobotHP` 是**两个不同类型**（前者多
  `enemy_outpost_hp/outpost_hp/base_hp/team_color/enemy_color` 字段，后者没有 `base_hp`）。本节点写的是
  `robot_msgs` 版本，`IsFriendOK` 读的却是 `rm_decision_interfaces` 版本 → 两节点连不起来（见 §22）。
- 文档说“`topic_name` 在 XML 中给静态话题名（如 `/game_status`）”，源码更严格：**不给就加载失败**。

---

## 2. SubRobotStatus

- 文件：`plugins/action/sub_robot_status.cpp` + 同名 hpp
- C++ 类 / 基类：`SubRobotStatusAction` : `BT::RosTopicSubNode<robot_msgs::msg::RobotStatus>`
- 注册：✅ `RegisterRosNode`（`msg_update_plugin_libs`，库 `sub_robot_status`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `robot_status` | out | `robot_msgs::msg::RobotStatus` | 无 |

运行期行为：有新消息才写 `robot_status`（`current_hp`、`shooter_heat`、`shooter_barrel_heat_limit`、`robot_id`、`team_color`、`is_attacked`）；始终 SUCCESS。

测试要点：喂 `/robot_status`（`current_hp`、`shooter_heat` 为 `uint16`），下游接 `IsStatusOK message="{robot_status}"` 验证三个阈值的正负例；下游接 `IsAttacked message="{robot_status}"` 时注意它只认 HP 下降沿。

坑：`robot_msgs::msg::RobotStatus` 与 `rm_decision_interfaces::msg::RobotStatus` 字段不同（后者没有
`shooter_barrel_heat_limit`），`IsStatusOK` 读的是 `robot_msgs` 版本，一致；但别拿旧包的 `rm_decision` 版本类型去填黑板。

---

## 3. SubGameStatus

- 文件：`plugins/action/sub_game_status.cpp` + 同名 hpp
- C++ 类 / 基类：`SubGameStatusAction` : `BT::RosTopicSubNode<robot_msgs::msg::GameStatus>`
- 注册：✅ `RegisterRosNode`（`msg_update_plugin_libs`，库 `sub_game_status`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `game_status` | out | `robot_msgs::msg::GameStatus` | 无 |

运行期行为：有新消息才写 `game_status`；始终 SUCCESS。

测试要点：`ros2 topic pub --qos-durability transient_local --qos-reliability reliable /game_status robot_msgs/msg/GameStatus "{game_progress: 4, stage_remain_time: 200}"`；下游 `IsGameTime message="{game_status}" game_progress="4" ...` 与 `IsTimeToBullet message="{game_status}" ...`。
负例：不发 → `{game_status}` 不存在 → `IsGameTime` 切到自己的裸订阅（拿不到就 RUNNING），`IsTimeToBullet` 直接 FAILURE。

坑：文件内注释说“基类会根据端口 topic_name（默认取自节点参数中的默认话题）创建订阅”
（`sub_game_status.cpp:9-10`），但 `params_update_msg` 没有默认话题 → 不写 `topic_name` 就抛异常。**文档未提这点**。

---

## 4. SubAutoAimStatus

- 文件：`plugins/action/sub_auto_aim_status.cpp` + 同名 hpp
- C++ 类 / 基类：`SubAutoAimStatusAction` : `BT::RosTopicSubNode<robot_msgs::msg::AutoAimStatus>`
- 注册：✅ `RegisterRosNode`（`msg_update_plugin_libs`，库 `sub_auto_aim_status`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `auto_aim_status` | out | `robot_msgs::msg::AutoAimStatus` | 无 |

运行期行为：有新消息才写；始终 SUCCESS。消息字段 `uint8 auto_aim_status`（0=正常，1=不正常）。

测试要点：与 `IsAutoAimOK message="{auto_aim_status}" target_status="0"` 配合；注意 `IsAutoAimOK` 还有自己的裸订阅 `/auto_aim_status`（硬编码），两条数据源都在时 **message 端口优先**。

---

## 5. SubChassisEnergy

- 文件：`plugins/action/sub_chassis_energy.cpp` + 同名 hpp
- C++ 类 / 基类：`SubChassisEnergyAction` : `BT::RosTopicSubNode<robot_msgs::msg::ChassisEnergy>`
- 注册：✅ `RegisterRosNode`（`msg_update_plugin_libs`，库 `sub_chassis_energy`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `chassis_energy` | out | `robot_msgs::msg::ChassisEnergy` | 无 |

运行期行为：有新消息才写；始终 SUCCESS。

坑/不一致：消息定义 `uint16 chassis_energy # 底盘剩余能量 (J)`（`robot_msgs/msg/ChassisEnergy.msg`），
但 `IsChassisEnergyOK` 把它当**布尔flag**（`==1` 判“能量不足”）。测试喂 `1` 会得到“能量不足”，喂真实能量值（如 60）会得到“能量 OK”。

---

## 6. SubRuneStatus

- 文件：`plugins/action/sub_rune_status.cpp` + 同名 hpp
- C++ 类 / 基类：`SubRuneStatusAction` : `BT::RosTopicSubNode<robot_msgs::msg::RuneStatus>`
- 注册：✅ `RegisterRosNode`（`msg_update_plugin_libs`，库 `sub_rune_status`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `rune_status` | out | `robot_msgs::msg::RuneStatus` | 无 |

运行期行为：有新消息才写；始终 SUCCESS。字段 `uint8 rune_count`（0/1/2）。

坑：头文件注释写“输入端口 topic_name: 要订阅的话题名（默认 /rune_status）”（`sub_rune_status.hpp:13`），
源码**没有默认值**，不写就抛异常。

---

## 7. SubRemoteStatus

- 文件：`plugins/action/sub_remote_status.cpp` + 同名 hpp
- C++ 类 / 基类：`SubRemoteStatusAction` : `BT::RosTopicSubNode<robot_msgs::msg::RemoteStatus>`
- 注册：✅ `RegisterRosNode`（`msg_update_plugin_libs`，库 `sub_remote_status`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `remote_status` | out | `robot_msgs::msg::RemoteStatus` | 无 |

运行期行为：有新消息才写；始终 SUCCESS。字段 `uint8 remote_control_status`（1 正常 / 2 保守 / 3 激进）。

测试要点：与 `IsRemoteStatus message="{remote_status}" target_status="1|2|3"` 配合；`IsRemoteStatus` 另有硬编码裸订阅 `/remote_status`。

---

## 8. SubDecisionNum

- 文件：`plugins/action/sub_decision_num.cpp` + 同名 hpp
- C++ 类 / 基类：`SubDecisionNumAction` : `BT::RosTopicSubNode<rm_decision_interfaces::msg::DecisionNum>`
- 注册：✅ `RegisterRosNode`（`msg_update_plugin_libs`，库 `sub_decision_num`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `decision_num` | out | `rm_decision_interfaces::msg::DecisionNum` | 无 |

运行期行为：有新消息才写；始终 SUCCESS。字段 `uint8 decision_num`。

测试要点：`DecisionSwitch` / `ActionSelector` 的 `decision_num` 端口读的就是这个类型（`rm_decision_interfaces`，与 `robot_msgs` 无同名类型，不会混淆）。拨杆值直接当动作号用。

坑：`ActionSelector` 里 `decision_num` 是**可选**输入端口（无默认值 → 不给就 getInput 失败），此时只能靠 `action_id` 驱动；两个都没有 → FAILURE。

---

## 9. SubTargetPos（自带订阅 + TF，`SyncActionNode`）

- 文件：`plugins/action/sub_target_pos.cpp`、`include/.../action/sub_target_pos.hpp`
- C++ 类 / 基类：`SubTargetPosAction` : `BT::SyncActionNode`（**不是** `RosTopicSubNode`）
- 注册：✅ `RegisterRosNode`（`msg_update_plugin_libs`，库 `sub_target_pos`，`params_update_msg`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | `auto_aim_target_pos` |
| `target_frame` | in | `std::string` | `map` |
| `source_frame` | in | `std::string` | `camera_init` |
| `armors` | out | `auto_aim_interfaces::msg::Armors` | 无 |

运行期行为：
- 数据源是 `std_msgs::msg::String`，格式 **`"x,y,z,w"`（逗号分隔，≥4 个数）**；解析失败整条丢弃。
- first tick 才建订阅（`rclcpp::QoS(10).reliable()`，**非 latched**），`node_ = params_update_msg.nh`（该节点在 executor 名单里，回调会被处理）。
- `(x,y)≈0` → 视为敌人消失：清空 `raw_armor_` 与 `last_armors_.armors`，并 **FAILURE**。
- 10 s 无消息 → 清空 `armors`。
- 有原始点 → `lookupTransform(target_frame, source_frame, Time(0), 0.5s)` 变换；TF 失败时**沿用上一帧有效点**；若没有历史有效点，则输出 (0,0,0) 的 armor 但**仍返回 SUCCESS**（`sub_target_pos.cpp:261-299`）。
- 返回：第 246 行分支恒 FAILURE；正常路径 `subscription_created_ ? SUCCESS : FAILURE`（首 tick 建完订阅后即真）。

测试要点：
- 正例：`ros2 topic pub -r 20 /auto_aim_target_pos std_msgs/msg/String "{data: '3.0,2.0,0.2,1.0'}"`，并且 `map<-camera_init`（或 XML 里改 `source_frame`）的 TF 存在（静态 TF 即可）→ 输出 armor 的 map 坐标。
- 负例：`data: '0,0,0,1'` → FAILURE 且 armors 为空（`IsDetectEnemy`、`PursueGoal` 会随之为负）。
- 负例：把 TF 删掉 → 若从未成功过，得到 `(0,0,0)` 的 armor + SUCCESS（**“看似成功其实无效”**）；若有历史，得到旧点（会追着一个静止幽灵点跑）。

坑：
1. 头文件写 `source_frame` 默认 `camera_init`，构造函数里却把成员设成 `"chassis"`（`sub_target_pos.cpp:23`）——
   构造期默认是 `chassis`，但因为端口有默认值 `camera_init`，tick 里必然被覆盖成 `camera_init`，那个 `chassis` 是死代码。
2. `parseTargetPos` 只支持逗号；若发布端发分号/空格 → 全部丢弃，表现为“永远没有敌人”。
3. 输出端口 `armors` 不在 XML 里声明 → 写入被 `setOutput` 静默丢弃（返回的 `Result` 没人检查）。
4. 该节点是**裸订阅**，靠 `params_update_msg.nh` 被主 executor spin；测试若自己 new 一个节点承载、又没 spin，永远收不到数据。

---

## 10. SubGimbalTarget（自带订阅 + 静态 TF 广播，`SyncActionNode`）

- 文件：`plugins/action/sub_gimbal_target.cpp`、`include/.../action/sub_gimbal_target.hpp`
- C++ 类 / 基类：`SubGimbalTargetAction` : `BT::SyncActionNode`
- 注册：✅ `RegisterRosNode`（`msg_update_plugin_libs`，库 `sub_gimbal_target`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | `gimbal_target_point` |
| `gimbal_frame` | in | `std::string` | `gimbal_field` |
| `gimbal_origin_x` | in | `double` | `0.0` |
| `gimbal_origin_y` | in | `double` | `0.0` |
| `gimbal_origin_z` | in | `double` | `0.0` |
| `gimbal_yaw` | in | `double` | `0.0` |
| `target_pose` | out | `std::string`（格式 `x;y;z;qx;qy;qz;qw`，6 位小数） | 无 |
| `target_valid` | out | `bool` | 无 |
| `target_x` | out | `double`（gimbal 系原始值，无效时 0.0） | 无 |
| `target_y` | out | `double`（同上） | 无 |

运行期行为：
- 所有端口（含 `topic_name`）都在 **tick 里** 读，所以字面量/黑板都能改；但 `sub_` 只在首次 tick 创建一次，之后再改 `topic_name` 无效。
- `target_frame_` **硬编码 `"map"`**，没有端口。
- 首个 tick 广播静态 TF `map -> gimbal_field`（平移 `gimbal_origin_*`、yaw `gimbal_yaw_`），只发一次。
- 收到 `GimbalTarget`（`float32 x,y`）且非 (0,0) → `has_valid_target_=true`；收到 (0,0) → 置 false。
- tick：`has_valid_target_` 时做 TF 变换 → 成功则输出 pose 字符串；变换失败则 `has_valid_target_=false` 并输出空串。
- 返回：`subscription_created_` 为真（首 tick 建完订阅后恒真）→ **SUCCESS**；否则 FAILURE。

测试要点：
- 正例：`ros2 topic pub /gimbal_target_point robot_msgs/msg/GimbalTarget "{x: 3.0, y: 4.0}"`（QoS reliable，非 latched 也行），且静态 TF 已被该节点自己发布 → `target_pose` 出现 `;` 分隔 7 个数。
- 负例：只发 `{x: 0, y: 0}` → `target_valid=false`、`target_pose=""`、`target_x/y=0`；下游 `IsGimbalTargetValid`、`IsGimbalTargetInZone` 应为 FAILURE。
- **顺序坑**：首个有效目标到达的那个 tick，`publishStaticTf()` 刚发出 `map->gimbal_field`，同 tick 的
  `transformPointToMap()` 很可能还查不到这个 TF（TransformListener 需要至少一次 spin/回调）→
  该目标被丢、`has_valid_target_` 被置回 false。测试必须**连发多帧**并等待至少 1 个 tick 才看到 `target_valid=true`。
- 负例：把 `gimbal_frame` 改成与静态 TF 不一致的名字（不可能，因为 TF 的 child 就用它）——真正能做负例的是删掉 `map` 系（`target_frame_` 硬编码），或让 TF 超时 0.5 s 失败。

坑：
1. `target_frame_` 硬编码 `"map"`，文档也没提；想让云台系变成别的父系做不到。
2. `has_valid_target_` 一旦因 TF 失败被置 false，**只有新消息**才能恢复（不会自愈）。
3. 输出端口 4 个，全部要写入 XML 才能进黑板。
4. 该节点是裸订阅（`params_update_msg.nh`，在 executor 里）+ `tf2_ros::TransformListener`（同节点）→ 依赖主 executor spin。
5. 文档说“订阅创建成功即返回 SUCCESS，否则 FAILURE”，与源码一致；但文档没说首帧会被 TF 竞态吃掉。

---
# 二、发布类动作节点（`BT::RosTopicPubNode<T>`，7 个）

公共事实（源码：`bt_topic_pub_node.hpp`）：

- 基类同样是 **`BT::ConditionNode`**；`tick()` 每次调用 `setMessage(msg)` → `publisher_->publish(msg)` → 返回 `SUCCESS`；
  `setMessage` 返回 false 时返回 `FAILURE`（**本工程 7 个节点里只有 `PubRobotState`/`PubChassisMode`/`PubAutoAimMode`/
  `PubSuperCapacitor` 会在缺端口时返回 false，其余都恒 true**）。
- 发布端 QoS 是 `create_publisher<T>(topic, 1)`（`bt_topic_pub_node.hpp:161`）= **KeepLast(1)、volatile、reliable**。
- `topic_name` 端口来自 `providedBasicPorts(...)`，默认字符串是 `"__default__placeholder__"`。
  **BT.CPP 在解析 XML 时会把端口默认值写进 `NodeConfig.input_ports`**（`behaviortree_cpp/src/xml_parsing.cpp:818-836`），
  所以 XML 省略 `topic_name` 时构造函数看到的是占位符 → 走 `params.default_port_value` 分支。
- `RobotControl` 是唯一**没有** `topic_name` 端口的发布节点（它重写 `providedPorts()` 时没调 `providedBasicPorts`）。

关键默认值（`rm_behavior_tree.cpp`）：

| 注册参数 | `default_port_value` | 影响的节点 |
| --- | --- | --- |
| `params_robot_state` | `robot_state` | `PubRobotState` ✅ 可以不写 `topic_name` |
| `params_robot_control` | `robot_control` | `RobotControl`（无该端口） |
| `params_pub_default` | **空** | `PubAutoAimMode`、`PubChassisMode`、`PubSuperCapacitor`、`PubEnemyStatus`、`PubAutoaimTarget` → **必须写 `topic_name`，否则 `createTreeFromFile` 抛 `std::logic_error`** |

---

## 11. PubRobotState

- 文件：`plugins/action/pub_robot_state.cpp`、`include/.../action/pub_robot_state.hpp`
- C++ 类 / 基类：`PubRobotStateAction` : `BT::RosTopicPubNode<robot_msgs::msg::RobotState>`
- 注册：✅ `RegisterRosNode`（`params_robot_state`，库 `pub_robot_state`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | `robot_state`（来自 `RosNodeParams`，**可省略**） |
| `robot_state` | in | `int`（写入 `int8 state`） | `1` |

运行期行为：每 tick 读 `robot_state`，`msg.state = *state`；读不到 → `setMessage` 返回 false → **FAILURE**（端口有默认值 1，实际不会发生）；否则 SUCCESS。`RobotState.msg`：1=移动 2=防御 3=攻击 4=待机（**代码注释只写了 1/2/3，漏了 4**）。

测试要点：订阅 `robot_state`（话题名）看每 tick 都收到一条；`robot_state="3"` → `state=3`；`robot_state="300"` → `int8` 溢出（未截断检查）。

---

## 12. PubChassisMode

- 文件：`plugins/action/pub_chassis_mode.cpp` + 同名 hpp
- C++ 类 / 基类：`PubChassisModeAction` : `BT::RosTopicPubNode<robot_msgs::msg::ChassisMode>`
- 注册：✅ `RegisterRosNode`（`params_pub_default`，库 `pub_chassis_mode`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填**（占位符 + params 无默认 → 抛异常） |
| `chassis_mode` | in | `int`（写入 `uint8 mode`） | `0` |

运行期行为：每 tick 发布 `mode`；缺 `chassis_mode` → FAILURE（有默认值，实际不会发生）；否则 SUCCESS。注释：0 正常 / 1 加速旋转 / 2 过起伏 / 3 过盲道前跟随 / 4 回盲道前跟随（`ChassisMode.msg` 只定义了 0/1/2）。

坑：**端口注释里 3/4 在 `ChassisMode.msg` 中没有定义**（msg 注释只有 0/1/2），电控侧行为未知 → 测试 3/4 只能验证“发出去了”。

---

## 13. PubAutoAimMode

- 文件：`plugins/action/pub_auto_aim_mode.cpp` + 同名 hpp
- C++ 类 / 基类：`PubAutoAimModeAction` : `BT::RosTopicPubNode<robot_msgs::msg::AutoAimMode>`
- 注册：✅ `RegisterRosNode`（`params_pub_default`，库 `pub_auto_aim_mode`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `key_name` | in | `int` | `0`（**死端口**，见下） |
| `auto_aim_mode` | in | `int`（写入 `uint8 mode`） | `0` |

运行期行为：`setMessage` 先 `getInput<int>("auto_aim_mode")`，失败才回退 `key_name`；两者都没有 → ERROR 日志 + false → FAILURE。否则每 tick 发布并 SUCCESS。

坑（文档也提到，已核对源码一致）：`auto_aim_mode` 有默认值 0，`getInput` 永远成功 → **`key_name` 分支是死代码**。
`<PubAutoAimMode key_name="3"/>` 实际发 `mode=0`。测试必须用 `auto_aim_mode`。

---

## 14. PubSuperCapacitor

- 文件：`plugins/action/pub_super_capacitor.cpp` + 同名 hpp
- C++ 类 / 基类：`PubSuperCapacitorAction` : `BT::RosTopicPubNode<robot_msgs::msg::SuperCapacitor>`
- 注册：✅ `RegisterRosNode`（`params_pub_default`，库 `pub_super_capacitor`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `super_capacitor_state` | in | `int`（写入 `uint8 state`） | `0` |

运行期行为：每 tick 发布 `state`（0=关，1=开）；缺端口 → FAILURE（有默认值）。

---

## 15. PubEnemyStatus

- 文件：`plugins/action/pub_enemy_status.cpp` + 同名 hpp
- C++ 类 / 基类：`PubEnemyStatusAction` : `BT::RosTopicPubNode<sp_msgs::msg::EnemyStatusMsg>`
- 注册：✅ `RegisterRosNode`（`params_pub_default`，库 `pub_enemy_status`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `invincible_enemy_ids` | in | `std::string`（解析成 `int8[]`） | `""` |

运行期行为：先按 `std::string` 读端口，用 `parseIdList` 解析（去掉 `[`/`]`/空格后按逗号切分，失败 token 忽略）；
字符串端口取不到才尝试 `std::vector<int8_t>`（本仓库没注册该转换器，会打一条 `convertFromString` 错误）；
最后 `msg.timestamp = node_->now()`，每 tick 发布 → SUCCESS。

测试要点：`invincible_enemy_ids="1,3,5"` → `[1,3,5]`；`""` → 空数组；`"[1, 3]"` → `[1,3]`；`"a,b"` → 空数组（token 全被忽略，只有日志没有报错）；`"300"` → `static_cast<int8_t>` 溢出成 44。

坑：`parseIdList` 在 `pub_enemy_status.cpp` 与 `pub_autoaim_target.cpp` 里各定义一份（**同名字、外部链接、定义在
`namespace rm_behavior_tree` 内**），两个库分开编译所以不冲突；但若将来把两个 cpp 编进同一个库会 ODR 冲突。

---

## 16. PubAutoaimTarget

- 文件：`plugins/action/pub_autoaim_target.cpp` + 同名 hpp
- C++ 类 / 基类：`PubAutoaimTargetAction` : `BT::RosTopicPubNode<sp_msgs::msg::AutoaimTargetMsg>`
- 注册：✅ `RegisterRosNode`（`params_pub_default`，库 `pub_autoaim_target`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `target_ids` | in | `std::string`（解析成 `int8[]`） | `""` |

运行期行为：同 §15，字段是 `target_ids`，`timestamp = node_->now()`；每 tick 发布 → SUCCESS。

---

## 17. RobotControl

- 文件：`plugins/action/robot_control.cpp`、`include/.../action/robot_control.hpp`
- C++ 类 / 基类：`RobotControlAction` : `BT::RosTopicPubNode<rm_decision_interfaces::msg::RobotControl>`
- 注册：✅ `RegisterRosNode`（`params_robot_control`，库 `robot_control`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `stop_gimbal_scan` | in | `bool` | **必填**（声明无默认；但读不到也不报错，见下） |
| `chassis_spin_vel` | in | `float` | **必填**（同上） |
| ~~`topic_name`~~ | — | — | **没有这个端口**，话题固定为 `robot_control`（来自 `RosNodeParams`） |

运行期行为：`setMessage` 里 `getInput("stop_gimbal_scan", msg.stop_gimbal_scan)` / `getInput("chassis_spin_vel", msg.chassis_spin_vel)`，
**两个返回值都丢弃**，然后 `return true` → 恒 SUCCESS + 每 tick 发布。

坑（重要）：
1. 缺端口时静默发 `false` / `0.0`，**不报错、不 FAILURE** —— 负例是“什么都没变”而不是“节点失败”。
2. XML 里写 `topic_name="..."` 会被 BT.CPP 当成未知端口 → **抛异常 `unknown port`**（端口清单里没有它）。
   文档这一点是对的（“XML 中写 topic_name 无效”）。
3. `chassis_spin_vel` 是 `float` 端口，XML 传 `1.5` 会走 `convertFromString<float>`；传 `1.234567890123` 会丢精度。

---

# 三、导航 / 运动 / 查询类动作节点

## 18. SendGoal

- 文件：`plugins/action/send_goal.cpp`、`include/.../action/send_goal.hpp`
- C++ 类 / 基类：`SendGoalAction` : `BT::RosActionNode<nav2_msgs::action::NavigateToPose>`
- 注册：✅ `RegisterRosNode`（`params_send_goal`，库 `send_goal`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `action_name` | in | `std::string` | 占位符 → 落到 `params_send_goal.default_port_value` = **`goal_pose`**（见坑 1） |
| `goal_pose` | in | `std::string`，格式 `x;y;z;qx;qy;qz;qw`（只把 `;` 换成空格后 `sscanf` 式读 7 个数，**空格分隔的也接受**） | **必填**（缺失时 `setGoal` 抛 `BT::RuntimeError`） |

基类参数：`server_timeout = 20 s`，`wait_for_server_timeout = 2 s`（`ros_node_params.hpp` 默认值，`rm_behavior_tree.cpp` 未覆盖）。

运行期行为：
1. `setGoal`：解析字符串（`std::replace(';'→' ')` + `ss >>` 7 个 double），`ss.fail()` → 抛 `BT::RuntimeError`；
   `goal.pose.header.frame_id = "map"`（**硬编码**），`stamp = node_->now()`（跟随 `use_sim_time`）；重置 `last_distance_remaining_=DBL_MAX`、`close_count_=0`。
2. 首次 tick 发目标 → RUNNING；构造时 `createClient` 会 `wait_for_action_server(2 s)`，找不到只打 ERROR 不抛。
3. `onFeedback`：
   - `goal_pose` 端口字符串**与发出去的不同**（且非空）→ `halt()` + 返回 FAILURE（上层换新目标重发）；
   - `distance_remaining > 1e10` → RUNNING（异常值保护）；
   - `distance_remaining < 0.2` **连续 5 次** → 返回 SUCCESS（基类随即 `cancelGoal()`）；
   - 否则 RUNNING。
4. `onResultReceived`：`SUCCEEDED` → SUCCESS；`ABORTED`/`CANCELED`/其它 → FAILURE。
5. `onFailure(Nav)`：`SEND_GOAL_TIMEOUT`、`GOAL_REJECTED_BY_SERVER`、`ACTION_ABORTED`、`ACTION_CANCELLED` 等 → FAILURE。
6. 基类兜底：RUNNING 超过 `10 × server_timeout = 200 s` 无结果 → FAILURE（并 cancel）。
7. `onHalt`：只重置 `close_count_`；基类负责 `async_cancel_goal`。

测试要点：
- 正例（不需要真 Nav2）：起一个 dummy `navigate_to_pose` action server，周期性发 feedback `distance_remaining` 从 5.0 → 0.1（≥5 帧）→ 观察节点 SUCCESS 且 `cancel` 被调用；
  或直接让 server `succeed` → SUCCESS。
- 负例：server 回 `abort` → FAILURE；server 不存在 → 构造期 ERROR + tick 后 `SEND_GOAL_TIMEOUT` FAILURE；
  `goal_pose="abc"` → tick 内抛 `RuntimeError`（**主循环没有 try/catch，会终止进程**）。
- 目标抖动测试：RUNNING 期间把 `goal_pose` 端口改值（用 `SetBlackboard` 或 `{变量}`）→ 下一帧 feedback 触发 halt + FAILURE。

坑：
1. **`params_send_goal.default_port_value = "goal_pose"`**（`rm_behavior_tree.cpp:38`）：这是给 `action_name` 用的默认值，
   属于复制粘贴错误。XML 省略 `action_name` 时会去连名为 `goal_pose` 的 action server → 2 s 后 ERROR，之后必失败。
   **每棵测试树的 `SendGoal` 都必须显式写 `action_name="navigate_to_pose"`。** 文档已提示，源码确认。
2. `frame_id` 硬编码 `"map"`，无法发 `odom` 目标。
3. 到达判定看 feedback 而不是 result（与 Nav2 语义不完全一致：如果 server 只发一次 0.05 的 feedback 就成功，
   会走 `onResultReceived` SUCCESS；如果 server 一直不发 feedback，则永远 RUNNING 直到 200 s 超时）。
4. 文档说“`onFeedback` 里若发现 `goal_pose` 端口值变了 → `halt()` 并返回 FAILURE” ✔；文档说“到达判定不看 result”
   ✘ —— 源码里 `SUCCEEDED` 的 result 也会直接 SUCCESS（两者都成立，文档漏了 result 分支）。

---

## 19. MoveAround

- 文件：`plugins/action/move_around.cpp`、`include/.../action/move_around.hpp`
- C++ 类 / 基类：`MoveAroundAction` : `BT::SyncActionNode`
- 注册：✅ `RegisterRosNode`（`params_move_around` = `params_update_msg`，库 `move_around`；链接 `map_accessor/key_point_loader/visibility_checker`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `geometry_msgs::msg::TransformStamped` | **必填**（来自 `GetCurrentLocation`） |
| `expected_dis` | in | `double` | `0.3`（构造期读） |
| `map_topic` | in | `std::string` | `/map`（构造期读） |
| `goal_clearance` | in | `double` | `0.0`（构造期读） |
| `max_resample` | in | `int` | `20`（构造期读，`<1` 强制 1） |
| `allow_unknown` | in | `bool` | `false`（构造期读） |
| `goal_pose` | out | `std::string`，`"x;y;z;0;0;0;1"`，fixed 4 位小数 | 无 |

运行期行为（`tick` 是纯函数式的一次采样）：
1. 读 `message`；缺失 → ERROR + **FAILURE**（注释明确“绝不能返回 RUNNING”）。
2. 没有地图（`latest_map_` 空）→ WARN + **FAILURE**（保守：校验不了就不发点）。
3. 在 `[0,2π)` 均匀采样角度，落点 = 圆心 + `expected_dis` 方向向量；最多 `max_resample` 次，每次三级校验：
   ① 在 `info.origin` 之上且在栅格范围内；② 占用值 `> 50` 判障碍、`< 0` 且 `allow_unknown=false` 判未知；
   ③ `VisibilityChecker::isVisible(from, to, map, clearance_cells)`，`clearance_cells = goal_clearance/resolution`。
4. 命中 → `setOutput("goal_pose", ...)` + **SUCCESS**；`max_resample` 次都失败 → WARN + **FAILURE**。

测试要点：
- 正例：`ros2 topic pub --qos-durability transient_local --qos-reliability reliable /map nav_msgs/msg/OccupancyGrid ...`（**必须 latched**，
  构造期订阅用 `transient_local+reliable`；由于它在构造期就订阅，普通 volatile 发布者也能连上，但**订阅时刻早**，所以
  “先起树、后发地图”仍能收到最后一条，前提是发布者用 transient_local）；`message="{current_location}"` 由一个
  `GetCurrentLocation` 或 `SetBlackboard` 提供。
- 负例：不发 `/map` → 恒 FAILURE（并且每 tick 打 WARN）；`message` 缺失 → FAILURE；把机器人放在障碍里/图外 → FAILURE。
- **构造期端口坑**：`expected_dis` / `map_topic` / `goal_clearance` / `max_resample` / `allow_unknown` 都在构造函数
  （`move_around.cpp:19-40`）读一次，XML 里写成 `{变量}` 时构造期黑板可能还没有值 → 静默落回成员默认值，
  运行期改黑板**不会**生效。测试必须用字面量。

坑：
1. 构造期 `initMapSubscription()`，`node_ = params_update_msg.nh`（在 executor 名单里）→ 订阅回调能被处理；
   测试若拿别的节点承载必须自己 spin。
2. `isPointUsable` 在持锁（`map_mutex_`）状态下调用 `VisibilityChecker`，地图越大 tick 越慢（不是 bug，但会影响 10 ms 循环频率）。
3. **SUCCESS ≠ 走到了**：它只表示撒出了一个合法点，必须配 `SendGoal`。
4. `formatGoalPose` 固定 `0;0;0;1` 姿态，朝向不受控。

---

## 20. PursueGoal

- 文件：`plugins/action/pursue_goal.cpp`、`include/.../action/pursue_goal.hpp`
- C++ 类 / 基类：`PursueGoalAction` : `BT::SyncActionNode`
- 注册：✅ `RegisterRosNode`（`params_pursue_goal` = `params_update_msg`，库 `pursue_goal`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `armors` | in | `auto_aim_interfaces::msg::Armors` | **必填**（运行期读） |
| `current_location` | in | `geometry_msgs::msg::TransformStamped` | **必填**（运行期读） |
| `pursue_radius` | in | `double` | `2.0`（**构造期读**） |
| `update_interval` | in | `double` | `0.5`（**构造期读**） |
| `min_goal_change` | in | `double` | `0.3`（**构造期读**） |
| `target_frame` | in | `std::string` | `map`（**构造期读**，只用于 `PoseStamped.header.frame_id`，不参与变换） |
| `map_topic` | in | `std::string` | `/map`（**构造期读**） |
| `wall_check_clearance` | in | `double` | `0.0`（**构造期读**；`>0` 才订阅地图） |
| `goal_pose` | out | `std::string`，4 位小数 `x;y;z;qx;qy;qz;qw` | 无 |
| `enemy_distance` | out | `double` | 无 |

运行期行为：
1. `armors` 缺失或 `armors.armors` 为空 → 3 s 节流 WARN + `resetSession()` + **FAILURE**。
2. `current_location` 缺失 → WARN + **FAILURE**（不清 session）。
3. `getEnemyPosition`：跳过三轴都为 0 的装甲板；全无效 → 返回 `NaN` 点 → **FAILURE** + `resetSession()`。
4. 3 帧滑窗均值（`kEnemyHistorySize=3`）→ `filtered`；`setOutput("enemy_distance", hypot(enemy-robot))`。
5. `wall_check_clearance_>0` 且 `hasObstacleBetween(robot, 原始 enemy)` → **FAILURE**（注意用的是**未滤波**点；
   `/map` 还没到时**放行**并每 5 s WARN）。
6. 节流：`has_last_goal_ && has_last_update_ && update_interval_>0` 且距上次重算 `< update_interval_` → **SUCCESS**（沿用旧 `goal_pose` 字符串）。
7. 计算新点（`generatePursuePose`：敌人→自身连线上距敌人 `pursue_radius`，z 用自身 z，yaw 朝向敌人）。
8. 与上次输出点比较：`jump < min_goal_change_` → **SUCCESS 但不改写 `goal_pose`**（防 `SendGoal` 抖动）。
9. 否则 `setOutput("goal_pose", ...)` → **SUCCESS**。

测试要点：
- 正例：`armors` 里放一个非零点（`5,2,0.2`），`current_location` 给 `(2,2,0.2)`，`pursue_radius="2.5"` → 首帧输出
  `2.5000;2.0000;0.2000;0.0000;0.0000;1.0000;0.0000`。
  推导：`dx = robot-enemy = -3`、`dy = 0` → 落点 `enemy + (dx,dy)*2.5/3 = (2.5, 2)`；
  `yaw = atan2(dy,dx) = π` → `qz=sin(π/2)=1, qw=cos(π/2)=0`（与文档给的第二组实测值一致）。
  ⚠ **这个 yaw 是「敌人→自身」的方向，从落点看是背对敌人的（差 180°）**，见下方坑 5。
- 负例：`armors` 为空 → FAILURE 且 `enemy_distance`/`goal_pose` 保留旧值（**不清空**，下游 `SendGoal` 可能继续跑旧目标——
  这是最容易写出“明明没敌人还在走”的原因）；`armors=[0,0,0]` → FAILURE；`current_location` 缺失 → FAILURE。
- 负例：`wall_check_clearance="0.3"` + `/map` 中间有障碍 → FAILURE；不发布 `/map` → 放行（假正例）。
- 防抖：敌人在 0.1 m 内抖动 → `goal_pose` 字符串不变；一次移动 1.0 m → 才改。
- **必须放 `ReactiveSequence`**（头文件注释也这么写）：普通 `Sequence` 里它 SUCCESS 后不会再被 tick，追不上移动目标。

坑：
1. `target_frame` 只写进 `header.frame_id`，**不做任何 TF 变换**；如果 `armors` 是 `odom` 系而 `target_frame="map"`，输出点坐标是错的但不报错。
2. 所有数值端口构造期读取（`pursue_goal.cpp:18-23`），XML 写字面量才可靠。
3. `enemy_distance` 用**原始**最近点算，`goal_pose` 用**滤波后**点算 —— 两者不同步。
4. `resetSession()` 清 `enemy_pos_history_`/`last_goal_*`，但**不写空字符串到 `goal_pose`**。
5. **输出姿态的 yaw 方向可疑**：`generatePursuePose` 里 `yaw = atan2(robot-enemy)`，得到的是“敌人→自身”的方向，
   从追击点看等于**背对敌人 180°**（`pursue_enemy.cpp` 里旧节点用同样的公式，属同源写法）。若 Nav2 的
   `NavigateToPose` 会按目标 yaw 调整终态朝向，实车会“倒着贴近”。测试断言 `goal_pose` 的 qz/qw 时按源码公式算，别按“朝向敌人”的直觉。
6. `SendGoal` 判定“`goal_pose` 端口变化就重新发目标”，而 `PursueGoal` 的 `update_interval` 节流会返回 SUCCESS 但**不更新**输出 →
   这正是设计意图（不抖动），但意味着“敌人静止时不会重发目标”，若导航失败要先让点变化才能触发重发。

---
## 21. PursueEnemy

- 文件：`plugins/action/pursue_enemy.cpp`、`include/.../action/pursue_enemy.hpp`（另有 `plugins/action/pursue_enemy_analysis.md` 设计说明）
- C++ 类 / 基类：`PursueEnemyAction` : `BT::StatefulActionNode`
- 注册：✅ `RegisterRosNode`（`params_pursue_enemy`，库 `pursue_enemy`；`nh` 是**独立节点 `pursue_enemy`**）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `armors` | in | `auto_aim_interfaces::msg::Armors` | **必填**（`onStart`/`onRunning` 读） |
| `current_location` | in | `geometry_msgs::msg::TransformStamped` | **必填** |
| `pursue_radius` | in | `double` | `2.0`（`onStart` 读） |
| `pursue_distance_threshold` | in | `double` | `0.5`（`onStart` 读，但**从不使用**） |
| `max_pursuit_points` | in | `int` | `10`（`onStart` 读） |
| `update_interval` | in | `double` | `0.5`（`onStart` 读；**单位被当 ms 用**，见坑 3） |
| `enemy_timeout` | in | `double` | `3.0`（`onStart` 读，**随后被硬编码 10.0 覆盖**） |
| `action_name` | in | `std::string` | `navigate_to_pose`（`onStart` 读，`create_client` 用） |
| `target_frame` | in | `std::string` | `map`（`onStart` 读） |
| `robot_base_frame` | in | `std::string` | `gimbal_yaw`（`onStart` 读，**从不使用**） |
| `map_topic` | in | `std::string` | `/map`（`onStart` 读） |
| `wall_check_clearance` | in | `double` | `0.0`（`onStart` 读；`>0` 才订阅地图） |
| 输出端口 | — | — | **无**（全部副作用是发 action / 取消 action） |

运行期行为：
- `onStart`：读全部端口 → `armors` 空 → FAILURE；`current_location` 缺 → FAILURE；`enemy_timeout_ = 10.0`（覆盖端口）；
  可选建 `/map` 订阅（**`rclcpp::QoS(5).best_effort()`**）+ 贴墙检查 → 被挡 FAILURE；`create_client` 一次；
  计算点 → `async_send_goal` → **RUNNING**。
- `onRunning`：
  - `current_location` 缺 → 取消并 FAILURE；
  - 敌人丢失（`armors` 空 / `armors[0]` 三轴为 0 / `now-last_armor_stamp > enemy_timeout_`）→ 取消并 FAILURE；
  - 未到 `update_interval`（且 `wall_check_clearance_<=0`）→ 直接 **RUNNING**；
  - `pursuit_points_sent_ >= max_pursuit_points_` → 取消并 **SUCCESS**（**唯一的 SUCCESS 路径**）；
  - 否则取消旧目标 → 发新点 → **RUNNING**。
- `onHalted`：取消并清标志。

测试要点：
- 正例：喂 `armors=[5,2,0.2]` + `current_location` → 首 tick RUNNING + 收到一个 `navigate_to_pose` goal；
  `max_pursuit_points="1"` 时下一轮 `update_interval` 后立刻 SUCCESS（**最快的正例**）。
- 负例：`armors` 空 → `onStart` 立即 FAILURE；`armors[0]=(0,0,0)` → FAILURE 且 `async_cancel_goal` 被调用（如果 handle 存在）。
- 状态机：StatefulActionNode 只在 `IDLE→RUNNING` 调 `onStart`；`halt()` 后重新 tick 才会再次 `onStart`。

坑（全部已核对源码）：
1. **`enemy_timeout` 端口无效**：`onStart` 第 79-81 行读完，第 105 行 `enemy_timeout_ = 10.0;` 无条件覆盖 → 永远 10 s。
2. **`pursue_distance_threshold` / `robot_base_frame` 是死端口**（只被赋值，从不参与逻辑）。
3. **`update_interval` 单位错**：`onRunning` 里 `elapsed_ms >= update_interval_ * 1000`（毫秒比较），
   而 `elapsed_ms` 是毫秒 —— 数值上等价于“秒”，逻辑刚好成立；但注释说“0 表示每 tick 更新”，实际 0 时
   `elapsed_ms >= 0` 恒真 → 确实每 tick 更新。**能用，但语义脆**。
4. **`nh` 是独立节点 `pursue_enemy`，没有加入主 executor**（`rm_behavior_tree.cpp:149-154` 只加了 `node`、
   `update_msg`、`get_current_location`、`send_goal`、`pub_default`）→ 该节点上的 `/map` 订阅回调、
   action 的 `goal_response_callback`/`result_callback` **都不会被处理** → `goal_handle_` 永远为空 →
   `async_cancel_goal` 全部无效；`arrived_at_enemy_`/`navigation_complete_` 永远是 false。
5. 地图订阅用 `best_effort`（`pursue_enemy.cpp:495`），与 `MoveAround`/`PursueGoal` 的 `transient_local+reliable` 不一致：
   `/map` 若是默认 reliable latched 发布者，best_effort 订阅者仍能连（reliable publisher + best_effort subscriber 兼容），
   但**订阅晚了就收不到历史帧** → “贴墙检查永远放行”。
6. `pursuit_points_sent_` 每次 `onRunning` 发点自增，但 `onStart` 也会置 1 —— 计数语义是“发过几个点”，
   `max_pursuit_points="1"` 时实际第一个点发出后下一轮就 SUCCESS。
7. 文档说的“`arrived_at_enemy_` 未使用、SUCCESS 只有发点用尽” ✔ 一致。

---

## 22. GetCurrentLocation

- 文件：`plugins/action/get_current_location.cpp`、`include/.../action/get_current_location.hpp`
- C++ 类 / 基类：`GetCurrentLocationAction` : `BT::SyncActionNode`
- 注册：✅ `RegisterRosNode`（`params_get_current_location`，库 `get_current_location`，**独立节点 `get_current_location`，在 executor 名单里**）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `current_location` | out | `geometry_msgs::msg::TransformStamped` | 无 |
| `target_frame` | in | `std::string` | `map`（tick 里读） |
| `source_frame` | in | `std::string` | `gimbal_yaw`（tick 里读） |

运行期行为：`tf_buffer_->lookupTransform(target_frame, source_frame, Time(0), 100 ms)`；
成功 → `setOutput("current_location", t)` + **SUCCESS**；`tf2::TransformException` → 3 s 节流 WARN + **FAILURE**。
构造函数里建 `TransformListener`（`node_` 的 clock）。

测试要点：
- 正例：`ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map gimbal_yaw`（或 tf2_ros 的 `static_transform_publisher` 参数形式），
  然后 `GetCurrentLocation` 每 tick SUCCESS，`translation`/`rotation` 是 `map<-gimbal_yaw` 的变换。
- 负例：不发 TF → 100 ms 后 FAILURE（不会抛），下游 `MoveAround`/`PursueGoal` 缺 `message` → FAILURE。
- **注意方向**：`lookupTransform(target, source)` 得到的是“把 source 系数据变到 target 系”的变换，即
  `translation` 是 `gimbal_yaw` 原点在 `map` 中的坐标。`MoveAround`/`PursueGoal` 直接把它的 `translation` 当机器人位置用，语义正确。

坑：
1. 每次 tick 阻塞最多 100 ms（TF 没到时）→ 主循环 10 ms 节奏会被拖慢，`ReactiveSequence` 里放它要小心。
2. `logger_` 用的是 `params.nh->get_logger()`；`tf_buffer_` 用 `node_->get_clock()`，**跟随 `use_sim_time`**。
3. 没有任何 `timeout` 端口（100 ms 硬编码）。
4. 输出端口必须写 `current_location="{current_location}"`，否则写入被静默丢弃。

---

## 23. WaitDuration

- 文件：`plugins/action/wait_duration.cpp`、`include/.../action/wait_duration.hpp`
- C++ 类 / 基类：`WaitDuration` : `BT::StatefulActionNode`
- 注册：✅ `registerFromPlugin`（在 `bt_plugin_libs`，库 `wait_duration`；cpp 内 `BT_REGISTER_NODES` → `"WaitDuration"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `duration_msec` | in | `unsigned` | `1000` |

运行期行为：`onStart` 读端口（覆盖成员默认 1000）；`==0` → **立即 SUCCESS**；否则记 `steady_clock` 起点 → **RUNNING**。
`onRunning`：`elapsed >= duration_msec` → **SUCCESS**，否则 **RUNNING**。`onHalted` 空实现（不重置、不返回 FAILURE）。

测试要点：`duration_msec="0"` → 同一 tick SUCCESS；`"200"` → 首 tick RUNNING，~200 ms 后 SUCCESS；
`{blackboard}` 动态值在**每次 onStart** 时重新读取（可重复使用）；负例：`duration_msec="-5"` 会走 `convertFromString<unsigned>` → 抛异常（字符串转 unsigned 失败）。
被 `ReactiveFallback` halt 后再次进入会重新 `onStart` 计时。

坑：
1. `onHalted` 注释说“Reset state - node will return FAILURE when halted”，**实际什么都没做**（注释误导，halt 不会返回 FAILURE）。
2. 计时用 `steady_clock`，与 `use_sim_time` 无关 → Gazebo 暂停时它照走。

---

## 24. PrintMessage

- 文件：`plugins/action/print_message.cpp`、`include/.../action/print_message.hpp`
- C++ 类 / 基类：`PrintMessageAction` : `BT::SyncActionNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `print_message`；`BT_REGISTER_NODES` → `"PrintMessage"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message_to_print` | in | `std::string` | **必填** |

运行期行为：读端口 → 失败则 `std::cerr` + **FAILURE**；成功则 `std::cout` + `RCLCPP_INFO` + **SUCCESS**。

测试要点：纯调试节点，`message_to_print="hello"` 一次 tick 输出两遍（stdout + ROS 日志）。可用作“树走到这里”的探针。

---

## 25. RMSetBlackboard

- 文件：`plugins/action/set_blackboard.cpp`（实现全在头文件 `include/.../action/set_blackboard.hpp`）
- C++ 类 / 基类：`SetBlackboardAction` : `BT::SyncActionNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs` 里的库名 **`rm_set_blackboard`**，`BT_REGISTER_NODES` → 标签 **`"RMSetBlackboard"`**）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `key` | in | `std::string` | **必填** |
| `value` | in | `bool` | **必填** |

运行期行为：缺 `key` → ERROR + FAILURE；缺 `value` → ERROR + FAILURE；否则 `config().blackboard->set<bool>(key, value)` + INFO + **SUCCESS**。

测试要点：`<RMSetBlackboard key="flag" value="true"/>` 之后 `<GetBlackboard key="flag"/>` 应 SUCCESS；
`value` 只有 bool（**不能写字符串/数字以外的类型**，`"1"`/`"0"` 由 `convertFromString<bool>` 支持）。

坑：
1. **库名与标签不同**（`rm_set_blackboard` vs `RMSetBlackboard`），写 XML 时用标签。
2. `blackboard->set<bool>` 在 `key` 已存在且**类型不是 bool** 时会 `throw RuntimeError`（`behaviortree_cpp/include/behaviortree_cpp/blackboard.h:260,292`），
   主循环没有 try/catch → 进程终止。测试顺序：先 `SetBlackboard`(内置，字符串) 建同名键再 `RMSetBlackboard` 会炸。
3. 与 BT.CPP 内置的 `SetBlackboard`（`output_key`/`value`，支持任意类型）**不是同一个节点**，别混用端口名。

---

## 26. SubArmors（**未注册 / 未编译**）

- 文件：`plugins/action/sub_armors.cpp`、`include/.../action/sub_armors.hpp`（实现完整，`CreateRosNodePlugin(..., "SubArmors")` 也在）
- C++ 类 / 基类：`SubArmorsAction` : `BT::RosTopicSubNode<auto_aim_interfaces::msg::Armors>`（若被编译）
- 注册：❌ **没有**
  1. `CMakeLists.txt:236-239` 的 `ament_auto_add_library(sub_armors ...)` 被注释 → `install/rm_behavior_tree/lib/` 里**没有 `libsub_armors.so`**（已核实目录列表）；
  2. `rm_behavior_tree.cpp:45` 的 `"sub_armors",` 也被注释。
- 端口（代码里声明的，仅供将来参考）：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | **必填** |
| `armors` | out | `auto_aim_interfaces::msg::Armors` | 无 |

运行期行为（若启用）：有新消息才写 `armors`；始终 SUCCESS。

测试要点 / 坑：
- XML 里写 `<SubArmors .../>` → `createTreeFromFile` 报 “Node not found: SubArmors” 并抛异常。
- 现在获取装甲板的正路是 `SubTargetPos`（字符串话题 `/auto_aim_target_pos`），不是这个节点。
- 文档说“有完整实现与 CreateRosNodePlugin，但 CMake target 与主程序注册列表均被注释，未编译、未注册” ✔ **源码一致**。

---

## 27. posture_change.cpp（**空文件，无节点**）

- 文件：`plugins/action/posture_change.cpp` —— **0 字节**（`ls -l` 确认 size 0，2026-03-27）。
- 没有类、没有 `providedPorts`、没有注册宏、CMake 里也没有 target。
- 影响：不产生任何 XML 标签；`grep posture` 在整包无其它引用。姿态切换请用 `PubRobotState`。

---
# 四、条件节点（`plugins/condition/`，22 个）

条件节点的两种基类：

- `BT::ConditionNode` + `tick()`：可拿 `params.nh` 建**裸订阅**（依赖主 executor spin）或纯读黑板。
- `BT::SimpleConditionNode`（构造时传入成员函数 `std::bind`）：`tick()` 被基类固定为“调用回调并返回”，
  **无法**在 tick 外做状态更新，也没有 `params`（所以不能建订阅）。

所有条件节点的 XML 标签都在 `<Condition ID="...">` 或 `<Action ID="...">` 下都能加载，但按 BT.CPP 语义它们只返回
`SUCCESS`/`FAILURE`（`IsGameTime` 例外，会返回 `RUNNING`）。

---

## 28. GetBlackboard

- 文件：`plugins/condition/get_blackboard.cpp`（实现全在 `include/.../condition/get_blackboard.hpp`）
- C++ 类 / 基类：`GetBlackboardCondition` : `BT::ConditionNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `get_blackboard`；`BT_REGISTER_NODES` → `"GetBlackboard"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `key` | in | `std::string` | **必填** |

运行期行为：缺 `key` 端口 → ERROR + FAILURE；`blackboard->get<bool>(key)` 为 true → **SUCCESS**，false → **FAILURE**；
key 不存在 / 类型不是 bool（抛 `std::exception` 被捕获）→ INFO 日志 + **FAILURE**。

测试要点：配 `RMSetBlackboard key="x" value="true"` 造正例；直接 `key="不存在"` 是负例（**不报错、不打 ERROR，只 INFO**）；
key 是用内置 `SetBlackboard` 写的字符串 → 类型不匹配 → FAILURE（不是异常）。

坑：只支持 `bool`。想读 int/double/消息类型做不到（源码写死 `get<bool>`）。

---

## 29. Not（**未注册**）

- 文件：`plugins/condition/not_condition.cpp`（实现全在头文件 `not_condition.hpp`）
- C++ 类 / 基类：`NotCondition` : `BT::DecoratorNode`
- 注册：❌ **没有**
  - `CMakeLists.txt:165-168` 有 `ament_auto_add_library(not_condition ...)` → `install/.../lib/libnot_condition.so` **存在**；
  - 但 `rm_behavior_tree.cpp` 的 `bt_plugin_libs`（第 55-72 行）**没有 `not_condition`**，也没有 `RegisterRosNode` 调用
    → 库被编译、安装，却**从未加载**。
- 端口：无（`providedPorts()` 返回 `{}`）。
- 运行期行为（若注册）：`child()->executeTick()`，SUCCESS↔FAILURE 互换，RUNNING→RUNNING，其它原样返回。

测试要点：XML 写 `<Not>` / `<Inverter>`：**`Not` 会报 “Node not found”**；需要取反请用 BT.CPP 内置的
`<Inverter>`（或 `<ForceFailure>` 包一层）。这是本清单里最“隐蔽”的未注册项——因为 `.so` 明明存在。

坑：源码里 `NotCondition` 的 `tick()` 不会调用 `setStatus()`，也不处理 `child()->halt()`，与内置 Inverter 行为不完全一致（若将来启用需复测）。

---

## 30. IsAttacked

- 文件：`plugins/condition/is_attacked.cpp`、`include/.../condition/is_attacked.hpp`
- C++ 类 / 基类：`IsAttackedAction` : `BT::ConditionNode`
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_attacked`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `robot_msgs::msg::RobotStatus` | **必填**（无订阅兜底！） |
| `hp_threshold` | in | `uint16_t` | `10` |
| `timeout_msec` | in | `int` | `10000` |

运行期行为（`is_attacked.cpp:12-58`）：
- 缺 `message` → **FAILURE**；
- 首次 tick：`last_hp_ = current_hp`、记时间、`is_initialized_=true` → **FAILURE**（**第一帧永远是 FAILURE**）；
- `hp_drop = last_hp_ - current_hp >= hp_threshold` → 更新 `last_hp_`/时间 → **SUCCESS**；
- 否则读 `elapsed`；`elapsed*1000 < timeout_msec` → **FAILURE**；超时后 → 若 `current_hp > last_hp_` 才更新 `last_hp_` → **FAILURE**。

**关键：`timeout_msec` 是死端口** —— 两条分支都返回 FAILURE，SUCCESS 只在“本帧掉血 ≥ 阈值”的那一 tick 出现。

测试要点：
- 正例：连发 `current_hp = 200`（第一帧 FAILURE，初始化）→ 再发 `current_hp = 180`（掉 20 ≥ 10）→ 这一帧 SUCCESS；下一帧立刻 FAILURE。
- 负例：`current_hp` 不变 → 永远 FAILURE；掉 5（< 10）→ FAILURE。
- 想“被攻击后保持 SUCCESS 10 s”的预期（头文件注释、文档语义）**做不到**。

坑/不一致：
1. **头文件注释说“被击打后保持 SUCCESS 的时间（默认 10000ms）”，源码不实现**（`timeout_msec` 只影响一条恒 FAILURE 的分支）。
   与 `IsBaseAttacked`（真实现保持逻辑）**语义相反**，很容易写错树。
2. 需要外部 `SubRobotStatus` 先喂 `{robot_status}`；本节点不看 `msg.is_attacked` 字段，只看 HP 差分（字段白给）。
3. `hp_threshold` 是 `uint16_t` 端口，XML 传负数会溢出。

---

## 31. IsBaseAttacked

- 文件：`plugins/condition/is_base_attacked.cpp`、`include/.../condition/is_base_attacked.hpp`
- C++ 类 / 基类：`IsBaseAttackedAction` : `BT::ConditionNode`
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_base_attacked`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `all_robot_hp` | in | `robot_msgs::msg::AllRobotHP` | **必填** |
| `hp_threshold` | in | `int` | `10` |
| `timeout_msec` | in | `int` | `10000` |

运行期行为：缺输入 → FAILURE；首帧初始化 → **FAILURE**；`base_hp` 掉 ≥ 阈值 → SUCCESS + 重置计时；
否则 `last_base_hp_ = base_hp`（**每帧都更新**），`elapsed*1000 < timeout_msec` → **SUCCESS（保持）**，超过 → **FAILURE**。

测试要点：`{robot_hp}` 来自 `SubAllRobotHP`；`base_hp` 从 5000 → 4900（掉 100）触发 SUCCESS，之后 10 s 内持续 SUCCESS，10 s 后转 FAILURE。
负例：`base_hp` 恒定 → 首帧 FAILURE 后，只要已初始化且距首帧 < 10 s 仍会 SUCCESS（**注意：首帧之后会白送 10 s SUCCESS**）→ 测负例要等 > 10 s。
断言建议：用 `timeout_msec="100"` 加速。

坑：
1. `hp_threshold` 是 `int`（`IsAttacked` 是 `uint16_t`），两个节点端口类型不一致。
2. `last_base_hp_` 在判定前就被更新 → 只能捕捉“相邻 tick 之间”的掉血，掉血被拆成多次小 dropdown 时会漏。
3. 与 `IsAttacked` 的保持语义相反（这个是“真保持”，那个是“不保持”）。

---

## 32. IsBaseStatusOK

- 文件：`plugins/condition/is_base_status_ok.cpp`、`include/.../condition/is_base_status_ok.hpp`
- C++ 类 / 基类：`IsBaseStatusOKAction` : `BT::ConditionNode`（构造函数 `(void)params;`，**不建订阅**）
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_base_status_ok`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `robot_hp` | in | `robot_msgs::msg::AllRobotHP` | **必填** |
| `hp_threshold` | in | `int` | `0` |

运行期行为：输入缺失 → **SUCCESS**（故意：启动阶段不阻塞）；`base_hp >= hp_threshold` → SUCCESS，否则 FAILURE。

测试要点：`hp_threshold="1"` 时 `base_hp=0` 是负例；`hp_threshold="0"`（默认）时 `base_hp=0` 仍是 SUCCESS（**`>=` 让默认配置恒真**）。
缺输入（还没收到 `{robot_hp}`）→ SUCCESS，这是“假正例”，测试必须显式区分“没数据”和“血量足”。

坑：与 `IsOutpostOK`（缺输入 → FAILURE）、`IsEnemyOutpostOK`（缺输入 → FAILURE）策略相反。

---

## 33. IsOutpostOK

- 文件：`plugins/condition/is_outpost_ok.cpp`、`include/.../condition/is_outpost_ok.hpp`
- C++ 类 / 基类：`IsOutpostOKAction` : `BT::ConditionNode`
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_outpost_ok`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `robot_hp` | in | `robot_msgs::msg::AllRobotHP` | **必填** |
| `hp_threshold` | in | `int` | `0` |

运行期行为：缺输入 → **FAILURE**；`outpost_hp > hp_threshold` → SUCCESS，否则 FAILURE（**严格大于**）。

坑：`outpost_hp` 字段来自 `robot_msgs::msg::AllRobotHP`（referee 版才有这个字段；`rm_decision_interfaces` 版没有）。
默认 `hp_threshold=0` 时 `outpost_hp=0` → FAILURE（与 `IsBaseStatusOK` 的 `>=` 不同）。

---

## 34. IsEnemyOutpostOK

- 文件：`plugins/condition/is_enemy_outpost_ok.cpp`、`include/.../condition/is_enemy_outpost_ok.hpp`
- C++ 类 / 基类：`IsEnemyOutpostOKAction` : `BT::ConditionNode`
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_enemy_outpost_ok`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `robot_hp` | in | `robot_msgs::msg::AllRobotHP` | **必填** |
| `hp_threshold` | in | `int` | `0` |

运行期行为：缺输入 → FAILURE；`enemy_outpost_hp >= hp_threshold` → SUCCESS（**大于等于**，与 §33 的 `>` 不一致）。

测试要点：`hp_threshold="1"` + `enemy_outpost_hp=0` → FAILURE（说明敌方前哨已被打掉）；`enemy_outpost_hp=1500` → SUCCESS。

坑：三个血条条件的比较符各不相同（`>=` / `>` / `>=`），批量生成测试树时不能套用同一个期望值。

---

## 35. IsAutoAimOK

- 文件：`plugins/condition/is_auto_aim_ok.cpp`、`include/.../condition/is_auto_aim_ok.hpp`
- C++ 类 / 基类：`IsAutoAimOKCondition` : `BT::ConditionNode`
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_auto_aim_ok`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `robot_msgs::msg::AutoAimStatus` | 可选（无默认，缺失时回退到订阅缓存） |
| `target_status` | in | `int` | `0` |

运行期行为：
- 构造期在 `params_update_msg.nh` 上**裸订阅 `/auto_aim_status`**（**硬编码话题名**，`qos=10`，reliable/volatile）→ 依赖主 executor spin。
- tick：读 `target_status`；`message` 端口优先；两者都没有 → 2 s 节流 WARN + **FAILURE**；
  `auto_aim_status == target_status` → SUCCESS，否则 FAILURE。

测试要点：正例两条路——
① 只发 `/auto_aim_status`（`auto_aim_status=0`），`target_status="0"` → SUCCESS（需要 spin `update_msg` 节点，主程序已加）；
② 用 `SubAutoAimStatus` 写 `{auto_aim_status}` 并 `message="{auto_aim_status}"` → 不依赖订阅。
负例：`target_status="1"` 而状态为 0 → FAILURE；不发布任何消息 → FAILURE（**不是 RUNNING**，会让 `Sequence` 卡在 FAILURE）。

坑：`message` 端口不能写 `topic_name`（本节点没有该端口）；硬编码话题意味着**改 namespace/话题名无效**。

---

## 36. IsRemoteStatus

- 文件：`plugins/condition/is_remote_status.cpp`、`include/.../condition/is_remote_status.hpp`
- C++ 类 / 基类：`IsRemoteStatusCondition` : `BT::ConditionNode`
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_remote_status`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `robot_msgs::msg::RemoteStatus` | 可选（缺失时用订阅缓存） |
| `target_status` | in | `int` | **`1`** |

运行期行为：构造期裸订阅 `/remote_status`（硬编码）；tick 优先 `message`，无消息 → 2 s 节流 WARN + FAILURE；
`remote_control_status == target_status` → SUCCESS。

测试要点：`target_status` 默认 1（正常）；喂 2（保守）/3（激进）测负例；`SubRemoteStatus message="{remote_status}"` 是更可控的路径。

坑：默认值 1 意味着“什么都不配”时断言的是“正常模式”，容易在别人以为默认是 0 时误判。

---

## 37. IsGameTime

- 文件：`plugins/condition/is_game_time.cpp`、`include/.../condition/is_game_time.hpp`
- C++ 类 / 基类：`IsGameTimeCondition` : `BT::ConditionNode`
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_game_time`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `robot_msgs::msg::GameStatus` | 可选（缺失时用订阅缓存） |
| `game_progress` | in | `int` | **必填**（声明无默认） |
| `lower_remain_time` | in | `int` | **必填**（声明无默认） |
| `higher_remain_time` | in | `int` | **必填**（声明无默认） |

运行期行为：构造期裸订阅 `/game_status`（硬编码）；tick 先读三个 int（**返回值全丢弃**），再取消息：
- 两个来源都没有 → 2 s 节流 WARN + **RUNNING**（全仓库唯一返回 RUNNING 的条件节点）；
- `game_progress == 期望 && lower <= stage_remain_time <= higher`（闭区间）→ **SUCCESS**；否则 **FAILURE**。

测试要点：
- 正例：`/game_status` 发 `{game_progress: 4, stage_remain_time: 200}`，端口 `game_progress="4" lower_remain_time="0" higher_remain_time="420"` → SUCCESS。
- 负例：`stage_remain_time=421` → FAILURE；`game_progress=3` → FAILURE；不发消息 → RUNNING（**在 `Sequence` 里会一直卡住**）。
- 加速测试：`lower == higher == 200` 做边界值（闭区间两端都 SUCCESS）。

坑：
1. **三个 int 端口没有默认值**，`getInput` 失败时局部变量 `game_progress/lower_remain_time/higher_remain_time`
   **未初始化**（`is_game_time.cpp:40`）→ 用垃圾值比较（UB，不崩溃但结果随机）。生成测试树时必须显式给全三个端口。
2. RUNNING 是条件节点里的异类：放在 `ReactiveSequence` 里会让后面的节点本 tick 不执行；放在 `Sequence` 里会让整棵树停在 RUNNING。
3. 硬编码 `/game_status`，`SubGameStatus` 的 `message` 端口才是可换话题的路径。

---

## 38. IsChassisEnergyOK

- 文件：`plugins/condition/is_chassis_energy_ok.cpp`、`include/.../condition/is_chassis_energy_ok.hpp`
- C++ 类 / 基类：`IsChassisEnergyOKCondition` : `BT::ConditionNode`
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_chassis_energy_ok`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `robot_msgs::msg::ChassisEnergy` | 可选（缺失时用订阅缓存） |

运行期行为：构造期裸订阅 `/chassis_energy`（硬编码）；tick 优先 `message`，都没有 → FAILURE；
**`chassis_energy == 1` → SUCCESS（“能量不足”）**，否则 **FAILURE（“能量 OK”）**；每 tick `std::cout` 打印原始值。

测试要点：**语义是反的**（节点名叫 OK，SUCCESS 表示不 OK）。
- “能量不足”正例：发 `chassis_energy: 1` → SUCCESS。
- “能量正常”负例：发 `chassis_energy: 0` 或 60 → FAILURE。
- 不发消息 → FAILURE（与“能量正常”无法区分）。

坑：
1. `ChassisEnergy.msg` 定义是 `uint16 chassis_energy # 底盘剩余能量 (J)`，代码却当 0/1 布尔 flag 用 →
   真实下位机若发能量焦耳值，这个条件**永远 FAILURE**。这是最值得上报的语义错位之一。
2. 端口只有 `message`，不能改话题名。
3. 每 tick 两行 `std::cout`（高频日志污染）。

---

## 39. IsStatusOK

- 文件：`plugins/condition/is_status_ok.cpp`、`include/.../condition/is_status_ok.hpp`
- C++ 类 / 基类：`IsStatusOKAction` : `BT::ConditionNode`（`(void)params;`，**无订阅**）
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_status_ok`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `robot_msgs::msg::RobotStatus` | **必填** |
| `hp_threshold` | in | `int` | **必填**（无默认） |
| `heat_threshold` | in | `int` | **必填**（无默认） |
| `barrel_heat_limit_threshold` | in | `int` | **必填**（无默认） |

运行期行为：
- 缺 `message` → **SUCCESS**（源码注释：让上层 `Inverter` 翻成 FAILURE 好切分支）；
- 三个阈值全 `getInput`（失败则保持局部初值 0）；
- `current_hp >= hp_threshold && shooter_heat >= heat_threshold && shooter_barrel_heat_limit <= barrel_heat_limit_threshold` → SUCCESS，否则 FAILURE。
- 每 tick 打印一整行字段（`std::cout`）。

测试要点：`shooter_heat` 语义是“**剩余弹量**（越大越好）”，`shooter_barrel_heat_limit` 是“枪口热量上限（越小越好）”，
两个字段的比较方向相反（`>=` vs `<=`），测试要分别构造正负例：
`hp_threshold="150" heat_threshold="10" barrel_heat_limit_threshold="200"` + `{current_hp: 200, shooter_heat: 20, shooter_barrel_heat_limit: 150}` → SUCCESS；
把 `current_hp` 降到 100 → FAILURE。

坑：
1. 缺 `message` 返回 SUCCESS —— 与“状态正常”无法区分（必须靠上层 Inverter 才有意义）。
2. 三个阈值**无默认值**，漏写时按 0 处理（`shooter_heat >= 0` 恒真、`barrel <= 0` 几乎恒假）→ 结果偏向 FAILURE，但不报错。
3. 依赖 `SubRobotStatus`（黑板上必须有 `robot_msgs` 类型）。

---

## 40. IsRuneStatus

- 文件：`plugins/condition/is_rune_status.cpp`、`include/.../condition/is_rune_status.hpp`
- C++ 类 / 基类：`IsRuneStatusAction` : `BT::ConditionNode`
- 注册：✅ `RegisterRosNode`（`params_update_msg`，库 `is_rune_status`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `rune_status` | in | `robot_msgs::msg::RuneStatus` | **必填** |
| `target_count` | in | `int` | `1` |

运行期行为：缺输入 → FAILURE；**`rune_count == target_count` → FAILURE（命中/打完）**；否则 → **SUCCESS**。

测试要点：逻辑是反的（等于目标值即“完成”，返回 FAILURE 让上层 fallback 换分支）。
正例（还没打完）：`rune_count=0, target_count=1` → SUCCESS；负例（打完了）：`rune_count=1` → FAILURE。
`target_count="2"` 时 `rune_count=0/1` → SUCCESS，`=2` → FAILURE。

坑：`RuneStatus` 只有 `uint8 rune_count`（0/1/2），端口是 int，比较前 `static_cast<int>`，OK。
与文档（附录未覆盖条件节点）无关；与 `SubRuneStatus` 配对使用。

---

## 41. IsSemantic

- 文件：`plugins/condition/is_semantic.cpp`、`include/.../condition/is_semantic.hpp`
- C++ 类 / 基类：`IsSemanticCondition` : `BT::RosTopicSubNode<std_msgs::msg::String>`
- 注册：✅ `RegisterRosNode`（**专用 `params_semantic`**，库 `is_semantic`，`default_port_value = "terrain_zone"`）

端口（`providedBasicPorts` + 2 个）：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `topic_name` | in | `std::string` | 占位符 → `terrain_zone`（相对名，跟随节点 namespace） |
| `semantic` | in | `std::string` | `""`（空串 = 断言“不在任何 zone”） |
| `received` | out | `std::string` | 无（必须写进 XML，否则丢弃） |

运行期行为：
- 基类订阅 QoS 是 `transient_local+reliable`，`tunnel_mode_manager` 发的是 latched 话题 → 首 tick 通常就能拿到。
- 每个 tick：`last_msg` 到了就 `trim` 后缓存并 `setOutput("received", ...)`；
- **从未收到消息 → FAILURE**（2 s 后打一次 WARN，节流用 `warned_no_msg_` 只报一次）；
- 收到过：每个 tick 重读 `semantic` 端口（支持 `{blackboard}`），`trim` 后与缓存比较 → 相等 SUCCESS / 不等 FAILURE。

测试要点：
- 正例：`ros2 topic pub --qos-durability transient_local --qos-reliability reliable -r 1 /terrain_zone std_msgs/msg/String "{data: 'tunnel'}"`，
  节点 `semantic="tunnel"` → SUCCESS；`semantic=" tunnel "`（带空格）也应 SUCCESS（trim）。
- 负例：`semantic="high_ground"` → FAILURE；不发消息 → FAILURE（**不是 RUNNING**）；
  发 `data: ""` 且 `semantic=""` → SUCCESS（“不在 zone”）。
- namespace：树带 `-r __ns:=/red_standard_robot1` 时相对名 `terrain_zone` 解析成 `/red_standard_robot1/terrain_zone`；
  测试发布端要发到同一个名字，或 XML 里显式写绝对名。

坑：
1. `received` 输出端口不写进 XML → `setOutput` 静默失败（返回的 `Result` 没人看）。
2. 未收到消息时**不执行** `semantic` 端口检查 → 端口缺失的报错被“没消息”掩盖。
3. `trim` 只去首尾空白，大小写敏感。

---
## 42. IsDetectEnemy

- 文件：`plugins/condition/is_detect_enemy.cpp`、`include/.../condition/is_detect_enemy.hpp`
- C++ 类 / 基类：`IsDetectEnemyAction` : `BT::SimpleConditionNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `is_detect_enemy`；`BT_REGISTER_NODES` → `"IsDetectEnemy"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `auto_aim_interfaces::msg::Armors` | **必填** |
| `max_range` | in | `double` | `-1.0`（≤0 表示不限距离） |
| `current_location` | in | `geometry_msgs::msg::TransformStamped` | 名义上 `""`（空串默认值不会写进 `input_ports`，实际等于**无默认**） |
| `debounce_ms` | in | `int` | `200`（**声明了但从不读取**） |

运行期行为：
1. 缺 `message` → `std::cerr` + FAILURE；
2. `armors.armors` 空 → FAILURE；
3. 所有 armor 的 `position(x,y,z)` 都 ≤0.001 → INFO + FAILURE（“全零视为无效”）；
4. `max_range > 0`：有 `current_location` 就用它算三维距离，否则用 armor 到**原点**的三维距离；任一 armor `distance <= max_range` → SUCCESS；全超范围 → INFO + FAILURE；
5. `max_range <= 0` → SUCCESS。

测试要点：
- 正例：`armors=[{pose.position: {x: 3, y: 1, z: 0.2}}]`，`max_range="-1"` → SUCCESS；`max_range="5"` → SUCCESS；`max_range="1"` → FAILURE（超范围日志）。
- 负例：`armors=[]` → FAILURE；`armors=[{pose.position:{x:0,y:0,z:0}}]` → FAILURE。
- `current_location` 缺失 + `max_range>0` 时用**场地原点**当自身位置 → 距离语义错误（机器人不在原点时判定不对），这是最容易写错负例的地方。

坑：
1. **`debounce_ms` 是死端口**：头文件注释“检测到敌人后需保持 debounce_ms 才确认，默认 200ms”**没有实现**；
   成员 `detecting_start_time_`/`lost_start_time_`/`was_enemy_detected_` 全部未被使用 → 无去抖，单帧噪声即翻转。
2. `current_location` 的默认值写法（`""`）对消息类型无意义，实际就是“没有默认”。
3. 文档（附录未覆盖条件节点）没有提到 `debounce_ms` 失效。

---

## 43. IsFriendOK

- 文件：`plugins/condition/is_friend_ok.cpp`、`include/.../condition/is_friend_ok.hpp`
- C++ 类 / 基类：`IsFriendOKAction` : `BT::SimpleConditionNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `is_friend_ok`；`BT_REGISTER_NODES` → `"IsFriendOK"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | **`rm_decision_interfaces::msg::AllRobotHP`** | **必填** |
| `friend_color` | in | `std::string` | **必填**（声明无默认） |

运行期行为：缺 `message` → FAILURE；`friend_color` 既不是 `"red"` 也不是 `"blue"` → cerr + FAILURE；
按 (1,3,4,7 号) 取**整数平均**（`/4` 截断）：red 方用 `red_*` 当己方；否则用 `blue_*` 当己方；
`friend_avg > enemy_avg` → SUCCESS，否则 FAILURE。

测试要点：
- 正例：`friend_color="red"`，`red_1/3/4/7 = 400`，`blue_* = 100` → SUCCESS；反过来 → FAILURE；相等 → FAILURE（严格大于）。
- 负例：`friend_color="green"` → FAILURE；`message` 缺失 → FAILURE。

坑（**高危**）：
1. **类型不匹配**：`SubAllRobotHP` 输出的是 `robot_msgs::msg::AllRobotHP`，本节点读 `rm_decision_interfaces::msg::AllRobotHP`
   → 把 `{robot_hp}` 接到 `message` **永远 FAILURE**（`getInput` 类型不符返回错误，代码只打 cerr）。
   要用它必须另有一条产生 `rm_decision_interfaces` 类型数据的路径（本包内**没有**这样的订阅节点）。
2. `friend_color` 缺端口时，`Expected<std::string>` 与 `"red"`/`"blue"` 比较都为 false → **静默走 else 分支（当蓝方）**，
   而不是报错。测试必须显式给 `friend_color`。
3. 平均是**整数除法**（`/4` 截断），`red=100,100,100,101` 与 `blue=100` 的边界会因截断翻转，边界值要小心。
4. 注释说“3V3 中无法较早确定对手上场机器人编号，故此处均设为 1/3/4/7”，即**不看 2/5 号**。

---

## 44. IsRfidDetected

- 文件：`plugins/condition/is_rfid_detected.cpp`、`include/.../condition/is_rfid_detected.hpp`
- C++ 类 / 基类：`IsRfidDetectedCondition` : `BT::SimpleConditionNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `is_rfid_detected`；`BT_REGISTER_NODES` → `"IsRfidDetected"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `key_port` | in | `robot_msgs::msg::RfidStatus` | **`{@referee_rfidStatus}`**（注意端口名不叫 `message`） |
| `base_gain_point` | in | `bool` | `false` |
| `friendly_fortress_gain_point` | in | `bool` | `false` |
| `center_gain_point` | in | `bool` | `false` |

运行期行为：缺 `key_port` → `std::cerr` + FAILURE；三个开关里**任一为 true 且对应字段为 true** → SUCCESS，否则 FAILURE（逻辑或）。

测试要点：
- 需要黑板上存在 `@referee_rfidStatus`（**不是** `referee_rfidStatus`，带 `@` 表示“父黑板条目”）这个键，
  且类型必须是 `robot_msgs::msg::RfidStatus`。**本包内没有任何节点写这个键**（`grep referee_rfidStatus` 只命中本文件），
  必须由外部进程/测试用 `SetBlackboard`（内置，支持任意类型）或 `Blackboard::createEntry` 预置。
- 正例：`key_port="{@referee_rfidStatus}"` + 黑板 `RfidStatus{base_gain_point: true}` + 端口 `base_gain_point="true"` → SUCCESS。
- 负例：`base_gain_point="true"` 但消息字段 false → FAILURE；`base_gain_point="false"`（默认）→ 即使消息字段全 true 也 FAILURE（**开关默认全关 = 恒 FAILURE**）。

坑：
1. 端口名是 `key_port`，其它条件节点都叫 `message` → 批量生成 XML 时容易套错模板。
2. 默认值 `{@referee_rfidStatus}` 只在黑板**已有该条目**时有效；不存在时 `getInput` 报“找不到键” → FAILURE（不崩溃）。
3. 依赖 `robot_msgs` 的 `RfidStatus`（含 `center_gain_point` 字段），不是 `rm_decision_interfaces/RFID`（那个只有 `rfid_patrol_status`）。

---

## 45. IsTargetInZone

- 文件：`plugins/condition/is_target_in_zone.cpp`、`include/.../condition/is_target_in_zone.hpp`
- C++ 类 / 基类：`IsTargetInZoneAction` : `BT::SimpleConditionNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `is_target_in_zone`；`BT_REGISTER_NODES` → `"IsTargetInZone"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `armors` | in | `auto_aim_interfaces::msg::Armors` | **必填** |
| `x1`..`y4` | in | `double` | 全部 `0.0`（8 个端口） |

运行期行为：缺 `armors` / `armors` 为空 → INFO + FAILURE；8 个角点有一个读不到 → WARN + FAILURE（有默认值，实际不会）；
遍历所有 armor，任一落在四边形内 → SUCCESS；全不在 → FAILURE。判定用叉积同号（支持凸四边形，顺/逆时针都行，边界算“在内”）。

测试要点：
- 正例：`x1..y4 = 0,0, 5,0, 5,5, 0,5`（矩形），`armors=[{pose.position:{x:2,y:2}}]` → SUCCESS。
- 负例：`armors=[{x:6,y:6}]` → FAILURE；`armors=[]` → FAILURE。
- **全 0 默认值的陷阱**：不给角点时四边形退化成点 (0,0)，`armor=(0,0)` 会 SUCCESS（`!(has_neg&&has_pos)` 全 0 视为在内）。
  `SubTargetPos` 在敌人消失时正好输出 `(0,0)` 或空数组 → 空数组走 FAILURE 分支，但“有 armor 且坐标为 0”会 **假 SUCCESS**。
- 日志：每 tick 每个 armor 打一行 INFO（`pointInQuadrilateral` 里），高频。

坑：`pointInQuadrilateral` 不检查四边形是否凸/自交；凹四边形会误判。文档（附录未覆盖）无此说明。

---

## 46. IsGimbalTargetInZone

- 文件：`plugins/condition/is_gimbal_target_in_zone.cpp`、`include/.../condition/is_gimbal_target_in_zone.hpp`
- C++ 类 / 基类：`IsGimbalTargetInZoneAction` : `BT::SimpleConditionNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `is_gimbal_target_in_zone`；`BT_REGISTER_NODES` → `"IsGimbalTargetInZone"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `target_x` | in | `double` | **必填**（声明无默认） |
| `target_y` | in | `double` | **必填**（声明无默认） |
| `x1`..`y4` | in | `double` | 全部 `0.0` |

运行期行为：缺 `target_x/y` → INFO + FAILURE；`(|x|<0.001 && |y|<0.001)` → INFO“Invalid target (0,0)” + FAILURE；
缺角点 → WARN + FAILURE；在四边形内 → SUCCESS，否则 FAILURE。**判定的是 gimbal 系原始坐标，不做 TF 变换**。

测试要点：配 `SubGimbalTarget` 的 `target_x="{gimbal_target_x}"` / `target_y="{gimbal_target_y}"`（注意 **SubGimbalTarget 无效时输出 0.0 → 本节点 FAILURE**，语义刚好衔接）。
正例：角点 `0,0,10,0,10,10,0,10` + `target_x="3" target_y="4"` → SUCCESS。负例：`target_x="0" target_y="0"` → FAILURE（即使 (0,0) 在矩形内）。

坑：
1. `target_x/target_y` 没有默认值，漏写 → FAILURE + INFO 日志，容易误以为“逻辑不通过”。
2. `(0,0)` 被硬编码当无效点 → 如果云台坐标系原点附近是合法目标区，永远判不到。
3. 头文件注释明确“不做 map 变换”，而 `SubGimbalTarget` 同时输出 map 版 `target_pose` —— 两个输出别接混。

---

## 47. IsGimbalTargetValid

- 文件：`plugins/condition/is_gimbal_target_valid.cpp`、`include/.../condition/is_gimbal_target_valid.hpp`（实现全在头文件）
- C++ 类 / 基类：`IsGimbalTargetValidCondition` : `BT::SimpleConditionNode`（构造函数里额外 `rclcpp::Node::make_shared("is_gimbal_target_valid_" + name)`）
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `is_gimbal_target_valid`；`BT_REGISTER_NODES` → `"IsGimbalTargetValid"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `bool` | `false`（收到有效目标标志，来自 `SubGimbalTarget::target_valid`） |
| `target_x` | in | `double` | `0.0` |
| `target_y` | in | `double` | `0.0` |
| `target_timeout_ms` | in | `int` | `5000` |

运行期行为（`is_gimbal_target_valid.hpp:52-113`）：
- `!message`（端口缺失或 false）→ `has_first_target_=false` + **FAILURE**；
- 缺 `target_x/target_y` → **FAILURE**；
- `!has_first_target_`（首次见到点）→ 记时间与坐标 → **SUCCESS**；
- 点相对“首次见到”的位置移动 `> 0.001`（x 或 y 任一）→ 重置计时/坐标 → **SUCCESS**（另打 INFO）；
- 同一点持续 `elapsed_ms > target_timeout_ms` → 2 s 节流 WARN + **FAILURE**；
- 否则 **SUCCESS**。

测试要点：
- 正例：`message="{gimbal_target_valid}" target_x="{gimbal_target_x}" target_y="{gimbal_target_y}" target_timeout_ms="5000"`，
  云台手持续发同一个点 → 前 5 s SUCCESS，之后 FAILURE（**即使电控还在发同一个点**）。
- 让点每次略变（>0.001）→ 一直 SUCCESS。这正是“云台手还在动 = 目标仍有效”的设计。
- 负例：`message="false"`（或 `SubGimbalTarget` 因 TF 竞态把 `target_valid` 置 false）→ 立即 FAILURE 并清 `has_first_target_`。

坑：
1. **自建 `rclcpp::Node`**（`is_gimbal_target_valid_<name>`）只为拿 `now()`；该节点没进 executor，且 **`use_sim_time` 参数不会传给它**
   → 仿真里它用系统时钟，和主时钟不同源（超时判定仍然自洽，因为只用同一个时钟做差，但和 `ros::Time` 日志对不上）。
   另外每实例创建一个节点，大量实例会有很多无名节点。
2. `epsilon=0.001` 硬编码；如果电控对同一点做小数点后 3 位的抖动，会被当“新点”无限续期（永远 SUCCESS）。
3. 首次见到点直接 SUCCESS，**没有“刚收到就可能是旧点”的保护**。

---

## 48. IsTimeToBullet

- 文件：`plugins/condition/is_time_to_bullet.cpp`、`include/.../condition/is_time_to_bullet.hpp`
- C++ 类 / 基类：`IsTimeToBulletCondition` : `BT::SimpleConditionNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `is_time_to_bullet`；`BT_REGISTER_NODES` → `"IsTimeToBullet"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `robot_msgs::msg::GameStatus` | **必填** |
| `game_progress` | in | `int` | **必填**（无默认） |
| `time_to_bullet` | in | `std::string`（空白分隔的整数列表，如 `"5 10 15"`） | **必填**（无默认） |

运行期行为：缺 `message` → INFO“没有” + FAILURE；解析 `time_to_bullet`（`iss >> int` 循环，非数字处停止）；
`game_progress == 期望 && stage_remain_time ∈ time_points` → SUCCESS，否则 FAILURE。

测试要点：`game_progress="4" time_to_bullet="200 300"` + `/game_status{game_progress:4, stage_remain_time:200}` → SUCCESS；
`stage_remain_time=201` → FAILURE；`time_to_bullet="abc"` → 列表为空 → 恒 FAILURE（且无报错）。

坑：
1. `game_progress` 缺端口时局部变量未初始化（`is_time_to_bullet.cpp:11`），比较结果随机。
2. 字符串解析用 `>>`，只支持空白分隔；`"200,300"` 会解析成 `200`（`iss` 在逗号处 `fail`）→ 只匹配 200，容易误判。
3. `RCLCPP_INFO` 的 logger 名写成 `"print_message"`（复制粘贴），日志来源误导。
4. 与 `IsGameTime` 功能重叠（区间 vs 点列表），但两者消息来源可以不同（本节点只吃 `message` 端口）。

---

## 49. IsBatteryOK（**未注册 / 未编译**）

- 文件：`plugins/condition/is_battery_ok.cpp`（**无对应头文件**，类直接定义在 cpp 里）
- C++ 类 / 基类：`IsBatteryOKAction` : `BT::SimpleConditionNode`
- 注册：❌ **没有**
  1. `CMakeLists.txt` 里**没有任何 `is_battery_ok` target** → `install/rm_behavior_tree/lib/` 下无 `libis_battery_ok.so`（已核实）；
  2. `rm_behavior_tree.cpp` 也没注册（`bt_plugin_libs` 与 `RegisterRosNode` 都没有）。
- 端口（代码里声明的）：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `message` | in | `robot_msgs::msg::RobotBatteryStatus` | **必填** |
| `bt_threshold` | in | `int` | **必填** |

- 为什么“未编译”是必然的：该文件**根本无法通过编译**——
  1. `return BT::NodeBattery::FAILURE;` / `BT::NodeBattery::SUCCESS`：`BT::NodeBattery` **不存在**（应为 `BT::NodeStatus`）；
  2. `getInput<robot_msgs::msg::RobotBattery>("message")`：`robot_msgs` 里**没有** `RobotBattery` 类型（只有 `RobotBatterystatus`），
     且与端口声明的 `RobotBatteryStatus` 不一致。
- 运行期行为（若修好）：`current_bt < bt_threshold` → FAILURE，否则 SUCCESS。

测试要点：XML 写 `<IsBatteryOK .../>` → “Node not found”。若需要电量判断，目前**没有可用节点**（需新写或改用 `IsStatusOK`）。

坑：这类“文件在、类在、就是没编译”的节点最容易被文档/`ls plugins/` 误判为可用；`install/.../lib/` 里没有 `.so` 是硬证据。

---

# 五、控制节点（`plugins/control/`，2 个）

## 50. ActionSelector

- 文件：`plugins/control/action_selector.cpp`、`include/.../control/action_selector.hpp`
- C++ 类 / 基类：`ActionSelector` : `BT::ControlNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `action_selector`；`BT_REGISTER_NODES` → `"ActionSelector"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `action_id` | in | `int` | `-1`（`>0` 时**优先于** `decision_num`） |
| `decision_num` | in | `rm_decision_interfaces::msg::DecisionNum` | **必填**（无默认，缺失时返回 -1） |
| `cooldown_ms` | in | `int` | `1000`（`<0` 强制 0） |

运行期行为（`action_selector.cpp:31-79`）：
1. `resolveAction()`：`action_id > 0` 用它；否则 `decision_num` 的 `decision_num` 字段；都没有 → -1。
2. `action < 1 || action > childrenCount()` → `std::cout` + **FAILURE**。
3. 动作号变化：`haltChild(旧 index)`，清 `waiting_`。
4. `waiting_ && now < next_tick_at_` → **RUNNING**（冷却中，**不 tick 子节点**）。
5. `executeTick(child[action-1])`：
   - 子 **RUNNING** → RUNNING；
   - 子 **SUCCESS** → 置 `waiting_=true`、`next_tick_at_ = now + cooldown_ms` → **返回 RUNNING**（**不向上返回 SUCCESS**！宏语义）；
   - 子 **FAILURE** → **FAILURE**（不缓存，下 tick 重试同一子节点）。
6. `halt()`：`current_action_=-1`、`waiting_=false`，再 `ControlNode::halt()` 全部子节点。

测试要点：
- 正例：`decision_num` 用 `SubDecisionNum` 写 `{decision_num}`，子树 1 返回 SUCCESS 且 `cooldown_ms="100"` →
  该 tick 返回 RUNNING，100 ms 后再次 tick 子树（可观察子树被重新执行）。
- 负例：`action_id="0"` 且无 `decision_num` → FAILURE；`action_id="99"`（> 子节点数）→ FAILURE；子树 FAILURE → FAILURE。
- 动作切换：运行中把 `decision_num` 从 1 改到 2 → 旧子树被 `haltChild`（若旧子树是 `SendGoal`，会取消导航）。

坑：
1. **子节点顺序即动作号**（第 N 个子节点 = 动作号 N），中间不能插空；用 `SubTree` 必须一一对应。
2. **永不向上返回 SUCCESS**（子树 SUCCESS 被吞成 RUNNING）→ 放在 `Sequence` 里会让整棵树卡在 RUNNING；
   适合放在 `Fallback`/`ReactiveFallback` 的第二分支或根下。
3. `cooldown_ms="0"` → 每 tick 重入子树（`SendGoal` 会反复取消/重发，需注意）。
4. `haltChild()` 只 halt 旧的那个；其它子节点若处于 RUNNING（不会发生，因为只 tick 一个）不管。

---

## 51. DecisionSwitch

- 文件：`plugins/control/decision_switch.cpp`、`include/.../control/decision_switch.hpp`
- C++ 类 / 基类：`DecisionSwitch` : `BT::ControlNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `decision_switch`；`BT_REGISTER_NODES` → `"DecisionSwitch"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `decision_num` | in | `rm_decision_interfaces::msg::DecisionNum` | **必填** |

运行期行为：缺输入 → `std::cout` + FAILURE；`decision_num==1` → tick `children_nodes_[0]`，SUCCESS 才 SUCCESS；
`==2` → tick `children_nodes_[1]`；其它值 → FAILURE；子节点非 SUCCESS（含 RUNNING）→ **FAILURE**。

测试要点：`<DecisionSwitch decision_num="{decision_num}">` + 两个子节点；`decision_num=1` 只走第一个；`=3` → FAILURE。
文档也说“只写了 case 1/2”。

坑（比文档更严重，按源码列出）：
1. **children < 2 时越界**：`decision_num=2` 而只有 1 个子节点 → `children_nodes_[1]` 越界（`std::vector` OOB，UB/崩溃）。生成测试树时若用 `DecisionSwitch` 必须给 ≥2 个子节点。
2. **子节点 RUNNING 被当成 FAILURE**：`if (executeTick() == SUCCESS)` 的写法把 RUNNING/FAILURE 一律折叠成 FAILURE，
   子节点状态没被 halt → 下一 tick 子树内部状态可能不一致（例如 `SendGoal` 还在 RUNNING 却返回了 FAILURE）。
3. 每 tick 重新 tick 子节点（无状态缓存），与 `ActionSelector` 的宏语义完全不同。

---

# 六、装饰节点（`plugins/decorator/`，1 个）

## 52. RateController

- 文件：`plugins/decorator/rate_controller.cpp`、`include/.../decorator/rate_controller.hpp`
- C++ 类 / 基类：`RateController` : `BT::DecoratorNode`
- 注册：✅ `registerFromPlugin`（`bt_plugin_libs`，库 `rate_controller`；`BT_REGISTER_NODES` → `"RateController"`）

端口：

| 端口 | 方向 | 类型 | 默认值 |
| --- | --- | --- | --- |
| `hz` | in | `double` | `10.0`（**构造期读一次**） |

运行期行为：
- 构造：`hz` 局部初值 1.0 → `getInput("hz", hz)` → `period_ = 1.0 / hz`。
- tick：`status()==IDLE` → 记 `start_`、`first_time_=true`；`setStatus(RUNNING)`；
  当 `first_time_ || child->status()==RUNNING || elapsed >= period_` 时 tick 子节点：
  子 RUNNING → RUNNING；子 SUCCESS → 重置 `start_` → **SUCCESS**；子 FAILURE → **FAILURE**；
  否则返回 `status()`（RUNNING）。

测试要点：`hz="2"` + 子节点是 `PrintMessage` → 每 500 ms 才打印一次，其余 tick 返回 RUNNING；
`hz="0"` → `period_ = inf` → 只有第 1 次执行（之后 `elapsed >= inf` 永假）；`hz="-1"` → `period_` 为负 → **每 tick 都执行**（无保护）。
子节点一旦 RUNNING，本装饰器每 tick 都转发（不再限速，这是设计）。

坑：
1. `hz` 在**构造期**读取（`rate_controller.cpp:12-14`），XML 写 `{blackboard}` 无效；`hz<=0` 无校验（inf/负周期）。
2. 构造函数参数类型是 `BT::NodeConfiguration`（BT.CPP 的旧别名，等同 `NodeConfig`），与其它节点写法不一致但能编译。
3. 子节点为 RUNNING 时**不受 hz 限制**（`child_node_->status()==RUNNING` 条件），限速只作用于“重新启动子节点”。

---

# 七、统计表

编号 1–52 共 52 个章节，其中 **§27（`posture_change.cpp`）是空文件、不是节点**，故：

| 指标 | 数量 | 说明 |
| --- | --- | --- |
| 范围内 `.cpp` 文件 | **52** | action 27（含 1 个 0 字节空文件）+ condition 22 + control 2 + decorator 1 |
| **节点总数（有 XML 标签、可实例化）** | **51** | 见下两行分解 |
| **已注册节点数** | **48** | 库已编译 + 注册入口已调用（`install/.../lib/` 里能找到对应 `.so`） |
| **未注册节点数** | **3** | `SubArmors`、`IsBatteryOK`（均未编译） + `Not`（库存在但未加载） |
| 其中 `RegisterRosNode` 注册 | 34 | `CreateRosNodePlugin` 声明 |
| 其中 `registerFromPlugin`（`BT_REGISTER_NODES`）注册 | 14 | 纯逻辑/简单条件节点 |
| `install/rm_behavior_tree/lib/` 里的插件 `.so` | **49** | 48 个已注册 + `libnot_condition.so`（已编译但从未 `registerFromPlugin`）；无 `libsub_armors.so`、无 `libis_battery_ok.so` —— 与上面 3 个未注册项完全对上 |

按基类分（“总数”含未注册项，“已注册”列相加 = 48）：

| 基类 | 总数 | 已注册 | 节点 |
| --- | --- | --- | --- |
| `BT::RosTopicSubNode<T>`（其本身是 `ConditionNode`） | 10 | 9 | 8 个 `Sub*` 消息订阅 + `IsSemantic`；`SubArmors` ❌ |
| `BT::RosTopicPubNode<T>`（其本身是 `ConditionNode`） | 7 | 7 | `PubRobotState`、`PubChassisMode`、`PubAutoAimMode`、`PubSuperCapacitor`、`PubEnemyStatus`、`PubAutoaimTarget`、`RobotControl` |
| `BT::RosActionNode<T>` | 1 | 1 | `SendGoal` |
| `BT::SyncActionNode` | 7 | 7 | `SubTargetPos`、`SubGimbalTarget`、`MoveAround`、`PursueGoal`、`GetCurrentLocation`、`PrintMessage`、`RMSetBlackboard` |
| `BT::StatefulActionNode` | 2 | 2 | `PursueEnemy`、`WaitDuration` |
| `BT::ConditionNode`（自实现 `tick()`） | 12 | 12 | `GetBlackboard`、`IsAttacked`、`IsBaseAttacked`、`IsBaseStatusOK`、`IsOutpostOK`、`IsEnemyOutpostOK`、`IsAutoAimOK`、`IsRemoteStatus`、`IsGameTime`、`IsChassisEnergyOK`、`IsStatusOK`、`IsRuneStatus` |
| `BT::SimpleConditionNode` | 8 | 7 | `IsDetectEnemy`、`IsFriendOK`、`IsRfidDetected`、`IsTargetInZone`、`IsGimbalTargetInZone`、`IsGimbalTargetValid`、`IsTimeToBullet`；`IsBatteryOK` ❌ |
| `BT::ControlNode` | 2 | 2 | `ActionSelector`、`DecisionSwitch` |
| `BT::DecoratorNode` | 2 | 1 | `RateController`；`Not` ❌ |
| **合计** | **51** | **48** | 9+7+1+7+2+12+7+2+1 = 48 ✅ |

下表是逐项清单（权威，含未注册项的 ❌ 标记）：

| # | XML 标签 | C++ 类 | 基类 | 注册 | 注册函数 |
| --- | --- | --- | --- | --- | --- |
| 1 | `SubAllRobotHP` | `SubAllRobotHPAction` | `RosTopicSubNode<robot_msgs::msg::AllRobotHP>` | ✅ | `RegisterRosNode` |
| 2 | `SubRobotStatus` | `SubRobotStatusAction` | `RosTopicSubNode<robot_msgs::msg::RobotStatus>` | ✅ | `RegisterRosNode` |
| 3 | `SubGameStatus` | `SubGameStatusAction` | `RosTopicSubNode<robot_msgs::msg::GameStatus>` | ✅ | `RegisterRosNode` |
| 4 | `SubAutoAimStatus` | `SubAutoAimStatusAction` | `RosTopicSubNode<robot_msgs::msg::AutoAimStatus>` | ✅ | `RegisterRosNode` |
| 5 | `SubChassisEnergy` | `SubChassisEnergyAction` | `RosTopicSubNode<robot_msgs::msg::ChassisEnergy>` | ✅ | `RegisterRosNode` |
| 6 | `SubRuneStatus` | `SubRuneStatusAction` | `RosTopicSubNode<robot_msgs::msg::RuneStatus>` | ✅ | `RegisterRosNode` |
| 7 | `SubRemoteStatus` | `SubRemoteStatusAction` | `RosTopicSubNode<robot_msgs::msg::RemoteStatus>` | ✅ | `RegisterRosNode` |
| 8 | `SubDecisionNum` | `SubDecisionNumAction` | `RosTopicSubNode<rm_decision_interfaces::msg::DecisionNum>` | ✅ | `RegisterRosNode` |
| 9 | `SubTargetPos` | `SubTargetPosAction` | `SyncActionNode` | ✅ | `RegisterRosNode` |
| 10 | `SubGimbalTarget` | `SubGimbalTargetAction` | `SyncActionNode` | ✅ | `RegisterRosNode` |
| 11 | `PubRobotState` | `PubRobotStateAction` | `RosTopicPubNode<robot_msgs::msg::RobotState>` | ✅ | `RegisterRosNode` |
| 12 | `PubChassisMode` | `PubChassisModeAction` | `RosTopicPubNode<robot_msgs::msg::ChassisMode>` | ✅ | `RegisterRosNode` |
| 13 | `PubAutoAimMode` | `PubAutoAimModeAction` | `RosTopicPubNode<robot_msgs::msg::AutoAimMode>` | ✅ | `RegisterRosNode` |
| 14 | `PubSuperCapacitor` | `PubSuperCapacitorAction` | `RosTopicPubNode<robot_msgs::msg::SuperCapacitor>` | ✅ | `RegisterRosNode` |
| 15 | `PubEnemyStatus` | `PubEnemyStatusAction` | `RosTopicPubNode<sp_msgs::msg::EnemyStatusMsg>` | ✅ | `RegisterRosNode` |
| 16 | `PubAutoaimTarget` | `PubAutoaimTargetAction` | `RosTopicPubNode<sp_msgs::msg::AutoaimTargetMsg>` | ✅ | `RegisterRosNode` |
| 17 | `RobotControl` | `RobotControlAction` | `RosTopicPubNode<rm_decision_interfaces::msg::RobotControl>` | ✅ | `RegisterRosNode` |
| 18 | `SendGoal` | `SendGoalAction` | `RosActionNode<nav2_msgs::action::NavigateToPose>` | ✅ | `RegisterRosNode` |
| 19 | `MoveAround` | `MoveAroundAction` | `SyncActionNode` | ✅ | `RegisterRosNode` |
| 20 | `PursueGoal` | `PursueGoalAction` | `SyncActionNode` | ✅ | `RegisterRosNode` |
| 21 | `PursueEnemy` | `PursueEnemyAction` | `StatefulActionNode` | ✅ | `RegisterRosNode` |
| 22 | `GetCurrentLocation` | `GetCurrentLocationAction` | `SyncActionNode` | ✅ | `RegisterRosNode` |
| 23 | `WaitDuration` | `WaitDuration` | `StatefulActionNode` | ✅ | `registerFromPlugin` |
| 24 | `PrintMessage` | `PrintMessageAction` | `SyncActionNode` | ✅ | `registerFromPlugin` |
| 25 | `RMSetBlackboard` | `SetBlackboardAction` | `SyncActionNode` | ✅ | `registerFromPlugin` |
| 26 | `SubArmors` | `SubArmorsAction` | `RosTopicSubNode<auto_aim_interfaces::msg::Armors>` | ❌ | — |
| 27 | `GetBlackboard` | `GetBlackboardCondition` | `ConditionNode` | ✅ | `registerFromPlugin` |
| 28 | `Not` | `NotCondition` | `DecoratorNode` | ❌ | — |
| 29 | `IsAttacked` | `IsAttackedAction` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 30 | `IsBaseAttacked` | `IsBaseAttackedAction` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 31 | `IsBaseStatusOK` | `IsBaseStatusOKAction` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 32 | `IsOutpostOK` | `IsOutpostOKAction` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 33 | `IsEnemyOutpostOK` | `IsEnemyOutpostOKAction` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 34 | `IsAutoAimOK` | `IsAutoAimOKCondition` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 35 | `IsRemoteStatus` | `IsRemoteStatusCondition` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 36 | `IsGameTime` | `IsGameTimeCondition` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 37 | `IsChassisEnergyOK` | `IsChassisEnergyOKCondition` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 38 | `IsStatusOK` | `IsStatusOKAction` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 39 | `IsRuneStatus` | `IsRuneStatusAction` | `ConditionNode` | ✅ | `RegisterRosNode` |
| 40 | `IsSemantic` | `IsSemanticCondition` | `RosTopicSubNode<std_msgs::msg::String>` | ✅ | `RegisterRosNode` |
| 41 | `IsDetectEnemy` | `IsDetectEnemyAction` | `SimpleConditionNode` | ✅ | `registerFromPlugin` |
| 42 | `IsFriendOK` | `IsFriendOKAction` | `SimpleConditionNode` | ✅ | `registerFromPlugin` |
| 43 | `IsRfidDetected` | `IsRfidDetectedCondition` | `SimpleConditionNode` | ✅ | `registerFromPlugin` |
| 44 | `IsTargetInZone` | `IsTargetInZoneAction` | `SimpleConditionNode` | ✅ | `registerFromPlugin` |
| 45 | `IsGimbalTargetInZone` | `IsGimbalTargetInZoneAction` | `SimpleConditionNode` | ✅ | `registerFromPlugin` |
| 46 | `IsGimbalTargetValid` | `IsGimbalTargetValidCondition` | `SimpleConditionNode` | ✅ | `registerFromPlugin` |
| 47 | `IsTimeToBullet` | `IsTimeToBulletCondition` | `SimpleConditionNode` | ✅ | `registerFromPlugin` |
| 48 | `IsBatteryOK` | `IsBatteryOKAction` | `SimpleConditionNode` | ❌ | — |
| 49 | `ActionSelector` | `ActionSelector` | `ControlNode` | ✅ | `registerFromPlugin` |
| 50 | `DecisionSwitch` | `DecisionSwitch` | `ControlNode` | ✅ | `registerFromPlugin` |
| 51 | `RateController` | `RateController` | `DecoratorNode` | ✅ | `registerFromPlugin` |

**已注册标签（48 个，按字母序，便于复制进测试生成器）：**

```
ActionSelector, DecisionSwitch, GetBlackboard, GetCurrentLocation, IsAttacked, IsAutoAimOK,
IsBaseAttacked, IsBaseStatusOK, IsChassisEnergyOK, IsDetectEnemy, IsEnemyOutpostOK, IsFriendOK,
IsGameTime, IsGimbalTargetInZone, IsGimbalTargetValid, IsOutpostOK, IsRemoteStatus, IsRfidDetected,
IsRuneStatus, IsSemantic, IsStatusOK, IsTargetInZone, IsTimeToBullet, MoveAround, PrintMessage,
PubAutoAimMode, PubAutoaimTarget, PubChassisMode, PubEnemyStatus, PubRobotState, PubSuperCapacitor,
PursueEnemy, PursueGoal, RMSetBlackboard, RateController, RobotControl, SendGoal, SubAllRobotHP,
SubAutoAimStatus, SubChassisEnergy, SubDecisionNum, SubGameStatus, SubGimbalTarget, SubRemoteStatus,
SubRobotStatus, SubRuneStatus, SubTargetPos, WaitDuration
```

**未注册（3 个，XML 里一写就 “Node not found”）：** `SubArmors`、`IsBatteryOK`、`Not`。

---

# 八、生成测试树时的全局注意事项

## 8.1 加载期就会抛异常的写法（必须避免）

| 触发条件 | 后果 | 依据 |
| --- | --- | --- |
| 任意 `RosTopicSubNode` 节点（8 个 `Sub*` + `IsSemantic`；未注册的 `SubArmors` 同理）漏写 `topic_name` | `std::logic_error("Both [topic_name] ... are empty")` → 整棵树加载失败 | `bt_topic_sub_node.hpp:207-234`；`params_update_msg` 无 `default_port_value` |
| `PubAutoAimMode`/`PubChassisMode`/`PubSuperCapacitor`/`PubEnemyStatus`/`PubAutoaimTarget` 漏写 `topic_name` | 同上（`__default__placeholder__` + 空 params） | `bt_topic_pub_node.hpp:120-150`；`params_pub_default` 无默认 |
| 给 `RobotControl` 写 `topic_name="..."` | BT.CPP 报未知端口异常（它的 `providedPorts` 里没有该端口） | `robot_control.hpp:20-23` |
| **任何端口名拼错 / 多写一个该节点没有的端口属性** | `RuntimeError("Possible typo? ... does not contain a port with this name")` → 整棵树加载失败 | `xml_parsing.cpp:740-752`（BT.CPP v4 对未知端口属性是**抛异常**，不是忽略） |
| 写 `SubArmors` / `IsBatteryOK` / `Not` | “Node not found” | 未注册（§26/§49/§29） |
| `SendGoal` 缺 `goal_pose` | `setGoal` 抛 `BT::RuntimeError` → **主循环无 try/catch，进程退出** | `send_goal.cpp:20-37`；`rm_behavior_tree.cpp:156-164` |
| `RMSetBlackboard` 写一个已存在但类型不是 bool 的键 | `blackboard->set<bool>` 抛 `RuntimeError` → 进程退出 | `blackboard.h:260-293` |

## 8.2 端口默认值 / 必填的真实规则

1. **端口默认值会被写进 `NodeConfig.input_ports`**（`xml_parsing.cpp:818-836`），所以 `getInput` 对“有默认值的端口”永远成功，
   代码里的“回退分支”是死代码（典型案例：`PubAutoAimMode.key_name`）。
2. **没有默认值的输入端口，漏写时 `getInput` 返回 `Expected` 错误（不抛）**，各节点处理方式不同：
   - 返回 FAILURE：`IsAttacked`、`IsBaseAttacked`、`IsOutpostOK`、`IsEnemyOutpostOK`、`IsRuneStatus`、`IsDetectEnemy`、`IsFriendOK`、`IsRfidDetected`、`IsTargetInZone`、`IsGimbalTargetInZone`、`IsGimbalTargetValid`、`IsTimeToBullet`、`MoveAround`、`PursueGoal`、`PursueEnemy`；
   - 返回 **SUCCESS**（“没数据不阻塞”）：`IsBaseStatusOK`、`IsStatusOK`；
   - **静默用未初始化变量**（结果随机）：`IsGameTime`（3 个 int）、`IsTimeToBullet`（`game_progress`）；
   - 静默用消息默认值发布：`RobotControl`（`false`/`0.0`）；
   - 回退到内部订阅：`IsAutoAimOK`、`IsRemoteStatus`、`IsGameTime`、`IsChassisEnergyOK`。
3. **输出端口必须写进 XML**，否则 `setOutput` 返回错误（被调用方忽略）→ 数据静默丢弃，黑板键根本不存在。
   唯一例外：端口默认值本身就是 `{...}` 时 BT.CPP 会把它加进 `output_ports`（本包内只有 `IsRfidDetected.key_port` 是输入，
   没有输出端口用这种默认值）。
4. 端口类型必须与 `providedPorts` 完全一致（`getInput<T>` 的类型不匹配 = 失败，`blackboard->set` 的类型不匹配 = 抛异常）。
   典型陷阱：`robot_msgs/AllRobotHP` vs `rm_decision_interfaces/AllRobotHP`（§43）、
   `rm_decision_interfaces/GameStatus` vs `robot_msgs/GameStatus`。

## 8.3 spin / executor 要求（按节点分三类）

| 类别 | 节点 | 测试时是否需要外部 spin |
| --- | --- | --- |
| `RosTopicSubNode<T>`（自带 callback group + executor，tick 内 `spin_some`） | 9 个 `Sub*`（含未注册的 `SubArmors`）+ `IsSemantic` | **不需要**（但 publisher 必须 `transient_local`，见 8.4） |
| `RosActionNode<T>`（自带 callback group + executor，tick 内 `spin_until_future_complete`） | `SendGoal` | **不需要** |
| 裸订阅 / TF（回调落在节点默认 callback group） | `SubTargetPos`、`SubGimbalTarget`（都在 `update_msg` 节点）、`MoveAround`/`PursueGoal` 的 `/map`（`update_msg`）、`GetCurrentLocation`（`get_current_location` 节点）、`IsAutoAimOK`/`IsRemoteStatus`/`IsGameTime`/`IsChassisEnergyOK`（`update_msg`） | **需要** —— 这些节点必须挂在被 spin 的 rclcpp 节点上；主程序已把 `update_msg`/`get_current_location` 加入 `SingleThreadedExecutor`（`rm_behavior_tree.cpp:150-154`） |
| 裸订阅但**没进 executor** | `PursueEnemy`（独立节点 `pursue_enemy`） | 严格说需要；**当前主程序没有 spin 它** → 它的 `/map` 订阅与 action 回调永不执行（见 §21 坑 4） |
| 无 ROS 交互 | 其余条件/控制/装饰/工具节点 | 不需要 |

> 测试若自建 harness：最省事的做法是模仿 `rm_behavior_tree.cpp` —— 一个 `SingleThreadedExecutor`，
> 循环 `spin_some(5ms)` + `tickOnce()` + `sleep(10ms)`（**不要用 `tickWhileRunning()`**，原因见该文件 156-164 行的注释：
> RUNNING 时永不返回，裸订阅永远收不到数据）。

## 8.4 QoS 兼容性（最容易导致“节点在跑但收不到数据”）

- 所有 `RosTopicSubNode` 节点（9 个 `Sub*` + `IsSemantic`，共 10 个）的订阅 QoS = **KeepLast(1) / reliable / transient_local**（源码与安装副本一致）。
  DDS 要求 publisher 的 durability ≥ subscriber 的 → **发布端必须 `transient_local`**，例如：
  `ros2 topic pub --qos-durability transient_local --qos-reliability reliable -r 10 /game_status ...`
  默认 `ros2 topic pub` 是 volatile → **一条都收不到**（也不报错）。
- `IsAutoAimOK`/`IsRemoteStatus`/`IsGameTime`/`IsChassisEnergyOK` 的裸订阅是 **depth 10 / reliable / volatile**（默认 QoS），
  对它们用 volatile 发布即可；**同一个话题喂两类节点时，用 transient_local 发布可同时满足两者**。
- 发布节点（`Pub*`）的 publisher 是 `create_publisher(topic, 1)` = depth 1 / volatile / reliable →
  测试订阅端用默认 QoS 就能收到，但**只能看到最近一条**，不要用 `ros2 topic echo` 的“历史”行为做断言。
- `MoveAround` / `PursueGoal` 的 `/map` 订阅是 `transient_local+reliable`（latched 语义），`PursueEnemy` 是 `best_effort`。
  用 `map_server`/`nav2_map_server` 发布的 `/map` 是 latched+reliable，两类都能连；**自己用 `ros2 topic pub` 发地图时必须 `transient_local`**，
  否则 `MoveAround`/`PursueGoal` 永远“没有地图”。

## 8.5 必须放 `ReactiveSequence` / 必须每 tick 重新 tick 的节点

| 节点 | 原因 |
| --- | --- |
| `PursueGoal` | 头文件注释明确要求；普通 `Sequence` 里 SUCCESS 后不再重算，追不上移动目标 |
| `MoveAround` + `SendGoal` 组合 | `MoveAround` 每次 tick 只撒一个点，`SendGoal` 需要新的 `goal_pose` 才会重发（见 §18 坑 4）；示例树用 `Fallback + RetryUntilSuccessful` |
| 所有 `Sub*` 数据面节点 | 只有被 tick 才会 `spin_some` 并把最新消息写黑板；放在 `Sequence` 里第一次 SUCCESS 后就不再更新 → 下游永远读旧值 |
| `ActionSelector` | 需要每 tick 检查动作号变化并 halt 旧子树 |
| `RateController` | 靠每 tick 判断 `elapsed`（放 `Sequence` 里只会执行一次） |
| `IsGameTime` | 会返回 RUNNING；放普通 `Sequence` 里会卡住整棵树 |

反过来：**`WaitDuration`、`SendGoal`、`PursueEnemy` 这类返回 RUNNING 的节点不能放在 `Sequence` 的“一次性”位置**，
需要父节点每 tick 重新 tick 它们（`Sequence` 本身会记住 RUNNING 子节点并继续 tick，这是允许的；但 `ReactiveSequence`
会在重新 tick 前先 halt 前面已成功的节点，对 `SendGoal` 意味着取消导航 —— 这两种语义要按测试目的选）。

## 8.6 构造期读取的端口（运行期改黑板/改 XML 无效）

| 节点 | 构造期读取的端口 | 备注 |
| --- | --- | --- |
| `MoveAround` | `map_topic`、`expected_dis`、`goal_clearance`、`max_resample`、`allow_unknown` | `message` 是运行期读；构造期还建 `/map` 订阅 |
| `PursueGoal` | `target_frame`、`map_topic`、`pursue_radius`、`update_interval`、`min_goal_change`、`wall_check_clearance` | `armors`/`current_location` 运行期读；`wall_check_clearance>0` 时构造期建订阅 |
| `RateController` | `hz` | `period_` 固定 |
| `SubGimbalTarget` | 无（全部 tick 读），但 `target_frame_` 硬编码 `map`、`sub_` 首次 tick 建一次 | `topic_name` 首次 tick 生效 |
| `SubTargetPos` | 无（全部 tick 读），`sub_` 首次 tick 建一次 | `source_frame_` 构造默认 `chassis` 被端口默认 `camera_init` 覆盖 |
| `SendGoal` | `action_name`（端口是字面量时构造期建 client，2 s 等待 server） | `goal_pose` 每次发目标时读 |
| 9 个 `Sub*` + `IsSemantic`（`RosTopicSubNode` 共 10 个，含未注册的 `SubArmors`） | `topic_name`（字面量时构造期建订阅；`{blackboard}` 时首 tick 建） | 二次 `createSubscriber` 会抛 `RuntimeError` |

## 8.7 时间 / 时钟

- 主程序用 `RCLCPP` 节点时钟 `node_->now()`（`rm_behavior_tree` 节点），`use_sim_time` 由 launch 传入（默认 `False`）。
- `GetCurrentLocation`/`SubTargetPos`/`SubGimbalTarget`/`PursueEnemy` 的 TF Buffer 都用各自节点的 `get_clock()` → 跟随 `use_sim_time`。
- `WaitDuration` 用 `steady_clock`、`RateController` 用 `high_resolution_clock`、`ActionSelector` 冷却用 `steady_clock`
  → **与仿真时间无关**，`use_sim_time=true` + Gazebo 暂停时这些节点照走。
- `IsGimbalTargetValid` 自建节点且不传 `use_sim_time` → 用系统时钟（内部自洽，但和日志时间轴不一致）。

## 8.8 消息/坐标系先后的硬依赖

```
SubTargetPos ──armors──► IsDetectEnemy / IsTargetInZone / PursueGoal / PursueEnemy
GetCurrentLocation ──current_location──► MoveAround / PursueGoal / PursueEnemy / IsDetectEnemy
MoveAround|PursueGoal ──goal_pose──► SendGoal ──navigate_to_pose──► Nav2
SubGimbalTarget ──gimbal_target_valid/x/y, target_pose──► IsGimbalTargetValid / IsGimbalTargetInZone / SendGoal
SubAllRobotHP ──robot_hp──► IsOutpostOK / IsEnemyOutpostOK / IsBaseAttacked / IsBaseStatusOK
SubRobotStatus ──robot_status──► IsStatusOK / IsAttacked
SubGameStatus ──game_status──► IsGameTime / IsTimeToBullet
SubRemoteStatus ──remote_status──► IsRemoteStatus（也可走裸订阅）
SubAutoAimStatus ──auto_aim_status──► IsAutoAimOK（也可走裸订阅）
SubChassisEnergy ──chassis_energy──► IsChassisEnergyOK（也可走裸订阅）
SubRuneStatus ──rune_status──► IsRuneStatus
SubDecisionNum ──decision_num──► ActionSelector / DecisionSwitch
```

**同 tick 顺序**：`Sub*` 写在 `ReactiveSequence` 最前面，后续节点同一 tick 就能读到本 tick 写入的黑板值
（`Sequence` 里如果 `Sub*` 已 SUCCESS，它不会再被 tick，值就不再刷新）。

## 8.9 明显的不一致 / 待上报问题清单（汇总）

1. `IsAttacked` 的 `timeout_msec` 不起作用（不会保持 SUCCESS），与同族 `IsBaseAttacked` 语义相反。
2. `IsChassisEnergyOK` 把“剩余能量(J)”当 0/1 flag，且 SUCCESS = 能量不足（名字与语义相反）。
3. `IsFriendOK` 的消息类型是 `rm_decision_interfaces/AllRobotHP`，与 `SubAllRobotHP` 输出的 `robot_msgs/AllRobotHP` 不兼容。
4. `IsDetectEnemy` 的 `debounce_ms` 声明了但完全不读（无去抖）；`current_location` 的默认值 `""` 对消息类型无意义。
5. `SendGoal` 的 `action_name` 默认值来自 `params_send_goal.default_port_value = "goal_pose"`（复制粘贴错误），漏写会连错 server。
6. `PursueEnemy` 的 `enemy_timeout` 端口被硬编码 10.0 覆盖；`pursue_distance_threshold`、`robot_base_frame` 是死端口；
   `arrived_at_enemy_`/`navigation_complete_` 从未参与返回判定；其 `nh` 没进 executor。
7. `PubAutoAimMode` 的 `key_name` 是死端口（`auto_aim_mode` 的默认值让它永不回退）。
8. `RateController` 的 `hz` 无有效性校验（0 → inf，负数 → 每 tick 都跑），且构造期读取。
9. `DecisionSwitch` 子节点少于 2 个时 `children_nodes_[1]` 越界；RUNNING 子节点被折叠成 FAILURE。
10. `IsGameTime` / `IsTimeToBullet` 的必填 int 端口缺失时使用未初始化变量（UB）。
11. `SubGimbalTarget` 的 `target_frame_` 硬编码 `map`；首帧受静态 TF 竞态影响会丢目标。
12. `SubTargetPos` 的 `source_frame_` 构造默认 `chassis` 是死代码（被端口默认 `camera_init` 覆盖）；
    TF 失败且无历史点时输出 (0,0,0) 的 armor 但仍返回 SUCCESS。
13. `Not` 的 `.so` 被编译安装却从未注册；`SubArmors`/`IsBatteryOK` 未编译（后者源码本身无法编译：`BT::NodeBattery`、`robot_msgs::msg::RobotBattery` 都不存在）。
14. 三个血条条件比较符不一致：`IsOutpostOK` 用 `>`，`IsEnemyOutpostOK`/`IsBaseStatusOK` 用 `>=`。
15. `IsTargetInZone` 全 0 角点 + armor(0,0) 会误判 SUCCESS；`SubTargetPos` 在敌人消失时正好会产出 (0,0) 点（另一分支是空数组）。

---

*本文档由静态源码清点生成，未修改任何源码；所有结论均可按条目中的文件/行号复核。*
