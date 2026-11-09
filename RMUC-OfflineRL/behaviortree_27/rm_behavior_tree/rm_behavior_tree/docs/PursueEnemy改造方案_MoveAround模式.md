# PursueEnemy 改造方案（对齐 MoveAround 模式）

> 状态：**方案稿，未改动任何代码**。
> 目标读者：后续实施者（人或 AI）。
> 相关文件：`plugins/action/pursue_enemy.cpp`、`include/rm_behavior_tree/plugins/action/pursue_enemy.hpp`、`src/rm_behavior_tree.cpp`、`config/rmul_2026_3v3_sentry.xml`、`config/test_move_around.xml`。

---

## 0. 结论

**可行。** 现有 PursueEnemy 的核心算法（敌人选取、滑动窗口滤波、交战点计算、贴墙检查）可以整体保留，需要重写的是"外围"：生命周期、ROS 通信、导航执行三部分。

改造同时能根除当前 4 个已确认缺陷（见 §1）。但改造**不是纯内部重构**：必须与 XML 同批修改，否则行为树直接加载失败（原因见 §8.1）。

本文给出两条路线：

- **路线 B（推荐，本文主体）**：完整对齐 MoveAround 模式——`SyncActionNode` 只算点、输出 `goal_pose`，执行交给 `SendGoal`。
- **路线 A（备选，§11）**：3 行最小修复，保留现有语义，只消灭缺陷。适合赛前只求稳。

---

## 1. 现状与问题（证据）

| # | 问题 | 证据位置 |
| --- | --- | --- |
| P1 | `enemy_timeout` 端口读了之后被硬编码覆盖成 10.0，端口实际无效（主树写的是 3.0） | `pursue_enemy.cpp:79-81`（读端口）、`:105`（覆盖） |
| P2 | `pursue_enemy` 这个 rclcpp 节点不在 executor 里 → `/map` 订阅回调与 action 结果回调都不被处理；`goal_handle_` 不会被赋值，`async_cancel_goal` 无效 | 节点创建 `rm_behavior_tree.cpp:92-94`；executor 名单 `:133-138`（只有 `node`/`update_msg`/`get_current_location`/`send_goal`/`pub_default`） |
| P3 | "到达追击点后停下自瞄"的 SUCCESS 分支未实现：`arrived_at_enemy_`、`navigation_complete_` 被回调赋值但 `onRunning` 从不使用；唯一 SUCCESS 路径是发点次数用尽 | 赋值 `:153,155,302,304`；SUCCESS 分支 `:267-272`；头文件注释 `pursue_enemy.hpp:30` 与实现不符 |
| P4 | `/map` 订阅用 `best_effort`，而 `/map` 通常是 latched，订阅晚了收不到（MoveAround 的注释已记录该坑） | `pursue_enemy.cpp:495` vs `move_around.cpp:50-53` |

附带事实（供实施者判断，不必修）：

- **TF 是死代码**：`calculatePursuePose` 里 `target_frame_ == robot_location.header.frame_id` 恒成立（`GetCurrentLocation` 返回的 `TransformStamped.header.frame_id` 就是它的 `target_frame`），所以 `:423-425` 提前返回，后面的 TF 查询与 fallback 永不执行。→ 改造可整体删除 `tf_buffer_`/`tf_listener_`。
- **`calculateAngleToEnemy` 只有声明没有定义**，也没人调用（`pursue_enemy.hpp:71-73`）→ 删除。
- **"敌人丢失"在上游已判**：`SubTargetPos` 在无目标时 `armors` 清空并返回 FAILURE（`sub_target_pos.cpp:246-253`），主树两个分支还各有一个 `IsDetectEnemy`（`:83-86`、`:164-167`）。PursueEnemy 内部的 `enemy_lost` 判定是重复劳动。
- **`plugins/action/pursue_enemy_analysis.md` 已过期**：描述的是"同时继承 `rclcpp::Node`"的旧版本，且其中的行为描述（敌人消失返回 SUCCESS、变化 10cm 才更新）与当前代码不符。改造时不要以它为准，建议一并标注废弃或删除。

---

## 2. "MoveAround 模式"的定义

拆成两条独立原则，**必须一起用**才成立：

1. **职责单一**：本节点只负责"读黑板 → 算点 → 写 `goal_pose`（字符串）"，不碰导航通信；执行、取消、重发全部由 `SendGoal`（`RosActionNode<NavigateToPose>`）承担——它的 `halt()` → `cancelGoal()` 是现成能力。
2. **挂到被 spin 的共享 ROS 节点上**：`RosNodeParams.nh` 传 `params_update_msg.nh`（在 executor 名单里），而不是自建节点。MoveAround 的注释（`rm_behavior_tree.cpp:102-104`）已经写明这个理由。

基类随之从 `StatefulActionNode` 变为 `SyncActionNode`：不再有 `onStart/onRunning/onHalted`，每 tick 一个结论。

---

## 3. 目标接口

### 3.1 头文件草案

```cpp
/**
 * @brief 追击敌人（MoveAround 模式）：只负责计算追击点，执行交给 SendGoal
 *
 * tick 逻辑：
 * - 读取 armors / current_location，缺失或无敌 → FAILURE
 * - 可选：机器人与敌人之间被墙挡住 → FAILURE
 * - 追击点相对上次输出变化超过 pursue_distance_threshold 时才更新 goal_pose
 * - 输出 "x;y;z;qx;qy;qz;qw" 字符串，配 SendGoal 使用
 */
class PursueEnemyAction : public BT::SyncActionNode
{
public:
  PursueEnemyAction(const std::string & name, const BT::NodeConfig & conf,
                    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<auto_aim_interfaces::msg::Armors>("armors"),
      BT::InputPort<geometry_msgs::msg::TransformStamped>("current_location"),
      BT::InputPort<double>("pursue_radius", 2.0, "保持的交战距离 / m"),
      BT::InputPort<double>("pursue_distance_threshold", 0.5, "追击点变化超过该距离才更新 / m"),
      BT::InputPort<double>("update_interval", 0.5, "更新间隔 / s，0 = 每 tick 更新"),
      BT::InputPort<std::string>("target_frame", "map", "输出目标坐标系"),
      BT::InputPort<std::string>("map_topic", "/map", "贴墙检查用的栅格地图话题"),
      BT::InputPort<double>("wall_check_clearance", 0.0, "贴墙检查安全间隔 / m，0 = 关闭"),
      // 兼容期保留（见 §8.2），实施后期删除
      BT::InputPort<double>("enemy_timeout", 3.0, "[deprecated] 敌人数据超时，已由上游判定"),
      BT::InputPort<int>("max_pursuit_points", 10, "[deprecated] 已无意义"),
      BT::InputPort<std::string>("action_name", "navigate_to_pose", "[deprecated] 改由 SendGoal 使用"),
      BT::InputPort<std::string>("robot_base_frame", "gimbal_yaw", "[deprecated] 未使用"),
      BT::OutputPort<std::string>("goal_pose", "输出目标 \"x;y;z;qx;qy;qz;qw\"")
    };
  }

  BT::NodeStatus tick() override;

private:
  // 保留
  geometry_msgs::msg::Point getEnemyPosition(const Armors & armors);
  geometry_msgs::msg::Point getFilteredEnemyPos();
  geometry_msgs::msg::PoseStamped generatePursuePose(const Point& enemy, const Point& robot, double radius);
  geometry_msgs::msg::PoseStamped calculatePursuePoseWithFilteredPos(const Point& filtered_enemy, const TransformStamped& robot_location);
  bool hasObstacleBetweenRobotAndEnemy(const Point& robot, const Point& enemy);
  void initMapSubscription();
  static std::string formatGoalPose(const geometry_msgs::msg::PoseStamped& pose);

  // 新增状态
  std::string last_goal_pose_str_;   // 已输出到黑板的字符串，用于"没变就不写"
  /* 保留：enemy_pos_history_、last_update_time_、map_sub_、latest_map_、map_mutex_ */
};
```

### 3.2 tick() 流程

```
tick():
  1. 读 armors / current_location
       缺失 / armors 为空 → FAILURE
  2. 若 wall_check_clearance > 0：
       无地图 → FAILURE（保守；与 MoveAround 的"校验不了就不发点"一致）
       机器人⇄敌人之间有墙 → FAILURE
  3. 需要更新吗？（距上次更新 < update_interval 且点未大改）→ 直接 SUCCESS，不写端口
  4. 滤波取最近敌人 → generatePursuePose → formatGoalPose
  5. 与 last_goal_pose_str_ 相同 → SUCCESS（不写，避免 SendGoal 抖动）
     不同 → setOutput("goal_pose", str) → SUCCESS
```

**与现状的语义差异（必须确认可接受）**：SUCCESS 现在表示"这一 tick 给出了一个有效追击点"，不再表示"追击结束"。"什么时候不再追、改为自瞄"必须由树结构或上游条件表达（见 §6.2）。

---

## 4. 代码级改动清单

### 4.1 删除

| 项 | 位置 | 理由 |
| --- | --- | --- |
| `nav_action_client_`、`goal_handle_`、`goal_sent_`、`navigation_complete_`、`arrived_at_enemy_`、`last_result_code_` | hpp:115-120 | 执行职责移交 SendGoal |
| 两个 `send_goal_options` 回调 + `async_send_goal` / `async_cancel_goal` | cpp:133-166、287-312 | 同上（P2 随之消失） |
| `tf_buffer_` / `tf_listener_` 及其构造、`calculatePursuePose` 的 TF 分支 | hpp:127-128、cpp:49-50、420-444 | 死代码（§1 附带事实） |
| `calculateAngleToEnemy` 声明 | hpp:71-73 | 无定义、无调用 |
| `enemy_lost` 判定与 `enemy_timeout_`、`last_armor_stamp_`、`first_armor_stamp_`、`has_received_armor_` | cpp:197-224、hpp:136-138 | 上游已判（P1 随之消失） |
| `pursuit_points_sent_`、`max_pursuit_points_` | cpp:267-272、hpp:96,135 | Sync 模式下"一次追击会话"语义不成立（§5） |
| `action_name_`、`robot_base_frame_` | hpp:91,93 | 不再使用 |
| `onStart` / `onRunning` / `onHalted` | hpp:55-59 | 基类更换 |
| 头文件里 `rclcpp_action`、`nav2_msgs`、`tf2_ros` 的 include | hpp:8-11 | 不再需要；`<optional>` 也可去 |

### 4.2 保留（核心算法，几乎不用改）

- `getEnemyPosition`（cppi:329-370）：选最近装甲，直接用 map 系坐标，**无 TF**。
- 滑动窗口滤波 `enemy_pos_history_` + `getFilteredEnemyPos`（cpp:570-587，`kEnemyHistorySize = 3`）。
- `generatePursuePose`（cpp:372-407）：按 `pursue_radius` 在"敌人→自身"方向退让，带 yaw。
- `calculatePursuePoseWithFilteredPos`（cpp:446-479）：**简化版**——去掉 TF 分支后只剩 `generatePursuePose(...)` 直接返回。
- `hasObstacleBetweenRobotAndEnemy`（cpp:504-568）+ `initMapSubscription`（cpp:481-502）：**QoS 要改成 MoveAround 的写法**：

```cpp
rclcpp::QoS map_qos(rclcpp::KeepLast(1));
map_qos.transient_local().reliable();     // 不要用 best_effort（见 P4）
```

- 墙检查的坐标不在图内时"放行"的宽松策略（cpp:555-557）保留，但建议加节流日志。

### 4.3 新增

| 项 | 说明 |
| --- | --- |
| `formatGoalPose` | 照抄 `move_around.cpp:169-178` 的格式化：`std::fixed << setprecision(4)`，拼成 `"x;y;z;qx;qy;qz;qw"`；**PursueEnemy 要把 `generatePursuePose` 算出的 yaw 转成四元数**（`qz=sin(yaw/2)`、`qw=cos(yaw/2)`），这点比 MoveAround 多一步 |
| `last_goal_pose_str_` | 与上次输出比对，避免无意义写黑板（写黑板会让 SendGoal 误判"目标变了"） |
| 节流判定 | `elapsed < update_interval` 且新点与上次点距离 < `pursue_distance_threshold` → 不更新 |

---

## 5. 状态与重置策略（SyncActionNode 的隐蔽差异）

`StatefulActionNode` 每次 `onStart` 都会重置状态；**`SyncActionNode` 的实例在整棵树生命周期内只构造一次**，成员变量跨"分支重启"一直存在。因此：

| 状态 | 处理 |
| --- | --- |
| `enemy_pos_history_` | 保留但需**显式重置条件**：例如连续 N tick 未更新点、或敌人 ID/位置突变（跳变 > 1 m）时 `clear()`，否则会把几分钟前的历史混进均值 |
| `last_update_time_` | 保留；`update_interval <= 0` 时视为"每 tick 更新" |
| `last_goal_pose_str_` | 保留；**节点重建时才清空**（正常情况不会重建） |
| 计数器类 | 一律删除，不要再引入"一次会话"计数器 |

> 补充：主循环是 `executor.spin_some(5ms)` + `tree.tickOnce()` + `sleep(10ms)`（`rm_behavior_tree.cpp:143-148`），即约 100 Hz tick。所以"每 tick 都算点"必须靠 `update_interval` 节流，否则会以 100 Hz 刷新黑板。

---

## 6. 注册、CMake 与 XML

### 6.1 注册（`rm_behavior_tree.cpp`）

```cpp
// 删除 92-94 行的 params_pursue_enemy（自建节点，不在 executor 里）
// 改为复用 update_msg（已在 :135 add_node），与 MoveAround 完全一致
RegisterRosNode(factory, BT::SharedLibrary::getOSName("pursue_enemy"), params_update_msg);
```

`plugins/action/pursue_enemy.cpp` 末尾的 `CreateRosNodePlugin(..., "PursueEnemy")` 保持不变（仍需要 `RosNodeParams` 拿共享节点做地图订阅）。CMake target 不用动（仍链接 `map_accessor.cpp`/`key_point_loader.cpp`/`visibility_checker.cpp`）。

**是否需要专门写一个收发 ROS 的节点？不需要。** 改造后 PursueEnemy 的收发需求全部落在现成设施上：

| 收发需求 | 现成承担者 | 依赖主 executor？ |
| --- | --- | --- |
| 敌人位置 `armors` | `SubTargetPos`（读黑板） | **是**（裸订阅，挂 `update_msg`） |
| 自身位姿 `current_location` | `GetCurrentLocation`（读黑板） | **是**（TF 监听器挂 `get_current_location` 节点） |
| 对手/赛场状态 | `Sub*` 系列 | 否（`RosTopicSubNode` 自带 callback group + 自带 `spin_some`，见 `bt_topic_sub_node.hpp:302`） |
| `/map` 贴墙检查 | 复用 `update_msg`（MoveAround 范例） | **是**（裸订阅，走节点默认 callback group） |
| Nav2 目标发送/反馈/结果 | `SendGoal`（`RosActionNode`） | 否（自带 callback group + `spin_some` / `spin_until_future_complete`，`bt_action_node.hpp:404-412`） |

结论：**只有"裸订阅"才要求把节点挂进主 executor**——这正是当前 P2 缺陷的成因。因此改造是"少建一个节点"，不是"多建一个节点"。若将来确实需要独立节点（独立参数/命名空间/QoS），两条路：① 建了就**必须** `executor.add_node(...)`；② 改用 `BT::RosTopicSubNode<T>` 基类，自带 spin、免管 executor。

### 6.2 XML：两处调用点都要改（**这是行为变了的根源**）

**调用点 1**（`config/rmul_2026_3v3_sentry.xml:75-114`，被 `ForceSuccess` 包着的**普通 `Sequence`**）：

现在是"PursueEnemy 返回 RUNNING 卡住，才不会走到后面那个硬编码固定点"。改成每 tick SUCCESS 后，**`Sequence` 会立刻推进到 `SendGoal(5.238;-4.222)`**——机器人直接跑去驻守点。必须重排：

```xml
<ForceSuccess>
  <Fallback>
    <ReactiveSequence>
      <GetCurrentLocation current_location="{current_location}" target_frame="map" source_frame="gimbal_yaw"/>
      <SubTargetPos topic_name="/auto_aim_target_pos" target_frame="map" source_frame="odom" armors="{armors}"/>
      <IsDetectEnemy message="{armors}" max_range="-1.0" current_location="{current_location}" debounce_ms="200"/>
      <!-- 其他发布节点... -->
      <PursueEnemy armors="{armors}" current_location="{current_location}"
                   pursue_radius="2.5" pursue_distance_threshold="0.5"
                   update_interval="0.5" target_frame="map"
                   map_topic="/map" wall_check_clearance="0.0"
                   goal_pose="{pursue_goal}"/>
      <SendGoal goal_pose="{pursue_goal}" action_name="navigate_to_pose"/>
    </ReactiveSequence>
    <!-- 追丢/追不到的备份驻守点 -->
    <SendGoal goal_pose="5.238;-4.222;0.182; 0; 0; 0; 1" action_name="navigate_to_pose"/>
  </Fallback>
</ForceSuccess>
```

**调用点 2**（`:168-184`，已经是 `ReactiveSequence` + `Fallback`）：把 `Fallback` 的第一支从"单独一个 PursueEnemy"改成 `ReactiveSequence{PursueEnemy → goal_pose, SendGoal}`，保留原来的固定点（`:181`）作为第二支。

**`<TreeNodesModel>`（`:313-330` 附近）**：同步更新 PursueEnemy 的端口声明（新增 `goal_pose`，删掉废弃端口），否则 Groot2 里显示的还是旧端口。

### 6.3 换点抖动问题（②号坑）

`SendGoal::onFeedback`（`send_goal.cpp:87-96`）发现 `goal_pose` 端口值变了会 `halt()` 并返回 **FAILURE**。在 `ReactiveSequence` 中这一次 FAILURE 会让整条序列失败 → 那一 tick 落到 `Fallback` 的备份分支发一次固定点目标 → 下一 tick 又切回追击 → Nav2 目标来回抖。三种处理方式：

| 方案 | 做法 | 评价 |
| --- | --- | --- |
| ① 改 SendGoal（推荐） | 加端口 `resend_on_change`（默认 `false` 保持现有行为）。为 `true` 时：`cancelGoal()` → 立即用新目标重发 → 继续返回 RUNNING，不进 FAILURE | 改动小、不影响其他分支，根治抖动 |
| ② 新建 SendGoalReactive | 独立节点，到达判定改用 Nav2 的 result，彻底替换"连续 5 次 <0.2"的 hack（`send_goal.cpp:107-115`） | 最干净，但要新增节点 + 改所有调用点，成本最高 |
| ③ 不改，接受抖动 | 靠 `pursue_distance_threshold` 把换点频率压低；或把备份固定点换成"最近一次有效点" | 最省事，但抖动依然存在，只是变少 |

---

## 7. 实施步骤（每步可编译、可单独验证）

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| 0 | 备份现有 `config/*.xml`；`pursue_enemy_analysis.md` 标注废弃 | — |
| 1 | 只做 §4 的 hpp/cpp 改造（**端口先全保留**，新增 `goal_pose`），注册改为 `params_update_msg` | `colcon build --packages-select rm_behavior_tree` 通过；老 XML 仍能加载（老端口还在，行为退化为"只算点不导航"） |
| 2 | 改 XML 调用点 1，在仿真实测"是否换点抖动" | 观察日志：追击点更新时是否出现固定点目标 |
| 3 | 按 §6.3 选一种方案消除抖动 | 目标切换时无固定点下发 |
| 4 | 改 XML 调用点 2 + `<TreeNodesModel>` | 两条分支都按预期跑 |
| 5 | 删除废弃端口（`enemy_timeout`/`max_pursuit_points`/`action_name`/`robot_base_frame`），并 grep 全仓 XML 确认无残留 | 加载无 `not in the providedPorts()` 异常 |
| 6 | 更新 `docs/行为树动作节点清单.md` 的 PursueEnemy 章节 | 文档与代码一致 |

---

## 8. 兼容性（**最容易翻车的地方**）

### 8.1 删端口 = 老 XML 直接加载失败

BehaviorTree.CPP 在实例化节点时会校验 XML 属性，**只要 XML 里出现 `providedPorts()` 中没有的端口就抛异常**：

```cpp
// behaviortree_cpp/src/xml_parsing.cpp:653-658
throw RuntimeError(StrCat("a port with name [", port_name,
                          "] is found in the XML, but not in the providedPorts()"));
```

所以：**删端口与改 XML 必须同批完成**，否则 `createTreeFromFile` 直接失败、机器人起不来。这也是 §7 阶段 1 要"先保留老端口"的原因。（注意 `<TreeNodesModel>` 只影响 Groot2 显示，运行时校验看的是 `providedPorts()`。）

### 8.2 兼容期策略

保留老端口声明（节点不再读取，静默忽略）一个版本，等所有 XML（含仿真/测试树、别人本地的树）都更新完再删。代价是端口列表暂时冗余。

---

## 9. 验证方案

1. **静态**：`grep -rn "PursueEnemy" config/ scripts/ launch/ docs/` 确认所有调用点都更新；编译无 warning（尤其 `-Wunused-variable` 会暴露没删干净的成员）。
2. **启动**：`bt_xml` 指向主树，确认无 `not in the providedPorts()` / `is not a registered node` 异常；Groot2（1667 端口）能连上并显示新端口。
3. **行为**（仿真）：
   - 敌人静止 → 追击点稳定，Nav2 只有一次目标，无反复重发；
   - 敌人移动 > `pursue_distance_threshold` → 恰好一次目标更新，无固定点串入；
   - 敌人消失 → `SubTargetPos` FAILURE → 分支失败并 `halt()` SendGoal（日志出现 `onHalt called`）→ 落到备份驻守点；
   - `update_interval=0` 时确认不会 100 Hz 刷黑板（看日志频率）。
4. **回归**：`test_move_around.xml` 与 `ces` 子树不受影响（它们不用 PursueEnemy，但共用 `SendGoal`，若做了 §6.3 方案①需回归）。

---

## 10. 风险与回滚

| 风险 | 等级 | 缓解 |
| --- | --- | --- |
| XML 未同批更新导致树加载失败 | 高 | 阶段 1 保留老端口；阶段 5 前 grep 全仓 |
| 换点 FAILURE 引发 Nav2 目标抖动 | 中 | §6.3 方案①；阶段 2 先实测 |
| Sync 模式状态不重置导致追击点被历史数据拖住 | 中 | §5 的重置条件；用日志断言滤波窗口内容 |
| 丢失"追击期间独占子树"的语义 | 中 | 父节点必须用 `ReactiveSequence`；逐个调用点确认（§6.2） |
| 与 `SendGoal` 的到达判定（5×0.2）叠加后行为变化 | 低 | 阶段 3 一起观察 `distance_remaining` 日志 |

**回滚**：改动集中在 2 个源文件 + 1 个 XML；保留分支/备份即可整体回退。建议新建 Git 分支实施，XML 改动单独一个 commit 便于回退。

---

## 11. 备选路线 A：最小修复（3 行）

只想消灭缺陷、不动语义时：

```cpp
// rm_behavior_tree.cpp:92-94 改一行：复用已在 executor 的 update_msg（修 P2）
RegisterRosNode(factory, BT::SharedLibrary::getOSName("pursue_enemy"), params_update_msg);

// pursue_enemy.cpp:105 删除硬编码覆盖（修 P1）
-  enemy_timeout_ = 10.0;

// pursue_enemy.cpp:495 改 QoS（修 P4）
-  rclcpp::QoS(5).best_effort(),
+  rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
```

- 优点：不碰接口、不碰 XML、无加载失败风险；P1/P2/P4 全消。
- 缺点：P3（到达判定未实现）仍在；仍然是"一个节点同时管算点 + 导航 + 取消"的胖节点；与 MoveAround/SendGoal 的模式不统一。
- 建议：**如果近期有比赛，先做路线 A；路线 B 放到赛后再落。**

---

## 12. 证据索引

| 结论 | 位置 |
| --- | --- |
| 端口硬编码覆盖 | `plugins/action/pursue_enemy.cpp:79-81, 105` |
| SUCCESS 仅由发点次数触发 | `plugins/action/pursue_enemy.cpp:267-272` |
| 到达标记未使用 | `plugins/action/pursue_enemy.cpp:153,155,302,304`（赋值）、`184-316`（onRunning 全文未用） |
| 自有节点未进 executor | `src/rm_behavior_tree.cpp:92-94`、`:133-138` |
| `/map` QoS 不匹配 | `plugins/action/pursue_enemy.cpp:495` vs `plugins/action/move_around.cpp:50-53` |
| MoveAround 模式参照实现 | `plugins/action/move_around.cpp:13-41`（构造/端口/订阅）、`:67-106`（tick）、`:169-178`（格式化） |
| SendGoal 换点即 FAILURE | `plugins/action/send_goal.cpp:87-96` |
| SendGoal 到达判定 | `plugins/action/send_goal.cpp:107-115` |
| 上游已判敌人丢失 | `plugins/action/sub_target_pos.cpp:246-253`；主树 `:83-86`、`:164-167`（两处 `IsDetectEnemy`） |
| Armors 头时间戳每 tick 重写（不能用作陈旧判定） | `plugins/action/sub_target_pos.cpp:236` |
| 未知端口直接抛异常 | `behaviortree_cpp/src/xml_parsing.cpp:653-658` |
