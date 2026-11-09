# RMul_27 — RMUL 2027 哨兵决策工作空间

从 `../behaviortree_27` 复制出来的干净工作空间，用来**接替**它的职责：
跑哨兵决策行为树（BT.CPP v4 + ROS2 Humble + Nav2），并承载 2027 的 12 动作决策层。

成员包（与原工作空间同名同版本，**不要同时 source 两个工作空间**）：

| 包 | 作用 | 来源 |
| --- | --- | --- |
| `rm_behavior_tree` | 决策行为树主程序 + 全部插件节点 | 原样复制 |
| `rm_decision_interfaces` | `GameStatus` / `RobotStatus` / `AllRobotHP` / `DecisionNum` / `RobotControl` … | 原样复制 |
| `robot_msgs` | 电控/裁判链路消息（`RobotState` / `ChassisMode` / `AutoAimMode` …） | 原样复制 |
| `auto_aim_interfaces` | `Armors`（自瞄输出） | 原样复制 |
| `sp_msgs` | `EnemyStatusMsg` / `AutoaimTargetMsg`（给自瞄的黑白名单） | 原样复制 |
| `behaviortree_cpp` | BehaviorTree.CPP v4 本体（源码内嵌） | 原样复制 |
| `behaviortree_ros2` / `btcpp_ros2_interfaces` | BT↔ROS2 适配层 | 原样复制（在 `rm_behavior_tree/BehaviorTree.ROS2/` 下） |
| `btcpp_ros2_samples` | 适配层示例 | 原样复制（可不编） |

**没有带过来的包**（与决策无关，需要时从旧工作空间再拷）：

- `small_gicp`：点云配准库，属于定位栈，BT 不依赖；
- `pb_rm_interfaces`：无人引用。

---

## 构建

```bash
cd /home/w/pd2027/RMul_27
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 3     # 本机 14 GB 内存，别用默认并行度
source install/setup.bash
```

全量编译约 4–6 分钟（`behaviortree_cpp` 占大头）。

## 运行

```bash
# 只跑行为树（导航已在别的终端跑着）
./run_bt_nav.sh --bt-only

# 仿真：Gazebo + 导航 + 行为树 + 假裁判数据
./run_bt_nav.sh --mock-referee

# 只做环境预检
./run_bt_nav.sh --check
```

跑 2026 3v3 哨兵树（`rmul_2026_3v3_sentry.xml`）：

```bash
./run_2026_3v3_sentry.sh                      # 仿真：Gazebo + 导航 + Subtree
./run_2026_3v3_sentry.sh --reality            # 实车：导航 + 该树
./run_2026_3v3_sentry.sh --mock-referee       # 仿真调试决策必加（否则 IsGameTime 恒假）
./run_2026_3v3_sentry.sh --bt-only            # 只起该树（导航已在别处运行）
./run_2026_3v3_sentry.sh --tree-id MainTree   # 跑文件里的斜坡占位树
./run_2026_3v3_sentry.sh --literal            # 严格按 XML 里的入口（当前 = MainTree）
```

封装脚本默认把入口改成 `Subtree`（完整 3v3 逻辑），因为 XML 里的
`main_tree_to_execute="MainTree"` 指向的只是两条 `SendGoal` 的斜坡占位树。
改写只发生在 `log/run_bt_nav/trees/` 的副本上，原 XML 不动。

等价写法：`./run_bt_nav.sh --tree rmul_2026_3v3_sentry.xml`（`--tree` 接受 config/ 下的
文件名、相对路径或绝对路径）。

`run_bt_nav.sh` 起 BT 前会先等 nav2 的 `bt_navigator` 变 `active`（默认最多 120s，
`--wait-nav` 可调）。仿真里 `local_costmap` 激活要等 `odom↔gimbal_yaw_fake` 的 TF，
常要 20s+；BT 提前发目标会得到 `Failed to send goal response ... (timeout)`，
表现就像"行为树没起来"。

`run_bt_nav.sh` 是 `../behaviortree_27/run_bt_nav.sh` 的副本，只改了工作空间名与默认树
（`rmul_2027_sentry.xml`）。它依赖 `/home/w/pd2027/beijixiong` 的导航栈。

手工启动：

```bash
ros2 run rm_behavior_tree rm_behavior_tree --ros-args \
  -p bt_xml:=$PWD/rm_behavior_tree/rm_behavior_tree/config/rmul_2027_sentry.xml \
  -p use_sim_time:=false
```

后台清理：

```bash
./cleanup_bg.sh        # 列出并询问
./cleanup_bg.sh -n     # 只列出，不动手
./cleanup_bg.sh -y     # 直接清理
```

只匹配本工作空间/这台仿真（Gazebo、pb2025 导航、beijixiong 下的节点、
`rm_behavior_tree`、`mock_referee`），不会动 ros2 daemon 和其他工程。
先 SIGINT 让节点优雅退出，超时再 SIGKILL；GNOME Terminal 的标签页需要自己关。

---

## 与原工作空间的差异

1. **新增 `ActionSelector` 控制节点**（`plugins/control/action_selector.cpp`）
   12 路宏动作选择器，支持 `action_id`（雷达）/ `decision_num`（拨杆）两种来源，
   处理 RUNNING、切换时 halt 旧子树、子树 SUCCESS 后保持并在 `cooldown_ms` 后重入。
   原 `DecisionSwitch` 只写了 `case 1/2`，保留但已不用。
2. **新增主树 `config/rmul_2027_sentry.xml`**
   12 个动作分支 + 数据面守卫 + 点位表（map 居中系）+ 本地兜底（协同防守）。
   12 个分支是**内联**的，不用 `<SubTree>`——BT.CPP v4 的 SubTree 会开独立黑板，
   运行期写入的 `{robot_hp}` / `{armors}` 在子树里读不到，会直接 terminate（见 `docs/节点链路.md` §4）。
   旧树 `rmul_2026_3v3_sentry.xml`、`rmul_2026_7v7.xml`、`test*.xml` 全部保留。
3. **新增文档**
   - `docs/节点链路.md`：话题/动作总表、一次 tick 的时序、12 动作链路、缺的节点与插接点；
   - `docs/RM2027_十二动作规划.md`：12 动作的语义、规则硬前置、树结构、数据面清单、阶段与验收。

## 目录

```
RMul_27/
├── rm_behavior_tree/            # 主包 + BehaviorTree.ROS2 适配层 + rm_decision_interfaces
├── robot_msgs/                  # 电控/裁判消息
├── auto_aim_interfaces/         # 自瞄消息
├── sp_msgs/                     # 自瞄控制消息
├── behaviortree_cpp/            # BT.CPP v4 源码
├── wildwolf_serial_nav2/        # 仅 robot_msgs 的宿主目录（与旧工作空间一致）
├── docs/                        # 节点链路 + 12 动作规划
├── run_bt_nav.sh                # 一键启动（导航 + 行为树，--tree 可换树）
├── run_2026_3v3_sentry.sh       # 上面的封装：跑 2026 3v3 哨兵树
├── cleanup_bg.sh                # 清理 Gazebo / 导航 / 行为树 / 模拟裁判
└── build/ install/ log/         # colcon 产物
```

---

## 已知限制（下一步）

1. **12 动作里"能不能做"还是近似判据**：现在只有前哨血量（`/robot_hp`），没有阵地控制状态
   （`0x0101`）、回血 buff（`0x0204`）、允许发弹量/金币（`0x0208`）、脱战/姿态（`0x020D`）。
   接上这些包后，把 `IsOutpostOK` 换成 `IsOutpostStatus` / `IsGainPointActive` 即可，子树结构不用动。
   见 `docs/节点链路.md` §5。
2. **雷达动作还没接**：`ActionSelector` 的 `action_id` 端口已留好，等 `SubRadarSuggestion` +
   `radar_decision_adapter`（`0x0301` 子内容 `0x0210`）。
3. **点位需要标定**：`rmul_2027_sentry.xml` 里的 `goal_*` 来自 Arena-1 `rmuc2027` 场地图
   （已与旧 7v7 草稿里的硬编码值对上），但 `map` 帧与官方场地帧的变换 `T_map_field`
   必须上实车前用四点标定复核；到达判定建议改用增益点激活位闭环（规划文档 §0 第 7 条）。
4. **姿态仍是 1/2/3**：`PubRobotState` 是旧口径，2027 的强化姿态（4/5/6）要走 `0x0120`，
   需要新增 `SentryPosture.msg` 并在串口驱动里做唯一的编码转换（`RobotState` 的 4=待机 ≠ 协议 4=强化进攻）。
5. **`/terrain_zone` 是占位**：`IsSemantic` 依赖的语义话题在当前部署里恒为空串，别拿它当到达依据。
