#!/usr/bin/env bash
# =============================================================================
#  run_move_around_sim.sh —— 在仿真里跑 MoveAround「原地抖动躲弹」演示树
#
#  用法：
#     ./run_move_around_sim.sh                       # 默认：5 下、半径 0.5 m（原地抖动）
#     ./run_move_around_sim.sh --points 8 --radius 0.3   # 更接近正式参数
#     ./run_move_around_sim.sh --radius 1.5          # 放大一点，Gazebo 里更好看清
#     ./run_move_around_sim.sh --action /red_standard_robot1/navigate_to_pose
#     ./run_move_around_sim.sh --map /red_standard_robot1/map
#     ./run_move_around_sim.sh --ns red_standard_robot1   # ★ 推荐：和仿真里的导航对齐
#     ./run_move_around_sim.sh --dry-run             # 只打印将要做的事，不启动
#     ./run_move_around_sim.sh --no-check            # 跳过前置检查（启动更快）
#
#  它做什么：
#     1) source ROS 2（默认 /opt/ros/humble）与本工作空间的 install/setup.bash
#     2) 自动探测仿真里 Nav2 的 navigate_to_pose action 名 和 地图话题名
#        （仿真里通常带命名空间，比如 /red_standard_robot1/...，写死很容易找不到）
#     3) 用探测结果从模板 config/test_move_around.xml 生成一个临时 XML
#     4) 启动行为树节点，日志直接打在终端上
#
#  想看效果时你需要同时在跑：仿真（Gazebo）+ Nav2 + TF(map←gimbal_yaw)
# =============================================================================

set -uo pipefail   # 故意不加 -e：出问题要给人话提示，而不是默默退出

# ---------------------------------------------------------------------------
# 默认参数
# ---------------------------------------------------------------------------
POINTS=5                 # 抖几下（对应 XML 里的 <Repeat num_cycles>）
RADIUS=0.5               # 抖动半径 / m（演示 0.5 看得清；正式按 Project.btproj 用 0.3）
CLEARANCE=0.0            # 直线可达检查的安全间隔 / m
ALLOW_UNKNOWN=false      # 是否允许落在未知区域（栅格值 -1）
ACTION=auto              # auto = 自动探测 Nav2 action 名
MAP_TOPIC=auto           # auto = 自动探测地图话题名
NS=""                    # 命名空间（仿真里导航通常在 red_standard_robot1 下）
SIM_TIME=true            # 仿真里应为 true（节点会用 /clock）
DO_CHECK=true
DRY_RUN=false
ROS_SETUP=/opt/ros/humble/setup.bash

# ---------------------------------------------------------------------------
# 解析命令行参数
# ---------------------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --points)        POINTS="${2:-}";        shift 2 ;;
    --radius)        RADIUS="${2:-}";        shift 2 ;;
    --clearance)     CLEARANCE="${2:-}";     shift 2 ;;
    --allow-unknown) ALLOW_UNKNOWN="${2:-}"; shift 2 ;;
    --action)        ACTION="${2:-}";        shift 2 ;;
    --map)           MAP_TOPIC="${2:-}";     shift 2 ;;
    --ns)            NS="${2:-}";            shift 2 ;;
    --ros-setup)     ROS_SETUP="${2:-}";     shift 2 ;;
    --sim-time)      SIM_TIME="${2:-}";      shift 2 ;;
    --no-check)      DO_CHECK=false;         shift   ;;
    --dry-run)       DRY_RUN=true;           shift   ;;
    -h|--help)       sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "未知参数：$1（用 --help 看用法）"; exit 2 ;;
  esac
done

# ---------------------------------------------------------------------------
# 路径：脚本在  <ws>/rm_behavior_tree/rm_behavior_tree/scripts/  下
#       所以工作空间根目录 = scripts/../../..
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
TEMPLATE_XML="${SCRIPT_DIR}/../config/test_move_around.xml"
RUNTIME_XML="${TMPDIR:-/tmp}/move_around_test.xml"
INSTALL_SETUP="${WS_DIR}/install/setup.bash"

echo "=============================================================="
echo " MoveAround 仿真演示"
echo "=============================================================="
echo "  工作空间   : ${WS_DIR}"
echo "  模板 XML   : ${TEMPLATE_XML}"
echo "  运行 XML   : ${RUNTIME_XML}  （每次运行都会重新生成）"
echo "  点数/半径  : ${POINTS} 个点，半径 ${RADIUS} m"
echo "  sim time   : ${SIM_TIME}"
echo "  命名空间   : ${NS:-（无，节点跑在根命名空间下）}"
echo "--------------------------------------------------------------"

# ---------------------------------------------------------------------------
# 1) 环境
# ---------------------------------------------------------------------------
if [ ! -f "${ROS_SETUP}" ]; then
  echo "!! 找不到 ${ROS_SETUP}，请用 --ros-setup 指定你的 ROS 2 setup.bash"
  exit 1
fi

if [ ! -f "${INSTALL_SETUP}" ]; then
  echo "!! 找不到 ${INSTALL_SETUP}"
  echo "   先把工作空间编译一遍：cd ${WS_DIR} && colcon build --packages-select rm_behavior_tree"
  exit 1
fi

# ★ ROS 的 setup.bash 内部会读一些"可能没定义"的环境变量（例如 AMENT_TRACE_SETUP_FILES），
#   这和本脚本开头的 `set -u`（未定义变量立刻报错）冲突，
#   所以 source 期间临时关掉 nounset，source 完再打开。
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${INSTALL_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
  echo "!! source 之后仍然没有 ros2 命令，环境有问题"
  exit 1
fi

if [ ! -f "${TEMPLATE_XML}" ]; then
  echo "!! 找不到模板 XML：${TEMPLATE_XML}"
  exit 1
fi

# ---------------------------------------------------------------------------
# 2) 前置检查（只读操作，不会让机器人动）
# ---------------------------------------------------------------------------
if [ "${DO_CHECK}" = true ]; then
  echo ">>> 前置检查（最多等 10 秒）..."

  # 2.1 是否已经有另一个行为树实例在跑？
  #     （两个实例会抢同一个 Nav2，而且都要占 Groot2 的 1667 端口）
  if timeout 10 ros2 node list 2>/dev/null | grep -qx "/rm_behavior_tree"; then
    echo "!! 已经有一个 /rm_behavior_tree 节点在运行了"
    echo "   两组行为树会同时往 Nav2 发目标，请先把它关掉（Ctrl-C）"
    exit 1
  fi

  # 2.2 自动探测 Nav2 的 action 名
  if [ "${ACTION}" = "auto" ]; then
    DETECTED="$(timeout 10 ros2 action list 2>/dev/null | grep 'navigate_to_pose' | head -1 || true)"
    if [ -n "${DETECTED}" ]; then
      ACTION="${DETECTED}"
      echo "    自动探测到 action : ${ACTION}"
    else
      ACTION="navigate_to_pose"
      echo "!! 没有探测到 navigate_to_pose —— Nav2 可能还没起来"
      echo "   先按默认名 ${ACTION} 试；如果日志报 'not reachable'，请先启动 Nav2"
    fi
  fi

  # 2.3 自动探测地图话题名
  if [ "${MAP_TOPIC}" = "auto" ]; then
    DETECTED="$(timeout 10 ros2 topic list 2>/dev/null | grep -E '(^|/)map$' | head -1 || true)"
    if [ -n "${DETECTED}" ]; then
      MAP_TOPIC="${DETECTED}"
      echo "    自动探测到地图   : ${MAP_TOPIC}"
    else
      MAP_TOPIC="/map"
      echo "!! 没有探测到地图话题 —— MoveAround 校验落点时会一直失败"
      echo "   先确认 Nav2 的 map_server 起来了：ros2 topic hz ${MAP_TOPIC}"
    fi
  fi

  # 2.4 TF / 里程计是否在发（GetCurrentLocation 靠它拿位置）
  #     注意：仿真里 TF 通常带命名空间（/red_standard_robot1/tf），所以这里只要找到
  #     任何一个以 tf 结尾的话题就算通过；真正对齐要靠下面的 --ns 选项。
  if ! timeout 10 ros2 topic list 2>/dev/null | grep -Eq '(^|/)tf$'; then
    echo "!! 没有看到 /tf 话题 —— GetCurrentLocation 会失败，机器人位置拿不到"
    echo "   如果仿真里的 TF 被重映射到了别的名字（例如 /red_standard_robot1/tf），"
    echo "   请加上 --ns red_standard_robot1 重跑（本脚本会做和 launch 文件一样的 TF remap）"
  fi

  # 2.5 MoveAround 插件是否已经被编译出来了
  if [ ! -f "${WS_DIR}/build/rm_behavior_tree/libmove_around.so" ]; then
    echo "!! 找不到 build/rm_behavior_tree/libmove_around.so"
    echo "   请先编译：cd ${WS_DIR} && colcon build --packages-select rm_behavior_tree"
    exit 1
  fi
fi

# ---------------------------------------------------------------------------
# 3) 从模板生成运行用 XML（把参数替换进去）
# ---------------------------------------------------------------------------
sed -e "s|num_cycles=\"[0-9]*\"|num_cycles=\"${POINTS}\"|" \
    -e "s|expected_dis=\"[0-9.]*\"|expected_dis=\"${RADIUS}\"|" \
    -e "s|goal_clearance=\"[0-9.]*\"|goal_clearance=\"${CLEARANCE}\"|" \
    -e "s|allow_unknown=\"[a-zA-Z0-9]*\"|allow_unknown=\"${ALLOW_UNKNOWN}\"|" \
    -e "s|action_name=\"[^\"]*\"|action_name=\"${ACTION}\"|" \
    -e "s|map_topic=\"[^\"]*\"|map_topic=\"${MAP_TOPIC}\"|" \
    "${TEMPLATE_XML}" > "${RUNTIME_XML}"

# 如果 sed 出问题（比如某个模式没匹配到），这里能立刻发现
if ! grep -q "num_cycles=\"${POINTS}\"" "${RUNTIME_XML}"; then
  echo "!! 生成的 XML 里没有找到 num_cycles=${POINTS}，模板可能被改动过"
  exit 1
fi

echo "--------------------------------------------------------------"
echo "  最终使用："
echo "    action_name : ${ACTION}"
echo "    map_topic   : ${MAP_TOPIC}"
echo "    goal_clearance=${CLEARANCE}  allow_unknown=${ALLOW_UNKNOWN}"
echo "--------------------------------------------------------------"

CMD=(ros2 run rm_behavior_tree rm_behavior_tree
     --ros-args
     -p "bt_xml:=${RUNTIME_XML}"
     -p "use_sim_time:=${SIM_TIME}")

# 和 bejixiong_2/bj2_nav_bt_sentry.launch.py 里行为树节点的写法保持一致：
#   namespace=red_standard_robot1   →   -r __ns:=/red_standard_robot1
#   TF 必须 remap：ros2 的 TransformListener 订阅的是【绝对名】/tf，
#   而带命名空间的仿真把 TF 发在 /red_standard_robot1/tf 上；
#   "-r /tf:=tf" 把绝对名改写成相对名 tf，从而跟随命名空间。
if [ -n "${NS}" ]; then
  CMD+=(-r "__ns:=/${NS}" -r "/tf:=tf" -r "/tf_static:=tf_static")
fi

if [ "${DRY_RUN}" = true ]; then
  echo ">>> --dry-run：不启动，下面是将要执行的命令"
  printf '    %q' "${CMD[@]}"; echo
  echo ">>> 生成的 XML 内容（前 40 行）："
  sed -n '1,40p' "${RUNTIME_XML}" | sed 's/^/    /'
  exit 0
fi

# ---------------------------------------------------------------------------
# 4) 启动行为树
# ---------------------------------------------------------------------------
cat <<'TIP'
>>> 启动行为树（Ctrl-C 退出）。正常情况下你应该看到这样的日志：

    MoveAround: subscribing map topic '/map'                 ← 插件加载成功
    MoveAround: goal=(3.1234,-2.0456) center=(3.0000,-2.0000) r=1.50
    [SendGoal] Sending goal: [3.1, -2.0]
    [SendGoal] Arrived! distance=0.150, returning SUCCESS
    MoveAround: goal=(...)                                   ← 第 2 个点，重复到第 3 个

  要警惕的日志：
    MoveAround: no map on '...' yet            → 地图没收到（话题名/QoS/没起 map_server）
    MoveAround: no usable point after 20 tries → 20 次都落在墙里/未知区域
    [SendGoal] ... not reachable               → Nav2 没起，或 action_name 不对
    MoveAround: missing required input [message] → TF map←gimbal_yaw 没起来

--------------------------------------------------------------------------------
TIP

exec "${CMD[@]}"
