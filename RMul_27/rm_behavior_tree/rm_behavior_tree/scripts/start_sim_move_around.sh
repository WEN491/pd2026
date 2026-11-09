#!/usr/bin/env bash
# =============================================================================
#  start_sim_move_around.sh —— 一键起仿真 + 导航 + MoveAround「原地抖动躲弹」演示树
#
#  它按正确顺序做四件事：
#     1) 清 snap/GTK 污染变量 + 按 [ROS → 决策工作区 → BJ2 导航] 的顺序 source
#        （顺序错 = 找到错的包 = 导航"看起来起了实际没起"，所以这一步是硬编码的）
#     2) 修 LD_LIBRARY_PATH（/opt/MVS 的老 libusb 与 PCL 冲突、缺 tdt 的 libOsqpEigen）
#     3) 后台起 Gazebo，等仿真真的就绪（轮询 /clock）再继续
#     4) 前台起 导航 + 数据面 + 行为树（复用 bejixiong_2/run_bj2_bt.sh）
#
#  用法：
#     ./start_sim_move_around.sh                      # 一键全套（默认 5 下、半径 0.5 m）
#     ./start_sim_move_around.sh --radius 0.3 --points 8
#     ./start_sim_move_around.sh --no-gazebo          # Gazebo 已在别的终端跑着
#     ./start_sim_move_around.sh --nav-only           # 只起 Gazebo + 导航，不起行为树
#     ./start_sim_move_around.sh --bt /abs/other.xml  # 换一棵树（默认用抖动演示树）
#     ./start_sim_move_around.sh --no-rviz            # 不起 RViz
#     ./start_sim_move_around.sh --dry-run            # 只打印将要做的事
#     ./start_sim_move_around.sh --help
#
#  退出：Ctrl-C 会同时收掉行为树/导航和本脚本起的 Gazebo
#        （如果 Gazebo 是你用 --no-gazebo 方式自己起的，本脚本不会去动它）
# =============================================================================

set +u   # 和 run_bj2_bt.sh 保持一致：ROS 的 setup.bash 会引用未绑定变量，开 set -u 会中途退出

# ---------------------------------------------------------------------------
# 路径（可用环境变量覆盖）
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BT_WS="${BT_WS:-$(cd "${SCRIPT_DIR}/../../.." && pwd)}"        # 决策工作区（本仓库）
BJ2_WS="${BJ2_WS:-/home/w/pd2027/bejixiong_2}"                  # 导航工作区
TEMPLATE_XML="${SCRIPT_DIR}/../config/test_move_around.xml"
RUNTIME_XML="${TMPDIR:-/tmp}/move_around_test.xml"
GZ_LOG="${TMPDIR:-/tmp}/gz_sim_move_around.log"

# ---------------------------------------------------------------------------
# 默认参数
# ---------------------------------------------------------------------------
POINTS=5
RADIUS=0.5
BT_XML=""                 # 空 = 用模板 + POINTS/RADIUS 生成
START_GZ=true
START_NAV=true
USE_RVIZ=true
DRY_RUN=false

while [ $# -gt 0 ]; do
  case "$1" in
    --points)  POINTS="${2:-}";  shift 2 ;;
    --radius)  RADIUS="${2:-}";  shift 2 ;;
    --bt)      BT_XML="${2:-}";  shift 2 ;;
    --no-gazebo) START_GZ=false;  shift ;;
    --nav-only)  START_NAV=false; shift ;;
    --no-rviz)   USE_RVIZ=false;  shift ;;
    --dry-run)   DRY_RUN=true;    shift ;;
    -h|--help)   sed -n '2,26p' "$0"; exit 0 ;;
    *) echo "未知参数：$1（用 --help 看用法）"; exit 2 ;;
  esac
done

[ -n "${BT_XML}" ] && TEMPLATE_XML="${BT_XML}"

echo "================================================================"
echo " 一键仿真：Gazebo + 导航 + MoveAround 抖动演示"
echo "================================================================"
echo "  决策工作区 : ${BT_WS}"
echo "  导航工作区 : ${BJ2_WS}"
echo "  行为树     : ${TEMPLATE_XML}   （${POINTS} 下，半径 ${RADIUS} m）"
echo "  起 Gazebo  : ${START_GZ}"
echo "  起导航/树  : ${START_NAV}（RViz: ${USE_RVIZ}）"
echo "----------------------------------------------------------------"

die() { echo "[错误] $*" >&2; exit 1; }

# ---------------------------------------------------------------------------
# 0) 清掉 snap VSCode 注入的污染变量（否则 rviz2 / gz GUI 会崩）
# ---------------------------------------------------------------------------
for _v in $(env | grep -E '^(SNAP|GTK|GDK)' | cut -d= -f1) \
          GTK_EXE_PREFIX GTK_PATH GTK_MODULES \
          GDK_PIXBUF_MODULE_FILE GSETTINGS_SCHEMA_DIR GSETTINGS_BACKEND; do
  unset "$_v" 2>/dev/null
done
unset _v

# ---------------------------------------------------------------------------
# 1) 按固定顺序 source 三个环境（顺序不能变）
# ---------------------------------------------------------------------------
[ -f "${BT_WS}/install/setup.bash" ]  || die "找不到 ${BT_WS}/install/setup.bash（决策工作区没编译？）"
[ -f "${BJ2_WS}/install/setup.bash" ] || die "找不到 ${BJ2_WS}/install/setup.bash（导航工作区没编译？）"
[ -f "${BJ2_WS}/run_bj2_bt.sh" ]      || die "找不到 ${BJ2_WS}/run_bj2_bt.sh"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "${BT_WS}/install/setup.bash"
# shellcheck disable=SC1090
source "${BJ2_WS}/install/setup.bash"      # 最后 source → 同名包以导航为准

# ---------------------------------------------------------------------------
# 2) 动态库路径修复（不修这两条，导航会起来后立刻挂）
#    坑 1: /opt/MVS（海康 SDK）的老 libusb 缺 libusb_set_option，而 PCL 要用它
#          → gicp_node 秒退 → 没有 map->odom → TF 里没有 map → 行为树发不出目标
#    坑 2: TdtMincoController 依赖 libOsqpEigen/libosqp，只在 tdt-nav-kit/3rd/install/lib
# ---------------------------------------------------------------------------
TDT_LIB_DIR="${BJ2_WS}/beijixiong/tdt-nav-kit/3rd/install/lib"
_ldp="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS' | grep -v '^$' | paste -sd:)"
_ldp="$(echo "${_ldp}" | sed 's/^://')"        # 去掉可能的前导冒号（空项=当前目录，别留）
export LD_LIBRARY_PATH="${_ldp}:/lib/x86_64-linux-gnu:${TDT_LIB_DIR}"

# ---------------------------------------------------------------------------
# 3) 通信：RMW 与仿真分区
#    ⚠ 本机只装了 rmw_fastrtps_cpp，设成 cyclonedds 会让每个 rcl 进程直接退出；
#      Gazebo 和导航必须在同一个 GZ/IGN 分区里，否则互相看不见。
# ---------------------------------------------------------------------------
if [ -n "${RMW_IMPLEMENTATION:-}" ] \
   && [ ! -f "/opt/ros/humble/lib/lib${RMW_IMPLEMENTATION}.so" ] \
   && [ ! -f "/opt/ros/humble/lib/x86_64-linux-gnu/lib${RMW_IMPLEMENTATION}.so" ]; then
  die "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} 但本机没装它。请 unset RMW_IMPLEMENTATION 后重试"
fi
export GZ_PARTITION="${GZ_PARTITION:-bjx_sim}"
export IGN_PARTITION="${IGN_PARTITION:-bjx_sim}"

echo "  RMW        : ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp（系统默认）}"
echo "  仿真分区   : GZ_PARTITION=${GZ_PARTITION}  IGN_PARTITION=${IGN_PARTITION}"
echo "----------------------------------------------------------------"

# ---------------------------------------------------------------------------
# 4) 生成运行用 XML（把 --points/--radius 替换进模板）
# ---------------------------------------------------------------------------
if [ -f "${TEMPLATE_XML}" ]; then
  sed -e "s|num_cycles=\"[0-9]*\"|num_cycles=\"${POINTS}\"|" \
      -e "s|expected_dis=\"[0-9.]*\"|expected_dis=\"${RADIUS}\"|" \
      "${TEMPLATE_XML}" > "${RUNTIME_XML}"
  if ! grep -q "num_cycles=\"${POINTS}\"" "${RUNTIME_XML}"; then
    die "生成的 XML 里没找到 num_cycles=${POINTS}；模板是不是被改过了？"
  fi
  BT_TO_RUN="${RUNTIME_XML}"
  echo "  运行 XML   : ${BT_TO_RUN}"
else
  die "找不到行为树文件：${TEMPLATE_XML}"
fi

# launch 文件的布尔参数用 "True"/"False"（和它声明的默认值写法保持一致）
if [ "${USE_RVIZ}" = true ]; then RV="True"; else RV="False"; fi
LAUNCH_ARGS=("bt_xml:=${BT_TO_RUN}" "use_rviz:=${RV}")

if [ "${DRY_RUN}" = true ]; then
  echo ">>> --dry-run：不启动，实际会执行："
  if [ "${START_GZ}" = true ]; then
    echo "    (后台) GZ_PARTITION=${GZ_PARTITION} ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
  fi
  if [ "${START_NAV}" = true ]; then
    echo "    (前台) ${BJ2_WS}/run_bj2_bt.sh ${LAUNCH_ARGS[*]}"
  fi
  exit 0
fi

# ---------------------------------------------------------------------------
# 5) 后台起 Gazebo，并等它真的就绪
# ---------------------------------------------------------------------------
GZ_PID=""
STARTED_GZ=false
cleanup() {
  echo ""
  if [ "${STARTED_GZ}" = true ]; then
    echo ">>> 收尾：关闭本脚本起的 Gazebo（日志：${GZ_LOG}）"
    if [ -n "${GZ_PID}" ] && kill -0 "${GZ_PID}" 2>/dev/null; then
      kill -INT  "${GZ_PID}" 2>/dev/null || true
      sleep 2
      kill -TERM "${GZ_PID}" 2>/dev/null || true
    fi
    # Gazebo 会 fork 出一堆子进程，兜底再扫一遍
    pkill -f 'gz sim|gzserver|ign gazebo' 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "${START_GZ}" = true ]; then
  if pgrep -f 'gz sim|gzserver|ign gazebo' >/dev/null 2>&1; then
    echo "!! 检测到已经有 Gazebo 在跑，本次跳过启动（要重起请先关掉旧的）"
  else
    echo ">>> 后台启动 Gazebo（日志：${GZ_LOG}）"
    nohup ros2 launch rmu_gazebo_simulator bringup_sim.launch.py > "${GZ_LOG}" 2>&1 &
    GZ_PID=$!
    STARTED_GZ=true

    echo -n "    等仿真就绪（轮询 /clock，最多 60 秒）"
    _t=0
    while [ "${_t}" -lt 60 ]; do
      if ! kill -0 "${GZ_PID}" 2>/dev/null; then
        echo ""
        echo "!! Gazebo 进程已经退出了，看一下日志尾部："
        tail -20 "${GZ_LOG}" | sed 's/^/    /'
        die "Gazebo 启动失败"
      fi
      if timeout 5 ros2 topic list 2>/dev/null | grep -qx "/clock"; then
        echo " → 就绪（${_t}s）"
        break
      fi
      sleep 3; _t=$((_t + 3)); echo -n "."
    done
    [ "${_t}" -ge 60 ] && echo " → 60 秒没等到 /clock，继续往下（可能你的仿真不桥接 /clock）"
    sleep 3   # 再给机器人 spawn/TF 一点时间
  fi
else
  echo ">>> --no-gazebo：假设 Gazebo 已经在跑"
fi

# ---------------------------------------------------------------------------
# 6) 前台起 导航 + 数据面 + 行为树
# ---------------------------------------------------------------------------
if [ "${START_NAV}" = true ]; then
  echo ""
  echo "==================== 启动导航 + 行为树 ===================="
  cat <<'TIP'
>>> 正常情况下你应该在日志里看到：

    [update_msg]: MoveAround: subscribing map topic 'map'        ← 插件加载
    [update_msg]: MoveAround: goal=(3.4521,-1.8873) center=(3.0000,-2.0000) r=0.50
    [send_goal]:  [SendGoal] Sending goal: [3.5, -1.9]
    [send_goal]:  [SendGoal] Arrived! distance=0.150, returning SUCCESS
    ...（共 5 下，★ 5 行的 center(...) 必须完全一样 ★）

  要警惕：
    MoveAround: no map on 'map' yet            → 地图没收到
    MoveAround: no usable point after 20 tries → 落点全不合格（调小 --radius 试）
    MoveAround: missing required input [message] → TF 没起来
    [SendGoal] ... not reachable               → 导航没起来（看上面的预检查）

  Ctrl-C 退出（会一起收掉本脚本起的 Gazebo）
----------------------------------------------------------
TIP
  cd "${BJ2_WS}" || die "cd ${BJ2_WS} 失败"
  "${BJ2_WS}/run_bj2_bt.sh" "${LAUNCH_ARGS[@]}"
else
  echo ">>> --nav-only：导航没起，Gazebo 会一直开着"
  echo "    另开一个终端起导航+树："
  echo "      cd ${BJ2_WS} && ./run_bj2_bt.sh bt_xml:=${BT_TO_RUN}"
  echo "    （或直接 Ctrl-C 结束本脚本，Gazebo 也会被关掉）"
  if [ -n "${GZ_PID}" ]; then
    wait "${GZ_PID}" 2>/dev/null || true
  else
    echo "    （本次没有起 Gazebo，脚本直接结束）"
  fi
fi
