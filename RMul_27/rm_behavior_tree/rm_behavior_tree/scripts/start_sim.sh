#!/usr/bin/env bash
# =============================================================================
#  start_sim.sh —— 一键起仿真 + 导航 + 行为树（MoveAround 抖动演示）
#
#  可以放在两个地方，脚本会自己认路：
#     · /home/w/pd2027/start_sim.sh                      ← 推荐（和 env_setup.sh 同级）
#     · .../behaviortree_26/rm_behavior_tree/rm_behavior_tree/scripts/start_sim.sh
#
#  用法：
#     ./start_sim.sh                        # 一键全套（默认 5 下、半径 0.5 m）
#     ./start_sim.sh --radius 0.3 --points 8
#     ./start_sim.sh --tree /abs/other.xml  # 换一棵树
#     ./start_sim.sh --no-gazebo            # Gazebo 已在别的终端跑着
#     ./start_sim.sh --nav-only             # 只起 Gazebo + 导航，树自己起
#     ./start_sim.sh --no-rviz              # 不起 RViz
#     ./start_sim.sh --skip-precheck        # 跳过"残留进程"自检
#     ./start_sim.sh --dry-run              # 只打印将要做的事
#     ./start_sim.sh --help
#
#  它按正确顺序做这些事：
#     0) 自检：有没有残留的仿真/导航进程（优先调用同目录的 kill_ros.sh -n）
#     1) 清 snap/GTK 污染变量 + 按 [ROS → 决策工作区 → BJ2 导航] 顺序 source
#     2) 修 LD_LIBRARY_PATH（/opt/MVS 老 libusb 与 PCL 冲突、缺 tdt 的 libOsqpEigen）
#     3) 后台起 Gazebo，轮询 /clock 等它真就绪
#     4) 前台起 导航 + 数据面 + 行为树（复用 bejixiong_2/run_bj2_bt.sh）
#
#  Ctrl-C 一次全收：行为树、导航、以及【本脚本起的】Gazebo
# =============================================================================

set +u   # ROS 的 setup.bash 会引用未绑定变量，开 set -u 会在 source 那行直接退出

# ---------------------------------------------------------------------------
# 0) 自己认路：找出 /home/w/pd2027（含 behaviortree_26 与 bejixiong_2 的那一层）
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PD_DIR="${PD2027_DIR:-}"
if [ -z "${PD_DIR}" ]; then
  for cand in "$SCRIPT_DIR" "$SCRIPT_DIR/.." "$SCRIPT_DIR/../.." \
              "$SCRIPT_DIR/../../.." "$SCRIPT_DIR/../../../.."; do
    if [ -d "$cand/bejixiong_2" ] && [ -d "$cand/behaviortree_26" ]; then
      PD_DIR="$(cd "$cand" && pwd)"; break
    fi
  done
fi
[ -n "${PD_DIR}" ] || PD_DIR="/home/w/pd2027"

BT_WS="${BT_WS:-${PD_DIR}/behaviortree_26}"
BJ2_WS="${BJ2_WS:-${PD_DIR}/bejixiong_2}"
DEFAULT_XML="${DEFAULT_XML:-${BT_WS}/rm_behavior_tree/rm_behavior_tree/config/test_move_around.xml}"
KILL_SH="${PD_DIR}/kill_ros.sh"
RUNTIME_XML="${TMPDIR:-/tmp}/move_around_test.xml"
GZ_LOG="${TMPDIR:-/tmp}/gz_sim_move_around.log"

# ---------------------------------------------------------------------------
# 参数
# ---------------------------------------------------------------------------
POINTS=5
RADIUS=0.5
TREE=""
START_GZ=true
START_NAV=true
USE_RVIZ=true
PRECHECK=true
DRY_RUN=false

while [ $# -gt 0 ]; do
  case "$1" in
    --points)        POINTS="${2:-}";  shift 2 ;;
    --radius)        RADIUS="${2:-}";  shift 2 ;;
    --tree|--bt)     TREE="${2:-}";    shift 2 ;;
    --no-gazebo)     START_GZ=false;   shift ;;
    --nav-only)      START_NAV=false;  shift ;;
    --no-rviz)       USE_RVIZ=false;   shift ;;
    --skip-precheck) PRECHECK=false;   shift ;;
    --dry-run)       DRY_RUN=true;     shift ;;
    -h|--help)       sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数：$1（用 --help 看用法）"; exit 2 ;;
  esac
done
[ -n "${TREE}" ] && DEFAULT_XML="${TREE}"

die() { echo "[错误] $*" >&2; exit 1; }

echo "================================================================"
echo " 一键仿真：Gazebo + 导航 + 行为树"
echo "================================================================"
echo "  PD 目录    : ${PD_DIR}"
echo "  决策工作区 : ${BT_WS}"
echo "  导航工作区 : ${BJ2_WS}"
echo "  行为树     : ${DEFAULT_XML}   （${POINTS} 下，半径 ${RADIUS} m）"
echo "  起 Gazebo  : ${START_GZ}    起导航/树 : ${START_NAV}（RViz: ${USE_RVIZ}）"
echo "----------------------------------------------------------------"

# ---------------------------------------------------------------------------
# 1) 残留进程自检（两个实例同时跑=最难查的问题）
# ---------------------------------------------------------------------------
if [ "${PRECHECK}" = true ] && [ "${DRY_RUN}" = false ] && [ -x "${KILL_SH}" ]; then
  echo ">>> 残留进程自检（${KILL_SH} -n）"
  if "${KILL_SH}" -n | grep -qE '\[(launch|gazebo|nav|bt)\]'; then
    echo ""
    echo "!! 上面这些进程还在跑。两套导航/仿真会互相抢话题和 TF，建议先清掉："
    echo "     ${KILL_SH} -y"
    printf "   仍然继续启动吗？[y/N] "
    read -r ans
    case "${ans}" in y|Y|yes|YES) ;; *) echo "  已取消。"; exit 0 ;; esac
  else
    echo "    ✓ 没有残留"
  fi
  echo "----------------------------------------------------------------"
fi

# ---------------------------------------------------------------------------
# 2) 清 snap VSCode 注入的污染变量（否则 rviz2 / gz GUI 会崩）
# ---------------------------------------------------------------------------
for _v in $(env | grep -E '^(SNAP|GTK|GDK)' | cut -d= -f1) \
          GTK_EXE_PREFIX GTK_PATH GTK_MODULES \
          GDK_PIXBUF_MODULE_FILE GSETTINGS_SCHEMA_DIR GSETTINGS_BACKEND; do
  unset "$_v" 2>/dev/null
done
unset _v

# ---------------------------------------------------------------------------
# 3) 按固定顺序 source（顺序不能变：BJ2 最后 → 同名包以导航为准）
# ---------------------------------------------------------------------------
[ -f "${BT_WS}/install/setup.bash" ]  || die "找不到 ${BT_WS}/install/setup.bash（决策工作区没编译？）"
[ -f "${BJ2_WS}/install/setup.bash" ] || die "找不到 ${BJ2_WS}/install/setup.bash（导航工作区没编译？）"
[ -f "${BJ2_WS}/run_bj2_bt.sh" ]      || die "找不到 ${BJ2_WS}/run_bj2_bt.sh"
[ -f "${DEFAULT_XML}" ]               || die "找不到行为树文件：${DEFAULT_XML}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "${BT_WS}/install/setup.bash"
# shellcheck disable=SC1090
source "${BJ2_WS}/install/setup.bash"

# ---------------------------------------------------------------------------
# 4) 动态库路径修复（不修这两条，导航起来后立刻挂）
#    坑1: /opt/MVS 的老 libusb 缺 libusb_set_option，PCL 要用 → gicp_node 秒退
#         → 没有 map->odom → TF 里没有 map → 行为树发不出目标
#    坑2: TdtMincoController 依赖的 libOsqpEigen/libosqp 只在 tdt-nav-kit/3rd/install/lib
# ---------------------------------------------------------------------------
TDT_LIB_DIR="${BJ2_WS}/beijixiong/tdt-nav-kit/3rd/install/lib"
_ldp="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '^/opt/MVS' | grep -v '^$' | paste -sd:)"
_ldp="$(echo "${_ldp}" | sed 's/^://')"     # 去掉可能的前导冒号（空项=当前目录，别留）
export LD_LIBRARY_PATH="${_ldp}:/lib/x86_64-linux-gnu:${TDT_LIB_DIR}"

# ---------------------------------------------------------------------------
# 5) RMW 与仿真分区
#    ⚠ 本机只装了 rmw_fastrtps_cpp；设成 cyclonedds 会让每个 rcl 进程直接退出
#    ⚠ Gazebo 与导航必须在同一个 GZ/IGN 分区，否则互相看不见
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
# 6) 生成运行用 XML（把 --points/--radius 替换进模板）
# ---------------------------------------------------------------------------
sed -e "s|num_cycles=\"[0-9]*\"|num_cycles=\"${POINTS}\"|" \
    -e "s|expected_dis=\"[0-9.]*\"|expected_dis=\"${RADIUS}\"|" \
    "${DEFAULT_XML}" > "${RUNTIME_XML}"

# 模板是"你自己的树"，可能没有 <Repeat>（就没有 num_cycles）、也可能改了端口名
# → 这里【有则替换、无则提示】，绝不因为替换不到就直接退出
if grep -q 'num_cycles="' "${RUNTIME_XML}"; then
  grep -q "num_cycles=\"${POINTS}\"" "${RUNTIME_XML}" || die "num_cycles 替换失败，请检查模板"
else
  echo "  ! 模板里没有 num_cycles（树里没有 <Repeat>）→ 本次 --points 参数无效"
fi
if grep -q 'expected_dis="' "${RUNTIME_XML}"; then
  grep -q "expected_dis=\"${RADIUS}\"" "${RUNTIME_XML}" || die "expected_dis 替换失败，请检查模板"
else
  echo "  ! 模板里没有 expected_dis → 本次 --radius 参数无效"
fi
echo "  运行 XML   : ${RUNTIME_XML}"

if [ "${USE_RVIZ}" = true ]; then RV="True"; else RV="False"; fi
LAUNCH_ARGS=("bt_xml:=${RUNTIME_XML}" "use_rviz:=${RV}")

if [ "${DRY_RUN}" = true ]; then
  echo ">>> --dry-run：不启动。实际会执行："
  [ "${START_GZ}" = true ] && echo "    (后台) GZ_PARTITION=${GZ_PARTITION} ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
  [ "${START_NAV}" = true ] && echo "    (前台) ${BJ2_WS}/run_bj2_bt.sh ${LAUNCH_ARGS[*]}"
  exit 0
fi

# ---------------------------------------------------------------------------
# 7) 后台起 Gazebo，等它真的就绪
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
    pkill -f 'gz sim|gzserver|ign gazebo' 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "${START_GZ}" = true ]; then
  if pgrep -f 'gz sim|gzserver|ign gazebo' >/dev/null 2>&1; then
    echo "!! 已经检测到 Gazebo 在跑，本次跳过启动（要重起请先关掉旧的）"
  else
    echo ">>> 后台启动 Gazebo（日志：${GZ_LOG}）"
    nohup ros2 launch rmu_gazebo_simulator bringup_sim.launch.py > "${GZ_LOG}" 2>&1 &
    GZ_PID=$!
    STARTED_GZ=true

    echo -n "    等仿真就绪（轮询 /clock，最多 60 秒）"
    _t=0
    while [ "${_t}" -lt 60 ]; do
      if ! kill -0 "${GZ_PID}" 2>/dev/null; then
        echo ""; echo "!! Gazebo 已退出，日志尾部："
        tail -20 "${GZ_LOG}" | sed 's/^/    /'
        die "Gazebo 启动失败"
      fi
      if timeout 5 ros2 topic list 2>/dev/null | grep -qx "/clock"; then
        echo " → 就绪（${_t}s）"; break
      fi
      sleep 3; _t=$((_t + 3)); echo -n "."
    done
    [ "${_t}" -ge 60 ] && echo " → 60 秒没等到 /clock，继续往下"
    sleep 3
  fi
else
  echo ">>> --no-gazebo：假设 Gazebo 已经在跑"
fi

# ---------------------------------------------------------------------------
# 8) 前台起 导航 + 数据面 + 行为树
# ---------------------------------------------------------------------------
if [ "${START_NAV}" = true ]; then
  echo ""
  echo "==================== 启动导航 + 行为树 ===================="
  cat <<'TIP'
>>> 正常情况下你应该看到：
    [update_msg]: MoveAround: subscribing map topic 'map'
    [update_msg]: MoveAround: goal=(3.4521,-1.8873) center=(3.0000,-2.0000) r=0.50
    [send_goal]:  [SendGoal] Sending goal: [3.5, -1.9]
    [send_goal]:  [SendGoal] Arrived! distance=0.150, returning SUCCESS
    ...（共 5 下，★ 5 行的 center(...) 必须完全一样 ★）

  要警惕：
    MoveAround: no map on 'map' yet            → 地图没收到
    MoveAround: no usable point after 20 tries → 落点全不合格（调小 --radius）
    MoveAround: missing required input [message] → TF 没起来
    [SendGoal] ... not reachable               → 导航没起来

  Ctrl-C 退出（会一起收掉本脚本起的 Gazebo）；收不干净就再跑 kill_ros.sh -y
----------------------------------------------------------
TIP
  cd "${BJ2_WS}" || die "cd ${BJ2_WS} 失败"
  "${BJ2_WS}/run_bj2_bt.sh" "${LAUNCH_ARGS[@]}"
else
  echo ">>> --nav-only：导航没起，Gazebo 会一直开着"
  echo "    另开终端起： cd ${BJ2_WS} && ./run_bj2_bt.sh bt_xml:=${RUNTIME_XML}"
  echo "    （Ctrl-C 结束本脚本，Gazebo 也会被关掉）"
  if [ -n "${GZ_PID}" ]; then wait "${GZ_PID}" 2>/dev/null || true; fi
fi
