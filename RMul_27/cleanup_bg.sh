#!/bin/bash
# =============================================================================
# cleanup_bg.sh — 清理 run_bt_nav.sh / run_2026_3v3_sentry.sh 留在后台的进程
# =============================================================================
# 会停掉这些（按本工作空间 / 本仿真识别，不动 ros2 daemon、编辑器和别的工程）：
#   - 行为树 rm_behavior_tree（含 ros2 run 包装进程）
#   - 模拟裁判 mock_referee.py
#   - 导航栈 pb2025_nav_bringup / nav2 容器 / beijixiong 下的节点
#   - Gazebo（gz sim / ign gazebo / gzserver / gzclient）、ros_gz 桥
#   - 红/蓝方 robot_state_publisher 等带 __ns:=/red_standard_robot* 的节点
#
# 用法：
#   ./cleanup_bg.sh              # 列出并询问；确认后先 SIGINT，超时再 SIGKILL
#   ./cleanup_bg.sh -y           # 不询问，直接清理
#   ./cleanup_bg.sh -n           # 只列出，不动手
#   ./cleanup_bg.sh --timeout 15 # 等优雅退出的秒数（默认 8）
#
# 终端里那些跑完停在“按回车关闭”的标签页不会被关掉，需要自己按回车 / Ctrl-D。
# =============================================================================

set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_WS="/home/w/pd2027/beijixiong"

ASSUME_YES=0
DRY_RUN=0
TIMEOUT=8
PIDS=()

while [ $# -gt 0 ]; do
  case "$1" in
    -y|--yes)       ASSUME_YES=1 ;;
    -n|--dry-run|--list) DRY_RUN=1 ;;
    --timeout)      TIMEOUT="${2:-8}"; shift ;;
    -h|--help)      awk 'NR>1 { if ($0 ~ /^#/ || $0 == "") print; else exit }' "$0"; exit 0 ;;
    *) echo "未知参数：$1（用 --help 查看用法）" >&2; exit 2 ;;
  esac
  shift
done

case "$TIMEOUT" in
  ''|*[!0-9]*) echo "--timeout 需要非负整数：$TIMEOUT" >&2; exit 2 ;;
esac

say()  { echo -e "\033[1;36m[cleanup_bg]\033[0m $*"; }
warn() { echo -e "\033[1;33m[警告]\033[0m $*"; }
err()  { echo -e "\033[1;31m[错误]\033[0m $*"; }

# 只匹配本工作空间 / 本仿真用到的进程；宁可漏杀，不要误杀
is_target() {
  local args="$1"
  case "$args" in
    # 自己（含从本脚本派生的子壳）
    *"cleanup_bg.sh"*) return 1 ;;

    # 行为树
    *"$WS/install/rm_behavior_tree/lib/rm_behavior_tree/rm_behavior_tree"*) return 0 ;;
    *"ros2 run rm_behavior_tree"*) return 0 ;;

    # 模拟裁判
    *"mock_referee.py"*) return 0 ;;

    # 导航栈（launch 命令、beijixiong 下所有节点）
    *"pb2025_nav_bringup"*) return 0 ;;
    *"$NAV_WS/install/"*) return 0 ;;
    *"$NAV_WS/"*"_launch.py"*) return 0 ;;

    # Gazebo / 仿真桥
    *"rmu_gazebo_simulator"*) return 0 ;;
    *"ros_gz_bridge"*) return 0 ;;
    *"gz.msgs"*) return 0 ;;
    *"ignition.msgs"*) return 0 ;;
    *"gz sim"*) return 0 ;;
    *"/bin/gz "*) return 0 ;;
    *"ign gazebo"*) return 0 ;;
    *"gzserver"*|*"gzclient"*) return 0 ;;

    # 红/蓝方机器人命名空间下的节点（robot_state_publisher、parameter_bridge 等）
    *"__ns:=/red_standard_robot"*) return 0 ;;
    *"__ns:=/blue_standard_robot"*) return 0 ;;

    *) return 1 ;;
  esac
}

collect_targets() {
  PIDS=()
  local pid etimes args state
  while read -r pid etimes args; do
    [ -n "$pid" ] || continue
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "$PPID" ] && continue
    is_target "$args" || continue
    # 跳过僵尸
    state=$(ps -o stat= -p "$pid" 2>/dev/null)
    case "$state" in Z*|"") continue ;; esac
    PIDS+=("$pid")
  done < <(ps -eww -o pid=,etimes=,args=)
}

show_targets() {
  local p etimes args
  for p in "${PIDS[@]}"; do
    read -r etimes args < <(ps -eww -o etimes=,args= -p "$p" 2>/dev/null)
    [ -n "$etimes" ] || continue
    printf '  %-7s %5ss  %s\n' "$p" "$etimes" "${args:0:260}"
  done
}

kill_list() {
  local sig="$1"; shift
  [ $# -gt 0 ] || return 0
  kill "-$sig" "$@" 2>/dev/null
}

collect_targets

if [ ${#PIDS[@]} -eq 0 ]; then
  say "没有发现需要清理的进程（Gazebo / 导航 / 行为树 / 模拟裁判都没在跑）"
  exit 0
fi

say "发现 ${#PIDS[@]} 个后台进程："
show_targets

if [ "$DRY_RUN" = "1" ]; then
  echo
  say "--dry-run：只列出，不动手。去掉 -n 或加 -y 执行清理。"
  exit 0
fi

if [ "$ASSUME_YES" -eq 0 ]; then
  if [ -t 0 ]; then
    echo
    read -r -p "确认停止以上 ${#PIDS[@]} 个进程? [y/N] " ans
    case "$ans" in
      y|Y|yes|YES) ;;
      *) say "已取消"; exit 1 ;;
    esac
  else
    err "非交互环境，未确认；请加 -y 执行"
    exit 1
  fi
fi

# 第一轮：SIGINT 优雅退出（ROS 节点会关 lifecycle、Gazebo 会收尾）
say "发送 SIGINT（最多等 ${TIMEOUT}s）..."
kill_list INT "${PIDS[@]}"

for ((i = 0; i < TIMEOUT; i++)); do
  alive=0
  for p in "${PIDS[@]}"; do
    state=$(ps -o stat= -p "$p" 2>/dev/null)
    case "$state" in
      ""|Z*) ;;
      *) alive=1; break ;;
    esac
  done
  [ "$alive" -eq 0 ] && break
  sleep 1
done

# 第二轮：没退的、以及 launch 又 respawn 出来的，统一 SIGKILL
collect_targets
if [ ${#PIDS[@]} -gt 0 ]; then
  warn "还有 ${#PIDS[@]} 个没退出（或已被 launch 重新拉起），发送 SIGKILL："
  show_targets
  kill_list KILL "${PIDS[@]}"
  sleep 1
fi

# 最终检查
collect_targets
if [ ${#PIDS[@]} -eq 0 ]; then
  say "清理完成"
else
  warn "仍有 ${#PIDS[@]} 个残留："
  show_targets
  warn "可再跑一次 ./cleanup_bg.sh -y，或手动 kill -9 上面的 PID"
  exit 1
fi

cat <<'EOF'

提示：gnome-terminal 里那些「已退出，按回车关闭」的标签页不会被脚本关掉；
      在对应标签页按回车 / Ctrl-D 即可。
EOF
