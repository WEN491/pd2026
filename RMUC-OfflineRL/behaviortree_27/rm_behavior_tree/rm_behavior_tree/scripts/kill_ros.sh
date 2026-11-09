#!/usr/bin/env bash
# =============================================================================
#  kill_ros.sh —— 找出并收掉仿真/导航/行为树相关的残留进程
#
#  为什么要用它：一套仿真会拉起几十个进程（Gazebo + Nav2 + 行为树 + RViz +
#  数据面 + 各种 bridge），出问题时 Ctrl-C 常常收不干净：
#    · 有些节点是 `respawn=True`，杀了还会被 launch 拉起来 → 必须先杀 launch 父进程；
#    · FastDDS 崩过之后会留下 /dev/shm 里的共享内存文件；
#    · ros2 CLI 的 daemon 会缓存旧的节点列表，让你"看到已经不存在的节点"。
#
#  安全设计：
#    · 默认【只列出 + 交互确认】，按 y 才真的杀；
#    · 默认【不发 SIGKILL】，先 SIGINT（等同 Ctrl-C，能优雅退出）→ 再 SIGTERM；
#      只有显式加 --force 才会 SIGKILL；
#    · 永不杀自己（本脚本、启动它的 shell）以及 dsh/编辑器/ssh 等无关进程。
#
#  用法：
#     ./kill_ros.sh                 # 列出 → 问 y/N → 优雅关闭
#     ./kill_ros.sh -n              # dry-run：只列不杀（先跑这个看看）
#     ./kill_ros.sh -y              # 不询问，直接杀
#     ./kill_ros.sh -y --force      # 先优雅，最后兜底 SIGKILL
#     ./kill_ros.sh --only gazebo    # 只清某一类（launch/gazebo/nav/bt/rviz/tools）
#     ./kill_ros.sh -y --daemon --shm   # 顺带停 ros2 daemon、清 fastrtps 共享内存
#     ./kill_ros.sh --timeout 15     # 优雅等待时间（秒，默认 8）
# =============================================================================

set -uo pipefail

# ---------------------------------------------------------------------------
# 参数
# ---------------------------------------------------------------------------
DRY_RUN=false
ASSUME_YES=false
FORCE=false
ONLY=""
DO_DAEMON=false
DO_SHM=false
GRACE=8
QUIET=false

while [ $# -gt 0 ]; do
  case "$1" in
    -n|--dry-run|--list) DRY_RUN=true; shift ;;
    -y|--yes)            ASSUME_YES=true; shift ;;
    -f|--force)          FORCE=true; shift ;;
    --only)              ONLY="${2:-}"; shift 2 ;;
    --daemon)            DO_DAEMON=true; shift ;;
    --shm)               DO_SHM=true; shift ;;
    --timeout)           GRACE="${2:-8}"; shift 2 ;;
    -q|--quiet)          QUIET=true; shift ;;
    -h|--help)           sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "未知参数：$1（用 --help 看用法）"; exit 2 ;;
  esac
done

# ---------------------------------------------------------------------------
# 分类规则：类别 → 正则（匹配的是 ps 的完整命令行）
#   注意：写太宽的词（比如只写 "python3"）会误杀别的程序，所以每一条都尽量带上下文
# ---------------------------------------------------------------------------
CLASS_NAMES=(launch gazebo nav bt rviz tools)
PAT_launch='ros2 launch|ros2 run'
PAT_gazebo='gz sim|gzserver|ign gazebo|ruby .*gz|parameter_bridge|rmu_gazebo_simulator|spawn_robots|referee_system|rmua19'
PAT_nav='nav2|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|lifecycle_manager|map_server|map_saver|amcl|robot_state_publisher|small_gicp|gicp_node|pointlio|loam_interface|terrainAnalysis|static_transform_publisher|joy_node|fake_vel_transform|sensor_scan_generation|ign_sim_pointcloud_tool|pointcloud_to_laserscan|localization|component_container|fake_odom_tf'
PAT_bt='rm_behavior_tree'
PAT_rviz='rviz2'
PAT_tools='port_simulator|bench\.py|bt_monitor|fake_map|tf2_echo'

# 白名单：命令行里出现这些词的进程【永远不碰】
NEVER_KILL='kill_ros\.sh|dsh|deepseek|/bin/bash -c|sshd|systemd|Xorg|gnome|code |vim|nano|tmux|screen|colcon'

# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------
c_red=$'\033[31m'; c_grn=$'\033[32m'; c_yel=$'\033[33m'; c_dim=$'\033[2m'; c_off=$'\033[0m'

# 收集进程：输出 "pid|ppid|etime|class|args"
collect() {
  # -ww 防止命令行被截断；用 bash 自己匹配，避免 ps|grep 时把 grep 自己也匹配进去
  ps -eww -o pid=,ppid=,etime=,args= 2>/dev/null | while read -r pid ppid etime args; do
    [ -z "${pid:-}" ] && continue
    [ "$pid" = "$$" ] && continue                 # 不碰自己
    [ "$pid" = "${PPID:-0}" ] && continue         # 不碰父 shell
    [[ "$args" =~ $NEVER_KILL ]] && continue      # 白名单
    [[ "$args" =~ ^\[ ]] && continue              # 内核线程 / defunct

    for cls in "${CLASS_NAMES[@]}"; do
      local pat_var="PAT_$cls"
      local pat="${!pat_var}"
      if [[ "$args" =~ $pat ]]; then
        echo "${pid}|${ppid}|${etime}|${cls}|${args}"
        break
      fi
    done
  done
}

kill_pid() {   # kill_pid PID SIG
  local pid="$1" sig="$2"
  [ -d "/proc/$pid" ] || return 1
  kill "-$sig" "$pid" 2>/dev/null || return 1
  return 0
}

alive_count() {   # 传进去一批 pid，返回还活着的个数
  local n=0 p
  for p in "$@"; do [ -d "/proc/$p" ] && n=$((n+1)); done
  echo "$n"
}

# ---------------------------------------------------------------------------
# 1) 扫描
# ---------------------------------------------------------------------------
ALL_ROWS="$(collect)"
if [ -n "${ONLY}" ]; then
  ROWS="$(echo "${ALL_ROWS}" | awk -F'|' -v o="${ONLY}" '$4==o')"
else
  ROWS="${ALL_ROWS}"
fi

echo "================================================================"
echo " ROS 进程清理"
echo "================================================================"
if [ -z "${ROWS}" ]; then
  echo "  ${c_grn}没有找到需要清理的进程${c_off}（或者它们本来就没在跑）"
  [ "${DO_DAEMON}" = true ] && { echo ">>> 停 ros2 daemon"; ros2 daemon stop 2>/dev/null || true; }
  [ "${DO_SHM}" = true ] && { echo ">>> 清 fastrtps 共享内存"; rm -f /dev/shm/fastrtps_* 2>/dev/null || true; }
  exit 0
fi

# 按类别统计 + 打印明细
printf '  %-8s %-8s %-10s %s\n' "PID" "PGID" "运行时长" "命令行"
echo "  ------------------------------------------------------------"
PIDS_ALL=()
PIDS_LAUNCH=()
while IFS='|' read -r pid ppid etime cls args; do
  [ -z "${pid:-}" ] && continue
  printf '  %-8s %-8s %-10s [%s] %s\n' "$pid" "$ppid" "$etime" "$cls" "${args:0:110}"
  PIDS_ALL+=("$pid")
  [ "$cls" = "launch" ] && PIDS_LAUNCH+=("$pid")
done <<< "${ROWS}"

echo "  ------------------------------------------------------------"
echo "  共 ${#PIDS_ALL[@]} 个进程（其中 launch 父进程 ${#PIDS_LAUNCH[@]} 个）"
echo ""

if [ "${DRY_RUN}" = true ]; then
  echo ">>> --dry-run：只列不杀。确认没问题后去掉 -n 再跑一次"
  exit 0
fi

# ---------------------------------------------------------------------------
# 2) 确认
# ---------------------------------------------------------------------------
if [ "${ASSUME_YES}" != true ]; then
  printf "  确认要收掉上面这些进程吗？[y/N] "
  read -r ans
  case "${ans}" in
    y|Y|yes|YES) ;;
    *) echo "  已取消。"; exit 0 ;;
  esac
fi

# ---------------------------------------------------------------------------
# 3) 关：先父后子，SIGINT → SIGTERM →(可选) SIGKILL
# ---------------------------------------------------------------------------
still_alive() {   # 打印还活着的
  local p
  for p in "${PIDS_ALL[@]}"; do
    [ -d "/proc/$p" ] && ps -o pid=,args= -p "$p" 2>/dev/null | sed 's/^/    /'
  done
}

echo ""
echo ">>> 第 1 步：先给 ${#PIDS_LAUNCH[@]} 个 ros2 launch/run 父进程发 SIGINT（等同 Ctrl-C）"
echo "    （必须先杀父进程：launch 里 respawn=True 的节点，杀了会被重新拉起来）"
for p in "${PIDS_LAUNCH[@]}"; do kill_pid "$p" INT; done
sleep 2

echo ">>> 第 2 步：给其余所有进程发 SIGINT"
for p in "${PIDS_ALL[@]}"; do kill_pid "$p" INT; done

# 轮询等待
t=0
while [ "$t" -lt "${GRACE}" ]; do
  n=0
  for p in "${PIDS_ALL[@]}"; do [ -d "/proc/$p" ] && n=$((n+1)); done
  [ "$n" -eq 0 ] && break
  sleep 1; t=$((t+1))
done

N_LEFT=$(alive_count "${PIDS_ALL[@]}")
if [ "${N_LEFT}" -gt 0 ]; then
  echo ">>> 第 3 步：还有 ${N_LEFT} 个没退出，发 SIGTERM"
  for p in "${PIDS_ALL[@]}"; do [ -d "/proc/$p" ] && kill_pid "$p" TERM; done
  sleep 3
fi

N_LEFT=$(alive_count "${PIDS_ALL[@]}")
if [ "${N_LEFT}" -gt 0 ] && [ "${FORCE}" = true ]; then
  echo ">>> 第 4 步（--force）：还有 ${N_LEFT} 个，发 SIGKILL"
  for p in "${PIDS_ALL[@]}"; do [ -d "/proc/$p" ] && kill "-9" "$p" 2>/dev/null; done
  sleep 2
fi

# ---------------------------------------------------------------------------
# 4) 收尾
# ---------------------------------------------------------------------------
echo ""
N_LEFT=$(alive_count "${PIDS_ALL[@]}")
if [ "${N_LEFT}" -eq 0 ]; then
  echo "  ${c_grn}✓ 全部已退出${c_off}"
else
  echo "  ${c_yel}还有 ${N_LEFT} 个没退出${c_off}："
  still_alive
  echo ""
  echo "  这些可能是："
  echo "    · 不响应 SIGINT/SIGTERM 的（加 --force 用 SIGKILL）"
  echo "    · 变成僵尸(defunct)的：需要它的父进程回收，杀掉父进程即可"
fi

# 残留的启动父进程可能还在往外拉新进程，这里再扫一遍提示
sleep 1
RESPAWNED="$(collect | awk -F'|' -v o="${ONLY}" 'o==""||$4==o' | wc -l)"
[ "${RESPAWNED}" -gt 0 ] && echo "  ${c_yel}注意：又出现了 ${RESPAWNED} 个新进程（可能被某个 launch 反复拉起），再跑一次本脚本${c_off}"

if [ "${DO_DAEMON}" = true ]; then
  echo ">>> 停 ros2 daemon（它会缓存旧的节点/话题列表，停掉后 ros2 CLI 重新发现）"
  ros2 daemon stop 2>/dev/null || true
fi

if [ "${DO_SHM}" = true ]; then
  echo ">>> 清 FastDDS 共享内存（崩溃后才会残留；正常退出会自动清）"
  ls /dev/shm/fastrtps_* >/dev/null 2>&1 \
    && rm -f /dev/shm/fastrtps_* && echo "    已清 /dev/shm/fastrtps_*" \
    || echo "    没有残留"
fi

echo ""
echo "  提示：想确认还有没有残留 → 再跑一次 $0 -n"
