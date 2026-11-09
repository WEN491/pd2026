#!/bin/bash
# =============================================================================
# run_bt_nav.sh — 用 beijixiong 的导航包跑 behaviortree_27 的决策行为树
# =============================================================================
# 行为树：/home/w/pd2027/behaviortree_27/rm_behavior_tree/rm_behavior_tree/config/rmul_2026_3v3_sentry.xml
# 导航栈：/home/w/pd2027/beijixiong（pb2025_nav_bringup + 定位/nav2/地形分析等）
#
# 用法：
#   ./run_bt_nav.sh                  # 仿真：Gazebo + 导航栈 + 行为树（默认）
#   ./run_bt_nav.sh --reality        # 实车：导航栈 + 行为树（不含 Gazebo）
#   ./run_bt_nav.sh --mock-referee   # 仿真 + 模拟裁判数据（仿真里没有 /game_status，
#                                    #   不开这个的话 IsGameTime 恒假、整棵树不会推进）
#   ./run_bt_nav.sh --nav-only       # 只起导航（调试导航用）
#   ./run_bt_nav.sh --bt-only        # 只起行为树（导航已在别的终端跑着）
#   ./run_bt_nav.sh --no-rviz        # 不开 RViz
#   ./run_bt_nav.sh --namespace red_standard_robot1   # 覆盖导航命名空间
#   ./run_bt_nav.sh --check          # 只做环境预检，不启动任何进程
#   ./run_bt_nav.sh --dry-run        # 只打印将要执行的命令，不真启动
#   ./run_bt_nav.sh --jobs 2         # 编译并行度上限（默认 3；本机 32核/14GB，别用默认无限）
#   ./run_bt_nav.sh --repair-bt      # 清空并重建 behaviortree_27（install 损坏时用）
#   ./run_bt_nav.sh --repair-nav     # 修 beijixiong：重建 + 自动拼回被清空的那几个包
#   ./run_bt_nav.sh --assemble-plugins   # 只重跑"拼装"这一步（build/ 已有产物时）
#
# ─────────────────────────────── 已知问题（重要） ───────────────────────────────
# beijixiong 的 install/ 在 2026-10-12 被整体清空（1866 个文件全为 0 字节，且缺
# .colcon_install_layout），直接 colcon build 会报：
#     The install directory 'install' was created with the layout ''
# 另外下面 5 个包的源码也被清空（CMakeLists.txt/package.xml/源码全空），无法重新编译：
#     controller            nav2 控制器插件 controller::TdtMincoController（sim yaml 在用）
#     planning              nav2 全局规划插件 planning::TdtGlobalPlanner，以及
#                           is_path_valid_server / tunnel_mode_manager
#     dynamic_scan_filter   节点 dynamic_scan_filter_node
#     rog_map / tdt_nav_stack  静态库依赖（运行期不需要）
# --repair-nav 会自动处理：先重建其余包，再用 **build/ 里的预编译产物 + wolf_sentry
# 里同源完好的 install 作 ament 管道模板** 把这 3 个包拼回 install/。已验证这些预编译
# 插件在本机的依赖可全部解析（0 个 not found；前提是 LD_LIBRARY_PATH 带上
# tdt-nav-kit/3rd/install/lib —— 脚本已自动加）。
# 预编译产物已抢救到 beijixiong/_prebuilt_rescue/（含 planning 的
# libplanning.so / libplanning_bt_plugins.so / is_path_valid_server / plugin_check /
# tunnel_mode_manager 原版二进制，controller 的 libcontroller.so，dynamic_scan_filter 的节点）。
# ⚠ 残留缺口：只有 controller / planning / dynamic_scan_filter / rog_map / tdt_nav_stack
# 这 5 个包的**源码**没恢复，改不了它们的代码；跑起来不受影响，但要改这三个插件的行为，
# 就得先从 git/备份恢复源码再正常编译。
# =============================================================================

# 注意：不要开 set -u —— ROS 的 setup.bash 会引用未绑定的 AMENT_TRACE_SETUP_FILES
set -o pipefail

BT_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_WS="/home/w/pd2027/beijixiong"
BT_XML="$BT_WS/rm_behavior_tree/rm_behavior_tree/config/rmul_2026_3v3_sentry.xml"
BT_PKG="rm_behavior_tree"

# 自带的 GTSAM（不是 apt 装的）：localization_fusion 靠它编译；
# 注意它的布局是 <前缀>/lib/cmake/GTSAM/GTSAMConfig.cmake + <前缀>/lib/x86_64-linux-gnu/libgtsam.so
GTSAM_PREFIX="/home/w/pd2027/.thirdparty/gtsam/opt/ros/humble"
if [ -f "$GTSAM_PREFIX/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
  GTSAM_LIB_DIR="$GTSAM_PREFIX/lib/x86_64-linux-gnu"
else
  GTSAM_PREFIX=""; GTSAM_LIB_DIR=""
fi

# ───────────────────────────── 参数解析 ─────────────────────────────
MODE="sim"          # sim | reality
NS_OVERRIDE=""      # 空 = 用各 launch 的默认命名空间
RUN_NAV=1; RUN_BT=1; RUN_GZ=1; USE_RVIZ=1; MOCK=0
REPAIR_BT=0; REPAIR_NAV=0; CHECK_ONLY=0; ASSEMBLE_ONLY=0
JOBS="${JOBS:-3}"
DRY_RUN=0                      # 并行度上限；本机 32核/14GB，默认 3（0 = 用 colcon 默认，危险）
WORLD="rmuc_2025"
ODOM="small_point_lio"; RELOC="gicp"   # 仿真默认（与 one_click_nav.sh 一致）

while [ $# -gt 0 ]; do
  case "$1" in
    --reality)       MODE="reality" ;;
    --sim)           MODE="sim" ;;
    --nav-only)      RUN_BT=0 ;;
    --bt-only)       RUN_NAV=0; RUN_GZ=0 ;;
    --no-gazebo)     RUN_GZ=0 ;;
    --no-rviz)       USE_RVIZ=0 ;;
    --mock-referee)  MOCK=1 ;;
    --namespace)     NS_OVERRIDE="${2:-}"; shift ;;
    --world)         WORLD="${2:-$WORLD}"; shift ;;
    --jobs)          JOBS="${2:-3}"; shift ;;
    --check)         CHECK_ONLY=1 ;;
    --dry-run)       DRY_RUN=1 ;;
    --repair-bt)     REPAIR_BT=1 ;;
    --repair-nav)    REPAIR_NAV=1 ;;
    --assemble-plugins) ASSEMBLE_ONLY=1 ;;
    -h|--help)       sed -n '2,35p' "$0"; exit 0 ;;
    *) echo "[警告] 未知参数：$1（用 --help 查看用法）" ;;
  esac
  shift
done

# 默认命名空间：仿真栈把机器人挂在 red_standard_robot1 下；实车 launch 默认根命名空间
if [ -z "$NS_OVERRIDE" ]; then
  if [ "$MODE" = "sim" ]; then NS="red_standard_robot1"; else NS=""; fi
else
  NS="$NS_OVERRIDE"
fi

LOG_DIR="$BT_WS/log/run_bt_nav"
say()  { echo -e "\033[1;36m[run_bt_nav]\033[0m $*"; }
warn() { echo -e "\033[1;33m[警告]\033[0m $*"; }
err()  { echo -e "\033[1;31m[错误]\033[0m $*"; }

# ───────────────────── 预检：install 是否真的可用 ─────────────────────
# 判据：① setup.bash 非空；② ament_index 标记在（setup.bash 被清空时 source 是静默
#       无效的，必须显式检查，否则后面报错会很难看懂）
# ⚠ 用 --symlink-install 时这些标记是**符号链接**（指向 build/…/ament_cmake_index/…），
#   所以绝不能用 `find -type f` 去匹配 —— 会全部漏掉、误报"install 不可用"。
install_ok() {
  local ws="$1" probe_pkg="$2"
  [ -s "$ws/install/setup.bash" ] || return 1
  local marker="$ws/install/$probe_pkg/share/ament_index/resource_index/packages/$probe_pkg"
  { [ -e "$marker" ] || [ -L "$marker" ]; } || return 1
  return 0
}

# 更权威的一票：真去 source 一遍，看 ros2 能不能找到这个包
# ⚠ 子壳里必须先 unset RMW_IMPLEMENTATION：若父 shell 设了一个没装的实现
#   （本机常见：RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 但只装了 fastrtps），
#   ros2 CLI 会在 rcl 初始化阶段就 exit 1，导致这里误判"install 不可用"。
pkg_resolvable() {
  local ws="$1" probe_pkg="$2"
  bash -c "set +u; unset RMW_IMPLEMENTATION; \
           source /opt/ros/humble/setup.bash >/dev/null 2>&1; \
           source '$ws/install/setup.bash' >/dev/null 2>&1; \
           ros2 pkg prefix '$probe_pkg' >/dev/null 2>&1"
}

preflight() {
  local fail=0
  say "预检："
  if [ -s "$BT_XML" ]; then
    echo "  ✔ 行为树 XML：$BT_XML ($(wc -c < "$BT_XML") 字节)"
  else
    err "行为树 XML 不存在或为空：$BT_XML"; fail=1
  fi

  if [ "$RUN_NAV" = "1" ]; then
    if install_ok "$NAV_WS" "pb2025_nav_bringup"; then
      if pkg_resolvable "$NAV_WS" "pb2025_nav_bringup"; then
        echo "  ✔ 导航 install：$NAV_WS/install 可用（ros2 能找到 pb2025_nav_bringup）"
      else
        warn "install 文件在，但 source 后 ros2 找不到 pb2025_nav_bringup —— setup.bash 可能是旧的，建议重跑 colcon build"
        fail=1
      fi
    else
      err "导航 install 不可用：$NAV_WS/install（setup.bash 为空或缺 ament 标记）"
      echo "      → 修复：$0 --repair-nav     （或先 rm -rf install 再 colcon build，见文件头“已知问题”）"
      fail=1
    fi
    # 拼装出来的 3 个包不是 colcon 登记的，单独确认一下
    for _p in planning controller dynamic_scan_filter localization_fusion; do
      if [ -e "$NAV_WS/install/$_p/lib" ]; then
        echo "  ✔ 拼装包 $_p：$(find "$NAV_WS/install/$_p/lib" -maxdepth 2 -type f 2>/dev/null | wc -l) 个库/可执行"
      else
        warn "拼装包 $_p 不在 install 里 → 跑 $0 --assemble-plugins"
      fi
    done
  else
    echo "  · 跳过导航预检（--bt-only）"
  fi

  local bt_exe="$BT_WS/install/$BT_PKG/lib/$BT_PKG/$BT_PKG"
  if install_ok "$BT_WS" "$BT_PKG" && [ -x "$bt_exe" ]; then
    echo "  ✔ 行为树 install：$bt_exe"
  else
    err "行为树 install 不可用或可执行文件缺失：$bt_exe"
    echo "      → 修复：$0 --repair-bt"
    fail=1
  fi
  return $fail
}

# ───────────────────────────── 修复 / 编译 ─────────────────────────────
source /opt/ros/humble/setup.bash

# ── RMW 实现选择：绝不能硬设一个没装的实现 ──
# 实测过：RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 但本机未安装 →
#   "failed to load shared library 'librmw_cyclonedds_cpp.so'" → 每个 rcl 进程秒退
#   （Gazebo 终端里的 bridge 也会挂，表现成"Gazebo 已退出"）
rmw_available() {
  [ -f "/opt/ros/humble/lib/lib$1.so" ] || [ -f "/opt/ros/humble/lib/x86_64-linux-gnu/lib$1.so" ]
}
if [ -n "${RMW_IMPLEMENTATION:-}" ] && ! rmw_available "$RMW_IMPLEMENTATION"; then
  RMW_LINE="unset RMW_IMPLEMENTATION   # 继承来的实现没装，清掉用系统默认"
  warn "你的 shell 里 RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}，但本机没装它！"
  warn "  → 本脚本的终端会自动清掉；但你**手动开终端**跑 ros2 命令时也要先执行："
  warn "       unset RMW_IMPLEMENTATION          # 或 export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
  warn "  → 否则会报 'failed to load shared library lib${RMW_IMPLEMENTATION}.so' 并秒退"
  warn "  （来源可能是别处 source 过的脚本：one_click_nav.sh / save_map.sh / tools/run_eval.sh 里都有这一行）"
elif [ -n "${RMW_IMPLEMENTATION:-}" ] && rmw_available "$RMW_IMPLEMENTATION"; then
  RMW_LINE="export RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
  say "RMW：沿用当前环境已设的 ${RMW_IMPLEMENTATION}"
elif rmw_available rmw_cyclonedds_cpp; then
  RMW_LINE="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  say "RMW：rmw_cyclonedds_cpp（与旧 one_click_nav.sh 一致）"
elif rmw_available rmw_fastrtps_cpp; then
  RMW_LINE="unset RMW_IMPLEMENTATION   # 本机只有 fastrtps，用它就等于系统默认"
  say "RMW：本机没装 rmw_cyclonedds_cpp → 用系统默认 rmw_fastrtps_cpp"
else
  RMW_LINE="unset RMW_IMPLEMENTATION"
  warn "没探测到任何 RMW 实现，交给 ROS 自己决定"
fi


# 用 build/ 里的预编译产物 + 同源完好 install 的 ament 管道，把源码被清空的那几个包
# 拼回 beijixiong/install。原理：
#   ① ament 管道（package.xml / *_plugin.xml / ament_index 标记 / cmake 导出）从
#      wolf_sentry/install/<pkg> 复制 —— 它是同一份代码在 9月28-30 的完好安装，
#      插件类名一致（controller::TdtMincoController / planning::TdtGlobalPlanner）；
#   ② 二进制（.so / .a / 可执行）用 beijixiong 自己 build/ 里的，版本与它的 yaml 匹配；
#   ③ tunnel_mode_manager 在 beijixiong 的 build/ 里根本不存在（源码被清空前就没编过），
#      装一个占位实现，避免 navigation_launch.py 因找不到可执行文件而整体失败。
TPL_WS="/home/w/pd2027/wolf_sentry"        # 同源完好 install 模板
RESCUE_DIR="/home/w/pd2027/beijixiong/_prebuilt_rescue"
PLUGIN_PKGS="planning controller dynamic_scan_filter localization_fusion"

# 把被清空包的预编译产物抢救到独立目录：之后无论 build/ 被挪走还是重编，
# 拼装都还有料可用（这一步很关键——build/ 会被整体挪走重建）。
rescue_prebuilt() {
  local pkg
  mkdir -p "$RESCUE_DIR"
  for pkg in $PLUGIN_PKGS rog_map; do
    [ -d "$NAV_WS/build/$pkg" ] || continue
    mkdir -p "$RESCUE_DIR/$pkg"
    cp -f "$NAV_WS/build/$pkg"/*.so "$RESCUE_DIR/$pkg/" 2>/dev/null
    cp -f "$NAV_WS/build/$pkg"/*.a  "$RESCUE_DIR/$pkg/" 2>/dev/null
    local f base
    for f in "$NAV_WS/build/$pkg"/*; do
      [ -f "$f" ] && [ -x "$f" ] || continue
      base="$(basename "$f")"
      case "$base" in
        *.so|*.a|*.cmake|*.sh|*.py|Makefile|*.dsv|*.bash|*.zsh|*.rc|*.txt|*.json|*.ini|*.log) continue ;;
      esac
      cp -f "$f" "$RESCUE_DIR/$pkg/"
    done
    say "抢救 $pkg 预编译产物 → $RESCUE_DIR/$pkg（$(ls "$RESCUE_DIR/$pkg" | wc -l) 个文件）"
  done
}

assemble_stub_tunnel() {                    # 占位：恒发 terrain_zone="" / tunnel_mode=false
  local dst="$1"
  cat > "$dst" <<'PYEOF'
#!/usr/bin/env python3
# tunnel_mode_manager 占位实现（原版源码在 2026-10-12 被清空，这里只保证 launch 不失败）
# 语义地图功能缺失：/terrain_zone 恒为空串，/tunnel_mode 恒 false。
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Bool


class Stub(Node):
    def __init__(self):
        super().__init__("tunnel_mode_manager")
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.zone = self.create_publisher(String, "terrain_zone", qos)
        self.mode = self.create_publisher(Bool, "tunnel_mode", qos)
        self.create_timer(1.0, self.tick)
        self.get_logger().warn("tunnel_mode_manager: 占位实现，terrain_zone 恒为 ''")

    def tick(self):
        self.zone.publish(String(data=""))
        self.mode.publish(Bool(data=False))


def main():
    rclpy.init()
    node = Stub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
PYEOF
  chmod +x "$dst"
}

assemble_plugins() {
  local pkg src dst
  if [ ! -d "$TPL_WS/install/planning" ]; then
    err "找不到同源模板 $TPL_WS/install/planning，无法拼装插件"
    return 1
  fi
  for pkg in $PLUGIN_PKGS; do
    # 优先用抢救出来的产物；没有就退回 beijixiong/build/
    src="$RESCUE_DIR/$pkg"
    [ -d "$src" ] && [ -n "$(ls -A "$src" 2>/dev/null)" ] || src="$NAV_WS/build/$pkg"
    if [ ! -d "$src" ]; then warn "跳过 $pkg（既没有抢救目录也没有 build/ 产物）"; continue; fi
    say "拼装 $pkg（源码：$src）→ $NAV_WS/install/$pkg"
    mkdir -p "$NAV_WS/install/$pkg/lib/$pkg"
    # ① ament 管道
    [ -d "$TPL_WS/install/$pkg/share" ] && cp -a "$TPL_WS/install/$pkg/share/." "$NAV_WS/install/$pkg/share/"
    [ -d "$TPL_WS/install/$pkg/include" ] && cp -a "$TPL_WS/install/$pkg/include" "$NAV_WS/install/$pkg/" 2>/dev/null
    # ② 库与可执行
    cp -f "$src"/*.so "$NAV_WS/install/$pkg/lib/" 2>/dev/null
    cp -f "$src"/*.a  "$NAV_WS/install/$pkg/lib/" 2>/dev/null
    local f base
    for f in "$src"/*; do
      [ -f "$f" ] && [ -x "$f" ] || continue
      base="$(basename "$f")"
      case "$base" in
        *.so|*.a|*.cmake|*.sh|*.py|Makefile|*.dsv|*.bash|*.zsh|*.rc|*.txt|*.json|*.ini|*.log) continue ;;
      esac
      cp -f "$f" "$NAV_WS/install/$pkg/lib/$pkg/"
    done
    # localization_fusion 的 config 用 beijixiong 源码里的覆盖模板版本（参数才与本机一致）
    if [ "$pkg" = "localization_fusion" ] && [ -f "$NAV_WS/localization_fusion/config/localization_fusion.yaml" ]; then
      mkdir -p "$NAV_WS/install/$pkg/share/$pkg/config"
      cp -f "$NAV_WS/localization_fusion/config/localization_fusion.yaml" "$NAV_WS/install/$pkg/share/$pkg/config/"
    fi
    # beijixiong build/ 里没有的可执行文件，从模板补齐
    if [ -d "$TPL_WS/install/$pkg/lib/$pkg" ]; then
      for f in "$TPL_WS/install/$pkg/lib/$pkg"/*; do
        [ -f "$f" ] || continue
        [ -e "$NAV_WS/install/$pkg/lib/$pkg/$(basename "$f")" ] || cp -f "$f" "$NAV_WS/install/$pkg/lib/$pkg/"
      done
    fi
  done
  # ③ tunnel_mode_manager：优先用抢救出来的真二进制（_prebuilt_rescue 里已确认有 387KB 的原版），
  #    只有真的缺失时才退化成占位实现
  if [ -d "$NAV_WS/install/planning" ] && [ ! -e "$NAV_WS/install/planning/lib/planning/tunnel_mode_manager" ]; then
    assemble_stub_tunnel "$NAV_WS/install/planning/lib/planning/tunnel_mode_manager"
    warn "planning/tunnel_mode_manager 用占位实现顶上（语义地图功能为空）"
  fi
  say "拼装完成：$PLUGIN_PKGS"
}

# 并行度控制：本机 32 核 / 14GB 内存，colcon 默认是「32 个包并行 × 每包 -j32」，
# 会瞬间把内存打爆。这里两处一起限：
#   ① --parallel-workers N  ：同时最多编译 N 个包
#   ② MAKEFLAGS="-jN -lN"   ：每个包内部最多 N 个编译任务（colcon 检测到 MAKEFLAGS
#      里已有 -j/-l 就不再自己追加 -j$(nproc)，见 colcon_cmake/task/cmake/build.py:291）
#      再配 CMAKE_BUILD_PARALLEL_LEVEL 兜住 Ninja 生成的包。
COLCON_PARALLEL=""
setup_jobs() {
  if [ "${JOBS}" -gt 0 ] 2>/dev/null; then
    export MAKEFLAGS="-j${JOBS} -l${JOBS}"
    export CMAKE_BUILD_PARALLEL_LEVEL="${JOBS}"
    COLCON_PARALLEL="--parallel-workers ${JOBS}"
    say "并行度限制：--parallel-workers ${JOBS} 且 MAKEFLAGS='-j${JOBS} -l${JOBS}'（总编译进程 ≤ $((JOBS*JOBS))）"
  else
    warn "未限制并行度：colcon 默认 32 个包并行、每包 -j$(nproc)，本机 14GB 内存极易 OOM"
  fi
}

if [ "$REPAIR_BT" = "1" ]; then
  say "重建 behaviortree_27（清 build/install/log 后全量编译）..."
  setup_jobs
  cd "$BT_WS"
  rm -rf build install log
  colcon build --symlink-install $COLCON_PARALLEL || { err "behaviortree_27 编译失败"; exit 1; }
  say "behaviortree_27 编译完成"
fi

if [ "$REPAIR_NAV" = "1" ]; then
  cd "$NAV_WS"
  say "修复 beijixiong：抢救预编译产物 → 清理外来 build 树 → 重建 → 拼装"
  # ① 先抢救被清空包的预编译产物（必须在动 build/ 之前）
  rescue_prebuilt

  # ② install 布局损坏 → 挪走
  if [ ! -s install/.colcon_install_layout ]; then
    local_bad="install.broken.$(date +%m%d_%H%M%S)"
    mv install "$local_bad"
    warn "旧的（已损坏且全空的）install 已改名为 $local_bad"
  fi

  # ③ build/ 是在原机器 /home/zjl/BJ/beijixiong 生成的，CMakeCache 里的源码路径对不上，
  #    会导致几乎每个包都报 “source ... does not match ... used to generate cache”，
  #    必须整体挪走重来（用 mv 不用 rm：万一还需要里面的产物还能捞回来）
  foreign_caches=$(grep -ls "/home/zjl/BJ/beijixiong" build/*/CMakeCache.txt 2>/dev/null | wc -l)
  if [ "$foreign_caches" -gt 0 ]; then
    foreign="build.foreign.$(date +%m%d_%H%M%S)"
    mv build "$foreign"
    warn "检测到 $foreign_caches 个包的 CMakeCache 指向原机器路径 → 整个 build 树已改名为 $foreign（新构建确认正常后可自行删除）"
  fi

  # ④ 重建：被清空的 5 个包必然失败，用 --continue-on-error 让其余包照常装
  setup_jobs
  # ⚠ localization_fusion 不从源码重编：它的源码用 std::shared_ptr 调 GTSAM，
  #   而本机可用的 GTSAM(4.2.0, 自带那份) 头文件要 boost::shared_ptr → 必然编译失败。
  #   方案：用旧 build 里的预编译产物拼进 install（见 assemble_plugins），构建时跳过它。
  export COLCON_SKIP_PKGS="localization_fusion rog_map_config_check"
  say "开始 colcon 全量重建（包多，耗时较长，耐心等）..."
  colcon build --symlink-install --continue-on-error $COLCON_PARALLEL \
    --packages-skip $COLCON_SKIP_PKGS 2>&1 | tail -30
  warn "上面若列出 controller / planning / dynamic_scan_filter / rog_map / tdt_nav_stack 失败，属预期（源码被清空）"
  warn "⚠ 绝不要用 --packages-ignore 跳过 localization_fusion：它在 launch 期被 get_package_share_directory 用到，"
  warn "  包不在 install 里 → launch 直接加载失败（报 PackageNotFoundError）"
  warn "其它包若也失败，把报错贴给我 —— 多半是与那 5 个包有编译依赖，或还有别的源码损坏"

  # ⑤ 拼回被清空的包
  assemble_plugins
  say "beijixiong 重建 + 拼装流程结束"
fi

if [ "$ASSEMBLE_ONLY" = "1" ]; then
  assemble_plugins
  exit $?
fi

if [ "$CHECK_ONLY" = "1" ]; then
  preflight && say "预检通过" || warn "预检未通过（见上）"
  exit 0
fi

if ! preflight; then
  err "预检未通过，已中止。先跑：$0 --repair-nav  /  $0 --repair-bt"
  exit 1
fi

# ───────────────────── 开终端（有 DISPLAY 用标签页，否则后台+日志）─────────────────────
mkdir -p "$LOG_DIR"
HAS_GUI=0
if [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null 2>&1; then HAS_GUI=1; fi

# run_term <标题> <日志名> <命令字符串>
run_term() {
  local title="$1" log="$2" cmd="$3"
  if [ "$DRY_RUN" = "1" ]; then
    echo "──────── [dry-run] 本会在「$title」终端里执行："
    echo "$cmd" | sed 's/^/    /'
    return 0
  fi
  if [ "$HAS_GUI" = "1" ]; then
    gnome-terminal --tab --title="$title" -- bash -c "$cmd; echo; echo '[${title}] 已退出，按回车关闭'; exec bash" >/dev/null 2>&1
  else
    say "（无 DISPLAY，后台运行：$title → $LOG_DIR/$log）"
    nohup bash -c "$cmd" >"$LOG_DIR/$log" 2>&1 &
  fi
}

# 每个终端里都要重新 source（gnome-terminal 是独立 shell）
# ⚠ LD_LIBRARY_PATH 有三件事必须做，少一件导航就起不来：
#   ① 剔除 /opt/MVS（相机 SDK 自带老 libusb，排在 ~/.bashrc 最前面）：
#      不剔 → PCL 报 "libpcl_io.so.1.12: undefined symbol: libusb_set_option"
#      → libcontroller.so 插件加载失败 → controller_server 起不来 → navigate_to_pose 不可达，
#        同时 gicp_node 秒退 → 没有 map→odom
#   ② 补 tdt-nav-kit/3rd/install/lib：controller/planning 插件的 RUNPATH 指向原机器
#      (/home/zjl/BJ/...)，缺了会报 libOsqpEigen.so.0.8.0 cannot open shared object file
#   ③ 补 GTSAM 的 lib/x86_64-linux-gnu：localization_fusion 的 fusion_backend_node 等要用
ENV_SETUP="source /opt/ros/humble/setup.bash
source $NAV_WS/install/setup.bash
source $BT_WS/install/setup.bash
export LD_LIBRARY_PATH=\$(echo \"\${LD_LIBRARY_PATH:-}\" | tr ':' '\n' | grep -v '^/opt/MVS' | grep -v '^$' | paste -sd:)
# ⚠ 顺序极其重要：ROS 的路径必须留在**前面**，/lib/x86_64-linux-gnu 只能**追加**。
#   因为 /lib/x86_64-linux-gnu 是 /usr/lib/x86_64-linux-gnu 的软链，里面有老版
#   libclass_loader.so.0.5.0（ROS1 时代，没有 ClassLoader::getLibraryPath），
#   一旦前置 → 所有 rclcpp 节点报 undefined symbol: ...getLibraryPathB5cxx11Ev 并 127 退出。
export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH:-}:/lib/x86_64-linux-gnu:$NAV_WS/tdt-nav-kit/3rd/install/lib${GTSAM_LIB_DIR:+:$GTSAM_LIB_DIR}
# 拼装出来的 3 个包不是 colcon 构建时登记的，显式补进环境（pluginlib 靠 AMENT_PREFIX_PATH 找插件）
for _p in planning controller dynamic_scan_filter localization_fusion; do
  _d=$NAV_WS/install/\$_p
  [ -d \"\$_d\" ] || continue
  export AMENT_PREFIX_PATH=\"\$_d:\${AMENT_PREFIX_PATH:-}\"
  export LD_LIBRARY_PATH=\"\$_d/lib:\${LD_LIBRARY_PATH:-}\"
done
$RMW_LINE
export GZ_PARTITION=bjx_sim
export IGN_PARTITION=bjx_sim"

# ───────────────────────────── 1) Gazebo ─────────────────────────────
if [ "$RUN_GZ" = "1" ]; then
  say "[1/4] 启动 Gazebo（world 取自 rmu_gazebo_simulator/config/gz_world.yaml，当前=$WORLD）..."
  run_term "Gazebo" "gz.log" "$ENV_SETUP
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py"
  [ "$DRY_RUN" = "1" ] || sleep 8   # 等 Gazebo/模型 spawn 起来，再起导航
fi

# ───────────────────────────── 2) 导航栈 ─────────────────────────────
if [ "$RUN_NAV" = "1" ]; then
  RVIZ_ARG=$( [ "$USE_RVIZ" = 1 ] && echo true || echo false )
  if [ "$MODE" = "sim" ]; then
    say "[2/4] 启动导航栈（仿真：slam=False relocalization=$RELOC odometry=$ODOM rviz=$RVIZ_ARG）..."
    NAV_CMD="ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=$WORLD slam:=False relocalization:=$RELOC odometry:=$ODOM namespace:=$NS use_sim_time:=true use_rviz:=$RVIZ_ARG"
  else
    say "[2/4] 启动导航栈（实车：namespace='$NS' use_sim_time=false）..."
    NAV_CMD="ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py namespace:=$NS use_sim_time:=false use_rviz:=$RVIZ_ARG"
  fi
  run_term "Navigation" "nav.log" "$ENV_SETUP
$NAV_CMD"
  [ "$DRY_RUN" = "1" ] || sleep 6
fi

# ───────────────────── 2.5) 可选：模拟裁判数据 ─────────────────────
# 仿真里没有 /game_status 发布者（只有 /referee_system/<robot>/robot_status），
# 而树的入口是 IsGameTime(message="{game_status}", game_progress=4...)，所以仿真下必须补。
if [ "$MOCK" = "1" ]; then
  say "[3/4] 启动模拟裁判（/game_status + /robot_status，供仿真下推进行为树）..."
  MOCK_PY="$LOG_DIR/mock_referee.py"
  cat > "$MOCK_PY" <<'PYEOF'
import rclpy
from rclpy.node import Node
from robot_msgs.msg import GameStatus, RobotStatus

class MockReferee(Node):
    def __init__(self):
        super().__init__("mock_referee")
        self.gs = self.create_publisher(GameStatus, "/game_status", 10)
        self.rs = self.create_publisher(RobotStatus, "/robot_status", 10)
        self.create_timer(0.2, self.tick)   # 5 Hz，latched 与否都能被订阅到

    def tick(self):
        g = GameStatus()
        g.game_progress = 4          # 4 = 比赛中（IsGameTime 的判据）
        g.stage_remain_time = 300
        self.gs.publish(g)

        r = RobotStatus()
        r.robot_id = 1               # 红方 1 号
        r.current_hp = 400
        r.shooter_barrel_heat_limit = 300
        r.shooter_heat = 0
        r.team_color = False         # False = 红
        r.is_attacked = False
        self.rs.publish(r)

def main():
    rclpy.init()
    n = MockReferee()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
PYEOF
  run_term "MockReferee" "mock.log" "$ENV_SETUP
python3 $MOCK_PY"
  [ "$DRY_RUN" = "1" ] || sleep 2
fi

# ───────────────────────────── 3) 行为树 ─────────────────────────────
if [ "$RUN_BT" = "1" ]; then
  USE_SIM_TIME=$([ "$MODE" = "sim" ] && echo true || echo false)
  # 行为树节点跑在根命名空间，而导航栈挂在 /<ns> 下 → 话题/动作/TF 都要 remap 过去，
  # 否则 SendGoal 找不到 navigate_to_pose 动作服务器、GetCurrentLocation 查不到 TF。
  BT_ARGS="-p bt_xml:=$BT_XML -p use_sim_time:=$USE_SIM_TIME"
  if [ -n "$NS" ]; then
    # 关键：让 BT 节点跑在导航的命名空间下，相对动作名 navigate_to_pose 才会解析成
    # /<ns>/navigate_to_pose。⚠ 不要试图用 "-r /navigate_to_pose:=/<ns>/navigate_to_pose"
    # 来 remap action：ROS2 里 action 的底层实体是 /<name>/_action/{send_goal,cancel_goal,
    # get_result,feedback,status}，只 remap 基名不会传导，客户端仍然找不到服务器
    # （实测：remap 基名 → 一直 "Action server ... is not reachable"；改用 __ns → 正常）。
    # ⚠ __ns 必须是全限定名（带前导 /）：写成 "__ns:=red_standard_robot1" 会被 rcl 拒绝
    #   （"Namespace not remapped to a fully qualified name" → Failed to parse global arguments → 进程直接 abort）
    BT_ARGS="$BT_ARGS -r __ns:=/$NS -r /tf:=tf -r /tf_static:=tf_static -r /map:=/$NS/map"
  fi

  say "[4/4] 启动行为树：$BT_XML"
  run_term "BehaviorTree" "bt.log" "$ENV_SETUP
ros2 run $BT_PKG $BT_PKG --ros-args $BT_ARGS"
fi

# ───────────────────────────── 收尾提示 ─────────────────────────────
cat <<EOF

$(say "启动流程结束")模式：$MODE   命名空间：'$NS'   行为树：rmul_2026_3v3_sentry.xml

接下来怎么看：
  ros2 node list                      # 应能看到行为树节点与 nav2 各节点
  ros2 topic list | grep -E "game_status|robot_status|terrain_map|auto_aim_target_pos"
  ros2 topic echo /game_status        # 需有数据，否则树被 IsGameTime 挡住（仿真请用 --mock-referee）
  ros2 action list | grep navigate_to_pose
  Groot2 连 1667 端口看实时状态（树里每个节点的颜色）

已知会挡路的两件事（详见本文件头部注释）：
  1) beijixiong 的 controller / planning 等 5 个包源码被清空，未恢复前
     controller_server、planner_server 加载插件会失败 → 不会真正移动；
  2) 仿真里没有裁判数据，调试决策逻辑请加 --mock-referee。

排查小抄：
  ros2 run tf2_ros tf2_echo map gimbal_yaw     # 树的 GetCurrentLocation 用这个变换
  ros2 topic echo /chassis_energy --once       # 缺数据时对应分支会被条件节点挡住
  ros2 node list | grep -E "controller_server|planner_server|bt_navigator"
EOF
