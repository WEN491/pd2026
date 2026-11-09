#!/bin/bash
# =============================================================================
# run_2026_3v3_sentry.sh — 用 beijixiong 导航栈跑 2026 3v3 哨兵行为树
# =============================================================================
# 行为树：rm_behavior_tree/rm_behavior_tree/config/rmul_2026_3v3_sentry.xml
# 导航栈：/home/w/pd2027/beijixiong（通过 run_bt_nav.sh 启动）
#
# 用法（除 --tree-id 外的参数都原样透传给 run_bt_nav.sh）：
#   ./run_2026_3v3_sentry.sh                  # 仿真：Gazebo + 导航栈 + 该树
#   ./run_2026_3v3_sentry.sh --reality        # 实车：导航栈 + 该树
#   ./run_2026_3v3_sentry.sh --bt-only        # 只起行为树（导航已在别的终端跑着）
#   ./run_2026_3v3_sentry.sh --mock-referee   # 仿真 + 模拟裁判数据
#   ./run_2026_3v3_sentry.sh --check          # 只做环境预检
#   ./run_2026_3v3_sentry.sh --dry-run        # 只打印将要执行的命令
#   ./run_2026_3v3_sentry.sh --tree-id MainTree  # 跑文件里的 MainTree（斜坡占位树）
#   ./run_2026_3v3_sentry.sh --literal        # 严格按 XML 里写的入口跑，不做改写
#   ./run_2026_3v3_sentry.sh --wait-nav 60     # 等 nav2 变 active 的上限（默认 120s）
#
# 默认跑 ID="Subtree"（完整 3v3 哨兵逻辑）：文件里的 main_tree_to_execute 指向
# MainTree，而 MainTree 只是两条 SendGoal 的斜坡占位（按 terrain_zone 选一个点）。
# 脚本只改副本的入口属性，不动原 XML。
# ⚠ 仿真是没有 /game_status 的；不启动 mock referee 时 IsGameTime 恒假，
#   完整树会被挡住。调试决策请加 --mock-referee。
# =============================================================================

set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
XML="$WS/rm_behavior_tree/rm_behavior_tree/config/rmul_2026_3v3_sentry.xml"
TREE_ID="Subtree"    # 默认跑完整 3v3 逻辑；--tree-id MainTree 可跑占位树
RAW=0                # 1 = 严格按 XML 入口，不改写
PASSTHRU=()

while [ $# -gt 0 ]; do
  case "$1" in
    --tree-id)
      TREE_ID="${2:-}"
      RAW=0
      shift
      ;;
    --literal|--as-is)
      RAW=1
      TREE_ID=""
      ;;
    -h|--help)
      awk 'NR>1 { if ($0 ~ /^#/ || $0 == "") print; else exit }' "$0"
      exit 0
      ;;
    *)
      PASSTHRU+=("$1")
      ;;
  esac
  shift
done

if [ ! -s "$XML" ]; then
  echo "错误：行为树文件不存在或为空：$XML" >&2
  exit 1
fi

BT_XML="$XML"

if [ "$RAW" = "0" ] && [ -n "$TREE_ID" ]; then
  if ! [[ "$TREE_ID" =~ ^[A-Za-z0-9_]+$ ]]; then
    echo "错误：--tree-id 只允许字母、数字、下划线：$TREE_ID" >&2
    exit 1
  fi
  if ! grep -q "BehaviorTree ID=\"$TREE_ID\"" "$XML"; then
    echo "错误：$XML 里没有 ID=\"$TREE_ID\" 的树" >&2
    echo "      可用的树 ID：" >&2
    grep -o 'BehaviorTree ID="[^"]*"' "$XML" | sed 's/^/        /' >&2
    exit 1
  fi
  # 生成只改入口的副本（Sed 只动 <root> 上的 main_tree_to_execute 属性）
  GEN_DIR="$WS/log/run_bt_nav/trees"
  mkdir -p "$GEN_DIR"
  GEN="$GEN_DIR/$(basename "${XML%.xml}").${TREE_ID}.xml"
  sed -E 's/main_tree_to_execute="[^"]*"/main_tree_to_execute="'"$TREE_ID"'"/' "$XML" > "$GEN"
  echo "[run_2026_3v3_sentry] 入口树：$TREE_ID（生成副本：$GEN）"
  BT_XML="$GEN"
fi

# 仿真里没有 /game_status：从参数里认一下，没加 --mock-referee 就提醒
has_arg() {
  local a
  for a in "${PASSTHRU[@]}"; do
    [ "$a" = "$1" ] && return 0
  done
  return 1
}
if ! has_arg --reality && ! has_arg --mock-referee && ! has_arg --nav-only \
   && ! has_arg --check && ! has_arg --dry-run; then
  echo "[提示] 仿真没有 /game_status，IsGameTime 会挡住完整树；调试决策请加 --mock-referee。" >&2
fi

exec "$WS/run_bt_nav.sh" --tree "$BT_XML" "${PASSTHRU[@]}"
