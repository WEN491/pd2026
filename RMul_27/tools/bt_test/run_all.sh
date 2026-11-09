#!/bin/bash
# 一键运行逐节点测试。
# 必须在同一个 job 内同时起 mock 与 BT：跨 job 的 DDS 共享内存数据面不通。
set -o pipefail
cd "$(dirname "$0")"
WS="$(cd ../.. && pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export HOME="$WS/.testhome"
export ROS_HOME="$HOME"
export ROS_LOG_DIR="$HOME/log"
mkdir -p "$HOME/log" results
rm -f results/record.jsonl

python3 mock_stack.py --record results/record.jsonl > results/mock.log 2>&1 &
MOCK=$!
sleep 2

python3 run_cases.py "$@"
RC=$?

kill -INT "$MOCK" 2>/dev/null
sleep 0.5
kill -9 "$MOCK" 2>/dev/null
wait "$MOCK" 2>/dev/null
exit "$RC"
