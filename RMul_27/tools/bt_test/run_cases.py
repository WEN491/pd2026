#!/usr/bin/env python3
"""逐节点运行测试执行器。

必须由 run_all.sh 在同一个进程/命名空间里启动 mock_stack.py 和本脚本：
后台 job 与前台命令之间 DDS 数据面（共享内存）不通，只有同一 job 内的进程能互收。

用法：
  python3 run_cases.py [--only CASE,CASE] [--duration 4.0] [--list]
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import cases as case_mod

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.abspath(os.path.join(HERE, "..", ".."))
BT_EXE = os.path.join(WS, "build", "rm_behavior_tree", "rm_behavior_tree")
RECORD = os.path.join(HERE, "results", "record.jsonl")
CASE_DIR = os.path.join(HERE, "cases")
RESULT_DIR = os.path.join(HERE, "results")
WALL_T0 = None


class RunnerNode(Node):
    def __init__(self):
        super().__init__("bt_test_runner")
        self.cfg_pub = self.create_publisher(String, "/bt_test/config", 10)
        self.mark_pub = self.create_publisher(String, "/bt_test/mark", 10)

    def set_config(self, cfg, settle=0.35):
        cfg = dict(cfg)
        cfg["reset"] = True
        msg = String()
        msg.data = json.dumps(cfg)
        self.cfg_pub.publish(msg)
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def mark(self, text, settle=0.05):
        msg = String()
        msg.data = text
        self.mark_pub.publish(msg)
        time.sleep(settle)


def start_bt(xml_path, tag):
    env = os.environ.copy()
    env["HOME"] = os.path.join(WS, ".testhome")
    env["ROS_LOG_DIR"] = os.path.join(WS, ".testhome", "log")
    env["ROS_HOME"] = os.path.join(WS, ".testhome")
    # 每个 case 用独立日志文件，避免 BT.CPP 的 spdlog 互相覆盖
    os.makedirs(env["ROS_LOG_DIR"], exist_ok=True)
    # 直接起可执行文件：ros2 run 的 python 包装层被 SIGKILL 时子进程会成为孤儿，
    # 一直占着 Groot2 的 1667 端口，后续 case 会 "Address already in use"。
    cmd = [BT_EXE, "--ros-args", "-p", f"bt_xml:={xml_path}"]
    proc = subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, env=env,
    )
    lines = []
    lock = threading.Lock()

    def reader():
        for line in proc.stdout:
            with lock:
                lines.append((time.time(), line.rstrip("\n")))

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    return proc, lines, lock, th


def wait_port_free(port=1667, timeout=8.0):
    import socket
    end = time.time() + timeout
    while time.time() < end:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind(("0.0.0.0", port))
            s.close()
            return True
        except OSError:
            s.close()
            time.sleep(0.2)
    return False


def stop_bt(proc, th):
    if proc.poll() is None:
        try:
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=1.5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=1)
    th.join(timeout=2)


def read_records(path, offset):
    recs = []
    try:
        with open(path, "r", encoding="utf-8") as f:
            f.seek(offset)
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    recs.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
            new_offset = f.tell()
    except FileNotFoundError:
        new_offset = offset
    return recs, new_offset


def get_path(obj, path):
    cur = obj
    for part in path.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        else:
            return None
    return cur


def check_topic(recs, spec, start_t):
    topic = spec["topic"]
    field = spec.get("field")
    found = []
    for r in recs:
        if r["topic"] != topic or r.get("t", -1) < start_t:
            continue
        val = r["data"] if field is None else get_path(r["data"], field)
        found.append((r.get("t"), val))
    errors = []
    min_count = spec.get("min_count", 1)
    if len(found) < min_count:
        return [f"{topic}{'.' + field if field else ''}: 期望至少 {min_count} 条，实际 {len(found)}"]
    if "equals" in spec:
        if not any(v == spec["equals"] for _, v in found):
            errors.append(f"{topic}{'.' + field if field else ''}: 未出现期望值 {spec['equals']!r}，实际样本 {found[:5]}")
    if "approx" in spec:
        tol = spec.get("tol", 0.05)
        if not any(isinstance(v, (int, float)) and abs(v - spec["approx"]) <= tol for _, v in found):
            errors.append(f"{topic}{'.' + field if field else ''}: 未出现 ≈{spec['approx']} (tol {tol})，实际样本 {found[:5]}")
    if "contains" in spec:
        if not any(v is not None and spec["contains"] in str(v) for _, v in found):
            errors.append(f"{topic}{'.' + field if field else ''}: 未包含 {spec['contains']!r}")
    if "absent" in spec:
        if any(v == spec["absent"] for _, v in found):
            errors.append(f"{topic}{'.' + field if field else ''}: 不应出现 {spec['absent']!r}")
    return errors


def check_goals(recs, spec, start_t):
    goals = [r["data"] for r in recs if r["topic"] == "__goal__" and r.get("t", -1) >= start_t]
    if not goals:
        return ["navigate_to_pose: 没有收到 goal"]
    tol = spec.get("tol", 0.15)
    hit = [g for g in goals
           if abs(g["x"] - spec["x"]) <= tol and abs(g["y"] - spec["y"]) <= tol]
    if "count" in spec:
        if len(goals) != spec["count"]:
            return [f"goal 数量期望 {spec['count']}，实际 {len(goals)}: {[(g['x'], g['y']) for g in goals]}"]
    if not hit:
        return [f"goal 未命中 ({spec['x']}, {spec['y']}) ±{tol}; 实际 {[(round(g['x'],2), round(g['y'],2)) for g in goals]}"]
    return []


def check_no_goal(recs, start_t):
    goals = [r["data"] for r in recs if r["topic"] == "__goal__" and r.get("t", -1) >= start_t]
    return [] if not goals else [f"不应发导航目标，实际 {[(g['x'], g['y']) for g in goals]}"]


def check_stdout(lines, patterns, start_t, mode="all"):
    # C++ 的 std::cout 在管道下是块缓冲的，进程退出时才一次性刷出，
    # 所以裸 stdout 行的时间戳不可信（会全部堆到结束时）。PrintMessage 同时打
    # RCLCPP_INFO，因此所有 CASE_RESULT 判定只看 [print_message] 行。
    probe_text = "\n".join(l for t, l in lines if t >= start_t and "[print_message]" in l)
    all_text = "\n".join(l for t, l in lines if t >= start_t)

    def haystack(pattern):
        return probe_text if "CASE_RESULT" in pattern else all_text

    if mode == "none":
        hits = [p for p in patterns if re.search(p, haystack(p), re.M)]
        return [] if not hits else [f"stdout 不应命中 {hits}"]
    missing = [p for p in patterns if not re.search(p, haystack(p), re.M)]
    return [] if not missing else [f"stdout 缺少 {missing}"]


def check_timing(lines, spec, start_t):
    events = [(t, line) for t, line in lines if t >= start_t and "[print_message]" in line]
    start_pat, end_pat = spec["start"], spec["end"]
    t0 = next((t for t, l in events if re.search(start_pat, l)), None)
    if t0 is None:
        return [f"timing: 未找到起点 {start_pat!r}"]
    t1 = next((t for t, l in events if t > t0 and re.search(end_pat, l)), None)
    if t1 is None:
        return [f"timing: 未找到终点 {end_pat!r}"]
    dt = (t1 - t0) * 1000.0
    lo, hi = spec.get("min_ms", 0), spec.get("max_ms", 1e9)
    if not (lo <= dt <= hi):
        return [f"timing: {start_pat!r}→{end_pat!r} = {dt:.0f} ms，期望 [{lo}, {hi}]"]
    return []


def count_stdout(lines, pattern, start_t):
    # 只数 ROS 日志行，避免裸 stdout 的重复行和退出时批量刷出干扰
    return sum(1 for t, l in lines
               if t >= start_t and "[print_message]" in l and re.search(pattern, l))


def run_case(node, case):
    name = case["name"]
    print(f"\n=== CASE {name} ===", flush=True)
    node.set_config(case.get("config", {}))
    if case.get("xml_file"):
        path = os.path.join(WS, case["xml_file"])
    else:
        xml = case["xml"]
        os.makedirs(CASE_DIR, exist_ok=True)
        path = os.path.join(CASE_DIR, f"{name}.xml")
        with open(path, "w", encoding="utf-8") as f:
            f.write(xml)

    if not wait_port_free(1667, 8.0):
        print(f"[WARN] {name}: Groot2 port 1667 仍被占用，继续尝试", flush=True)
    offset = os.path.getsize(RECORD) if os.path.exists(RECORD) else 0
    node.mark(name)
    t_start = time.time()
    proc, lines, lock, th = start_bt(path, name)
    duration = case.get("duration", 4.0)
    time.sleep(duration)
    stop_bt(proc, th)
    t_end = time.time()
    recs, _ = read_records(RECORD, offset)
    with lock:
        out = list(lines)

    warm = case.get("warmup", 0.8)
    check_from = t_start + warm
    case_rel = t_start - WALL_T0
    errors = []
    for spec in case.get("topics", []):
        errors += check_topic(recs, spec, case_rel)
    if case.get("no_goal"):
        errors += check_no_goal(recs, case_rel)
    for spec in case.get("goals", []):
        errors += check_goals(recs, spec, case_rel)
    if case.get("stdout_all"):
        errors += check_stdout(out, case["stdout_all"], check_from, "all")
    if case.get("stdout_none"):
        errors += check_stdout(out, case["stdout_none"], check_from, "none")
    if "timing" in case:
        errors += check_timing(out, case["timing"], check_from)
    for ck in case.get("counts", []):
        n = count_stdout(out, ck["pattern"], check_from)
        if not (ck.get("min", 0) <= n <= ck.get("max", 10 ** 9)):
            errors.append(f"count({ck['pattern']!r})={n}，期望 [{ck.get('min')}, {ck.get('max')}]")
    if "min_count_any" in case:
        n = count_stdout(out, case["min_count_any"]["pattern"], check_from)
        if n < case["min_count_any"]["min"]:
            errors.append(f"count({case['min_count_any']['pattern']!r})={n} < {case['min_count_any']['min']}")
    if case.get("expect_load_failure"):
        text = "\n".join(l for _, l in out)
        if proc.returncode == 0:
            errors.append("期望加载失败，但进程正常退出")
        if not re.search(case["expect_load_failure"], text):
            errors.append(f"加载失败输出中未匹配 {case['expect_load_failure']!r}")
    elif case.get("expect_no_crash", True):
        text = "\n".join(l for _, l in out)
        if "terminate called" in text and "signal_handler" not in text:
            errors.append("进程异常 terminate")
        if "what():" in text and proc.returncode not in (0, -2, 130):
            errors.append(f"异常退出 returncode={proc.returncode}")

    status = "PASS" if not errors else "FAIL"
    print(f"[{status}] {name}: {errors if errors else 'ok'}", flush=True)

    with open(os.path.join(RESULT_DIR, f"{name}.log"), "w", encoding="utf-8") as f:
        f.write(f"case: {name}\nstatus: {status}\n")
        f.write(f"duration: {t_end - t_start:.2f}s\n")
        f.write(f"returncode: {proc.returncode}\n")
        f.write("errors:\n")
        for e in errors:
            f.write(f"  - {e}\n")
        f.write("\n--- stdout ---\n")
        for t, l in out:
            f.write(f"{t - t_start:8.3f} {l}\n")
        f.write("\n--- records ---\n")
        for r in recs:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    return {
        "name": name,
        "status": status,
        "errors": errors,
        "duration": round(t_end - t_start, 2),
        "returncode": proc.returncode,
        "stdout_lines": len(out),
        "goal_count": sum(1 for r in recs if r["topic"] == "__goal__"),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="")
    ap.add_argument("--exclude", default="")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    if args.list:
        for c in case_mod.CASES:
            print(c["name"])
        return

    only = [s.strip() for s in args.only.split(",") if s.strip()]
    exclude = [s.strip() for s in args.exclude.split(",") if s.strip()]
    selected = [c for c in case_mod.CASES
                if (not only or c["name"] in only)
                and not any(c["name"].startswith(p) for p in exclude)]
    os.makedirs(RESULT_DIR, exist_ok=True)

    global WALL_T0
    while not os.path.exists(RECORD) or os.path.getsize(RECORD) == 0:
        time.sleep(0.2)
    with open(RECORD, "r", encoding="utf-8") as f:
        meta = json.loads(f.readline())
    WALL_T0 = meta["data"]["wall_t0"]

    rclpy.init()
    node = RunnerNode()
    results = []
    try:
        # 等 mock stack 起来并收到配置
        node.set_config({}, settle=1.5)
        for case in selected:
            if args.duration is not None:
                case["duration"] = args.duration
            results.append(run_case(node, case))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    with open(os.path.join(RESULT_DIR, "results.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    passed = sum(1 for r in results if r["status"] == "PASS")
    print(f"\n===== {passed}/{len(results)} passed =====", flush=True)
    for r in results:
        if r["status"] != "PASS":
            print(f"FAIL {r['name']}: {r['errors']}", flush=True)


if __name__ == "__main__":
    main()
