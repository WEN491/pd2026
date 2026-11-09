#!/usr/bin/env python3
"""listen_semantic.py — 监听语义地图（tunnel_mode_manager）发布的当前语义

发布接口（实测于本机，2026-10-17）：
  /terrain_zone        std_msgs/String           当前地形类型；"" = 不在任何 zone
  /tunnel_mode         std_msgs/Bool             type==tunnel 时为 true（兼容旧链路）
  /terrain_zones_viz   visualization_msgs/MarkerArray   RViz 可视化
  /set_tunnel_mode     std_srvs/SetBool          手动模式（auto_enable=false 时用）

  ⚠ QoS = RELIABLE + TRANSIENT_LOCAL（latched）。订阅端必须请求 TRANSIENT_LOCAL，
    否则只有"订阅之后发生的变化"才收得到；ROS 里 latched 的历史值会丢。

  ⚠ 由导航 launch 启动时，节点在命名空间里 → 话题变成
    /<ns>/terrain_zone 等（默认 ns = red_standard_robot1）。

用法：
  python3 listen_semantic.py                                  # 根命名空间
  python3 listen_semantic.py --namespace red_standard_robot1   # 仿真/实车常见
  python3 listen_semantic.py -n red_standard_robot1 -d 30      # 只监听 30 秒
  python3 listen_semantic.py -n red_standard_robot1 --call true  # 顺手手动置 tunnel_mode
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--namespace", default="", help="导航命名空间，如 red_standard_robot1")
    ap.add_argument("-d", "--duration", type=float, default=0, help="监听秒数，0 = 一直听")
    ap.add_argument("--call", choices=["true", "false"], help="调用 /set_tunnel_mode 手动置位")
    args = ap.parse_args()

    ns = args.namespace.strip("/")
    prefix = f"/{ns}" if ns else ""

    rclpy.init()
    node = Node("semantic_listener")

    # 关键：必须 TRANSIENT_LOCAL 才能拿到 latched 的当前值
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE

    state = {"zone": None, "mode": None}

    def fmt(t):
        return time.strftime("%H:%M:%S", time.localtime(t))

    def on_zone(msg):
        if msg.data != state["zone"]:
            state["zone"] = msg.data
            shown = msg.data if msg.data else "(不在任何 zone)"
            print(f"[{fmt(time.time())}] terrain_zone = {shown}")

    def on_mode(msg):
        if msg.data != state["mode"]:
            state["mode"] = msg.data
            print(f"[{fmt(time.time())}] tunnel_mode  = {msg.data}")

    node.create_subscription(String, f"{prefix}/terrain_zone", on_zone, qos)
    node.create_subscription(Bool, f"{prefix}/tunnel_mode", on_mode, qos)

    if args.call:
        from std_srvs.srv import SetBool
        cli = node.create_client(SetBool, f"{prefix}/set_tunnel_mode")
        if cli.wait_for_service(timeout_sec=3.0):
            req = SetBool.Request()
            req.data = args.call == "true"
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=3.0)
            print(f"[set_tunnel_mode] -> {fut.result()}")
        else:
            print("[warn] 找不到 /set_tunnel_mode 服务（节点没起？）")

    print(f"监听中：{prefix or ''}/terrain_zone 与 {prefix or ''}/tunnel_mode（Ctrl-C 退出）")
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.duration and time.time() - t0 >= args.duration:
                break
        print(f"最终状态：{state}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
