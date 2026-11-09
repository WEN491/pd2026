#!/usr/bin/env python3
"""BT 节点运行测试的 mock 数据面。

提供 2027 主树 / 单节点测试树需要的全部输入：
  - 裁判/电控话题：/game_status /robot_status /robot_hp /chassis_energy
    /auto_aim_status /remote_status /rune_status /decision_num
    /auto_aim_target_pos (auto_aim_interfaces/Armors)
  - 语义话题：/terrain_zone
  - TF：map->odom、map->gimbal_yaw（恒等）
  - 地图：/map（40m x 40m 全空闲，transient_local）
  - Nav2 action：/navigate_to_pose

配置通过 /bt_test/config (std_msgs/String, JSON) 热更新；记录所有 BT 输出话题到
JSONL，供 run_cases.py 做断言。

用法：
  python3 mock_stack.py --record results/record.jsonl
"""
import argparse
import array
import json
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Time
from tf2_ros import StaticTransformBroadcaster

from robot_msgs.msg import (
    AutoAimStatus,
    AllRobotHP,
    ChassisEnergy,
    ChassisMode,
    GameStatus,
    RemoteStatus,
    RobotState,
    RobotStatus,
    RuneStatus,
    SuperCapacitor,
    AutoAimMode,
    GimbalTarget,
)
from rm_decision_interfaces.msg import DecisionNum, RobotControl
from auto_aim_interfaces.msg import Armor, Armors
from sp_msgs.msg import AutoaimTargetMsg, EnemyStatusMsg

DEFAULTS = {
    "game_status": {"game_progress": 4, "stage_remain_time": 300},
    "robot_status": {
        "robot_id": 1,
        "current_hp": 400,
        "shooter_barrel_heat_limit": 300,
        "shooter_heat": 100,
        "team_color": False,
        "is_attacked": False,
        "hp_pattern": "steady",  # steady | sawtooth
    },
    "robot_hp": {
        "red_outpost_hp": 500,
        "blue_outpost_hp": 400,
        "outpost_hp": 500,
        "enemy_outpost_hp": 400,
        "base_hp": 5000,
        "enemy_base_hp": 5000,
        "red_base_hp": 5000,
        "blue_base_hp": 5000,
        "team_color": True,
        "enemy_color": False,
    },
    "chassis_energy": {"chassis_energy": 0},
    "auto_aim_status": {"auto_aim_status": 0},
    "remote_status": {"remote_control_status": 1},
    "rune_status": {"rune_count": 0},
    "decision_num": {"decision_num": 0},
    "armors": {"x": 5.0, "y": 2.0, "z": 0.0, "frame": "odom", "count": 1},
    "terrain_zone": {"data": ""},
    "gimbal_target": {"x": 3.0, "y": 4.0},
    "nav": {"mode": "arrive", "feedback_distance": 0.1, "duration": 0.5},
    "tf": {"odom_xyz": [0.0, 0.0, 0.0], "gimbal_xyz": [0.0, 0.0, 0.0]},
}


def deep_merge(base, upd):
    for k, v in upd.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def to_dict(msg):
    """把消息转成可 JSON 化的 dict（只保留标量/数组/嵌套）。"""
    out = {}
    for field in msg.get_fields_and_field_types():
        val = getattr(msg, field)
        if hasattr(val, "get_fields_and_field_types"):
            out[field] = to_dict(val)
        elif isinstance(val, (list, tuple, array.array)):
            out[field] = [
                to_dict(x) if hasattr(x, "get_fields_and_field_types") else x for x in val
            ]
        elif hasattr(val, "sec") and hasattr(val, "nanosec"):
            out[field] = {"sec": val.sec, "nanosec": val.nanosec}
        else:
            out[field] = val
    return out


class MockStack(Node):
    def __init__(self, record_path):
        super().__init__("bt_test_mock_stack")
        self.cfg = json.loads(json.dumps(DEFAULTS))
        self.record_path = record_path
        self.record_lock = threading.Lock()
        self.goals = []
        self.goal_lock = threading.Lock()
        self.wall_t0 = time.time()
        self.t0 = self.wall_t0

        qos = QoSProfile(depth=10)
        # 输入话题统一用 transient_local：订阅端（BT::RosTopicSubNode）要求
        # durable，volatile 发布者会被判 QoS 不兼容而收不到任何消息。
        latched = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        cbg = ReentrantCallbackGroup()

        # ---- 输入话题 ----
        self.pub_game = self.create_publisher(GameStatus, "/game_status", latched)
        self.pub_robot = self.create_publisher(RobotStatus, "/robot_status", latched)
        self.pub_hp = self.create_publisher(AllRobotHP, "/robot_hp", latched)
        self.pub_energy = self.create_publisher(ChassisEnergy, "/chassis_energy", latched)
        self.pub_aim_status = self.create_publisher(AutoAimStatus, "/auto_aim_status", latched)
        self.pub_remote = self.create_publisher(RemoteStatus, "/remote_status", latched)
        self.pub_rune = self.create_publisher(RuneStatus, "/rune_status", latched)
        self.pub_decision = self.create_publisher(DecisionNum, "/decision_num", latched)
        # SubTargetPos 订阅的是 std_msgs/String（"x,y,z,w"），不是 Armors。
        # 节点源码（plugins/action/sub_target_pos.cpp）里写得明确；docs/节点链路.md 的
        # “auto_aim_interfaces/Armors” 与源码不一致，这里按源码契约发布。
        self.pub_target = self.create_publisher(String, "/auto_aim_target_pos", latched)
        self.pub_terrain = self.create_publisher(String, "/terrain_zone", latched)
        self.pub_gimbal = self.create_publisher(GimbalTarget, "/gimbal_target_point", latched)
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", latched)

        # ---- 输出记录 ----
        self.subs = []
        for topic, typ in [
            ("/robot_state", RobotState),
            ("/chassis_mode", ChassisMode),
            ("/robot_control", RobotControl),
            ("/auto_aim_mode", AutoAimMode),
            ("/super_capacitor", SuperCapacitor),
            ("/enemy_status", EnemyStatusMsg),
            ("/autoaim_target", AutoaimTargetMsg),
            ("/robot_status", RobotStatus),
        ]:
            self.subs.append(
                self.create_subscription(
                    typ,
                    topic,
                    lambda msg, t=topic: self.record(t, to_dict(msg)),
                    qos,
                    callback_group=cbg,
                )
            )

        # ---- 配置入口 ----
        self.create_subscription(String, "/bt_test/config", self.on_config, qos, callback_group=cbg)
        self.create_subscription(
            String, "/bt_test/mark", self.on_mark, qos, callback_group=cbg
        )

        # ---- TF 静态变换 ----
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_tf()

        # ---- 地图 ----
        self.publish_map()

        # ---- Nav2 action server ----
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_goal,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=cbg,
        )

        self.create_timer(0.1, self.tick, callback_group=cbg)
        self.create_timer(1.0, self.publish_map, callback_group=cbg)
        self.record("__meta__", {"event": "ready", "pid": os.getpid(), "wall_t0": self.wall_t0})

    # ------------------------------------------------------------------ 记录
    def record(self, topic, data):
        try:
            line = {"t": round(time.time() - self.t0, 6), "topic": topic, "data": data}
            text = json.dumps(line, ensure_ascii=False) + "\n"
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"record skipped {topic}: {exc}")
            return
        with self.record_lock:
            with open(self.record_path, "a", encoding="utf-8") as f:
                f.write(text)

    def on_mark(self, msg):
        self.record("__mark__", {"text": msg.data})

    # ------------------------------------------------------------------ 配置
    def on_config(self, msg):
        try:
            upd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"bad config: {e}")
            return
        if upd.get("reset"):
            self.cfg = json.loads(json.dumps(DEFAULTS))
        deep_merge(self.cfg, {k: v for k, v in upd.items() if k != "reset"})
        self.record("__config__", self.cfg)
        self.broadcast_tf()

    # ------------------------------------------------------------------ 发布
    def tick(self):
        now = time.time() - self.t0
        c = self.cfg

        g = GameStatus()
        g.game_progress = c["game_status"]["game_progress"]
        g.stage_remain_time = c["game_status"]["stage_remain_time"]
        self.pub_game.publish(g)

        r = RobotStatus()
        rs = c["robot_status"]
        for k in ("robot_id", "current_hp", "shooter_barrel_heat_limit", "shooter_heat",
                  "team_color", "is_attacked"):
            setattr(r, k, rs[k])
        if rs.get("hp_pattern") == "sawtooth" and now > 1.0:
            r.current_hp = 400 if int(now) % 2 == 0 else 300
        self.pub_robot.publish(r)

        hp = AllRobotHP()
        for k, v in c["robot_hp"].items():
            if hasattr(hp, k):
                setattr(hp, k, v)
        self.pub_hp.publish(hp)

        e = ChassisEnergy()
        e.chassis_energy = c["chassis_energy"]["chassis_energy"]
        self.pub_energy.publish(e)

        a = AutoAimStatus()
        a.auto_aim_status = c["auto_aim_status"]["auto_aim_status"]
        self.pub_aim_status.publish(a)

        rm = RemoteStatus()
        rm.remote_control_status = c["remote_status"]["remote_control_status"]
        self.pub_remote.publish(rm)

        ru = RuneStatus()
        ru.rune_count = c["rune_status"]["rune_count"]
        self.pub_rune.publish(ru)

        d = DecisionNum()
        d.decision_num = c["decision_num"]["decision_num"]
        self.pub_decision.publish(d)

        if int(c["armors"]["count"]) > 0:
            target = String()
            target.data = "{:.3f},{:.3f},{:.3f},1.0".format(
                float(c["armors"]["x"]), float(c["armors"]["y"]), float(c["armors"]["z"]))
        else:
            target = String()
            target.data = "0,0,0,1"  # SubTargetPos 约定 (0,0) = 敌人丢失
        self.pub_target.publish(target)

        s = String()
        s.data = c["terrain_zone"]["data"]
        self.pub_terrain.publish(s)

        gt = GimbalTarget()
        gt.x = float(c["gimbal_target"]["x"])
        gt.y = float(c["gimbal_target"]["y"])
        self.pub_gimbal.publish(gt)

    def broadcast_tf(self):
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for parent, frame, xyz in [
            ("map", "odom", self.cfg["tf"]["odom_xyz"]),
            ("map", "gimbal_yaw", self.cfg["tf"]["gimbal_xyz"]),
            ("odom", "base_link", [0.0, 0.0, 0.0]),
        ]:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = frame
            t.transform.translation.x = float(xyz[0])
            t.transform.translation.y = float(xyz[1])
            t.transform.translation.z = float(xyz[2])
            t.transform.rotation.w = 1.0
            transforms.append(t)
        # 必须一次性发整个列表：/tf_static 是 depth=1，分三次发只会留下最后一条
        self.tf_broadcaster.sendTransform(transforms)

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()
        res = 0.1
        n = 400  # 40 m x 40 m
        grid.info.resolution = res
        grid.info.width = n
        grid.info.height = n
        grid.info.origin.position.x = -20.0
        grid.info.origin.position.y = -20.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (n * n)
        self.pub_map.publish(grid)

    # ------------------------------------------------------------------ action
    def on_goal(self, goal_request):
        with self.goal_lock:
            self.goals.append(
                {
                    "t": round(time.time() - self.t0, 6),
                    "x": goal_request.pose.pose.position.x,
                    "y": goal_request.pose.pose.position.y,
                    "z": goal_request.pose.pose.position.z,
                    "frame": goal_request.pose.header.frame_id,
                }
            )
            self.record("__goal__", self.goals[-1])
        return GoalResponse.ACCEPT

    def execute_goal(self, goal_handle):
        """尽力发 feedback/结果；客户端会主动 cancel，任何 rclpy 异常都不能
        让 executor 线程退出（否则后续 case 全部收不到数据）。"""
        mode = self.cfg["nav"]["mode"]
        feedback = NavigateToPose.Feedback()
        try:
            if mode == "abort":
                time.sleep(0.3)
                if goal_handle.is_active:
                    goal_handle.abort()
                return NavigateToPose.Result()

            if mode == "hang":
                for _ in range(100):
                    if not goal_handle.is_active:
                        break
                    feedback.distance_remaining = 5.0
                    try:
                        goal_handle.publish_feedback(feedback)
                    except Exception:
                        break
                    time.sleep(0.1)
                if goal_handle.is_active:
                    goal_handle.canceled()
                return NavigateToPose.Result()

            # arrive：5 帧近距离反馈正好触发 SendGoal 的“连续 5 次 <0.2”判定；
            # 客户端随后会 cancel，之后的 publish/succeed 失败是正常现象。
            steps = 5
            for _ in range(steps):
                if not goal_handle.is_active:
                    break
                feedback.distance_remaining = self.cfg["nav"]["feedback_distance"]
                try:
                    goal_handle.publish_feedback(feedback)
                except Exception:
                    break
                time.sleep(self.cfg["nav"]["duration"] / steps)
            if goal_handle.is_active:
                goal_handle.succeed()
            else:
                # 被客户端 cancel 时必须显式置 CANCELED 并回 result，
                # 否则客户端 async_get_result 的 future 永远不完成（Nav2 会回）。
                try:
                    goal_handle.canceled()
                except Exception:
                    pass
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"execute_goal: {type(exc).__name__}: {exc}")
        return NavigateToPose.Result()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--record", default="results/record.jsonl")
    args = parser.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.record)), exist_ok=True)
    open(args.record, "w").close()

    rclpy.init()
    node = MockStack(os.path.abspath(args.record))
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    print(f"[mock_stack] ready, record={os.path.abspath(args.record)}", flush=True)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
