import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from robot_msgs.msg import GameStatus
rclpy.init(); n=Node('pubgame')
q=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
p=n.create_publisher(GameStatus,'/jobtest',q)
while rclpy.ok():
    m=GameStatus(); m.game_progress=4; m.stage_remain_time=300; p.publish(m); print('pub',flush=True); time.sleep(0.2)
