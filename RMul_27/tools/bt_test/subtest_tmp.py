import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from robot_msgs.msg import GameStatus

rclpy.init()
n = Node('subjob2')
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
               durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(GameStatus, '/jobtest', lambda m: print('GOT', m.game_progress, flush=True), q)
rclpy.spin(n)
