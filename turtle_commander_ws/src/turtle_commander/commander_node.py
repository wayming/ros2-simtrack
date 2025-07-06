
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class CommanderNode(Node):
    def __init__(self):
        super().__init__('commander_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.draw_square)
        self.phase = 0
        self.last_time = self.get_clock().now()
        self.side_duration = 2.0  # seconds
        self.turn_duration = 1.5  # seconds

    def draw_square(self):
        msg = Twist()
        now = self.get_clock().now()
        elapsed = (now - self.last_time).nanoseconds / 1e9

        if self.phase % 2 == 0:
            # 直行
            if elapsed < self.side_duration:
                msg.linear.x = 2.0
            else:
                self.phase += 1
                self.last_time = now
        else:
            # 转弯
            if elapsed < self.turn_duration:
                msg.angular.z = math.pi / 2
            else:
                self.phase += 1
                self.last_time = now

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CommanderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
