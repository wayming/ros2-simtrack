# src/laser_avoider.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random

class LaserAvoider(Node):
    def __init__(self):
        super().__init__('laser_avoider')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        front_ranges = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
        min_front = min(front_ranges)

        twist = Twist()

        if min_front < 0.5:
            self.get_logger().info("Obstacle detected! Turning...")
            twist.angular.z = random.choice([-1.0, 1.0])
        else:
            twist.linear.x = 0.2

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = LaserAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
