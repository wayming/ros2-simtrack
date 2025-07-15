# src/laser_avoider.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
import random

class LaserAvoider(Node):
    def __init__(self):
        super().__init__('laser_avoider')
        
        # Declare parameters with default values
        self.declare_parameter('forward_speed', 0.2)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('obstacle_distance', 0.5)
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        if len(msg.ranges) < 20:
            self.get_logger().warn("Not enough range data received.")
            return
        front_ranges = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
        min_front = min(front_ranges)

        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        
        if min_front < self.obstacle_distance:
            self.get_logger().info("Obstacle detected! Turning...")
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = self.turn_speed * random.choice([-1, 1])  # Randomly turn left or right
        else:
            twist_stamped.twist.linear.x = self.forward_speed

        self.cmd_pub.publish(twist_stamped)

def main():
    rclpy.init()
    node = LaserAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
