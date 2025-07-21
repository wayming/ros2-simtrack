# src/laser_avoider.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion
from rclpy.duration import Duration
from enum import Enum
from math import pi
import random

class State(Enum):
    FORWARD = 1
    TURNING = 2
    BLOCKING = 3

EPSILON = 1e-3

# Define the LaserAvoider class that will handle laser scan data and control the robot
class LaserAvoider(Node):
    def __init__(self):
        super().__init__('laser_avoider')
        
        # Declare parameters with default values
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('obstacle_distance', 0.5)
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_angular_velocity_z = 0.0
        
        self.turn_start_time = self.get_clock().now()
        self.turn_start_yaw = 0.0
        self.state = State.FORWARD
        
        
    def scan_callback(self, msg):
        if len(msg.ranges) < 20:
            self.get_logger().warn("Not enough range data received.")
            return
        
        # Step 1: 计算正前方索引
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        front_angle = 0.0  # 正前方
        front_index = int((front_angle - angle_min) / angle_increment)

        # Step 2: 从前方 ±15° 范围内采样
        angle_width = 0.52  # ≈ 30° = 0.52 rad
        num_points = int(angle_width / angle_increment)
        start_index = max(front_index - num_points // 2, 0)
        end_index = min(front_index + num_points // 2, len(msg.ranges) - 1)
        
        front_ranges = msg.ranges[start_index:end_index]
        front_ranges = [r for r in front_ranges if 0.01 < r < msg.range_max]
        
        if not front_ranges:
            self.get_logger().warn("No valid laser readings in front sector.")
            return

        min_front = min(front_ranges)
        
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()

        if self.state == State.TURNING and abs(self.current_yaw - self.turn_start_yaw) < pi / 4:
            self.get_logger().info("Already turning, maintaining turn speed.")
            return

        self.get_logger().info(f"Minimum front distance: {min_front:.2f} m")
        if min_front < self.obstacle_distance:
            self.get_logger().info("Obstacle detected! Start to turn...")
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = self.turn_speed * random.choice([-1, 1])  # Randomly turn left or right
            self.turn_start_time = self.get_clock().now()
            self.state = State.TURNING
            self.turn_start_yaw = self.current_yaw
        else:
            if abs(self.current_angular_velocity_z) > EPSILON:
                self.get_logger().info("Defer moving forward.")
                twist_stamped.twist.linear.x = 0.0
                twist_stamped.twist.angular.z = 0.0
                self.state = State.FORWARD # set the state but waiting angular_velocity_z to 0
            else:
                twist_stamped.twist.linear.x = self.forward_speed
                twist_stamped.twist.angular.z = 0.0
                self.state = State.FORWARD

        self.cmd_pub.publish(twist_stamped)


    def odom_callback(self, msg):
        # Update the current angular velocity from odometry
        self.current_angular_velocity_z = msg.twist.twist.angular.z
        self.get_logger().debug(f"Current angular velocity (z): {self.current_angular_velocity_z:.3f} rad/s")
        
        orientation = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.get_logger().debug(f"Current orientation (roll, pitch, yaw): ({roll:.3f}, {pitch:.3f}, {yaw:.3f}) rad")
        self.current_yaw = yaw
        
def main():
    rclpy.init()
    node = LaserAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()