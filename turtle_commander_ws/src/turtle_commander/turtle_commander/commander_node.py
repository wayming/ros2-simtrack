import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class Commander(Node):
    def __init__(self):
        super().__init__('commander')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_square)
        self.step = 0
        self.start_time = time.time()

    def move_square(self):
        msg = Twist()
        duration = 2.0  # seconds per edge
        rotate_time = 1.0  # seconds to rotate 90 degrees

        elapsed = time.time() - self.start_time
        if self.step % 2 == 0:
            msg.linear.x = 2.0
            msg.angular.z = 0.0
            if elapsed >= duration:
                self.step += 1
                self.start_time = time.time()
        else:
            msg.linear.x = 0.0
            msg.angular.z = 1.57  # roughly 90 degrees/sec
            if elapsed >= rotate_time:
                self.step += 1
                self.start_time = time.time()

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Commander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
