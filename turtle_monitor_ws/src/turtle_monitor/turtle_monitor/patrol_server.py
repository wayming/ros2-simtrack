import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Vector3
from turtle_interfaces.action import Patrol
from math import pi
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class PatrolServer(Node):
    def __init__(self):
        super().__init__('patrol_server')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._action_server = ActionServer(self, Patrol, 'patrol', self.execute_callback)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', qos_profile=qos_profile)
        self.get_logger().info('Patrol Server Initialized')

    def execute_callback(self, goal_handle):
        feedback_msg = Patrol.Feedback()
        result = Patrol.Result()

        if goal_handle.is_cancel_requested:
            result.result = "Patrol canceled"
            goal_handle.canceled()
            return result

        goal_msg = goal_handle.request.goal
        feedback_msg.state = f'Arrived at waypoint {goal_msg}'
        
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f'move_to ({goal_msg.x}, {goal_msg.y}, {goal_msg.z})')
        self.move_to(goal_msg.x, goal_msg.y, goal_msg.z)
        self.get_logger().info(feedback_msg.state)

        result.result = "Patrol completed successfully"
        goal_handle.succeed()
        return result

    def move_to(self, x, y, theta, duration=2.0):
        twist_stamped = TwistStamped()
        twist_stamped.twist.linear.x = x
        twist_stamped.twist.linear.y = y
        twist_stamped.twist.angular.z = theta

        start_time = time.time()
        while time.time() - start_time < duration:
            # 更新时间戳
            twist_stamped.header.stamp = self.get_clock().now().to_msg()
            twist_stamped.header.frame_id = "base_footprint"
            self.cmd_vel_pub.publish(twist_stamped)
            time.sleep(0.1)

        # 停止移动
        stop_msg = TwistStamped()
        stop_msg.header.stamp = self.get_clock().now().to_msg()
        stop_msg.header.frame_id = "base_footprint"
        self.cmd_vel_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
