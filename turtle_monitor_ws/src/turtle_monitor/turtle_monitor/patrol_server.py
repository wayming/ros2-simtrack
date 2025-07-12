import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from turtle_interfaces.action import Patrol
from math import pi
import time

class PatrolServer(Node):
    def __init__(self):
        super().__init__('patrol_server')
        self._action_server = ActionServer(self, Patrol, 'patrol', self.execute_callback)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Patrol Server Initialized')

    def execute_callback(self, goal_handle):
        feedback_msg = Patrol.Feedback()
        result = Patrol.Result()

        waypoints = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, pi/2),
            (-1.0, 0.0, pi),
            (0.0, -1.0, -pi/2)
        ]

        for i, (x, y, theta) in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                result.result = "Patrol canceled"
                goal_handle.canceled()
                return result

            feedback_msg.state = f'Arrived at waypoint {i + 1}'
            goal_handle.publish_feedback(feedback_msg)
            self.move_to(x, y, theta)
            self.get_logger().info(feedback_msg.state)

        result.result = "Patrol completed successfully"
        goal_handle.succeed()
        return result

    def move_to(self, x, y, theta, duration=2.0):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = theta

        start_time = time.time()
        rate = self.create_rate(10, self.get_clock())  # 10 Hz

        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # 停止移动
        self.cmd_vel_pub.publish(Twist())  # all zeros

def main(args=None):
    rclpy.init(args=args)
    node = PatrolServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
