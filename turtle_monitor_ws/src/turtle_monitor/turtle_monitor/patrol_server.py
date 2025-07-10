
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlebot3_example.action import Patrol
from math import pi

class PatrolServer(Node):
    def __init__(self):
        super().__init__('patrol_server')
        self._action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            self.execute_callback)
        self.get_logger().info('Patrol Server Initialized')

    def execute_callback(self, goal_handle):
        feedback_msg = Patrol.Feedback()
        result = Patrol.Result()

        # Define patrol path (triangle)
        waypoints = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, pi / 2),
            (-1.0, 0.0, pi),
            (0.0, -1.0, -pi / 2)
        ]

        for i, (x, y, theta) in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                result.success = False
                return result
            feedback_msg.current_waypoint = i + 1
            goal_handle.publish_feedback(feedback_msg)
            self.move_to(x, y, theta)

        result.success = True
        return result

    def move_to(self, x, y, theta):
        cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        move_cmd = Twist()
        move_cmd.linear.x = x
        move_cmd.linear.y = y
        move_cmd.angular.z = theta
        cmd_vel_pub.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    patrol_server = PatrolServer()
    rclpy.spin(patrol_server)
    patrol_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

