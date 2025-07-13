import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from turtle_interfaces.action import Patrol
from geometry_msgs.msg import Vector3
from math import pi

class PatrolClient(Node):
    def __init__(self):
        super().__init__('patrol_client')
        self._action_client = ActionClient(self, Patrol, 'patrol')

    def send_goal(self):
        self.get_logger().info('Waiting for patrol action server...')
        self._action_client.wait_for_server()

        goal_msg = Patrol.Goal()
        goal_msg.goal = Vector3(x=1.0, y=0.0, z=0.0)  # 示例传目标

        waypoints = [
            Vector3(x=1.0, y=0.0, z=0.0),
            Vector3(x=0.0, y=1.0, z=pi/2),
            Vector3(x=-1.0, y=0.0, z=pi),
            Vector3(x=0.0, y=-1.0, z=-pi/2)
        ]
        
        for i, waypoint in enumerate(waypoints):
            goal_msg.goal = waypoint
            self.get_logger().info(f'Waypoint {i + 1}: {waypoint.x}, {waypoint.y}, {waypoint.z}')
            self.get_logger().info('Sending patrol goal...')
            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Patrol goal rejected')
            return

        self.get_logger().info('Patrol goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._goal_handle = goal_handle
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.state}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = PatrolClient()
    client.send_goal()
    rclpy.spin(client)

if __name__ == '__main__':
    main()
