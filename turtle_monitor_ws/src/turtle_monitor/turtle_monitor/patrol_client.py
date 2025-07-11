import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from turtlebot3_msgs.action import Patrol


class PatrolClient(Node):
    def __init__(self):
        super().__init__('patrol_client')
        self._action_client = ActionClient(self, Patrol, 'patrol')

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = Patrol.Goal()
        # 如果目标有字段可设置，可在此设置
        self.get_logger().info('Sending patrol goal...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

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
        self.get_logger().info(f'Received feedback: {feedback.state}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Patrol result: {result.result}')
        rclpy.shutdown()

    def cancel_goal(self):
        if hasattr(self, '_goal_handle'):
            self.get_logger().info('Canceling patrol...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Patrol goal canceled')
        else:
            self.get_logger().warn('Failed to cancel patrol goal')


def main(args=None):
    rclpy.init(args=args)
    patrol_client = PatrolClient()

    try:
        patrol_client.send_goal()
        rclpy.spin(patrol_client)
    except KeyboardInterrupt:
        patrol_client.get_logger().info('KeyboardInterrupt detected')
        patrol_client.cancel_goal()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
