import rclpy
from rclpy.node import Node

from turtle_interfaces.srv import ResetTurtle         # 自定义服务
from turtlesim.srv import TeleportAbsolute            # 系统内置服务

class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')

        # 创建服务服务器
        self.srv = self.create_service(ResetTurtle, '/reset_turtle', self.reset_callback)
        self.get_logger().info('Service /reset_turtle is ready.')

        # 创建客户端用于调用 turtlesim 的 teleport_absolute 服务
        self.cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /turtle1/teleport_absolute service...')

    def reset_callback(self, request, response):
        self.get_logger().info('Resetting turtle to origin...')

        req = TeleportAbsolute.Request()
        req.x = 5.544445
        req.y = 5.544445
        req.theta = 0.0

        future = self.cli.call_async(req)
        self.get_logger().info('Waiting for teleport service response...')

        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        # 健壮处理
        if future.done():
            if future.result() is not None:
                self.get_logger().info('Teleport result received.')
                response.success = True
            else:
                self.get_logger().warn('Teleport service returned None.')
                response.success = False
        else:
            self.get_logger().error('Teleport service call timed out.')
            response.success = False

        self.get_logger().info(f'Sending response: success={response.success}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
