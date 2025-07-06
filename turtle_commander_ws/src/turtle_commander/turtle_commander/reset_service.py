import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute

class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.srv = self.create_service(Empty, 'reset_turtle', self.reset_callback)
        self.cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for teleport_absolute service...')

    def reset_callback(self, request, response):
        self.get_logger().info('Resetting turtle to origin...')
        req = TeleportAbsolute.Request()
        req.x = 5.544445
        req.y = 5.544445
        req.theta = 0.0
        self.cli.call_async(req)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
