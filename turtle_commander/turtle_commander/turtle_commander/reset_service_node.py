
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from turtle_commander.srv import ResetTurtle

class ResetServiceNode(Node):
    def __init__(self):
        super().__init__('reset_service_node')
        self.srv = self.create_service(ResetTurtle, 'reset_turtle', self.reset_callback)

    def reset_callback(self, request, response):
        try:
            # 调用clear服务
            clear_client = self.create_client(Empty, '/clear')
            while not clear_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('等待/clear服务...')
            clear_client.call_async(Empty.Request())

            # 调用teleport服务将乌龟归位
            teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
            while not teleport_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('等待/teleport_absolute服务...')
            req = TeleportAbsolute.Request()
            req.x = 5.544445
            req.y = 5.544445
            req.theta = 0.0
            teleport_client.call_async(req)

            self.get_logger().info("乌龟已重置")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"重置失败: {e}")
            response.success = False
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ResetServiceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
