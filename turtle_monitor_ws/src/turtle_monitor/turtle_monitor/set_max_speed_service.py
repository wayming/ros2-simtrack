import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Parameter
from geometry_msgs.msg import Twist

class MaxSpeedService(Node):
    def __init__(self):
        super().__init__('max_speed_service')
        self.declare_parameter('max_speed', 0.2)
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.set_max_speed_srv = self.create_service(
            SetBool,
            '/set_max_speed',
            self.set_max_speed_callback)

    def set_max_speed_callback(self, request, response):
        if request.data:
            self.max_speed = 0.5
            self.get_logger().info('Max speed set to 0.5 m/s')
        else:
            self.max_speed = 0.2
            self.get_logger().info('Max speed set to 0.2 m/s')
        response.success = True
        response.message = f'Max speed set to {self.max_speed} m/s'
        return response

def main(args=None):
    rclpy.init(args=args)
    max_speed_service = MaxSpeedService()
    rclpy.spin(max_speed_service)
    max_speed_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

