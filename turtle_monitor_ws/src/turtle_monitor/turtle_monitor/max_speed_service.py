import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from turtle_interfaces.srv import SetMaxSpeed

class MaxSpeedService(Node):
    def __init__(self):
        super().__init__('max_speed_service')
        self.declare_parameter('max_speed', 0.2)
        self.max_speed = self.get_parameter('max_speed').value

        self.srv = self.create_service(SetMaxSpeed, 'set_max_speed', self.set_max_speed_callback)
        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info(f"Initial max speed: {self.max_speed}")

    def set_max_speed_callback(self, request, response):
        if request.max_speed <= 0.0:
            response.success = False
            response.message = "Speed must be positive"
            return response

        self.max_speed = request.max_speed
        self.set_parameters([Parameter('max_speed', Parameter.Type.DOUBLE, self.max_speed)])
        response.success = True
        response.message = f"Max speed set to {self.max_speed}"
        self.get_logger().info(response.message)
        return response

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_speed' and param.type_ == Parameter.Type.DOUBLE:
                self.max_speed = param.value
                self.get_logger().info(f"Max speed parameter updated to: {self.max_speed}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = MaxSpeedService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
