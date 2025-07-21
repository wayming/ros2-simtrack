import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class CameraTfListener(Node):
    def __init__(self):
        super().__init__('camera_tf_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(1.0, self.print_tf)

    def print_tf(self):
        try:
            t = self.buffer.lookup_transform('camera_link', 'base_link', rclpy.time.Time())
            self.get_logger().info(f"TF camera_link→base_link: "
                                   f"x={t.transform.translation.x:.2f}, "
                                   f"y={t.transform.translation.y:.2f}, "
                                   f"z={t.transform.translation.z:.2f}")
        except Exception as e:
            self.get_logger().warning(f'Could not get transform: {e}')

def main():
    rclpy.init()
    node = CameraTfListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
