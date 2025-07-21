import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TfDebugger(Node):
    def __init__(self):
        super().__init__('tf_debugger')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(1.0, self.show_all)

    def show_all(self):
        frames = self.buffer.all_frames_as_yaml()
        self.get_logger().info(f"All TF frames:\n{frames}")

def main():
    rclpy.init()
    node = TfDebugger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
