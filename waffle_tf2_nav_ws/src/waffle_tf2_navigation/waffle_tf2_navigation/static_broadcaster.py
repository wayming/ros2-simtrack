import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class LandmarkBroadcaster(Node):
    def __init__(self):
        super().__init__('static_landmark_broadcaster')
        self.broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'landmark'
        t.transform.translation.x = 2.0
        t.transform.translation.y = 0.5
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)
        self.get_logger().info('Published static transform world→landmark')

def main():
    rclpy.init()
    node = LandmarkBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
