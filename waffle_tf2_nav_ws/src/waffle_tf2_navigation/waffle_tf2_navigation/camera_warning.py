import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraWarningNode(Node):
    def __init__(self):
        super().__init__('camera_warning_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 图像处理：示例 - 边缘检测
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 100, 200)

            # 简单障碍物检测逻辑（示意）：如果图像中边缘太多，可能是障碍物
            edge_count = np.count_nonzero(edges)
            if edge_count > 10000:  # 这个阈值要你自己调试
                cv2.putText(cv_image, 'WARNING: Obstacle!',
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2)
                cv2.rectangle(cv_image, (20, 20), (620, 460), (0, 0, 255), 2)

            # 显示图像
            cv2.imshow("Camera View", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraWarningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
