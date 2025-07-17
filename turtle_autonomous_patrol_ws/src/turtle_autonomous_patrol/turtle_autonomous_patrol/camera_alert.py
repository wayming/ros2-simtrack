# src/camera_alert.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraAlert(Node):
    def __init__(self):
        super().__init__('image_alert')

        # 声明参数并读取
        self.declare_parameter('warning_distance', 0.7)
        self.warning_distance = self.get_parameter('warning_distance').value

        # 初始化工具
        self.bridge = CvBridge()
        self.closest = float('inf')

        # 订阅图像和激光雷达话题
        self.image_sub = self.create_subscription(Image, '/camera/rgb/image_raw', self.image_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # 创建 OpenCV 窗口
        cv2.namedWindow('camera_view', cv2.WINDOW_NORMAL)
        self.get_logger().info("Image alert node started and subscribing to /camera/rgb/image_raw")

    def scan_cb(self, msg: LaserScan):
        # 获取最近距离
        valid_ranges = [r for r in msg.ranges if 0.01 < r < msg.range_max]
        if valid_ranges:
            self.closest = min(valid_ranges)

    def image_cb(self, msg: Image):
        self.get_logger().info(f"Received image with size: {msg.width}x{msg.height}")
        try:
            # 转换 ROS 图像为 OpenCV 格式
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f"Converted image shape: {cv_img.shape}")
        except Exception as e:
            self.get_logger().error(f"CV bridge error: {e}")
            return

        h, w = cv_img.shape[:2]

        # 如果有障碍物太近，画红框
        if self.closest < self.warning_distance:
            cv2.rectangle(cv_img, (0, 0), (w, h), (0, 0, 255), 8)
            cv2.putText(cv_img, "WARNING: Obstacle Ahead!", (50, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        cv2.imshow('camera_view', cv_img)
        cv2.waitKey(1)
        cv2.imwrite("/ros2_ws/turtle_autonomous_patrol_ws/debug_image.png", cv_img)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()
        
def main():
    rclpy.init()
    node = CameraAlert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
