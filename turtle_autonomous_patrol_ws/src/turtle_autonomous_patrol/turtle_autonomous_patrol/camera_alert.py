# src/camera_alert.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraAlert(Node):
    def __init__(self):
        super().__init__('camera_alert')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)

    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 模拟障碍物颜色识别（比如检测红色区域）
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        if np.sum(mask) > 5000:
            cv2.putText(img, 'Warning: Obstacle!', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.rectangle(img, (20, 20), (300, 200), (0, 0, 255), 3)

        cv2.imshow("Camera View", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraAlert()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
