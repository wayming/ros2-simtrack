import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.alert_pub = self.create_publisher(String, '/alert', 10)
        self.get_logger().info('Monitor Node Initialized')

    def battery_callback(self, msg):
        if msg.percentage < 0.2:
            alert_msg = String()
            alert_msg.data = 'Battery low: {:.2f}%'.format(msg.percentage * 100)
            self.alert_pub.publish(alert_msg)

    def odom_callback(self, msg):
        if abs(msg.pose.pose.position.x) > 10 or abs(msg.pose.pose.position.y) > 10:
            alert_msg = String()
            alert_msg.data = 'Odometry out of bounds'
            self.alert_pub.publish(alert_msg)

def main(args=None):
    rclpy.init(args=args)
    monitor_node = MonitorNode()
    rclpy.spin(monitor_node)
    monitor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

