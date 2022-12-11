import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SelfDrive(Node):
    def __init__(self):
        super().__init__('self_drive')
        lidar_qos_profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                       history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                       depth=1)
        vel_qos_profile = QoSProfile(depth=10)
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.subscribe_scan,
            lidar_qos_profile)
        self.pub_velo = self.create_publisher(Twist, '/cmd_vel', vel_qos_profile)
        self.step = 0

    def subscribe_scan(self, scan):
        twist = Twist()

        twist.linear.x = 0.15
        twist.angular.z = 0.
        self.get_logger().info(f"scan: {scan.ranges[0]}, forward")

        if 0.01 < scan.ranges[10] <= 0.25 or 0.01 < scan.ranges[50] <= 0.25:
            if 0.01 < scan.ranges[10] <= 0.23 or 0.01 < scan.ranges[30] <= 0.23:
                twist.linear.x = 0.06
                twist.angular.z = -1.2
                self.get_logger().info(f"scan: {scan.ranges[30]}, right")
            elif 0.01 < scan.ranges[30] <= 0.21 or 0.01 < scan.ranges[50] <= 0.21:
                twist.linear.x = 0.08
                twist.angular.z = -1.0
                self.get_logger().info(f"scan: {scan.ranges[30]}, right")
            else:
                twist.linear.x = 0.03
                twist.angular.z = -1.5
       

def main(args=None):
    rclpy.init(args=args)
    node = SelfDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
