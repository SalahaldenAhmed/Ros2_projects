#!/usr/bin/env python3
"""
scan_restamper.py
Reads /scan, fixes the empty timestamp, republishes on /scan_stamped.
SLAM Toolbox listens to /scan_stamped instead of /scan.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanRestamper(Node):
    def __init__(self):
        super().__init__('scan_restamper')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_stamped', 10)
        self.get_logger().info('scan_restamper ready — /scan → /scan_stamped')

    def cb(self, msg: LaserScan):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
