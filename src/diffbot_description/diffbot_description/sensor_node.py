#!/usr/bin/env python3
"""
sensor_node.py
Live sensor dashboard — prints summary every 0.5s.
Reads: /scan, /imu/data, /camera/image_raw, /odom
"""
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.scan_sectors = {'front': float('inf'), 'back': float('inf'),
                             'left':  float('inf'), 'right': float('inf')}
        self.imu_ax = self.imu_ay = self.imu_az = 0.0
        self.imu_gz = 0.0
        self.imu_tilt = 0.0
        self.cam_w = self.cam_h = self.cam_frames = 0
        self.odom_x = self.odom_y = self.odom_yaw = self.odom_speed = 0.0
        self.total_dist = 0.0
        self._lx = self._ly = None

        self.create_subscription(LaserScan, '/scan',             self.scan_cb, 10)
        self.create_subscription(Imu,       '/imu/data',         self.imu_cb,  10)
        self.create_subscription(Image,     '/camera/image_raw', self.cam_cb,  10)
        self.create_subscription(Odometry,  '/odom',             self.odom_cb, 10)
        self.create_timer(0.5, self.dashboard)
        self.get_logger().info('sensor_node ready.')

    def scan_cb(self, msg):
        n = len(msg.ranges)
        def sec(cf, w=40):
            c = int(cf * n)
            s = int((w / 360.0) * n)
            idx = [((c + i) % n) for i in range(-s, s+1)]
            v = [msg.ranges[i] for i in idx if msg.range_min < msg.ranges[i] < msg.range_max]
            return min(v) if v else float('inf')
        self.scan_sectors['front'] = sec(0.5)
        self.scan_sectors['back']  = sec(0.0)
        self.scan_sectors['left']  = sec(0.25)
        self.scan_sectors['right'] = sec(0.75)

    def imu_cb(self, msg):
        a = msg.linear_acceleration
        self.imu_ax, self.imu_ay, self.imu_az = a.x, a.y, a.z
        self.imu_gz   = msg.angular_velocity.z
        self.imu_tilt = math.degrees(math.atan2(math.hypot(a.x, a.y), abs(a.z)))

    def cam_cb(self, msg):
        self.cam_w, self.cam_h = msg.width, msg.height
        self.cam_frames += 1

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        if self._lx is not None:
            self.total_dist += math.hypot(p.x - self._lx, p.y - self._ly)
        self._lx, self._ly = p.x, p.y
        self.odom_x, self.odom_y = p.x, p.y
        self.odom_yaw   = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y**2+q.z**2)))
        self.odom_speed = math.hypot(v.x, v.y)

    def fmt(self, v):
        return f'{v:.2f}m' if v < 9.99 else '  inf'

    def dashboard(self):
        S = self.scan_sectors
        cam = f'{self.cam_w}x{self.cam_h} frame#{self.cam_frames}' if self.cam_w else 'waiting...'
        self.get_logger().info(
            f'\n'
            f'  ╔═══════════════════════════════════════════════╗\n'
            f'  ║ LIDAR  F={self.fmt(S["front"])} B={self.fmt(S["back"])}'
            f' L={self.fmt(S["left"])} R={self.fmt(S["right"])}  ║\n'
            f'  ║ IMU    tilt={self.imu_tilt:.1f}deg  gz={self.imu_gz:+.2f}rad/s'
            f'  az={self.imu_az:.2f}      ║\n'
            f'  ║ CAM    {cam:<37}║\n'
            f'  ║ ODOM   x={self.odom_x:.2f}m y={self.odom_y:.2f}m'
            f' yaw={self.odom_yaw:.1f}deg dist={self.total_dist:.2f}m ║\n'
            f'  ╚═══════════════════════════════════════════════╝'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
