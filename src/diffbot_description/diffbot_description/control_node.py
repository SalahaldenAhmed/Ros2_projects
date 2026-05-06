#!/usr/bin/env python3
"""
control_node.py
Reads /scan and /imu/data, publishes /cmd_vel.
State machine: FORWARD → TURNING → REVERSE → ROTATE_90 → SHOCKED
"""
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu

FORWARD_SPEED   = 0.20
TURN_SPEED      = 0.55
ROTATE_90_SPEED = 0.60
REVERSE_SPEED   = -0.15
WARN_DIST       = 1.00
STOP_DIST       = 0.55
FRONT_ARC       = 40
CTRL_HZ         = 10
REVERSE_CYCLES  = 10
SHOCK_THRESHOLD = 3.0
ROTATE_90_CYCLES = int(math.ceil((math.pi / 2) / ROTATE_90_SPEED * CTRL_HZ))


class ControlNode(Node):
    FORWARD = 'FORWARD'
    TURNING = 'TURNING'
    REVERSE = 'REVERSE'
    ROTATE_90 = 'ROTATE_90'
    SHOCKED = 'SHOCKED'

    def __init__(self):
        super().__init__('control_node')
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan',     self.scan_cb, 10)
        self.create_subscription(Imu,       '/imu/data', self.imu_cb,  10)
        self.state           = self.FORWARD
        self.front_min       = float('inf')
        self.imu_shock       = False
        self.imu_tilt        = 0.0
        self.turn_dir        = 1
        self.reverse_counter = 0
        self.rotate_counter  = 0
        self.shocked_counter = 0
        self.timer = self.create_timer(1.0 / CTRL_HZ, self.loop)
        self.get_logger().info('control_node ready.')

    def scan_cb(self, msg):
        n         = len(msg.ranges)
        front_idx = n // 2
        arc_steps = int((FRONT_ARC / 360.0) * n)
        indices   = list(range(front_idx - arc_steps, front_idx + arc_steps + 1))
        valid     = [msg.ranges[i] for i in indices
                     if 0 <= i < n and msg.range_min < msg.ranges[i] < msg.range_max]
        self.front_min = min(valid) if valid else float('inf')

    def imu_cb(self, msg):
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        lateral        = math.hypot(ax, ay)
        self.imu_shock = lateral > SHOCK_THRESHOLD
        self.imu_tilt  = math.degrees(math.atan2(lateral, abs(az)))

    def loop(self):
        d   = self.front_min
        cmd = Twist()

        if self.imu_shock and self.state == self.FORWARD:
            self.state = self.SHOCKED
            self.shocked_counter = 15
            self.get_logger().warn('IMU SHOCK — emergency stop!')

        if self.state == self.SHOCKED:
            self.shocked_counter -= 1
            if self.shocked_counter <= 0:
                self.state = self.REVERSE
                self.reverse_counter = REVERSE_CYCLES
            self.vel_pub.publish(Twist())
            return

        if d < STOP_DIST and self.state in (self.FORWARD, self.TURNING):
            self.vel_pub.publish(Twist())
            self.state = self.REVERSE
            self.reverse_counter = REVERSE_CYCLES
            self.turn_dir *= -1
            self.get_logger().warn(f'EMERGENCY: {d:.2f}m')
            return

        if self.state == self.FORWARD:
            if d < WARN_DIST: self.state = self.TURNING
        elif self.state == self.TURNING:
            if d >= WARN_DIST: self.state = self.FORWARD
        elif self.state == self.REVERSE:
            self.reverse_counter -= 1
            if self.reverse_counter <= 0:
                self.state = self.ROTATE_90
                self.rotate_counter = ROTATE_90_CYCLES
        elif self.state == self.ROTATE_90:
            self.rotate_counter -= 1
            if self.rotate_counter <= 0:
                self.state = self.FORWARD
                self.get_logger().info('90 degrees done — FORWARD')

        if self.state == self.FORWARD:
            cmd.linear.x = FORWARD_SPEED;       cmd.angular.z = 0.0
        elif self.state == self.TURNING:
            cmd.linear.x = 0.0;                 cmd.angular.z = TURN_SPEED * self.turn_dir
        elif self.state == self.REVERSE:
            cmd.linear.x = REVERSE_SPEED;       cmd.angular.z = 0.0
        elif self.state == self.ROTATE_90:
            cmd.linear.x = 0.0;                 cmd.angular.z = ROTATE_90_SPEED * self.turn_dir

        self.vel_pub.publish(cmd)
        self.get_logger().info(
            f'[{self.state:9}] front={d:.2f}m '
            f'lin={cmd.linear.x:+.2f} ang={cmd.angular.z:+.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
