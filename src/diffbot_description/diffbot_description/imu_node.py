#!/usr/bin/env python3
"""
imu_node.py
Monitors /imu/data and publishes events to /imu/events.
Detects: gravity fault, tilt, shock, rotation.
"""
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.event_pub = self.create_publisher(String, '/imu/events', 10)
        self.get_logger().info('imu_node ready.')

    def imu_cb(self, msg):
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x,    msg.angular_velocity.y,    msg.angular_velocity.z
        gravity  = math.sqrt(ax**2 + ay**2 + az**2)
        tilt     = math.degrees(math.atan2(math.hypot(ax, ay), abs(az)))
        rot_rate = math.degrees(math.sqrt(gx**2 + gy**2 + gz**2))
        lateral  = math.hypot(ax, ay)
        events = []
        if not (9.0 < gravity < 10.5): events.append(f'GRAVITY_FAULT({gravity:.2f})')
        if tilt > 15.0:                events.append(f'TILT({tilt:.1f}deg)')
        if lateral > 3.0:              events.append(f'SHOCK(lateral={lateral:.2f})')
        if rot_rate > 60.0:            events.append(f'ROTATING({rot_rate:.1f}deg/s)')
        if events:
            out = String()
            out.data = ' | '.join(events)
            self.event_pub.publish(out)
            self.get_logger().warn(f'IMU EVENT: {out.data}')
        else:
            self.get_logger().info(
                f'IMU OK gravity={gravity:.2f} tilt={tilt:.1f}deg rot={rot_rate:.1f}deg/s',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
