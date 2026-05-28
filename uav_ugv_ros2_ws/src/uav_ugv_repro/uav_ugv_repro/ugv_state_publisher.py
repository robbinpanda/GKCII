"""Lightweight UGV kinematic simulator for early integration tests."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .common import yaw_to_quaternion


class UgvStatePublisher(Node):
    """Integrate /ugv/cmd_vel into /ugv/odom."""

    def __init__(self) -> None:
        super().__init__('ugv_state_publisher')
        self.declare_parameter('update_rate_hz', 30.0)
        self.declare_parameter('initial_x', -0.4)
        self.declare_parameter('initial_y', -0.4)
        self.declare_parameter('initial_yaw', 0.0)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.cmd = Twist()
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/ugv/odom', 10)
        self.create_subscription(Twist, '/ugv/cmd_vel', self._cmd_cb, 10)
        rate = float(self.get_parameter('update_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('UGV state publisher ready')

    def _cmd_cb(self, msg: Twist) -> None:
        self.cmd = msg

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        dt = max(0.0, min(dt, 0.2))

        v = float(self.cmd.linear.x)
        wz = float(self.cmd.angular.z)
        self.yaw += wz * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'ugv/base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist = self.cmd
        self.odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
