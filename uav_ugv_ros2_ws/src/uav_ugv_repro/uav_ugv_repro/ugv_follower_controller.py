"""UGV controller that follows a local goal or the UAV ground projection."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .common import clamp, wrap_to_pi


class UgvFollowerController(Node):
    """Pure pursuit style follower for a simple differential-drive UGV."""

    def __init__(self) -> None:
        super().__init__('ugv_follower_controller')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('kv', 0.45)
        self.declare_parameter('kw', 1.4)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_yaw_rate', 1.2)
        self.declare_parameter('stop_distance', 0.25)

        self.kv = float(self.get_parameter('kv').value)
        self.kw = float(self.get_parameter('kw').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)

        self.odom: Odometry | None = None
        self.goal: PoseStamped | None = None
        self.leader_pose: PoseStamped | None = None

        self.cmd_pub = self.create_publisher(Twist, '/ugv/cmd_vel', 10)
        self.create_subscription(Odometry, '/ugv/odom', self._odom_cb, 10)
        self.create_subscription(PoseStamped, '/ugv/local_goal', self._goal_cb, 10)
        self.create_subscription(PoseStamped, '/leader/pose_enu', self._leader_cb, 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('UGV follower controller ready')

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _goal_cb(self, msg: PoseStamped) -> None:
        self.goal = msg

    def _leader_cb(self, msg: PoseStamped) -> None:
        self.leader_pose = msg

    def _tick(self) -> None:
        cmd = Twist()
        if self.odom is None:
            self.cmd_pub.publish(cmd)
            return

        target = self.goal or self.leader_pose
        if target is None:
            self.cmd_pub.publish(cmd)
            return

        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        dx = target.pose.position.x - x
        dy = target.pose.position.y - y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = wrap_to_pi(target_yaw - yaw)

        if distance > self.stop_distance:
            cmd.linear.x = clamp(self.kv * distance, 0.0, self.max_speed)
            cmd.angular.z = clamp(self.kw * yaw_error, -self.max_yaw_rate, self.max_yaw_rate)
        self.cmd_pub.publish(cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvFollowerController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
