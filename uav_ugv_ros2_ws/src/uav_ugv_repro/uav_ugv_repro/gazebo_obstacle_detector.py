"""Truth-based obstacle detector used before adding Gazebo ray sensors."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from .common import parse_triplets


class GazeboObstacleDetector(Node):
    """Publish left/right obstacle booleans from configured obstacle positions."""

    def __init__(self) -> None:
        super().__init__('gazebo_obstacle_detector')
        self.declare_parameter('obstacles', ['0.75,0.25,0.20', '0.85,0.95,0.20'])
        self.declare_parameter('detection_range', 0.45)
        self.declare_parameter('lateral_width', 0.28)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.obstacles = parse_triplets(self.get_parameter('obstacles').value, [(0.75, 0.25, 0.2)])
        self.detection_range = float(self.get_parameter('detection_range').value)
        self.lateral_width = float(self.get_parameter('lateral_width').value)
        self.odom: Odometry | None = None

        self.left_pub = self.create_publisher(Bool, '/ugv/front_left_obstacle', 10)
        self.right_pub = self.create_publisher(Bool, '/ugv/front_right_obstacle', 10)
        self.create_subscription(Odometry, '/ugv/odom', self._odom_cb, 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'Obstacle detector loaded {len(self.obstacles)} obstacles')

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _tick(self) -> None:
        left = False
        right = False
        if self.odom is not None:
            x = self.odom.pose.pose.position.x
            y = self.odom.pose.pose.position.y
            q = self.odom.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            for ox, oy, radius in self.obstacles:
                dx = ox - x
                dy = oy - y
                forward = cos_y * dx + sin_y * dy
                lateral = -sin_y * dx + cos_y * dy
                if -radius <= forward <= self.detection_range + radius and abs(lateral) <= self.lateral_width + radius:
                    if lateral >= 0.0:
                        left = True
                    else:
                        right = True
        left_msg = Bool()
        left_msg.data = left
        right_msg = Bool()
        right_msg.data = right
        self.left_pub.publish(left_msg)
        self.right_pub.publish(right_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboObstacleDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
