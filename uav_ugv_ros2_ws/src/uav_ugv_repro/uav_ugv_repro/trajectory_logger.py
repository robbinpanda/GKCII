"""CSV trajectory logger for the reproduction experiment."""

from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Int32


class TrajectoryLogger(Node):
    """Record UAV/UGV state and obstacle flags to a CSV file."""

    def __init__(self) -> None:
        super().__init__('trajectory_logger')
        self.declare_parameter('output_path', '/home/robbinpanda/GKCII/uav_ugv_ros2_ws/results/run_001.csv')
        self.declare_parameter('sample_rate_hz', 10.0)

        output_path = Path(str(self.get_parameter('output_path').value)).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.file = output_path.open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time', 'uav_x', 'uav_y', 'uav_z', 'ugv_x', 'ugv_y', 'ugv_yaw',
            'waypoint_id', 'obstacle_left', 'obstacle_right',
        ])

        self.leader_pose: PoseStamped | None = None
        self.ugv_odom: Odometry | None = None
        self.waypoint_id = -1
        self.left = False
        self.right = False

        self.create_subscription(PoseStamped, '/leader/pose_enu', self._leader_cb, 10)
        self.create_subscription(Odometry, '/ugv/odom', self._ugv_cb, 10)
        self.create_subscription(Int32, '/mission/waypoint_id', self._wp_cb, 10)
        self.create_subscription(Bool, '/ugv/front_left_obstacle', self._left_cb, 10)
        self.create_subscription(Bool, '/ugv/front_right_obstacle', self._right_cb, 10)

        rate = float(self.get_parameter('sample_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'Logging trajectory to {output_path}')

    def _leader_cb(self, msg: PoseStamped) -> None:
        self.leader_pose = msg

    def _ugv_cb(self, msg: Odometry) -> None:
        self.ugv_odom = msg

    def _wp_cb(self, msg: Int32) -> None:
        self.waypoint_id = msg.data

    def _left_cb(self, msg: Bool) -> None:
        self.left = msg.data

    def _right_cb(self, msg: Bool) -> None:
        self.right = msg.data

    def _tick(self) -> None:
        if self.leader_pose is None or self.ugv_odom is None:
            return
        q = self.ugv_odom.pose.pose.orientation
        ugv_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        now = self.get_clock().now().nanoseconds / 1e9
        self.writer.writerow([
            f'{now:.3f}',
            f'{self.leader_pose.pose.position.x:.4f}',
            f'{self.leader_pose.pose.position.y:.4f}',
            f'{self.leader_pose.pose.position.z:.4f}',
            f'{self.ugv_odom.pose.pose.position.x:.4f}',
            f'{self.ugv_odom.pose.pose.position.y:.4f}',
            f'{ugv_yaw:.4f}',
            self.waypoint_id,
            int(self.left),
            int(self.right),
        ])
        self.file.flush()

    def destroy_node(self) -> bool:
        if not self.file.closed:
            self.file.flush()
            self.file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
