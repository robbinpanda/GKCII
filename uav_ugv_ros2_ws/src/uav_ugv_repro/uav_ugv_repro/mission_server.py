"""Mission server that advances UAV waypoints when the UGV catches up."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, String

from .common import parse_triplets


class MissionServer(Node):
    """Publish the current leader waypoint and mission state."""

    def __init__(self) -> None:
        super().__init__('mission_server')
        self.declare_parameter('waypoints', [
            '0.0,0.0,1.5',
            '1.5,0.0,1.5',
            '1.5,1.2,1.5',
            '0.0,1.2,1.5',
            '0.0,0.0,1.5',
        ])
        self.declare_parameter('switch_distance', 0.35)
        self.declare_parameter('publish_rate_hz', 5.0)

        self.waypoints = parse_triplets(
            self.get_parameter('waypoints').value,
            [(0.0, 0.0, 1.5), (1.5, 0.0, 1.5)],
        )
        self.switch_distance = float(self.get_parameter('switch_distance').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)

        self.leader_pose: PoseStamped | None = None
        self.ugv_odom: Odometry | None = None
        self.index = 0
        self.finished = False

        self.wp_pub = self.create_publisher(Float32MultiArray, '/mission/current_waypoint', 10)
        self.state_pub = self.create_publisher(String, '/mission/state', 10)
        self.id_pub = self.create_publisher(Int32, '/mission/waypoint_id', 10)
        self.finished_pub = self.create_publisher(String, '/mission/finished', 10)

        self.create_subscription(PoseStamped, '/leader/pose_enu', self._leader_cb, 10)
        self.create_subscription(Odometry, '/ugv/odom', self._ugv_cb, 10)
        self.create_timer(1.0 / publish_rate, self._tick)
        self.get_logger().info(f'Mission loaded with {len(self.waypoints)} waypoints')

    def _leader_cb(self, msg: PoseStamped) -> None:
        self.leader_pose = msg

    def _ugv_cb(self, msg: Odometry) -> None:
        self.ugv_odom = msg

    def _tick(self) -> None:
        if not self.finished and self.leader_pose is not None and self.ugv_odom is not None:
            dx = self.leader_pose.pose.position.x - self.ugv_odom.pose.pose.position.x
            dy = self.leader_pose.pose.position.y - self.ugv_odom.pose.pose.position.y
            if math.hypot(dx, dy) < self.switch_distance and self.index < len(self.waypoints) - 1:
                self.index += 1
                self.get_logger().info(f'Switching to waypoint {self.index}: {self.waypoints[self.index]}')
            elif math.hypot(dx, dy) < self.switch_distance and self.index == len(self.waypoints) - 1:
                self.finished = True
                done = String()
                done.data = 'finished'
                self.finished_pub.publish(done)
                self.get_logger().info('Mission finished')

        current = self.waypoints[self.index]
        wp_msg = Float32MultiArray()
        wp_msg.data = [float(current[0]), float(current[1]), float(current[2])]
        self.wp_pub.publish(wp_msg)

        id_msg = Int32()
        id_msg.data = self.index
        self.id_pub.publish(id_msg)

        state_msg = String()
        state_msg.data = 'finished' if self.finished else 'running'
        self.state_pub.publish(state_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
