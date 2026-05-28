"""Grid planner that sends a local goal to the UGV follower."""

from __future__ import annotations

from collections import deque
import math
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool

from .common import make_pose_stamped

GridCell = Tuple[int, int]


class GridObstaclePlanner(Node):
    """Maintain a small occupancy grid and publish a local UGV goal."""

    def __init__(self) -> None:
        super().__init__('grid_obstacle_planner')
        self.declare_parameter('width', 60)
        self.declare_parameter('height', 40)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('origin_x', -3.0)
        self.declare_parameter('origin_y', -2.0)
        self.declare_parameter('inflation_radius_cells', 2)
        self.declare_parameter('local_goal_lookahead_cells', 5)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.inflation = int(self.get_parameter('inflation_radius_cells').value)
        self.lookahead = int(self.get_parameter('local_goal_lookahead_cells').value)
        self.occupied: set[GridCell] = set()

        self.odom: Odometry | None = None
        self.leader_pose: PoseStamped | None = None
        self.left = False
        self.right = False

        self.goal_pub = self.create_publisher(PoseStamped, '/ugv/local_goal', 10)
        self.create_subscription(Odometry, '/ugv/odom', self._odom_cb, 10)
        self.create_subscription(PoseStamped, '/leader/pose_enu', self._leader_cb, 10)
        self.create_subscription(Bool, '/ugv/front_left_obstacle', self._left_cb, 10)
        self.create_subscription(Bool, '/ugv/front_right_obstacle', self._right_cb, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('Grid obstacle planner ready')

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _leader_cb(self, msg: PoseStamped) -> None:
        self.leader_pose = msg

    def _left_cb(self, msg: Bool) -> None:
        self.left = msg.data

    def _right_cb(self, msg: Bool) -> None:
        self.right = msg.data

    def _tick(self) -> None:
        if self.odom is None or self.leader_pose is None:
            return
        self._mark_detected_obstacles()

        start = self._world_to_cell(self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
        goal = self._world_to_cell(self.leader_pose.pose.position.x, self.leader_pose.pose.position.y)
        if start is None or goal is None:
            self._publish_direct_goal()
            return

        path = self._bfs(start, goal)
        if not path:
            self._publish_direct_goal()
            return
        target_cell = path[min(len(path) - 1, self.lookahead)]
        x, y = self._cell_to_world(target_cell)
        self.goal_pub.publish(make_pose_stamped(self, 'map', x, y, 0.0))

    def _mark_detected_obstacles(self) -> None:
        if not (self.left or self.right):
            return
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        for lateral in self._active_laterals():
            ox = x + math.cos(yaw) * 0.35 - math.sin(yaw) * lateral
            oy = y + math.sin(yaw) * 0.35 + math.cos(yaw) * lateral
            cell = self._world_to_cell(ox, oy)
            if cell is not None:
                self._add_obstacle(cell)

    def _active_laterals(self) -> Iterable[float]:
        if self.left:
            yield 0.18
        if self.right:
            yield -0.18

    def _add_obstacle(self, cell: GridCell) -> None:
        cx, cy = cell
        for dx in range(-self.inflation, self.inflation + 1):
            for dy in range(-self.inflation, self.inflation + 1):
                if dx * dx + dy * dy <= self.inflation * self.inflation:
                    inflated = (cx + dx, cy + dy)
                    if self._in_bounds(inflated):
                        self.occupied.add(inflated)

    def _publish_direct_goal(self) -> None:
        self.goal_pub.publish(make_pose_stamped(
            self,
            'map',
            self.leader_pose.pose.position.x,
            self.leader_pose.pose.position.y,
            0.0,
        ))

    def _world_to_cell(self, x: float, y: float) -> Optional[GridCell]:
        cx = int((x - self.origin_x) / self.resolution)
        cy = int((y - self.origin_y) / self.resolution)
        cell = (cx, cy)
        return cell if self._in_bounds(cell) else None

    def _cell_to_world(self, cell: GridCell) -> Tuple[float, float]:
        cx, cy = cell
        return (
            self.origin_x + (cx + 0.5) * self.resolution,
            self.origin_y + (cy + 0.5) * self.resolution,
        )

    def _in_bounds(self, cell: GridCell) -> bool:
        cx, cy = cell
        return 0 <= cx < self.width and 0 <= cy < self.height

    def _bfs(self, start: GridCell, goal: GridCell) -> List[GridCell]:
        queue: deque[GridCell] = deque([start])
        came_from: Dict[GridCell, Optional[GridCell]] = {start: None}
        while queue:
            current = queue.popleft()
            if current == goal:
                break
            for nxt in self._neighbors(current):
                if nxt in came_from or nxt in self.occupied:
                    continue
                came_from[nxt] = current
                queue.append(nxt)
        if goal not in came_from:
            return []
        path: List[GridCell] = []
        current: Optional[GridCell] = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def _neighbors(self, cell: GridCell) -> Iterable[GridCell]:
        cx, cy = cell
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (cx + dx, cy + dy)
            if self._in_bounds(nxt):
                yield nxt


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GridObstaclePlanner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
