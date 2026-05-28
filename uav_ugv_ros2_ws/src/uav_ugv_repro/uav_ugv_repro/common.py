"""Shared helpers for the UAV/UGV reproduction nodes."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

from geometry_msgs.msg import PoseStamped

Point3 = Tuple[float, float, float]
Point2 = Tuple[float, float]
Obstacle = Tuple[float, float, float]


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def parse_triplets(values: Iterable[str], default: Sequence[Point3]) -> List[Point3]:
    parsed: List[Point3] = []
    for raw in values:
        try:
            parts = [float(part.strip()) for part in str(raw).split(',')]
        except ValueError:
            continue
        if len(parts) == 3:
            parsed.append((parts[0], parts[1], parts[2]))
    return parsed or list(default)


def make_pose_stamped(node, frame_id: str, x: float, y: float, z: float) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = float(z)
    msg.pose.orientation.w = 1.0
    return msg


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)
