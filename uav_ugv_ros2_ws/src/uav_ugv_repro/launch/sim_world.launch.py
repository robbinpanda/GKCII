"""Launch the lightweight ROS-side UGV simulation without PX4."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('uav_ugv_repro')
    config = os.path.join(package_share, 'config', 'experiment.yaml')
    return LaunchDescription([
        Node(package='uav_ugv_repro', executable='ugv_state_publisher', name='ugv_state_publisher', parameters=[config], output='screen'),
        Node(package='uav_ugv_repro', executable='gazebo_obstacle_detector', name='gazebo_obstacle_detector', parameters=[config], output='screen'),
        Node(package='uav_ugv_repro', executable='grid_obstacle_planner', name='grid_obstacle_planner', parameters=[config], output='screen'),
        Node(package='uav_ugv_repro', executable='ugv_follower_controller', name='ugv_follower_controller', parameters=[config], output='screen'),
        Node(package='uav_ugv_repro', executable='trajectory_logger', name='trajectory_logger', parameters=[config], output='screen'),
    ])
