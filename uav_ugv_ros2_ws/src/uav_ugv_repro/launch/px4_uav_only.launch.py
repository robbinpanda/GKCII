"""Launch only the PX4 UAV leader path."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('uav_ugv_repro')
    config = os.path.join(package_share, 'config', 'experiment.yaml')
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    start_agent = LaunchConfiguration('start_agent')
    start_px4 = LaunchConfiguration('start_px4')
    return LaunchDescription([
        DeclareLaunchArgument('start_agent', default_value='true'),
        DeclareLaunchArgument('start_px4', default_value='true'),
        ExecuteProcess(cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'], output='screen', condition=IfCondition(start_agent)),
        ExecuteProcess(cmd=['make', 'px4_sitl', 'gz_x500'], cwd=px4_dir, output='screen', condition=IfCondition(start_px4)),
        Node(package='uav_ugv_repro', executable='mission_server', name='mission_server', parameters=[config], output='screen'),
        Node(package='uav_ugv_repro', executable='px4_uav_leader', name='px4_uav_leader', parameters=[config], output='screen'),
    ])
