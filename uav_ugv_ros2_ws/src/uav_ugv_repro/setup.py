from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'uav_ugv_repro'


def collect_data_files(directory):
    data_files = []
    for root, _, files in os.walk(directory):
        if not files:
            continue
        paths = [os.path.join(root, name) for name in files]
        data_files.append((os.path.join('share', package_name, root), paths))
    return data_files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ] + collect_data_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robbinpanda',
    maintainer_email='robbinpanda@users.noreply.github.com',
    description='ROS 2 reproduction package for UAV/UGV leader-follower control.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mission_server = uav_ugv_repro.mission_server:main',
            'px4_uav_leader = uav_ugv_repro.px4_uav_leader:main',
            'ugv_state_publisher = uav_ugv_repro.ugv_state_publisher:main',
            'ugv_follower_controller = uav_ugv_repro.ugv_follower_controller:main',
            'gazebo_obstacle_detector = uav_ugv_repro.gazebo_obstacle_detector:main',
            'grid_obstacle_planner = uav_ugv_repro.grid_obstacle_planner:main',
            'trajectory_logger = uav_ugv_repro.trajectory_logger:main',
        ],
    },
)
