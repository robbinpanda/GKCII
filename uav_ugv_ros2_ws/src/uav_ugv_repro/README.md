# uav_ugv_repro

ROS 2 package for reproducing the UAV/UGV leader-follower experiment on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and PX4 SITL.

## Typical launch

Stop any existing PX4/Gazebo/Agent processes first, then run:

```bash
cd /home/robbinpanda/GKCII/uav_ugv_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch uav_ugv_repro uav_ugv_experiment.launch.py
```

If PX4 and Micro XRCE-DDS Agent are already running in separate terminals, launch only the reproduction nodes:

```bash
ros2 launch uav_ugv_repro uav_ugv_experiment.launch.py start_agent:=false start_px4:=false
```
