# UAV/UGV ROS 2 Reproduction Progress - 2026-05-28

## Current Scope

The ROS 2 reproduction workspace is now located at:

```text
/home/robbinpanda/GKCII/uav_ugv_ros2_ws
```

The first custom reproduction package has been created at:

```text
/home/robbinpanda/GKCII/uav_ugv_ros2_ws/src/uav_ugv_repro
```

PX4 official packages are kept as local dependencies and are intentionally ignored by Git:

```text
uav_ugv_ros2_ws/src/px4_msgs/
uav_ugv_ros2_ws/src/px4_ros_com/
```

## Implemented Package

Package name:

```text
uav_ugv_repro
```

Implemented ROS 2 executables:

```text
mission_server
px4_uav_leader
ugv_state_publisher
ugv_follower_controller
gazebo_obstacle_detector
grid_obstacle_planner
trajectory_logger
```

Actual registered executable name for the UAV leader is:

```text
px4_uav_leader
```

## Main Files Added

```text
uav_ugv_ros2_ws/src/uav_ugv_repro/
├── README.md
├── config/experiment.yaml
├── launch/
│   ├── px4_uav_only.launch.py
│   ├── sim_world.launch.py
│   └── uav_ugv_experiment.launch.py
├── models/
│   ├── obstacle_box/
│   └── simple_ugv/
├── worlds/leader_follower_world.sdf
├── uav_ugv_repro/
│   ├── common.py
│   ├── gazebo_obstacle_detector.py
│   ├── grid_obstacle_planner.py
│   ├── mission_server.py
│   ├── px4_uav_leader.py
│   ├── trajectory_logger.py
│   ├── ugv_follower_controller.py
│   └── ugv_state_publisher.py
└── setup.py / package.xml / tests
```

## Implemented Behavior

- `px4_uav_leader` publishes PX4 Offboard setpoints to `/fmu/in/trajectory_setpoint`.
- It publishes `/fmu/in/offboard_control_mode` continuously.
- It sends Offboard mode and arm commands only after receiving PX4 odometry.
- It republishes PX4 NED odometry as ROS-style ENU pose on `/leader/pose_enu`.
- `mission_server` publishes `/mission/current_waypoint`, `/mission/state`, and `/mission/waypoint_id`.
- `ugv_state_publisher` provides a lightweight first-stage `/ugv/odom` by integrating `/ugv/cmd_vel`.
- `ugv_follower_controller` follows `/ugv/local_goal` or falls back to the UAV ground projection.
- `gazebo_obstacle_detector` uses configured obstacle truth positions as a first-stage substitute for ray sensors.
- `grid_obstacle_planner` maintains a 60x40 grid, inflates obstacle cells, runs BFS, and publishes `/ugv/local_goal`.
- `trajectory_logger` writes CSV output to `uav_ugv_ros2_ws/results/run_001.csv`.

## Important Fixes Already Applied

### PX4 Preflight / Arming Failure

During Offboard testing, PX4 refused to arm with messages like:

```text
Preflight Fail: system power unavailable
Preflight Fail: No connection to the GCS
Arming denied: Resolve system health failures first
```

For SITL only, this was handled from the PX4 `pxh>` shell with:

```bash
param set CBRK_SUPPLY_CHK 894281
param set NAV_DLL_ACT 0
param save
```

This workaround is documented in the main plan Markdown.

### PX4 Odometry QoS Mismatch

The first version of `px4_uav_leader` did not receive `/fmu/out/vehicle_odometry` because ROS 2 reported:

```text
New publisher discovered on topic '/fmu/out/vehicle_odometry', offering incompatible QoS.
No messages will be received from it. Last incompatible policy: RELIABILITY
```

The fix was to subscribe to PX4 odometry with best-effort QoS:

```python
QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
```

This has been applied in `px4_uav_leader.py`.

## Verification Completed

The following checks passed locally:

```bash
python3 -m compileall uav_ugv_repro/uav_ugv_repro uav_ugv_repro/launch
colcon build --packages-up-to uav_ugv_repro --symlink-install
colcon build --packages-select uav_ugv_repro --symlink-install
colcon test --packages-select uav_ugv_repro --event-handlers console_direct+
ros2 pkg executables uav_ugv_repro
ros2 launch uav_ugv_repro uav_ugv_experiment.launch.py --show-args
```

Registered executables were confirmed:

```text
uav_ugv_repro gazebo_obstacle_detector
uav_ugv_repro grid_obstacle_planner
uav_ugv_repro mission_server
uav_ugv_repro px4_uav_leader
uav_ugv_repro trajectory_logger
uav_ugv_repro ugv_follower_controller
uav_ugv_repro ugv_state_publisher
```

A short launch smoke test also started all ROS-side nodes successfully with:

```bash
ros2 launch uav_ugv_repro uav_ugv_experiment.launch.py start_agent:=false start_px4:=false
```

## How To Run Next

If no PX4/Gazebo/Agent process is already running:

```bash
cd /home/robbinpanda/GKCII/uav_ugv_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch uav_ugv_repro uav_ugv_experiment.launch.py
```

If PX4 and Micro XRCE-DDS Agent are already running in separate terminals:

```bash
cd /home/robbinpanda/GKCII/uav_ugv_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch uav_ugv_repro uav_ugv_experiment.launch.py start_agent:=false start_px4:=false
```

## Known Limitations

- The UGV is currently a lightweight kinematic ROS-side model, not yet a Gazebo physics-controlled vehicle.
- The obstacle detector uses configured truth positions, not Gazebo ray sensors yet.
- The included SDF world/model files are scaffolding and are not yet integrated with PX4's Gazebo world launch path.
- The one-file launch can start PX4, but manual three-terminal startup remains easier for debugging.
- Full visual validation of UAV/UGV motion after the QoS fix still needs to be done interactively.

## Recommended Next Steps

1. Stop all old launch/PX4/Gazebo/Agent processes.
2. Re-run the launch after the QoS fix.
3. Confirm `px4_uav_leader` logs `Offboard mode and arm commands sent` after PX4 odometry appears.
4. Confirm UAV takes off and follows the configured waypoints.
5. Confirm `/home/robbinpanda/GKCII/uav_ugv_ros2_ws/results/run_001.csv` receives trajectory rows.
6. Replace `ugv_state_publisher` with Gazebo bridge-based UGV pose/control once the ROS-side control loop is stable.
