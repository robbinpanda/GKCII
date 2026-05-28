"""PX4 Offboard leader node for the UAV."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String

from .common import make_pose_stamped


class Px4UavLeader(Node):
    """Send PX4 Offboard setpoints and republish UAV pose in ENU."""

    def __init__(self) -> None:
        super().__init__('px4_uav_leader')
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('arm_after_setpoints', 20)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('source_system', 1)
        self.declare_parameter('source_component', 1)
        self.declare_parameter('auto_land_when_finished', False)

        self.target_system = int(self.get_parameter('target_system').value)
        self.target_component = int(self.get_parameter('target_component').value)
        self.source_system = int(self.get_parameter('source_system').value)
        self.source_component = int(self.get_parameter('source_component').value)
        self.arm_after_setpoints = int(self.get_parameter('arm_after_setpoints').value)
        self.auto_land_when_finished = bool(self.get_parameter('auto_land_when_finished').value)

        self.current_waypoint = (0.0, 0.0, 1.5)
        self.setpoint_counter = 0
        self.has_odometry = False
        self.sent_mode_and_arm = False
        self.sent_land = False

        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.leader_pose_pub = self.create_publisher(PoseStamped, '/leader/pose_enu', 10)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self._odom_cb, px4_qos)
        self.create_subscription(Float32MultiArray, '/mission/current_waypoint', self._waypoint_cb, 10)
        self.create_subscription(String, '/mission/state', self._state_cb, 10)

        rate = float(self.get_parameter('command_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('PX4 UAV leader ready')

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        if len(msg.position) < 3:
            return
        self.has_odometry = True
        # PX4 odometry is NED. Publish a ROS-friendly ENU pose.
        ned_x, ned_y, ned_z = msg.position[0], msg.position[1], msg.position[2]
        pose = make_pose_stamped(self, 'map', ned_y, ned_x, -ned_z)
        self.leader_pose_pub.publish(pose)

    def _waypoint_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 3:
            self.current_waypoint = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))

    def _state_cb(self, msg: String) -> None:
        if msg.data == 'finished' and self.auto_land_when_finished and not self.sent_land:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.sent_land = True
            self.get_logger().info('Mission finished; land command sent')

    def _tick(self) -> None:
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()

        if not self.has_odometry:
            return

        if self.setpoint_counter == self.arm_after_setpoints and not self.sent_mode_and_arm:
            self._set_offboard_mode()
            self._arm()
            self.sent_mode_and_arm = True
            self.get_logger().info('Offboard mode and arm commands sent')

        if self.setpoint_counter <= self.arm_after_setpoints:
            self.setpoint_counter += 1

    def _publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self._now_us()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_pub.publish(msg)

    def _publish_trajectory_setpoint(self) -> None:
        enu_x, enu_y, enu_z = self.current_waypoint
        msg = TrajectorySetpoint()
        msg.timestamp = self._now_us()
        # ENU to NED: north=y, east=x, down=-z.
        msg.position = [float(enu_y), float(enu_x), float(-enu_z)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = 0.0
        msg.yawspeed = math.nan
        self.trajectory_pub.publish(msg)

    def _set_offboard_mode(self) -> None:
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def _arm(self) -> None:
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=float(VehicleCommand.ARMING_ACTION_ARM),
        )

    def _publish_vehicle_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.timestamp = self._now_us()
        msg.command = int(command)
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = self.source_system
        msg.source_component = self.source_component
        msg.from_external = True
        self.command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Px4UavLeader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
