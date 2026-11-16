"""Coordinate the X650 base and one-DOF arm from a world-frame tool pose."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from x650_sim.end_effector_kinematics import (
    base_for_configuration,
    solve_constraint,
    tool_direction,
)


class EndEffectorController(Node):
    """Stream coordinated PX4 and arm setpoints for a world-frame tool pose."""

    def __init__(self) -> None:
        super().__init__("x650_end_effector_controller")
        self.declare_parameter("target_topic", "/x650/end_effector/target_pose")
        self.declare_parameter("angle_topic", "/x650/manipulator/angle")
        self.declare_parameter("joint_state_topic", "/x650/manipulator/joint_state")
        self.declare_parameter("joint_name", "manipulator_tilt_joint")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("pivot_x", 0.15)
        self.declare_parameter("pivot_z", -0.10)
        self.declare_parameter("arm_length", 0.50)
        self.declare_parameter("min_joint_angle", -math.pi)
        self.declare_parameter("max_joint_angle", math.pi / 2.0)
        self.declare_parameter("max_abs_elevation", math.radians(80.0))
        self.declare_parameter("min_base_altitude", 0.25)
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("request_offboard", True)
        self.declare_parameter("auto_arm", False)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.joint_name = str(self.get_parameter("joint_name").value)
        self.pivot_x = float(self.get_parameter("pivot_x").value)
        self.pivot_z = float(self.get_parameter("pivot_z").value)
        self.arm_length = float(self.get_parameter("arm_length").value)
        self.min_joint_angle = float(self.get_parameter("min_joint_angle").value)
        self.max_joint_angle = float(self.get_parameter("max_joint_angle").value)
        self.max_abs_elevation = float(
            self.get_parameter("max_abs_elevation").value
        )
        self.min_base_altitude = float(
            self.get_parameter("min_base_altitude").value
        )
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.request_offboard = bool(
            self.get_parameter("request_offboard").value
        )
        self.auto_arm = bool(self.get_parameter("auto_arm").value)

        if self.arm_length <= 0.0 or self.control_rate <= 0.0:
            raise ValueError("arm_length and control_rate must be positive")
        if self.min_joint_angle >= self.max_joint_angle:
            raise ValueError("min_joint_angle must be less than max_joint_angle")
        if not 0.0 < self.max_abs_elevation < math.pi / 2.0:
            raise ValueError("max_abs_elevation must be between 0 and pi/2")

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", px4_qos
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", px4_qos
        )
        self.angle_pub = self.create_publisher(
            Float64, str(self.get_parameter("angle_topic").value), 10
        )
        self.joint_setpoint_pub = self.create_publisher(
            Float64, "/x650/end_effector/joint_setpoint", 10
        )
        self.base_setpoint_pub = self.create_publisher(
            PoseStamped, "/x650/end_effector/base_setpoint", 10
        )
        self.target_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("target_topic").value),
            self.on_target,
            10,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self.on_vehicle_status,
            px4_qos,
        )
        self.ack_sub = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.on_command_ack,
            px4_qos,
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self.on_vehicle_attitude,
            px4_qos,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self.on_joint_state,
            10,
        )

        self.target_position: tuple[float, float, float] | None = None
        self.target_direction: tuple[float, float, float] | None = None
        self.vehicle_pitch = 0.0
        self.vehicle_yaw = 0.0
        self.attitude_received = False
        self.joint_position: float | None = None
        self.armed = False
        self.offboard = False
        self.stream_count = 0
        self.last_arming_state = None
        self.last_nav_state = None
        self.timer = self.create_timer(1.0 / self.control_rate, self.control_step)
        self.get_logger().info(
            f"Waiting for a {self.world_frame}-frame PoseStamped on "
            f"{self.get_parameter('target_topic').value}; pose +X is tool forward"
        )

    def now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def on_target(self, msg: PoseStamped) -> None:
        if msg.header.frame_id not in {"", self.world_frame}:
            self.get_logger().error(
                f"Rejected frame '{msg.header.frame_id}'; expected '{self.world_frame}'"
            )
            return
        position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        try:
            direction = tool_direction(quaternion)
            base, joint, yaw, elevation = solve_constraint(
                position,
                direction,
                self.pivot_x,
                self.pivot_z,
                self.arm_length,
                self.vehicle_pitch,
            )
        except ValueError as error:
            self.get_logger().error(f"Rejected end-effector target: {error}")
            return
        if abs(elevation) > self.max_abs_elevation:
            self.get_logger().error(
                f"Rejected tool elevation {math.degrees(elevation):.1f} deg; "
                f"limit is +/-{math.degrees(self.max_abs_elevation):.1f} deg"
            )
            return
        if not self.min_joint_angle <= joint <= self.max_joint_angle:
            self.get_logger().error(
                f"Rejected required joint angle {math.degrees(joint):.1f} deg; "
                f"range is {math.degrees(self.min_joint_angle):.1f} to "
                f"{math.degrees(self.max_joint_angle):.1f} deg"
            )
            return
        if base[2] < self.min_base_altitude:
            self.get_logger().error(
                f"Rejected base altitude {base[2]:.2f} m; minimum is "
                f"{self.min_base_altitude:.2f} m"
            )
            return

        self.target_position = tuple(float(v) for v in position)
        self.target_direction = direction
        self.stream_count = 0
        self.get_logger().info(
            f"Accepted tool pose: p=({position[0]:.2f}, {position[1]:.2f}, "
            f"{position[2]:.2f}) m, yaw={math.degrees(yaw):.1f} deg, "
            f"elevation={math.degrees(elevation):.1f} deg"
        )

    def on_vehicle_attitude(self, msg: VehicleAttitude) -> None:
        if len(msg.q) != 4 or not all(math.isfinite(v) for v in msg.q):
            return
        norm = math.sqrt(sum(float(v) * float(v) for v in msg.q))
        if norm < 1e-6:
            return
        w, x, y, z = (float(v) / norm for v in msg.q)
        sin_pitch = 2.0 * (w * y - z * x)
        self.vehicle_pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        yaw_ned = math.atan2(
            2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
        )
        self.vehicle_yaw = -yaw_ned
        self.attitude_received = True

    def on_joint_state(self, msg: JointState) -> None:
        try:
            index = msg.name.index(self.joint_name)
        except ValueError:
            return
        if index >= len(msg.position) or not math.isfinite(msg.position[index]):
            return
        self.joint_position = float(msg.position[index])

    def on_vehicle_status(self, msg: VehicleStatus) -> None:
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        if msg.arming_state != self.last_arming_state:
            self.get_logger().info(
                "PX4 arming state: "
                + ("ARMED" if self.armed else f"DISARMED ({msg.arming_state})")
            )
            self.last_arming_state = msg.arming_state
        if msg.nav_state != self.last_nav_state:
            self.get_logger().info(
                "PX4 navigation state: "
                + ("OFFBOARD" if self.offboard else str(msg.nav_state))
            )
            self.last_nav_state = msg.nav_state

    def on_command_ack(self, msg: VehicleCommandAck) -> None:
        self.get_logger().info(
            f"PX4 command {msg.command} acknowledgement: result={msg.result}, "
            f"reason={msg.result_param1}"
        )

    def control_step(self) -> None:
        if self.target_position is None or self.target_direction is None:
            return
        desired_base, requested_joint, yaw, _ = solve_constraint(
            self.target_position,
            self.target_direction,
            self.pivot_x,
            self.pivot_z,
            self.arm_length,
            self.vehicle_pitch,
        )
        joint = max(
            self.min_joint_angle, min(self.max_joint_angle, requested_joint)
        )
        if joint != requested_joint:
            self.get_logger().warning(
                f"Pitch compensation saturated at {math.degrees(joint):.1f} deg",
                throttle_duration_sec=2.0,
            )

        # Use measured joint and vehicle attitude while converging. This makes
        # the drone translate with the actual linkage so the tip-position
        # constraint is maintained instead of only matching the final geometry.
        geometry_joint = self.joint_position if self.joint_position is not None else joint
        geometry_yaw = self.vehicle_yaw if self.attitude_received else yaw
        base, _ = base_for_configuration(
            self.target_position,
            geometry_yaw,
            self.vehicle_pitch,
            geometry_joint,
            self.pivot_x,
            self.pivot_z,
            self.arm_length,
        )
        if self.joint_position is None and not self.attitude_received:
            base = desired_base
        if base[2] < self.min_base_altitude:
            self.get_logger().warning(
                f"Constraint-following base altitude limited to "
                f"{self.min_base_altitude:.2f} m",
                throttle_duration_sec=2.0,
            )
            base = (base[0], base[1], self.min_base_altitude)

        angle = Float64()
        angle.data = joint
        self.angle_pub.publish(angle)
        self.joint_setpoint_pub.publish(angle)

        timestamp = self.now_us()
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.position = True
        mode.velocity = False
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.thrust_and_torque = False
        mode.direct_actuator = False
        self.offboard_pub.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        # Gazebo world is X-North, Y-West, Z-Up; PX4 local position is NED.
        setpoint.position = [base[0], -base[1], -base[2]]
        setpoint.yaw = -yaw
        setpoint.velocity = [math.nan, math.nan, math.nan]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yawspeed = math.nan
        self.trajectory_pub.publish(setpoint)
        self.publish_base_setpoint(base, yaw)

        request_period = max(1, round(self.control_rate))
        if self.stream_count >= request_period and self.stream_count % request_period == 0:
            if self.request_offboard and not self.offboard:
                self.vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
                )
                self.get_logger().info("Requesting Offboard mode")
            if self.auto_arm and not self.armed:
                self.vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0
                )
                self.get_logger().info("Requesting arm")
        self.stream_count += 1

    def publish_base_setpoint(
        self, base: tuple[float, float, float], yaw: float
    ) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = base
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.base_setpoint_pub.publish(msg)

    def vehicle_command(
        self, command: int, param1: float, param2: float = 0.0
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EndEffectorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
