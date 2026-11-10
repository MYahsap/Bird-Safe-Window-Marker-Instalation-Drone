"""Cartesian impedance control for the X650 one-DOF manipulator."""

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, PointStamped
from px4_msgs.msg import VehicleAttitude
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class ManipulatorController(Node):
    """Map Cartesian spring-damper force to the single joint's torque."""

    STS_TORQUE_ENABLE = 40
    STS_GOAL_POSITION = 42
    STS_WRITE = 0x03
    STS_STEPS_PER_TURN = 4096

    def __init__(self) -> None:
        super().__init__("x650_manipulator_controller")
        self.declare_parameter("mode", "sim")
        self.declare_parameter("angle_topic", "/x650/manipulator/angle")
        self.declare_parameter(
            "cartesian_target_topic", "/x650/manipulator/cartesian_target"
        )
        self.declare_parameter(
            "gz_effort_topic",
            "/model/x650_0/joint/manipulator_tilt_joint/cmd_force",
        )
        self.declare_parameter(
            "gz_state_topic", "/world/x650_world/model/x650_0/joint_state"
        )
        self.declare_parameter("joint_name", "manipulator_tilt_joint")
        self.declare_parameter("min_angle", -math.pi)
        self.declare_parameter("max_angle", math.pi / 2.0)
        self.declare_parameter("initial_target_angle", -math.pi / 2.0)
        self.declare_parameter("stabilize_forward", True)
        self.declare_parameter("forward_joint_angle", -math.pi / 2.0)
        self.declare_parameter("pitch_compensation_gain", 1.0)
        self.declare_parameter("attitude_topic", "/fmu/out/vehicle_attitude")
        self.declare_parameter("hardware_command_rate", 20.0)
        self.declare_parameter("pivot_x", 0.15)
        self.declare_parameter("pivot_z", -0.10)
        self.declare_parameter("arm_length", 0.50)
        self.declare_parameter("center_of_mass_length", 0.25)
        self.declare_parameter("moving_mass", 0.60)
        self.declare_parameter("gravity", 9.80665)
        self.declare_parameter("stiffness_x", 40.0)
        self.declare_parameter("stiffness_z", 40.0)
        self.declare_parameter("damping_x", 5.0)
        self.declare_parameter("damping_z", 5.0)
        self.declare_parameter("max_torque", 2.942)
        self.declare_parameter("max_torque_rate", 20.0)
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("state_timeout", 2.0)
        self.declare_parameter("gravity_compensation", True)
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 1_000_000)
        self.declare_parameter("servo_id", 1)
        self.declare_parameter("zero_position", 2048)
        self.declare_parameter("direction", 1)

        self.mode = str(self.get_parameter("mode").value).lower()
        self.joint_name = str(self.get_parameter("joint_name").value)
        self.min_angle = float(self.get_parameter("min_angle").value)
        self.max_angle = float(self.get_parameter("max_angle").value)
        initial_target_angle = float(
            self.get_parameter("initial_target_angle").value
        )
        self.stabilize_forward = bool(
            self.get_parameter("stabilize_forward").value
        )
        self.forward_joint_angle = float(
            self.get_parameter("forward_joint_angle").value
        )
        self.pitch_compensation_gain = float(
            self.get_parameter("pitch_compensation_gain").value
        )
        self.hardware_command_rate = float(
            self.get_parameter("hardware_command_rate").value
        )
        self.pivot_x = float(self.get_parameter("pivot_x").value)
        self.pivot_z = float(self.get_parameter("pivot_z").value)
        self.arm_length = float(self.get_parameter("arm_length").value)
        self.com_length = float(self.get_parameter("center_of_mass_length").value)
        self.moving_mass = float(self.get_parameter("moving_mass").value)
        self.gravity = float(self.get_parameter("gravity").value)
        self.kx = float(self.get_parameter("stiffness_x").value)
        self.kz = float(self.get_parameter("stiffness_z").value)
        self.dx = float(self.get_parameter("damping_x").value)
        self.dz = float(self.get_parameter("damping_z").value)
        self.max_torque = float(self.get_parameter("max_torque").value)
        self.max_torque_rate = float(self.get_parameter("max_torque_rate").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.gravity_compensation = bool(
            self.get_parameter("gravity_compensation").value
        )

        if self.mode not in {"sim", "hardware"}:
            raise ValueError("mode must be 'sim' or 'hardware'")
        if self.min_angle >= self.max_angle:
            raise ValueError("min_angle must be less than max_angle")
        if not self.min_angle <= initial_target_angle <= self.max_angle:
            raise ValueError("initial_target_angle is outside the joint limits")
        if not self.min_angle <= self.forward_joint_angle <= self.max_angle:
            raise ValueError("forward_joint_angle is outside the joint limits")
        if not math.isfinite(self.pitch_compensation_gain):
            raise ValueError("pitch_compensation_gain must be finite")
        positive_parameters = {
            "arm_length": self.arm_length,
            "center_of_mass_length": self.com_length,
            "moving_mass": self.moving_mass,
            "max_torque": self.max_torque,
            "max_torque_rate": self.max_torque_rate,
            "control_rate": self.control_rate,
            "state_timeout": self.state_timeout,
            "hardware_command_rate": self.hardware_command_rate,
        }
        invalid = [name for name, value in positive_parameters.items() if value <= 0.0]
        if invalid:
            raise ValueError(f"parameters must be positive: {', '.join(invalid)}")
        if min(self.kx, self.kz, self.dx, self.dz) < 0.0:
            raise ValueError("stiffness and damping gains must be non-negative")

        self.state_lock = threading.Lock()
        self.joint_position: Optional[float] = None
        self.joint_velocity = 0.0
        self.last_state_time = 0.0
        self.target_x, self.target_z = self.forward_kinematics(initial_target_angle)
        self.stabilized_angle = initial_target_angle
        self.last_hardware_command_time = 0.0
        self.last_hardware_angle: Optional[float] = None
        self.last_torque = 0.0
        self.warned_stale = False
        self.serial_port: Optional[object] = None
        self.torque_enabled = False

        self.joint_state_pub = self.create_publisher(
            JointState, "/x650/manipulator/joint_state", 10
        )
        self.tip_state_pub = self.create_publisher(
            PointStamped, "/x650/manipulator/tip_position", 10
        )
        self.torque_pub = self.create_publisher(
            Float64, "/x650/manipulator/commanded_torque", 10
        )
        self.stabilized_angle_pub = self.create_publisher(
            Float64, "/x650/manipulator/stabilized_angle", 10
        )
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.attitude_subscription = self.create_subscription(
            VehicleAttitude,
            str(self.get_parameter("attitude_topic").value),
            self.on_vehicle_attitude,
            px4_qos,
        )
        self.angle_subscription = self.create_subscription(
            Float64,
            str(self.get_parameter("angle_topic").value),
            self.on_angle,
            10,
        )
        self.cartesian_subscription = self.create_subscription(
            Point,
            str(self.get_parameter("cartesian_target_topic").value),
            self.on_cartesian_target,
            10,
        )

        if self.mode == "sim":
            from gz.msgs10.double_pb2 import Double
            from gz.msgs10.model_pb2 import Model
            from gz.transport13 import Node as GazeboNode

            self.gz_message_type = Double
            self.gz_node = GazeboNode()
            effort_topic = str(self.get_parameter("gz_effort_topic").value)
            state_topic = str(self.get_parameter("gz_state_topic").value)
            self.gz_effort_publisher = self.gz_node.advertise(
                effort_topic, self.gz_message_type
            )
            self.gz_node.subscribe(Model, state_topic, self.on_gazebo_state)
            self.control_timer = self.create_timer(
                1.0 / self.control_rate, self.control_step
            )
            self.get_logger().info(
                f"Cartesian impedance control: {state_topic} -> {effort_topic}; "
                f"K=({self.kx:.1f}, {self.kz:.1f}) N/m, "
                f"D=({self.dx:.1f}, {self.dz:.1f}) N.s/m"
            )
        else:
            import serial

            port = str(self.get_parameter("port").value)
            baud = int(self.get_parameter("baud").value)
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=0.05,
                write_timeout=0.1,
            )
            self.get_logger().warning(
                "Hardware mode uses position commands, not true impedance; "
                "torque feedback/control is not enabled for this STS3215 interface"
            )
            self.get_logger().info(f"STS3215 output: {port} at {baud} baud")

        if self.stabilize_forward:
            self.get_logger().info(
                "Forward stabilization enabled: "
                "joint_target = forward_joint_angle + vehicle_pitch"
            )

    def forward_kinematics(self, angle: float) -> tuple[float, float]:
        return (
            self.pivot_x - self.arm_length * math.sin(angle),
            self.pivot_z - self.arm_length * math.cos(angle),
        )

    @staticmethod
    def pitch_from_quaternion(quaternion) -> float:
        """Return aerospace pitch from PX4's [w, x, y, z] quaternion."""
        if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
            raise ValueError("attitude quaternion must contain four finite values")
        norm = math.sqrt(sum(float(v) * float(v) for v in quaternion))
        if norm < 1e-6:
            raise ValueError("attitude quaternion norm is zero")
        w, x, y, z = (float(v) / norm for v in quaternion)
        sin_pitch = 2.0 * (w * y - z * x)
        return math.asin(max(-1.0, min(1.0, sin_pitch)))

    def on_vehicle_attitude(self, msg: VehicleAttitude) -> None:
        if not self.stabilize_forward:
            return
        try:
            pitch = self.pitch_from_quaternion(msg.q)
        except ValueError as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return

        requested_angle = (
            self.forward_joint_angle + self.pitch_compensation_gain * pitch
        )
        angle = max(self.min_angle, min(self.max_angle, requested_angle))
        if angle != requested_angle:
            self.get_logger().warning(
                f"Forward stabilization limited at {math.degrees(angle):.1f} deg "
                f"for vehicle pitch {math.degrees(pitch):.1f} deg",
                throttle_duration_sec=2.0,
            )

        target_x, target_z = self.forward_kinematics(angle)
        with self.state_lock:
            self.stabilized_angle = angle
            self.target_x = target_x
            self.target_z = target_z

        target_msg = Float64()
        target_msg.data = angle
        self.stabilized_angle_pub.publish(target_msg)

        if self.mode == "hardware":
            now = time.monotonic()
            period = 1.0 / self.hardware_command_rate
            if now - self.last_hardware_command_time >= period:
                self.write_hardware_angle(angle)
                self.last_hardware_command_time = now
                self.last_hardware_angle = angle

    def on_angle(self, msg: Float64) -> None:
        if not math.isfinite(msg.data):
            self.get_logger().error("Rejected non-finite angle")
            return
        angle = max(self.min_angle, min(self.max_angle, float(msg.data)))
        if angle != msg.data:
            self.get_logger().warning(f"Clamped requested angle to {angle:.3f} rad")

        if self.stabilize_forward:
            self.get_logger().warning(
                "Ignoring manual angle command while forward stabilization is enabled",
                throttle_duration_sec=2.0,
            )
            return

        if self.mode == "sim":
            target_x, target_z = self.forward_kinematics(angle)
            with self.state_lock:
                self.target_x = target_x
                self.target_z = target_z
            self.get_logger().info(
                f"Angle target mapped to tip target x={target_x:.3f}, z={target_z:.3f} m"
            )
        else:
            self.write_hardware_angle(angle)
            self.get_logger().info(f"Manipulator target: {angle:.3f} rad")

    def on_cartesian_target(self, msg: Point) -> None:
        if not all(math.isfinite(value) for value in (msg.x, msg.y, msg.z)):
            self.get_logger().error("Rejected non-finite Cartesian target")
            return
        if self.mode != "sim":
            self.get_logger().error(
                "Cartesian impedance targets require simulation torque feedback"
            )
            return
        if self.stabilize_forward:
            self.get_logger().warning(
                "Ignoring Cartesian target while forward stabilization is enabled",
                throttle_duration_sec=2.0,
            )
            return
        if abs(msg.y) > 1e-6:
            self.get_logger().warning("Ignoring Cartesian target y; arm has one DOF")
        distance = math.hypot(msg.x - self.pivot_x, msg.z - self.pivot_z)
        if distance > 2.0 * self.arm_length:
            self.get_logger().error(
                f"Rejected target {distance:.3f} m from pivot; safety limit is "
                f"{2.0 * self.arm_length:.3f} m"
            )
            return
        with self.state_lock:
            self.target_x = float(msg.x)
            self.target_z = float(msg.z)
        self.get_logger().info(f"Cartesian target: x={msg.x:.3f}, z={msg.z:.3f} m")

    def on_gazebo_state(self, msg) -> None:
        for joint in msg.joint:
            if joint.name != self.joint_name:
                continue
            position = float(joint.axis1.position)
            velocity = float(joint.axis1.velocity)
            if not math.isfinite(position) or not math.isfinite(velocity):
                return
            with self.state_lock:
                self.joint_position = position
                self.joint_velocity = velocity
                self.last_state_time = time.monotonic()
            self.publish_state(position, velocity)
            return

    def publish_state(self, position: float, velocity: float) -> None:
        stamp = self.get_clock().now().to_msg()
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = [self.joint_name]
        joint_state.position = [position]
        joint_state.velocity = [velocity]
        joint_state.effort = [self.last_torque]
        self.joint_state_pub.publish(joint_state)

        x, z = self.forward_kinematics(position)
        tip = PointStamped()
        tip.header.stamp = stamp
        tip.header.frame_id = "base_link"
        tip.point.x = x
        tip.point.y = 0.0
        tip.point.z = z
        self.tip_state_pub.publish(tip)

    def control_step(self) -> None:
        now = time.monotonic()
        with self.state_lock:
            q = self.joint_position
            q_dot = self.joint_velocity
            target_x = self.target_x
            target_z = self.target_z
            state_age = now - self.last_state_time

        if q is None or target_x is None or target_z is None:
            return
        if state_age > self.state_timeout:
            self.publish_sim_torque(0.0)
            if not self.warned_stale:
                self.get_logger().error(
                    f"Joint state is stale ({state_age:.3f} s); commanding zero torque"
                )
                self.warned_stale = True
            return
        self.warned_stale = False

        x, z = self.forward_kinematics(q)
        jacobian_x = -self.arm_length * math.cos(q)
        jacobian_z = self.arm_length * math.sin(q)
        velocity_x = jacobian_x * q_dot
        velocity_z = jacobian_z * q_dot
        force_x = self.kx * (target_x - x) - self.dx * velocity_x
        force_z = self.kz * (target_z - z) - self.dz * velocity_z
        torque = jacobian_x * force_x + jacobian_z * force_z

        if self.gravity_compensation:
            torque += self.moving_mass * self.gravity * self.com_length * math.sin(q)

        torque = max(-self.max_torque, min(self.max_torque, torque))
        max_delta = self.max_torque_rate / self.control_rate
        torque = max(
            self.last_torque - max_delta,
            min(self.last_torque + max_delta, torque),
        )
        self.publish_sim_torque(torque)

    def publish_sim_torque(self, torque: float) -> None:
        command = self.gz_message_type()
        command.data = float(torque)
        if not self.gz_effort_publisher.publish(command):
            self.get_logger().warning(
                "Gazebo did not accept the torque command", throttle_duration_sec=2.0
            )
        self.last_torque = float(torque)
        if rclpy.ok():
            torque_msg = Float64()
            torque_msg.data = self.last_torque
            self.torque_pub.publish(torque_msg)

    def write_hardware_angle(self, angle: float) -> None:
        servo_id = int(self.get_parameter("servo_id").value)
        zero = int(self.get_parameter("zero_position").value)
        direction = int(self.get_parameter("direction").value)
        if not 0 <= servo_id <= 253:
            raise ValueError("servo_id must be between 0 and 253")
        if direction not in {-1, 1}:
            raise ValueError("direction must be -1 or 1")

        steps = round(
            zero + direction * angle * self.STS_STEPS_PER_TURN / (2.0 * math.pi)
        )
        steps = max(0, min(self.STS_STEPS_PER_TURN - 1, steps))
        if not self.torque_enabled:
            self.sts_write(servo_id, self.STS_TORQUE_ENABLE, bytes([1]))
            self.torque_enabled = True
        self.sts_write(
            servo_id,
            self.STS_GOAL_POSITION,
            bytes([steps & 0xFF, (steps >> 8) & 0xFF]),
        )

    def sts_write(self, servo_id: int, address: int, data: bytes) -> None:
        parameters = bytes([address]) + data
        length = len(parameters) + 2
        body = bytes([servo_id, length, self.STS_WRITE]) + parameters
        checksum = (~sum(body)) & 0xFF
        packet = b"\xff\xff" + body + bytes([checksum])
        self.serial_port.write(packet)
        self.serial_port.flush()

    def destroy_node(self) -> None:
        if self.mode == "sim" and hasattr(self, "gz_effort_publisher"):
            self.publish_sim_torque(0.0)
        if self.serial_port is not None:
            self.serial_port.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ManipulatorController()
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
