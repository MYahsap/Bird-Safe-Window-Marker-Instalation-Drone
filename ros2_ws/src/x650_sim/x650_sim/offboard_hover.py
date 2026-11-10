"""Minimal PX4 ROS 2 offboard takeoff/hover demonstration."""

import math

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class OffboardHover(Node):
    def __init__(self) -> None:
        super().__init__("x650_offboard_hover")
        self.declare_parameter("altitude", 3.0)
        self.altitude = float(self.get_parameter("altitude").value)
        if self.altitude <= 0.0:
            raise ValueError("altitude must be positive")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )
        self.ack_sub = self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self.on_command_ack,
            qos,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self.on_vehicle_status,
            qos,
        )
        self.armed = False
        self.offboard = False
        self.last_arming_state = None
        self.last_nav_state = None
        self.counter = 0
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"Streaming a NED hover setpoint at {self.altitude:.1f} m"
        )

    def now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def tick(self) -> None:
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
        setpoint.position = [0.0, 0.0, -self.altitude]
        setpoint.yaw = 0.0
        setpoint.velocity = [math.nan, math.nan, math.nan]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yawspeed = math.nan
        self.setpoint_pub.publish(setpoint)

        # PX4 requires setpoints before entering Offboard. Retry at 1 Hz until
        # status confirms both transitions; a one-shot best-effort command can
        # otherwise be missed or temporarily rejected during estimator startup.
        if self.counter >= 10 and self.counter % 10 == 0:
            if not self.offboard:
                self.vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
                )
                self.get_logger().info("Requesting Offboard mode")
            if not self.armed:
                self.vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0
                )
                self.get_logger().info("Requesting arm")
        self.counter += 1

    def on_vehicle_status(self, msg: VehicleStatus) -> None:
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if msg.arming_state != self.last_arming_state:
            state = "ARMED" if self.armed else f"DISARMED ({msg.arming_state})"
            self.get_logger().info(f"PX4 arming state: {state}")
            self.last_arming_state = msg.arming_state
        if msg.nav_state != self.last_nav_state:
            state = "OFFBOARD" if self.offboard else str(msg.nav_state)
            self.get_logger().info(f"PX4 navigation state: {state}")
            self.last_nav_state = msg.nav_state

    def on_command_ack(self, msg: VehicleCommandAck) -> None:
        result_names = {
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: "ACCEPTED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: "TEMPORARILY_REJECTED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: "DENIED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: "UNSUPPORTED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: "FAILED",
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: "IN_PROGRESS",
            VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: "CANCELLED",
        }
        result = result_names.get(msg.result, f"UNKNOWN({msg.result})")
        self.get_logger().info(
            f"PX4 command {msg.command} acknowledgement: {result} "
            f"(reason={msg.result_param1})"
        )

    def vehicle_command(self, command: int, param1: float, param2: float = 0.0) -> None:
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
    node = OffboardHover()
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

