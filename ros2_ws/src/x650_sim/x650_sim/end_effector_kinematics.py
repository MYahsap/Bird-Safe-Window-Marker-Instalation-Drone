"""Pure kinematics for coordinated X650 end-effector pose control."""

import math
from typing import Sequence


def tool_direction(quaternion: Sequence[float]) -> tuple[float, float, float]:
    """Return the world direction of the tool's local +X axis."""
    if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
        raise ValueError("orientation must contain four finite values")
    x, y, z, w = (float(v) for v in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-6:
        raise ValueError("orientation quaternion has zero length")
    x, y, z, w = (v / norm for v in (x, y, z, w))
    return (
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y + w * z),
        2.0 * (x * z - w * y),
    )


def solve_constraint(
    target_position: Sequence[float],
    direction: Sequence[float],
    pivot_x: float,
    pivot_z: float,
    arm_length: float,
    vehicle_pitch: float = 0.0,
) -> tuple[tuple[float, float, float], float, float, float]:
    """Solve base position, joint angle, yaw, and elevation for a tool pose.

    Positions and directions use the Gazebo world convention (X forward/North,
    Y left/West, Z up). The returned joint angle uses the model's revolute-joint
    convention. Vehicle pitch compensation matches manipulator_controller.py.
    """
    if len(target_position) != 3 or len(direction) != 3:
        raise ValueError("position and direction must contain three values")
    values = tuple(float(v) for v in (*target_position, *direction))
    if not all(math.isfinite(v) for v in values):
        raise ValueError("position and direction must be finite")
    dx, dy, dz = (float(v) for v in direction)
    direction_norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    if direction_norm < 1e-6:
        raise ValueError("tool direction has zero length")
    dx, dy, dz = (v / direction_norm for v in (dx, dy, dz))
    horizontal = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    elevation = math.atan2(dz, horizontal)

    # q=-pi/2 is horizontal/forward. Positive vehicle pitch is added so the
    # arm retains the requested world elevation while the airframe tilts.
    joint_angle = -elevation - math.pi / 2.0 + vehicle_pitch

    base_position, _ = base_for_configuration(
        target_position,
        yaw,
        vehicle_pitch,
        joint_angle,
        pivot_x,
        pivot_z,
        arm_length,
    )
    return base_position, joint_angle, yaw, elevation


def base_for_configuration(
    target_position: Sequence[float],
    vehicle_yaw: float,
    vehicle_pitch: float,
    joint_angle: float,
    pivot_x: float,
    pivot_z: float,
    arm_length: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return the base pose that holds the tip fixed for measured geometry."""
    values = (
        *target_position,
        vehicle_yaw,
        vehicle_pitch,
        joint_angle,
        pivot_x,
        pivot_z,
        arm_length,
    )
    if len(target_position) != 3 or not all(math.isfinite(float(v)) for v in values):
        raise ValueError("configuration values must be finite")
    if arm_length <= 0.0:
        raise ValueError("arm_length must be positive")

    # The model's q=-pi/2 points along body +X. Vehicle pitch rotates that
    # direction in the world while increasing q rotates it back down.
    elevation = vehicle_pitch - joint_angle - math.pi / 2.0
    cos_elevation = math.cos(elevation)
    link_direction = (
        math.cos(vehicle_yaw) * cos_elevation,
        math.sin(vehicle_yaw) * cos_elevation,
        math.sin(elevation),
    )

    # Rotate the off-axis pivot through current pitch, then current yaw.
    pitch_cos = math.cos(vehicle_pitch)
    pitch_sin = math.sin(vehicle_pitch)
    pivot_forward = pitch_cos * pivot_x - pitch_sin * pivot_z
    pivot_up = pitch_sin * pivot_x + pitch_cos * pivot_z
    pivot_world_x = math.cos(vehicle_yaw) * pivot_forward
    pivot_world_y = math.sin(vehicle_yaw) * pivot_forward

    tx, ty, tz = (float(v) for v in target_position)
    base_position = (
        tx - pivot_world_x - arm_length * link_direction[0],
        ty - pivot_world_y - arm_length * link_direction[1],
        tz - pivot_up - arm_length * link_direction[2],
    )
    return base_position, link_direction
