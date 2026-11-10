"""Launch PX4 SITL, Gazebo, the XRCE agent, and optionally offboard control."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("x650_sim"))
    default_px4 = str(Path.home() / "PX4-Autopilot")
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    resource_path = os.pathsep.join(
        filter(None, [str(share / "models"), str(share / "worlds"), existing_gz_path])
    )
    world_file = str(share / "worlds" / "x650_world.sdf")

    px4_path = LaunchConfiguration("px4_path")
    headless = LaunchConfiguration("headless")
    start_agent = LaunchConfiguration("start_agent")
    start_offboard = LaunchConfiguration("start_offboard")
    start_manipulator = LaunchConfiguration("start_manipulator")
    start_camera_bridge = LaunchConfiguration("start_camera_bridge")
    start_gazebo = LaunchConfiguration("start_gazebo")
    spawn_pose = LaunchConfiguration("spawn_pose")
    manipulator_stiffness_x = LaunchConfiguration("manipulator_stiffness_x")
    manipulator_stiffness_z = LaunchConfiguration("manipulator_stiffness_z")
    manipulator_damping_x = LaunchConfiguration("manipulator_damping_x")
    manipulator_damping_z = LaunchConfiguration("manipulator_damping_z")
    manipulator_stabilize_forward = LaunchConfiguration(
        "manipulator_stabilize_forward"
    )

    px4 = ExecuteProcess(
        cmd=[PathJoinSubstitution([px4_path, "build", "px4_sitl_default", "bin", "px4"])],
        cwd=px4_path,
        output="screen",
        additional_env={
            "PX4_SYS_AUTOSTART": "4001",
            "PX4_SIM_MODEL": "gz_x650",
            "PX4_GZ_WORLD": "x650_world",
            "PX4_GZ_STANDALONE": "1",
            "PX4_GZ_MODEL_POSE": spawn_pose,
            # Restore the proven gz_x500 allocation baseline. PX4 uses FRD
            # body coordinates, while Gazebo model poses are FLU; directly
            # copying SDF Y coordinates here reverses roll feedback.
            "PX4_PARAM_CA_ROTOR0_PX": "0.13",
            "PX4_PARAM_CA_ROTOR0_PY": "0.22",
            "PX4_PARAM_CA_ROTOR1_PX": "-0.13",
            "PX4_PARAM_CA_ROTOR1_PY": "-0.20",
            "PX4_PARAM_CA_ROTOR2_PX": "0.13",
            "PX4_PARAM_CA_ROTOR2_PY": "-0.22",
            "PX4_PARAM_CA_ROTOR3_PX": "-0.13",
            "PX4_PARAM_CA_ROTOR3_PY": "0.20",
            # Explicitly restore PX4 v1.17 multicopter defaults. These are
            # startup overrides because the earlier experimental values may
            # already be persisted in parameters.bson.
            "PX4_PARAM_MC_ROLL_P": "4.0",
            "PX4_PARAM_MC_PITCH_P": "4.0",
            "PX4_PARAM_MC_ROLLRATE_P": "0.15",
            "PX4_PARAM_MC_ROLLRATE_I": "0.20",
            "PX4_PARAM_MC_ROLLRATE_D": "0.003",
            "PX4_PARAM_MC_PITCHRATE_P": "0.15",
            "PX4_PARAM_MC_PITCHRATE_I": "0.20",
            "PX4_PARAM_MC_PITCHRATE_D": "0.003",
            "PX4_PARAM_MPC_XY_P": "0.95",
            "PX4_PARAM_MPC_XY_VEL_P_ACC": "1.80",
            "PX4_PARAM_MPC_XY_VEL_I_ACC": "0.40",
            "PX4_PARAM_MPC_XY_VEL_D_ACC": "0.20",
            "PX4_PARAM_MPC_Z_P": "1.00",
            "PX4_PARAM_MPC_Z_VEL_P_ACC": "4.00",
            "PX4_PARAM_MPC_Z_VEL_I_ACC": "2.00",
            "PX4_PARAM_MPC_Z_VEL_D_ACC": "0.00",
            "PX4_PARAM_MPC_ACC_HOR": "3.0",
            "PX4_PARAM_MPC_ACC_HOR_MAX": "5.0",
            "PX4_PARAM_MPC_ACC_UP_MAX": "4.0",
            "PX4_PARAM_MPC_ACC_DOWN_MAX": "3.0",
            "PX4_PARAM_MPC_TILTMAX_AIR": "45.0",
            "HEADLESS": PythonExpression(
                ["'1' if '", headless, "'.lower() == 'true' else ''"]
            ),
            "GZ_SIM_RESOURCE_PATH": resource_path,
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        },
    )

    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_file],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", start_gazebo, "'.lower() == 'true' and '", headless, "'.lower() != 'true'"]
            )
        ),
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", world_file],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", start_gazebo, "'.lower() == 'true' and '", headless, "'.lower() == 'true'"]
            )
        ),
    )

    agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        condition=IfCondition(start_agent),
    )

    offboard = Node(
        package="x650_sim",
        executable="offboard_hover",
        name="offboard_hover",
        output="screen",
        condition=IfCondition(start_offboard),
    )
    manipulator = Node(
        package="x650_sim",
        executable="manipulator_controller",
        name="manipulator_controller",
        output="screen",
        parameters=[
            {
                "mode": "sim",
                "stabilize_forward": ParameterValue(
                    manipulator_stabilize_forward, value_type=bool
                ),
                "stiffness_x": ParameterValue(
                    manipulator_stiffness_x, value_type=float
                ),
                "stiffness_z": ParameterValue(
                    manipulator_stiffness_z, value_type=float
                ),
                "damping_x": ParameterValue(manipulator_damping_x, value_type=float),
                "damping_z": ParameterValue(manipulator_damping_z, value_type=float),
            }
        ],
        condition=IfCondition(start_manipulator),
    )
    camera_bridge = Node(
        package="x650_sim",
        executable="camera_bridge",
        name="camera_bridge",
        output="screen",
        condition=IfCondition(start_camera_bridge),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_path",
                default_value=EnvironmentVariable("PX4_AUTOPILOT_PATH", default_value=default_px4),
                description="PX4-Autopilot source directory with a px4_sitl_default build",
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("start_agent", default_value="true"),
            DeclareLaunchArgument("start_offboard", default_value="false"),
            DeclareLaunchArgument("start_manipulator", default_value="true"),
            DeclareLaunchArgument("start_camera_bridge", default_value="true"),
            DeclareLaunchArgument("manipulator_stiffness_x", default_value="40.0"),
            DeclareLaunchArgument("manipulator_stiffness_z", default_value="40.0"),
            DeclareLaunchArgument("manipulator_damping_x", default_value="5.0"),
            DeclareLaunchArgument("manipulator_damping_z", default_value="5.0"),
            DeclareLaunchArgument(
                "manipulator_stabilize_forward", default_value="true"
            ),
            DeclareLaunchArgument(
                "start_gazebo",
                default_value="true",
                description="Start Gazebo; disable when a host Gazebo instance is already running",
            ),
            DeclareLaunchArgument("spawn_pose", default_value="0,0,0.25,0,0,0"),
            SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            LogInfo(msg=["X650 Gazebo resources: ", str(share)]),
            agent,
            gazebo_gui,
            gazebo_headless,
            px4,
            manipulator,
            camera_bridge,
            offboard,
        ]
    )

