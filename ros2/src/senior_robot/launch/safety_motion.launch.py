"""WHEELTEC drivers and shared hardware motion stack; mapping remains external.

The supervisor starts exactly one agent_adapter/CommandNode separately, preserving
its Unix peer identity. This launch owns the safety_gate decision owner and
its private native /cmd_vel transport.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from senior_control.real_runtime import load_real_parameters, validate_real_runtime
from senior_control.sensors.policy import ros_sensor_parameters


def _nodes(context):
    parameters = LaunchConfiguration("safety_parameters").perform(context)
    hardware = LaunchConfiguration("hardware_motion_profile").perform(context)
    robot_id = LaunchConfiguration("robot_id").perform(context)
    profile = validate_real_runtime(Path(hardware), Path(parameters))
    runtime = load_real_parameters(Path(parameters), profile)
    shared = {"use_sim_time": False, **ros_sensor_parameters(profile.diagnostics)}
    selected = {**shared, "hardware_motion_profile": hardware, "motion_core_profile": ""}
    # Driver topics from the WHEELTEC source: odom and imu/data_raw. Preserve
    # the original DDS publisher identity; a relay would hide the real producer.
    remaps = [("/wheel/odom", profile.wheel_topic), ("/imu/data", profile.imu_topic)]
    shutdown_requested = False

    def stop_required(event, launch_context):
        nonlocal shutdown_requested
        if shutdown_requested or launch_context.is_shutdown:
            return []
        shutdown_requested = True
        return [EmitEvent(event=Shutdown(reason="required hardware motion process exited"))]

    nodes = [
        # All included hardware children belong to this launch process group.
        # A required driver exit shuts down the shared stack as well.
        RegisterEventHandler(OnProcessExit(on_exit=stop_required)),
        *[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        Path(get_package_share_directory("turn_on_wheeltec_robot"))
                        / "launch"
                        / filename
                    )
                ),
            )
            for filename in (
                "turn_on_wheeltec_robot.launch.py",
                "wheeltec_lidar.launch.py",
            )
        ],
        Node(
            package="senior_sensor_ingress",
            executable="scan_receipt_node",
            parameters=[runtime["scan_receipt"], shared],
        ),
        Node(
            package="senior_robot",
            executable="scan_adapter",
            parameters=[runtime["scan_adapter"], shared],
        ),
        Node(
            package="senior_robot",
            executable="safety_scan_ingress",
            parameters=[runtime["safety_scan_ingress"], shared],
        ),
        Node(
            package="senior_scan_matcher",
            executable="scan_motion_node",
            parameters=[runtime["scan_motion_node"], shared],
        ),
        Node(
            package="senior_robot",
            executable="safety_node",
            parameters=[runtime["safety_gate"], selected, {"robot_id": robot_id}],
            remappings=remaps,
        ),
    ]

    navigation = LaunchConfiguration("navigation_parameters").perform(context)
    if navigation:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("senior_robot"), "launch", "navigation.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": "false",
                    "external_controller": "true",
                    "parameters": navigation,
                }.items(),
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("safety_parameters"),
            DeclareLaunchArgument("hardware_motion_profile"),
            DeclareLaunchArgument("robot_id"),
            DeclareLaunchArgument("navigation_parameters", default_value=""),
            OpaqueFunction(function=_nodes),
        ]
    )
