"""Shared hardware motion stack; vendor drivers and mapping/localization are external.

The supervisor starts exactly one agent_adapter/CommandNode separately, preserving
its Unix peer identity. This launch owns the sole safety_gate /cmd_vel output.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from senior_control.real_runtime import validate_real_runtime


def _nodes(context):
    parameters = LaunchConfiguration("safety_parameters").perform(context)
    hardware = LaunchConfiguration("hardware_motion_profile").perform(context)
    robot_id = LaunchConfiguration("robot_id").perform(context)
    profile = validate_real_runtime(Path(hardware), Path(parameters))
    shared = {"use_sim_time": False}
    selected = {**shared, "hardware_motion_profile": hardware, "motion_core_profile": ""}
    # Driver topics from the WHEELTEC source: odom and imu/data_raw. Preserve
    # the original DDS publisher identity; a relay would hide the real producer.
    remaps = [("/wheel/odom", profile.wheel_topic), ("/imu/data", profile.imu_topic)]
    stop = [EmitEvent(event=Shutdown(reason="required hardware motion process exited"))]
    nodes = [
        Node(
            package="senior_sensor_ingress",
            executable="scan_receipt_node",
            parameters=[parameters, shared],
            on_exit=stop,
        ),
        Node(
            package="senior_robot",
            executable="scan_adapter",
            parameters=[parameters, shared],
            on_exit=stop,
        ),
        Node(
            package="senior_robot",
            executable="safety_scan_ingress",
            parameters=[parameters, shared],
            on_exit=stop,
        ),
        Node(
            package="senior_scan_matcher",
            executable="scan_motion_node",
            parameters=[parameters, shared],
            on_exit=stop,
        ),
        Node(
            package="senior_robot",
            executable="motion_fusion",
            parameters=[parameters, selected],
            remappings=remaps,
            on_exit=stop,
        ),
        Node(
            package="senior_robot",
            executable="safety_node",
            parameters=[parameters, selected, {"robot_id": robot_id}],
            on_exit=stop,
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
