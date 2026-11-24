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
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from senior_robot.real_runtime import load_real_parameters, validate_real_runtime

from vehicle_core.launch_recovery import restart_prefix
from vehicle_core.lifecycle_events import report_ros_exit
from vehicle_core.sensor_parameters import ros_sensor_parameters


def _nodes(context):
    parameters = LaunchConfiguration("safety_parameters").perform(context)
    hardware = LaunchConfiguration("hardware_motion_profile").perform(context)
    robot_id = LaunchConfiguration("robot_id").perform(context)
    profile = validate_real_runtime(Path(hardware), Path(parameters))
    runtime = load_real_parameters(Path(parameters), profile)
    shared = {"use_sim_time": False, **ros_sensor_parameters(profile.diagnostics)}
    selected = {**shared, "hardware_motion_profile": hardware}
    # Original standard sensor publishers are consumed through native receipts.
    nodes = [
        # Observe every child exit without terminating healthy modules.
        RegisterEventHandler(OnProcessExit(on_exit=report_ros_exit)),
        GroupAction(
            scoped=True,
            actions=[
                SetLaunchConfiguration("launch-prefix", restart_prefix()),
                # Exact executable basenames from the vendor includes. The
                # scope excludes safety, navigation and all project nodes.
                SetLaunchConfiguration(
                    "launch-prefix-filter",
                    r"^(?:wheeltec_robot_node|yesense_node_publisher|ekf_node|"
                    r"robot_state_publisher|joint_state_publisher|static_transform_publisher|"
                    r"ultrasonic_enum\.py|ultrasonic_points\.py|lslidar_driver_node|"
                    r"ldlidar|rplidar_node|pointcloud_to_laserscan_node)$",
                ),
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
            ],
        ),
        Node(
            package="senior_sensor_ingress",
            executable="scan_receipt_node",
            prefix=restart_prefix(),
            parameters=[runtime["scan_receipt"], shared],
        ),
        Node(
            package="senior_perception",
            executable="scan_adapter",
            prefix=restart_prefix(),
            parameters=[runtime["scan_adapter"], shared],
        ),
        Node(
            package="senior_perception",
            executable="safety_scan_ingress",
            prefix=restart_prefix(),
            parameters=[runtime["safety_scan_ingress"], shared],
        ),
        Node(
            package="senior_motion_estimation",
            executable="scan_motion_node",
            prefix=restart_prefix(),
            parameters=[runtime["scan_motion_node"], shared],
        ),
        Node(
            package="senior_motion_control",
            executable="safety_node",
            prefix=restart_prefix(),
            sigterm_timeout="6.0",
            sigkill_timeout="1.0",
            parameters=[runtime["safety_gate"], selected, {"robot_id": robot_id}],
        ),
    ]

    navigation = LaunchConfiguration("navigation_parameters").perform(context)
    if navigation:
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("senior_navigation"), "launch", "navigation.launch.py"]
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
