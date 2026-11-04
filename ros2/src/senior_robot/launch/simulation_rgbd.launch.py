"""Explicit Fortress-only depth diagnostic composition; no coverage qualification."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from senior_robot.rgbd_transport import transport_environment


def _detect_rmw():
    from rclpy.utilities import get_rmw_implementation_identifier

    return get_rmw_implementation_identifier()


def _transport_actions(context):
    enabled = LaunchConfiguration("enable_rgbd_transport").perform(context).lower()
    if enabled not in {"true", "false"}:
        raise ValueError("enable_rgbd_transport must be true or false")
    profile = LaunchConfiguration("rgbd_transport_profile").perform(context)
    additions = transport_environment(
        context.environment, enabled == "true", profile, os.getcwd(), _detect_rmw
    )
    return [SetEnvironmentVariable(name, value) for name, value in additions.items()]


def generate_launch_description():
    robot = FindPackageShare("senior_robot")
    profile = PathJoinSubstitution([robot, "config", "simulation_rgbd.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent_socket"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("enable_rgbd_transport", default_value="true"),
            DeclareLaunchArgument(
                "rgbd_transport_profile",
                default_value=PathJoinSubstitution([robot, "config", "rgbd_transport.xml"]),
            ),
            GroupAction(
                scoped=True,
                actions=[
                    OpaqueFunction(function=_transport_actions),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([robot, "launch", "simulation_motion.launch.py"])
                        ),
                        launch_arguments={
                            "agent_socket": LaunchConfiguration("agent_socket"),
                            "headless": LaunchConfiguration("headless"),
                            "safety_overlay": profile,
                        }.items(),
                    ),
                    Node(
                        package="senior_sensor_ingress",
                        executable="raw_receipt_node",
                        parameters=[profile],
                    ),
                    Node(package="senior_robot", executable="depth_ingress", parameters=[profile]),
                    Node(
                        package="senior_robot", executable="semantic_ingress", parameters=[profile]
                    ),
                ],
            ),
        ]
    )
