"""Explicit Fortress-only depth diagnostic composition; no coverage qualification."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = FindPackageShare("senior_robot")
    profile = PathJoinSubstitution([robot, "config", "simulation_rgbd.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent_socket"),
            DeclareLaunchArgument("headless", default_value="true"),
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
            Node(package="senior_sensor_ingress", executable="raw_receipt_node",
                 parameters=[profile]),
            Node(package="senior_robot", executable="depth_ingress", parameters=[profile]),
            Node(package="senior_robot", executable="semantic_ingress", parameters=[profile]),
        ]
    )
