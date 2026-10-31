"""Explicit Fortress-only sensor, arbitration and safety composition."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = FindPackageShare("senior_sim")
    robot = FindPackageShare("senior_robot")
    params = PathJoinSubstitution([robot, "config", "simulation_motion.yaml"])
    rear_mask = PathJoinSubstitution([robot, "config", "simulation_motion_rear_mask.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent_socket"),
            DeclareLaunchArgument("headless", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([sim, "launch", "simulation.launch.py"])
                ),
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(package="senior_robot", executable="scan_adapter", parameters=[params, rear_mask]),
            Node(package="senior_scan_matcher", executable="scan_motion_node", parameters=[params]),
            Node(package="senior_robot", executable="motion_fusion", parameters=[params]),
            Node(package="senior_robot", executable="safety_node", parameters=[params, rear_mask]),
            Node(
                package="senior_robot",
                executable="agent_adapter",
                name="command_node",
                parameters=[params],
                additional_env={"SENIOR_AGENT_SOCKET": LaunchConfiguration("agent_socket")},
            ),
        ]
    )
