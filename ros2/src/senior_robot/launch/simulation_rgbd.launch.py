"""//Compatible: forward the original launch surface to senior_sim."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = FindPackageShare("senior_sim")
    robot = FindPackageShare("senior_robot")
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent_socket"),
            DeclareLaunchArgument("robot_id"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enable_rgbd_transport", default_value="true"),
            DeclareLaunchArgument(
                "rgbd_transport_profile",
                default_value=PathJoinSubstitution([robot, "config", "rgbd_transport.xml"]),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([sim, "launch", "simulation_rgbd.launch.py"])
                ),
                launch_arguments={
                    "agent_socket": LaunchConfiguration("agent_socket"),
                    "robot_id": LaunchConfiguration("robot_id"),
                    "headless": LaunchConfiguration("headless"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "enable_rgbd_transport": LaunchConfiguration("enable_rgbd_transport"),
                    "rgbd_transport_profile": LaunchConfiguration("rgbd_transport_profile"),
                }.items(),
            ),
        ]
    )
