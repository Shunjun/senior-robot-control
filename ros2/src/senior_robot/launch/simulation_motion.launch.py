"""//Compatible: forward the original launch surface to senior_sim."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = FindPackageShare("senior_sim")
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent_socket"),
            DeclareLaunchArgument("robot_id"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("navigation_enabled", default_value="false"),
            DeclareLaunchArgument("navigation_reference", default_value=""),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution([sim, "worlds", "motion_qualification.sdf"]),
            ),
            DeclareLaunchArgument(
                "hardware_motion_profile",
                default_value=PathJoinSubstitution([sim, "config", "motion_profile.json"]),
            ),
            DeclareLaunchArgument("video_enabled", default_value="false"),
            DeclareLaunchArgument("video_frame_path", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([sim, "launch", "simulation_motion.launch.py"])
                ),
                launch_arguments={
                    "agent_socket": LaunchConfiguration("agent_socket"),
                    "robot_id": LaunchConfiguration("robot_id"),
                    "headless": LaunchConfiguration("headless"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "navigation_enabled": LaunchConfiguration("navigation_enabled"),
                    "navigation_reference": LaunchConfiguration("navigation_reference"),
                    "map_yaml": LaunchConfiguration("map_yaml"),
                    "world": LaunchConfiguration("world"),
                    "hardware_motion_profile": LaunchConfiguration("hardware_motion_profile"),
                    "video_enabled": LaunchConfiguration("video_enabled"),
                    "video_frame_path": LaunchConfiguration("video_frame_path"),
                }.items(),
            ),
        ]
    )
