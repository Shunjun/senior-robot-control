"""//Compatible: original navigation launch arguments forward to the canonical owner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot = FindPackageShare("senior_robot")
    navigation = FindPackageShare("senior_navigation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "parameters", default_value=PathJoinSubstitution([robot, "config", "nav2.yaml"])
            ),
            DeclareLaunchArgument("external_controller", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([navigation, "launch", "navigation.launch.py"])
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "parameters": LaunchConfiguration("parameters"),
                    "external_controller": LaunchConfiguration("external_controller"),
                }.items(),
            ),
        ]
    )
