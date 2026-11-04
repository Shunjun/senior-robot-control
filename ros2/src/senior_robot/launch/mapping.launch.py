"""Pose-graph mapping. This launch does not start a simulator and publishes no velocity.

slam_toolbox publishes map to the single odom frame. The adapter keeps the
long-term occupancy layer beside that pose graph.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("senior_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = PathJoinSubstitution([share, "config", "slam.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[params, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="senior_robot",
                executable="slam_adapter",
                name="slam_adapter",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "odom_frame": "odom"}],
            ),
        ]
    )
