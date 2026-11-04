"""Pose-graph mapping. This launch does not start a simulator and publishes no velocity.

The default uses stock slam_toolbox and its adapter. A nonempty
capture_directory instead selects the optional senior_slam_bridge package
and exposes native pose-graph capture.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _mapping_nodes(context):
    share = FindPackageShare("senior_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = PathJoinSubstitution([share, "config", "slam.yaml"])
    capture_directory = LaunchConfiguration("capture_directory").perform(context)
    if capture_directory:
        if not Path(capture_directory).is_absolute():
            raise ValueError("capture_directory must be absolute")
        return [
            Node(
                package="senior_slam_bridge",
                executable="slam_graph_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    params,
                    {
                        "use_sim_time": use_sim_time,
                        "capture_directory": capture_directory,
                        "use_map_saver": False,
                        "enable_interactive_mode": False,
                    },
                ],
            )
        ]
    return [
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


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "capture_directory",
                default_value="",
                description=(
                    "Enable native graph capture with an empty absolute canonical directory "
                    "owned by the node user and mode 0700. Requires senior_slam_bridge."
                ),
            ),
            OpaqueFunction(function=_mapping_nodes),
        ]
    )
