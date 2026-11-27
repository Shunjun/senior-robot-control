"""Pose-graph mapping. This launch does not start a simulator and publishes no velocity.

The default uses stock slam_toolbox and its adapter. A nonempty
capture_directory instead selects the native senior_mapping runtime
and exposes native pose-graph capture. Bounded lidar input uses the existing
scan receipt process and standard TF.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from vehicle_core.launch_recovery import restart_prefix
from vehicle_core.lifecycle_events import resource_handlers


def _mapping_actions(context):
    share = FindPackageShare("senior_mapping")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = PathJoinSubstitution([share, "config", "slam.yaml"])
    odom_frame = LaunchConfiguration("odom_frame").perform(context)
    if not odom_frame or odom_frame == "map":
        raise ValueError("odom_frame must name a valid frame distinct from map")
    capture_directory = LaunchConfiguration("capture_directory").perform(context)
    verified_lidar = LaunchConfiguration("verified_static_lidar").perform(context) == "true"
    if capture_directory and not Path(capture_directory).is_absolute():
        raise ValueError("capture_directory must be absolute")
    if capture_directory or verified_lidar:
        bridge_parameters = {
            "use_sim_time": use_sim_time,
            "capture_directory": capture_directory,
            "odom_frame": odom_frame,
            "use_map_saver": False,
            "enable_interactive_mode": False,
        }
        nodes = []
        if verified_lidar:
            bounded_scan = LaunchConfiguration("bounded_scan_topic").perform(context)
            lidar_frame = LaunchConfiguration("lidar_frame").perform(context)
            if not bounded_scan.startswith("/") or bounded_scan in ("/", "/scan"):
                raise ValueError("bounded_scan_topic must be an absolute relay topic")
            if not lidar_frame or lidar_frame == "base_link":
                raise ValueError("lidar_frame must name a distinct sensor frame")
            bridge_parameters.update(
                {"bounded_scan_topic": bounded_scan, "lidar_frame": lidar_frame}
            )
        nodes.append(
            Node(
                package="senior_mapping",
                executable="slam_graph_node",
                name="slam_toolbox",
                output="screen",
                parameters=[params, bridge_parameters],
            )
        )
        return nodes
    return [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            prefix=restart_prefix(),
            name="slam_toolbox",
            output="screen",
            parameters=[params, {"use_sim_time": use_sim_time, "odom_frame": odom_frame}],
        ),
        Node(
            package="senior_mapping",
            executable="slam_adapter",
            prefix=restart_prefix(),
            name="slam_adapter",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "odom_frame": odom_frame}],
        ),
    ]


def _mapping_nodes(context):
    nodes = _mapping_actions(context)
    return [*resource_handlers("mapping", nodes), *nodes]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Single odometry frame shared by the selected graph owner and adapter.",
            ),
            DeclareLaunchArgument(
                "capture_directory",
                default_value="",
                description=(
                    "Enable native graph capture with an empty absolute canonical directory "
                    "owned by the node user and mode 0700. Requires native senior_mapping."
                ),
            ),
            DeclareLaunchArgument(
                "verified_static_lidar",
                default_value="false",
                choices=["true", "false"],
                description="Require the dedicated verified static lidar mount in bounded mapping.",
            ),
            DeclareLaunchArgument(
                "bounded_scan_topic",
                default_value="/safety/raw/scan",
                description="Existing scan_receipt output, used only in verified lidar mode.",
            ),
            DeclareLaunchArgument(
                "lidar_frame",
                default_value="laser_link",
                description="Expected fixed lidar target, used only in verified lidar mode.",
            ),
            OpaqueFunction(function=_mapping_nodes),
        ]
    )
