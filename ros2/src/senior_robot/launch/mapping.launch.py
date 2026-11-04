"""Pose-graph mapping. This launch does not start a simulator and publishes no velocity.

The default uses stock slam_toolbox and its adapter. A nonempty
capture_directory instead selects the optional senior_slam_bridge package
and exposes native pose-graph capture. The verified static lidar mode
requires the matching bridge consumer and an existing scan receipt process.
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
    verified_lidar = LaunchConfiguration("verified_static_lidar").perform(context) == "true"
    if capture_directory and not Path(capture_directory).is_absolute():
        raise ValueError("capture_directory must be absolute")
    if capture_directory or verified_lidar:
        bridge_parameters = {
            "use_sim_time": use_sim_time,
            "capture_directory": capture_directory,
            "use_map_saver": False,
            "enable_interactive_mode": False,
        }
        nodes = []
        if verified_lidar:
            bounded_scan = LaunchConfiguration("bounded_scan_topic").perform(context)
            lidar_frame = LaunchConfiguration("lidar_frame").perform(context)
            if not bounded_scan.startswith("/") or bounded_scan in ("/", "/scan"):
                raise ValueError("bounded_scan_topic must be an absolute relay topic")
            if (
                not lidar_frame
                or lidar_frame == "base_link"
                or len(lidar_frame) > 128
                or any(not 33 <= ord(character) <= 126 for character in lidar_frame)
            ):
                raise ValueError("lidar_frame must name a distinct valid sensor frame")

            relay_name = "slam_static_lidar_tf_receipt"
            receipt_topic = "/senior/slam/static_lidar_tf_receipt"
            raw_tf_topic = "/tf_static"
            raw_tf_node = "robot_state_publisher"
            nodes.append(
                Node(
                    package="senior_sensor_ingress",
                    executable="static_tf_receipt_node",
                    name=relay_name,
                    namespace="/",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "input_topic": raw_tf_topic,
                            "output_topic": receipt_topic,
                            "base_frame": "base_link",
                            # //Compatible: the relay's target parameter also supports lidar.
                            "camera_frame": lidar_frame,
                            "expected_node_name": raw_tf_node,
                            "expected_namespace": "/",
                        }
                    ],
                )
            )
            bridge_parameters.update(
                {
                    "bounded_scan_topic": bounded_scan,
                    "verified_static_lidar_tf_topic": receipt_topic,
                    "verified_static_lidar_frame": lidar_frame,
                    "verified_static_lidar_relay_node": f"/{relay_name}",
                    "verified_static_lidar_tf_source_topic": raw_tf_topic,
                    "verified_static_lidar_tf_source_node": f"/{raw_tf_node}",
                }
            )
        nodes.append(
            Node(
                package="senior_slam_bridge",
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
