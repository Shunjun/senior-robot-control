"""Opt-in SIM diagnostic chain; no motion qualification or safety input."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    remappings = [
        ("/rgbd_diagnostic/" + stream, "/safety/raw/" + stream)
        for stream in ("rgb", "depth", "rgb_info", "depth_info")
    ]
    common = {
        "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
        "camera_frame": LaunchConfiguration("camera_frame"),
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
            DeclareLaunchArgument("static_background_declared", default_value="false"),
            Node(
                package="senior_sensor_ingress",
                executable="static_tf_receipt_node",
                parameters=[common, {"output_topic": "/rgbd_diagnostic/tf"}],
            ),
            Node(
                package="senior_sensor_ingress",
                executable="rgbd_candidate_receipt_node",
                parameters=[common],
                remappings=remappings,
            ),
            Node(
                package="senior_robot",
                executable="rgbd_candidate_producer",
                parameters=[
                    common,
                    {
                        "static_background_declared": ParameterValue(
                            LaunchConfiguration("static_background_declared"), value_type=bool
                        )
                    },
                ],
                remappings=remappings,
            ),
        ]
    )
