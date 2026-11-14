"""Hardware safety processes; the supplied profile must be measured and calibrated."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    parameters = LaunchConfiguration("safety_parameters")
    return LaunchDescription(
        [
            DeclareLaunchArgument("safety_parameters"),
            DeclareLaunchArgument("robot_id"),
            DeclareLaunchArgument("motion_core_profile", default_value=""),
            Node(
                package="senior_sensor_ingress",
                executable="scan_receipt_node",
                parameters=[parameters],
            ),
            Node(package="senior_robot", executable="safety_scan_ingress", parameters=[parameters]),
            Node(package="senior_robot", executable="depth_ingress", parameters=[parameters]),
            Node(
                package="senior_robot",
                executable="safety_node",
                parameters=[
                    parameters,
                    {
                        "motion_core_profile": ParameterValue(
                            LaunchConfiguration("motion_core_profile"), value_type=str
                        )
                    },
                    {"robot_id": ParameterValue(LaunchConfiguration("robot_id"), value_type=str)},
                ],
            ),
        ]
    )
