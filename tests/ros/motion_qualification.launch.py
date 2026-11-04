"""Test-owned setup: no rear attachment, no agent and no production bypass."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = FindPackageShare("senior_sim")
    params = PathJoinSubstitution(
        [FindPackageShare("senior_robot"), "config", "simulation_motion.yaml"]
    )
    rear_mask = PathJoinSubstitution(
        [FindPackageShare("senior_robot"), "config", "simulation_motion_rear_mask.yaml"]
    )
    rear = LaunchConfiguration("rear_attachment")
    relay = LaunchConfiguration("scan_relay")
    world = LaunchConfiguration("world")
    safety_overlay = LaunchConfiguration("safety_overlay")
    fixed_stand = LaunchConfiguration("fixed_stand")

    def simulator():
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sim, "launch", "simulation.launch.py"])
            ),
            launch_arguments={
                "world": world,
                "headless": "true",
                "use_sim_time": "true",
                "rear_attachment": rear,
            }.items(),
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rear_attachment", default_value="false"),
            DeclareLaunchArgument("scan_relay", default_value="false"),
            DeclareLaunchArgument("fixed_stand", default_value="false"),
            DeclareLaunchArgument("safety_overlay", default_value=params),
            DeclareLaunchArgument(
                "world", default_value=str(Path(__file__).with_name("motion_qualification.sdf"))
            ),
            GroupAction(condition=UnlessCondition(relay), actions=[simulator()]),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": str(
                            Path(__file__).with_name("motion_fixed_stand_bridge.yaml")
                        ),
                        "use_sim_time": True,
                    }
                ],
                condition=IfCondition(fixed_stand),
            ),
            GroupAction(
                condition=IfCondition(relay),
                actions=[
                    SetRemap(src="/scan", dst="/fault_test/source_scan"),
                    simulator(),
                ],
            ),
            Node(
                package="senior_robot",
                executable="scan_adapter",
                parameters=[params],
                condition=UnlessCondition(rear),
            ),
            Node(
                package="senior_robot",
                executable="scan_adapter",
                parameters=[params, rear_mask],
                condition=IfCondition(rear),
            ),
            Node(package="senior_scan_matcher", executable="scan_motion_node", parameters=[params]),
            Node(package="senior_robot", executable="motion_fusion", parameters=[params]),
            Node(package="senior_sensor_ingress", executable="scan_receipt_node",
                 parameters=[params]),
            Node(package="senior_robot", executable="safety_scan_ingress", parameters=[params]),
            Node(
                package="senior_robot",
                executable="safety_node",
                parameters=[params, safety_overlay],
                condition=UnlessCondition(rear),
            ),
            Node(
                package="senior_robot",
                executable="safety_node",
                parameters=[params, safety_overlay, rear_mask],
                condition=IfCondition(rear),
            ),
        ]
    )
