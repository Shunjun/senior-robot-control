"""Explicit Fortress-only sensor, arbitration and safety composition."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = ParameterValue(LaunchConfiguration("robot_id"), value_type=str)
    sim = FindPackageShare("senior_sim")
    robot = FindPackageShare("senior_robot")
    world = PathJoinSubstitution([sim, "worlds", "baseline.sdf"])
    params = PathJoinSubstitution([robot, "config", "simulation_motion.yaml"])
    rear_mask = PathJoinSubstitution([robot, "config", "simulation_motion_rear_mask.yaml"])
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent_socket"),
            DeclareLaunchArgument("robot_id"),
            DeclareLaunchArgument("safety_state_path"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("qualified_reference", default_value="false"),
            DeclareLaunchArgument("safety_overlay", default_value=params),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([sim, "launch", "simulation.launch.py"])
                ),
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "world": world,
                    "use_sim_time": "true",
                }.items(),
            ),
            Node(package="senior_robot", executable="scan_adapter", parameters=[params, rear_mask]),
            Node(package="senior_scan_matcher", executable="scan_motion_node", parameters=[params]),
            Node(package="senior_robot", executable="motion_fusion", parameters=[params]),
            Node(
                package="senior_sensor_ingress", executable="scan_receipt_node", parameters=[params]
            ),
            Node(package="senior_robot", executable="safety_scan_ingress", parameters=[params]),
            Node(
                package="senior_robot",
                executable="safety_node",
                parameters=[
                    params,
                    LaunchConfiguration("safety_overlay"),
                    rear_mask,
                    {"robot_id": robot_id},
                ],
            ),
            Node(
                package="senior_robot",
                executable="agent_adapter",
                name="command_node",
                parameters=[
                    params,
                    {
                        "robot_id": robot_id,
                        "safety_state_path": LaunchConfiguration("safety_state_path"),
                        "use_sim_time": True,
                        "simulation_baseline_world": ParameterValue(world, value_type=str),
                        "simulation_qualified_reference": ParameterValue(
                            LaunchConfiguration("qualified_reference"), value_type=bool
                        ),
                    },
                ],
                additional_env={"SENIOR_AGENT_SOCKET": LaunchConfiguration("agent_socket")},
            ),
        ]
    )
