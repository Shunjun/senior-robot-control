"""Explicit Fortress-only sensor, arbitration and safety composition."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    robot_id = ParameterValue(LaunchConfiguration("robot_id"), value_type=str)
    sim = FindPackageShare("senior_sim")
    robot = FindPackageShare("senior_robot")
    world = LaunchConfiguration("motion_world")
    params = PathJoinSubstitution([robot, "config", "simulation_motion.yaml"])
    rear_mask = LaunchConfiguration("motion_attachment_profile")
    qualified = LaunchConfiguration("qualified_reference")

    def profile(context):
        use_reference = IfCondition(qualified).evaluate(context)
        return [
            SetLaunchConfiguration(
                "motion_world",
                PathJoinSubstitution(
                    [sim, "worlds", "motion_qualification.sdf" if use_reference else "baseline.sdf"]
                ),
            ),
            SetLaunchConfiguration("motion_rear_attachment", "false" if use_reference else "true"),
            # A new scan invalidates the previous worker ticket. Leave a full
            # evidence evaluation interval on this VM without extending freshness.
            SetLaunchConfiguration("motion_lidar_rate", "5" if use_reference else "10"),
            SetLaunchConfiguration("motion_gate_rate", "40.0" if use_reference else "20.0"),
            SetLaunchConfiguration(
                "motion_attachment_profile",
                PathJoinSubstitution(
                    [
                        robot,
                        "config",
                        "simulation_motion.yaml"
                        if use_reference
                        else "simulation_motion_rear_mask.yaml",
                    ]
                ),
            ),
            SetLaunchConfiguration(
                "default_safety_overlay",
                PathJoinSubstitution(
                    [
                        robot,
                        "config",
                        "simulation_motion_flat_lateral.yaml"
                        if use_reference
                        else "simulation_motion.yaml",
                    ]
                ),
            ),
        ]

    controller_params = RewrittenYaml(
        source_file=PathJoinSubstitution([robot, "config", "nav2.yaml"]),
        root_key="",
        param_rewrites={
            "use_sim_time": "true",
            # DDS request/response on the shared VM can exceed one BT tick.
            "bt_navigator.ros__parameters.default_server_timeout": "250",
            # Leave VM CPU for the scan and motion workers while MPPI is active.
            "controller_server.ros__parameters.FollowPath.batch_size": "400",
            "controller_server.ros__parameters.FollowPath.vx_max": "0.22",
            "controller_server.ros__parameters.FollowPath.vx_min": "-0.15",
            "controller_server.ros__parameters.FollowPath.wz_max": "0.4",
            "default_nav_to_pose_bt_xml": PathJoinSubstitution(
                [robot, "behavior_trees", "navigate.xml"]
            ),
            "default_nav_through_poses_bt_xml": PathJoinSubstitution(
                [robot, "behavior_trees", "navigate_through_poses.xml"]
            ),
        },
        convert_types=True,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent_socket"),
            DeclareLaunchArgument("robot_id"),
            DeclareLaunchArgument("safety_state_path"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("qualified_reference", default_value="false"),
            OpaqueFunction(function=profile),
            DeclareLaunchArgument(
                "safety_overlay", default_value=LaunchConfiguration("default_safety_overlay")
            ),
            DeclareLaunchArgument("video_enabled", default_value="false"),
            DeclareLaunchArgument("video_frame_path", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([sim, "launch", "simulation.launch.py"])
                ),
                launch_arguments={
                    "headless": LaunchConfiguration("headless"),
                    "world": world,
                    "rear_attachment": LaunchConfiguration("motion_rear_attachment"),
                    "lidar_rate": LaunchConfiguration("motion_lidar_rate"),
                    "use_sim_time": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([robot, "launch", "navigation.launch.py"])
                ),
                condition=IfCondition(qualified),
                launch_arguments={"external_controller": "true", "use_sim_time": "true"}.items(),
            ),
            Node(
                package="senior_robot",
                executable="simulation_camera",
                condition=IfCondition(LaunchConfiguration("video_enabled")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "frame_path": ParameterValue(
                            LaunchConfiguration("video_frame_path"), value_type=str
                        ),
                    }
                ],
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
                    {
                        "robot_id": robot_id,
                        "simulation_rate_hz": ParameterValue(
                            LaunchConfiguration("motion_gate_rate"), value_type=float
                        ),
                    },
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
                        "navigation_controller_params": ParameterValue(
                            controller_params, value_type=str
                        ),
                        "simulation_qualified_reference": ParameterValue(
                            LaunchConfiguration("qualified_reference"), value_type=bool
                        ),
                    },
                ],
                additional_env={"SENIOR_AGENT_SOCKET": LaunchConfiguration("agent_socket")},
            ),
        ]
    )
