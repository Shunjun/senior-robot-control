"""Standard 1x environment inputs composed with the common vehicle control path."""

import importlib.util
import secrets
import shutil
import tempfile
from importlib.machinery import SourceFileLoader
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

from vehicle_core.launch_recovery import (
    lifecycle_group_command,
    lifecycle_owner_prefix,
    restart_prefix,
)
from vehicle_core.lifecycle_events import report_ros_exit
from vehicle_core.sensor_parameters import ros_sensor_parameters


def environment_assets(context):
    """Generate environment map files without importing them into control code."""
    script = Path(get_package_prefix("senior_sim")) / "lib/senior_sim/world_reference"
    spec = importlib.util.spec_from_loader(
        "environment_world_reference", SourceFileLoader("environment_world_reference", str(script))
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    output = Path(tempfile.mkdtemp(prefix="senior-environment-"))
    try:
        reference = module.generate_reference(
            Path(LaunchConfiguration("world").perform(context)), output
        )
    except BaseException:
        shutil.rmtree(output)
        raise

    def cleanup(context):
        shutil.rmtree(output)
        return []

    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        SetLaunchConfiguration("navigation_reference", str(reference)),
        SetLaunchConfiguration("map_yaml", str(output / "map.yaml")),
    ]


def _simulation_nodes(context):
    sim = FindPackageShare("senior_sim")
    nav = FindPackageShare("senior_navigation")
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    navigation = IfCondition(LaunchConfiguration("navigation_enabled"))
    params = PathJoinSubstitution([sim, "config", "simulation_motion.yaml"])
    common = {"use_sim_time": sim_time}
    profile = ParameterValue(LaunchConfiguration("hardware_motion_profile"), value_type=str)
    controller_params = RewrittenYaml(
        source_file=PathJoinSubstitution([nav, "config", "nav2.yaml"]),
        root_key="",
        param_rewrites={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "bt_navigator.ros__parameters.default_server_timeout": "250",
            "bt_navigator.ros__parameters.wait_for_service_timeout": "4000",
            "controller_server.ros__parameters.FollowPath.batch_size": "400",
            "controller_server.ros__parameters.FollowPath.vx_max": "0.22",
            "controller_server.ros__parameters.FollowPath.vx_min": "-0.15",
            "controller_server.ros__parameters.FollowPath.wz_max": "0.4",
            "default_nav_to_pose_bt_xml": PathJoinSubstitution(
                [nav, "behavior_trees", "navigate.xml"]
            ),
            "default_nav_through_poses_bt_xml": PathJoinSubstitution(
                [nav, "behavior_trees", "navigate_through_poses.xml"]
            ),
        },
        convert_types=True,
    )
    return [
        RegisterEventHandler(OnProcessExit(on_exit=report_ros_exit)),
        OpaqueFunction(function=environment_assets, condition=navigation),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([sim, "launch", "simulation.launch.py"])
            ),
            launch_arguments={
                "headless": LaunchConfiguration("headless"),
                "world": LaunchConfiguration("world"),
                "rear_attachment": "false",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav, "launch", "navigation.launch.py"])
            ),
            condition=navigation,
            launch_arguments={
                "external_controller": "true",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        OpaqueFunction(function=_localization_entry, condition=navigation),
        Node(
            package="senior_sim",
            executable="simulation_camera",
            prefix=restart_prefix(),
            condition=IfCondition(LaunchConfiguration("video_enabled")),
            parameters=[
                common,
                {
                    "frame_path": ParameterValue(
                        LaunchConfiguration("video_frame_path"), value_type=str
                    )
                },
            ],
        ),
        *[
            Node(
                package=package,
                executable=executable,
                prefix=restart_prefix(),
                parameters=[params, ros_sensor_parameters(), common],
            )
            for package, executable in (
                ("senior_perception", "scan_adapter"),
                ("senior_motion_estimation", "scan_motion_node"),
                ("senior_sensor_ingress", "scan_receipt_node"),
                ("senior_perception", "safety_scan_ingress"),
            )
        ],
        Node(
            package="senior_motion_control",
            executable="safety_node",
            prefix=restart_prefix(),
            sigterm_timeout="6.0",
            sigkill_timeout="1.0",
            parameters=[
                params,
                ros_sensor_parameters(),
                common,
                {
                    "hardware_motion_profile": profile,
                    "robot_id": ParameterValue(LaunchConfiguration("robot_id"), value_type=str),
                },
            ],
        ),
        Node(
            package="senior_robot",
            executable="agent_adapter",
            name="command_node",
            parameters=[
                params,
                ros_sensor_parameters(),
                common,
                {
                    "robot_id": ParameterValue(LaunchConfiguration("robot_id"), value_type=str),
                    "ipc_replace_existing_socket": False,
                    "hardware_navigation_reference": ParameterValue(
                        LaunchConfiguration("navigation_reference"), value_type=str
                    ),
                    "navigation_controller_params": ParameterValue(
                        controller_params, value_type=str
                    ),
                },
            ],
            additional_env={
                "SENIOR_AGENT_SOCKET": LaunchConfiguration("agent_socket"),
                "SENIOR_LAUNCH_SOCKET_OWNER": secrets.token_hex(16),
            },
        ),
    ]


def _localization_members(context):
    nav = FindPackageShare("senior_navigation")
    common = {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}
    navigation = IfCondition(LaunchConfiguration("navigation_enabled"))
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        prefix=lifecycle_owner_prefix(),
        sigterm_timeout="2.5",
        sigkill_timeout="0.5",
        name="map_server",
        condition=navigation,
        parameters=[common, {"yaml_filename": LaunchConfiguration("map_yaml")}],
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        prefix=lifecycle_owner_prefix(),
        sigterm_timeout="2.5",
        sigkill_timeout="0.5",
        name="amcl",
        condition=navigation,
        parameters=[
            PathJoinSubstitution([nav, "config", "nav2.yaml"]),
            common,
            {
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "global_frame_id": "map",
                "scan_topic": "/scan",
                "set_initial_pose": True,
                "initial_pose.x": 0.0,
                "initial_pose.y": 0.0,
                "initial_pose.yaw": 0.0,
            },
        ],
    )
    manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        prefix=lifecycle_owner_prefix(),
        sigterm_timeout="2.5",
        sigkill_timeout="0.5",
        name="lifecycle_manager_localization",
        condition=navigation,
        parameters=[
            common,
            {
                "autostart": True,
                "attempt_respawn_reconnection": False,
                "node_names": ["map_server", "amcl"],
            },
        ],
    )
    return context.locals.senior_lifecycle_recovery.bind([map_server, amcl, manager])


def _localization_entry(context):
    arguments = {
        name: LaunchConfiguration(name).perform(context)
        for name in ("agent_socket", "robot_id", "map_yaml")
    }
    simulated = IfCondition(LaunchConfiguration("use_sim_time")).evaluate(context)
    arguments.update(use_sim_time="true" if simulated else "false", navigation_enabled="true")
    return [
        ExecuteProcess(
            cmd=lifecycle_group_command(
                str(Path(__file__).resolve()),
                arguments,
            ),
            prefix="",
            output="screen",
            sigterm_timeout="9.0",
            sigkill_timeout="1.0",
        )
    ]


def _simulation_entry(context):
    if context.get_locals_as_dict().get("senior_lifecycle_recovery") is not None:
        return _localization_members(context)
    return _simulation_nodes(context)


def generate_launch_description():
    sim = FindPackageShare("senior_sim")
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent_socket"),
            DeclareLaunchArgument("robot_id"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("navigation_enabled", default_value="false"),
            DeclareLaunchArgument("navigation_reference", default_value=""),
            DeclareLaunchArgument("map_yaml", default_value=""),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution([sim, "worlds", "motion_qualification.sdf"]),
            ),
            DeclareLaunchArgument(
                "hardware_motion_profile",
                default_value=PathJoinSubstitution([sim, "config", "motion_profile.json"]),
            ),
            DeclareLaunchArgument("video_enabled", default_value="false"),
            DeclareLaunchArgument("video_frame_path", default_value=""),
            OpaqueFunction(function=_simulation_entry),
        ]
    )
