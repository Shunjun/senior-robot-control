"""Nav2 for the differential chassis. This launch does not start a simulator.

The controller output is private to this launch. motion_output remains the only
/cmd_vel publisher. The behavior tree has no rotation before the path.
"""

import secrets
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

from senior_control.runtime.launch_recovery import (
    lifecycle_group_command,
    lifecycle_owner_prefix,
)


def _members(context):
    share = FindPackageShare("senior_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")
    external_controller = LaunchConfiguration("external_controller")
    params = RewrittenYaml(
        source_file=LaunchConfiguration("parameters"),
        root_key="",
        param_rewrites={
            "lattice_filepath": PathJoinSubstitution([share, "config", "lattice_diff.json"]),
            "default_nav_to_pose_bt_xml": PathJoinSubstitution(
                [share, "behavior_trees", "navigate.xml"]
            ),
            # //Compatible: Humble activates the through-poses navigator as well.
            "default_nav_through_poses_bt_xml": PathJoinSubstitution(
                [share, "behavior_trees", "navigate_through_poses.xml"]
            ),
            "use_sim_time": use_sim_time,
        },
        convert_types=True,
    )
    topic = f"/control/navigation/run_{secrets.token_hex(16)}/cmd_vel"
    remappings = [("cmd_vel", topic), ("/cmd_vel", topic)]
    lifecycle = [
        "controller_server",
        "planner_server",
        "bt_navigator",
    ]
    servers = [
        Node(
            condition=UnlessCondition(external_controller),
            package="nav2_controller",
            executable="controller_server",
            prefix=lifecycle_owner_prefix(),
            sigterm_timeout="2.5",
            sigkill_timeout="0.5",
            name="controller_server",
            output="screen",
            parameters=[params],
            remappings=remappings,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            prefix=lifecycle_owner_prefix(),
            sigterm_timeout="2.5",
            sigkill_timeout="0.5",
            name="planner_server",
            output="screen",
            parameters=[params],
            remappings=remappings,
        ),
        Node(
            condition=UnlessCondition(external_controller),
            package="nav2_bt_navigator",
            executable="bt_navigator",
            prefix=lifecycle_owner_prefix(),
            sigterm_timeout="2.5",
            sigkill_timeout="0.5",
            name="bt_navigator",
            output="screen",
            parameters=[params],
            remappings=remappings,
        ),
    ]
    managers = [
        Node(
            condition=IfCondition(external_controller),
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            prefix=lifecycle_owner_prefix(),
            sigterm_timeout="2.5",
            sigkill_timeout="0.5",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "attempt_respawn_reconnection": False,
                    "node_names": ["planner_server"],
                }
            ],
        ),
        Node(
            condition=UnlessCondition(external_controller),
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            prefix=lifecycle_owner_prefix(),
            sigterm_timeout="2.5",
            sigkill_timeout="0.5",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "autostart": True,
                    "attempt_respawn_reconnection": False,
                    "node_names": lifecycle,
                }
            ],
        ),
    ]
    adapters = [
        Node(
            condition=UnlessCondition(external_controller),
            package="senior_robot",
            executable="navigation_adapter",
            prefix=lifecycle_owner_prefix(),
            sigterm_timeout="2.5",
            sigkill_timeout="0.5",
            name="navigation_adapter",
            output="screen",
            parameters=[params],
        ),
    ]
    return context.locals.senior_lifecycle_recovery.bind([*servers, *managers, *adapters])


def _entry(context):
    if context.get_locals_as_dict().get("senior_lifecycle_recovery") is not None:
        return _members(context)
    external = IfCondition(LaunchConfiguration("external_controller")).evaluate(context)
    simulated = IfCondition(LaunchConfiguration("use_sim_time")).evaluate(context)
    arguments = {
        "use_sim_time": "true" if simulated else "false",
        "external_controller": "true" if external else "false",
        "parameters": LaunchConfiguration("parameters").perform(context),
    }
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


def generate_launch_description():
    share = FindPackageShare("senior_robot")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "parameters", default_value=PathJoinSubstitution([share, "config", "nav2.yaml"])
            ),
            DeclareLaunchArgument("external_controller", default_value="false"),
            OpaqueFunction(function=_entry),
        ]
    )
