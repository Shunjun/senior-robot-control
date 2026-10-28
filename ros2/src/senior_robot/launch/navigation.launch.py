"""Nav2 for the differential chassis. This launch does not start a simulator.

The controller output is /safety/command_in. safety_node remains the only
/cmd_vel publisher. The behavior tree has no rotation before the path.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = FindPackageShare("senior_robot")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = RewrittenYaml(
        source_file=PathJoinSubstitution([share, "config", "nav2.yaml"]),
        root_key="",
        param_rewrites={
            "lattice_filepath": PathJoinSubstitution([share, "config", "lattice_diff.json"]),
            "default_nav_to_pose_bt_xml": PathJoinSubstitution(
                [share, "behavior_trees", "navigate.xml"]
            ),
            "use_sim_time": use_sim_time,
        },
        convert_types=True,
    )
    remappings = [("cmd_vel", "/safety/command_in"), ("/cmd_vel", "/safety/command_in")]
    lifecycle = [
        "controller_server",
        "planner_server",
        "bt_navigator",
    ]
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[params],
                remappings=remappings,
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params],
                remappings=remappings,
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": lifecycle,
                    }
                ],
            ),
            Node(
                package="senior_robot",
                executable="navigation_adapter",
                name="navigation_adapter",
                output="screen",
                parameters=[params],
            ),
        ]
    )
