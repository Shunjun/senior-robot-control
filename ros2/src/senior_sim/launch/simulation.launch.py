"""Explicitly launched Fortress simulation; never invoked by environment detection."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("senior_sim")
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    sim_time = LaunchConfiguration("use_sim_time")
    description = Command(
        [
            "xacro ",
            PathJoinSubstitution([share, "urdf", "senior.urdf.xacro"]),
            " controllers_file:=",
            LaunchConfiguration("controllers_file"),
        ]
    )
    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world", default_value=PathJoinSubstitution([share, "worlds", "baseline.sdf"])
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution([share, "config", "controllers.yaml"]),
            ),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                condition=IfCondition(headless),
                launch_arguments={"gz_args": ["-r -s ", world]}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_launch),
                condition=UnlessCondition(headless),
                launch_arguments={"gz_args": ["-r ", world]}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description, "use_sim_time": sim_time}],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-name", "senior", "-topic", "robot_description", "-z", "0.06"],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                parameters=[
                    {
                        "config_file": PathJoinSubstitution([share, "config", "bridge.yaml"]),
                        "use_sim_time": sim_time,
                    }
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "drive",
                    "--controller-manager",
                    "/controller_manager",
                ],
            ),
        ]
    )
