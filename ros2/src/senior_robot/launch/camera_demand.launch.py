"""Keep vendor capture intact and launch optional image processors independently."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vehicle_core.ros_wiring import TOPICS


def camera_actions(context):
    component = LaunchConfiguration("component").perform(context)
    if component == "capture":
        vendor = Path(get_package_share_directory("turn_on_wheeltec_robot"))
        settings = yaml.safe_load((vendor / "config/wheeltec_param.yaml").read_text())
        mode = settings["camera_mode"]
        # Only package-local launch basenames from the vendor configuration are accepted.
        if not isinstance(mode, str) or not mode or not mode.replace("_", "").isalnum():
            raise ValueError("invalid vendor camera mode")
        if mode.startswith(("astra", "dabai", "gemini")):
            path = (
                Path(get_package_share_directory("astra_camera")) / "launch" / f"{mode}.launch.xml"
            )
            return [IncludeLaunchDescription(AnyLaunchDescriptionSource(str(path)))]
        if mode.startswith("usb"):
            path = Path(get_package_share_directory("usb_cam")) / "launch/demo.launch.py"
            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(str(path)),
                    launch_arguments={"video_device": "/dev/RgbCam"}.items(),
                )
            ]
        raise ValueError(f"unsupported vendor camera mode: {mode}")
    if component in {"jpeg", "depth_compressed"}:
        stream = "color" if component == "jpeg" else "depth"
        limited = getattr(TOPICS, f"senior_camera_{stream}_compression_input")
        limiter = Node(
            package="senior_sensor_ingress",
            executable="camera_rate_limit",
            name=f"limit_{stream}_compression",
            parameters=[{"input_topic": f"/camera/{stream}/image_raw", "output_topic": limited}],
        )
    if component == "jpeg":
        output = "/camera/color/image_raw/compressed"
        return [
            limiter,
            Node(
                package="image_transport",
                executable="republish",
                name="republish_color_compressed",
                arguments=["raw", "compressed"],
                remappings=[("in", limited), ("out/compressed", output)],
                parameters=[{"format": "jpeg", "jpeg_quality": 85}],
            ),
        ]
    if component == "depth_compressed":
        return [
            limiter,
            Node(
                package="image_transport",
                executable="republish",
                name="republish_depth_compressed",
                arguments=["raw", "compressedDepth"],
                remappings=[
                    ("in", limited),
                    ("out/compressedDepth", "/camera/depth/image_raw/compressedDepth"),
                ],
                output="screen",
            ),
        ]
    raise ValueError(f"unsupported camera component: {component}")


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("component", default_value="capture"),
            OpaqueFunction(function=camera_actions),
        ]
    )
