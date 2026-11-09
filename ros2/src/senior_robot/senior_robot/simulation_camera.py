"""Write one latest RGB image from the isolated Gazebo bridge."""

from pathlib import Path
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from senior_control.vision.simulation_video import publish_image
from senior_robot.motion_lifecycle import MotionExecutor, with_dds_info

_TOPIC = "/camera/color/image_raw"


class SimulationCamera(Node):
    def __init__(self) -> None:
        super().__init__("simulation_camera")
        self.declare_parameter("frame_path", "")
        value = str(self.get_parameter("frame_path").value)
        self._path = Path(value)
        if not self._path.is_absolute():
            raise ValueError("camera slot must have an absolute path")
        self._last_stamp = 0
        self.create_subscription(
            Image,
            _TOPIC,
            self._on_image,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    @with_dds_info
    def _on_image(self, image: Image, info: dict[str, Any]) -> None:
        sources = self.get_publishers_info_by_topic(_TOPIC)
        if len(sources) != 1 or (sources[0].node_name, sources[0].node_namespace) != (
            "senior_sim_bridge",
            "/",
        ):
            return
        # //Compatible: Humble may omit per-message publisher_gid.
        gid = info.get("publisher_gid")
        if gid is not None and bytes(gid) != bytes(sources[0].endpoint_gid):
            return
        stamp = image.header.stamp.sec * 10**9 + image.header.stamp.nanosec
        age = self.get_clock().now().nanoseconds - stamp
        if stamp <= 0 or stamp == self._last_stamp or not 0 <= age <= 500_000_000:
            return
        try:
            if publish_image(
                self._path,
                image.width,
                image.height,
                image.step,
                image.encoding,
                memoryview(image.data),
            ):
                self._last_stamp = stamp
        except OSError:
            self.get_logger().warning("simulation camera slot unavailable", throttle_duration_sec=3)


def main() -> None:
    rclpy.init()
    node = None
    executor = MotionExecutor()
    try:
        node = SimulationCamera()
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        # //Compatible: Humble's signal handler may already have shut down the context.
        rclpy.try_shutdown()
