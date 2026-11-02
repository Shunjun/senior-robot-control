"""ROS adapter for RGB-D perception. It publishes no velocity.

safety_node remains the only /cmd_vel publisher. The pipeline below is plain
Python: ``rclpy`` is imported only inside ``main`` so unit tests run without ROS.
Geometry is produced for every synchronized RGB/depth pair; the detector runs on
its own bounded worker and only adds semantics when it has fresh output.
"""

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from senior_control.vision import (
    CameraCalibration,
    DepthFrame,
    DynamicMask,
    GeometryConfig,
    GeometryObservation,
    ImageFrame,
    InferenceWorker,
    RgbdSynchronizer,
    Tracker,
    TrackingConfig,
    load_detector,
    project_depth,
)

# //Compatible: preserve the adapter module image-decoder imports.
from senior_control.vision.image_codec import _validate_layout, decode_depth, decode_rgb

DEFAULT_RGB_TOPIC = "/camera/color/image_raw"
DEFAULT_DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"


@dataclass(frozen=True)
class PerceptionOutput:
    geometry: GeometryObservation
    mask: DynamicMask
    detector_status: tuple[str, str]


class PerceptionAdapter:
    """In-process pipeline. ``published_topics`` stays empty: no /cmd_vel."""

    def __init__(
        self,
        *,
        calibration: CameraCalibration,
        synchronizer: RgbdSynchronizer,
        geometry_config: GeometryConfig,
        tracking_config: TrackingConfig,
        worker: InferenceWorker,
    ) -> None:
        self._calibration = calibration
        self._synchronizer = synchronizer
        self._geometry_config = geometry_config
        self._tracker = Tracker(tracking_config)
        self._worker = worker
        self.published_topics: list[str] = []

    def ingest_rgb(self, frame: ImageFrame) -> PerceptionOutput | None:
        pair = self._synchronizer.offer_rgb(frame)
        return None if pair is None else self._process(pair)

    def ingest_depth(self, frame: DepthFrame) -> PerceptionOutput | None:
        pair = self._synchronizer.offer_depth(frame)
        return None if pair is None else self._process(pair)

    def _process(self, pair: Any) -> PerceptionOutput:
        geometry = project_depth(pair, self._calibration, self._geometry_config)
        self._worker.submit(pair.rgb)
        batch = self._worker.latest()
        if batch is None:
            mask = self._tracker.update([], geometry)
        else:
            mask = self._tracker.update(batch.detections, geometry, semantics_stamp=batch.stamp)
        return PerceptionOutput(geometry, mask, self._worker.detector_status)

    def close(self) -> None:
        self._worker.close()




def create_perception_node() -> type:
    """ROS node type. Imported only when rclpy is installed. Publishes no velocity."""

    from rclpy.node import Node
    from sensor_msgs.msg import Image

    calibration_fields: tuple[str, ...] = (
        *("width", "height", "fx", "fy", "cx", "cy"),
        *("depth_unit_m", "min_range_m", "max_range_m"),
    )
    extrinsic_fields = ("x", "y", "z", "roll", "pitch", "yaw")

    def stamp_of(message: Any) -> float:
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9

    class PerceptionNode(Node):
        def __init__(self) -> None:
            super().__init__("perception")
            # Topics and units are parameters. Calibration has no defaults: an
            # uncalibrated camera raises here and the process exits.
            self.declare_parameter("rgb_topic", DEFAULT_RGB_TOPIC)
            self.declare_parameter("depth_topic", DEFAULT_DEPTH_TOPIC)
            self.declare_parameter("weights_path", "")
            self.declare_parameter("max_skew_s", 0.03)
            for name in calibration_fields:
                self.declare_parameter(f"calibration.{name}")
            for name in extrinsic_fields:
                self.declare_parameter(f"calibration.extrinsic.{name}")

            def value(name: str) -> object:
                return self.get_parameter(name).value

            raw: dict[str, object] = {
                name: value(f"calibration.{name}") for name in calibration_fields
            }
            raw["extrinsic"] = {
                name: value(f"calibration.extrinsic.{name}") for name in extrinsic_fields
            }
            calibration = CameraCalibration.from_mapping(raw)
            weights = str(value("weights_path"))
            detector = load_detector(Path(weights) if weights else None)
            self.adapter = PerceptionAdapter(
                calibration=calibration,
                synchronizer=RgbdSynchronizer(max_skew_s=float(str(value("max_skew_s")))),
                geometry_config=GeometryConfig(),
                tracking_config=TrackingConfig(),
                worker=InferenceWorker(detector, max_pending=1),
            )
            self.latest: PerceptionOutput | None = None
            self.get_logger().info(f"detector: {detector.enabled} ({detector.reason})")
            self.create_subscription(Image, str(value("rgb_topic")), self._on_rgb, 2)
            self.create_subscription(Image, str(value("depth_topic")), self._on_depth, 2)

        def _on_rgb(self, message: Any) -> None:
            def ingest() -> PerceptionOutput | None:
                size = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(str(message.encoding))
                if size is None:
                    raise ValueError("unsupported RGB encoding")
                _validate_layout(
                    int(message.width), int(message.height), int(message.step), size,
                    len(message.data),
                )
                frame = decode_rgb(
                    stamp_of(message), int(message.width), int(message.height), int(message.step),
                    str(message.encoding), bytes(message.data),
                )
                return self.adapter.ingest_rgb(frame)

            self._accept(ingest)

        def _on_depth(self, message: Any) -> None:
            def ingest() -> PerceptionOutput | None:
                size = {"16UC1": 2, "32FC1": 4}.get(str(message.encoding))
                if size is None:
                    raise ValueError("unsupported depth encoding")
                _validate_layout(
                    int(message.width), int(message.height), int(message.step), size,
                    len(message.data),
                )
                frame = decode_depth(
                    stamp_of(message),
                    int(message.width),
                    int(message.height),
                    int(message.step),
                    str(message.encoding),
                    bytes(message.data),
                    bool(message.is_bigendian),
                )
                return self.adapter.ingest_depth(frame)

            self._accept(ingest)

        def _accept(self, ingest: Callable[[], PerceptionOutput | None]) -> None:
            try:
                output = ingest()
            except ValueError as error:
                self.get_logger().warning(f"rejected frame: {error}")
                return
            if output is not None:
                self.latest = output

        def destroy_node(self) -> bool:
            self.adapter.close()
            return bool(super().destroy_node())

    return PerceptionNode


def main() -> None:
    import rclpy

    rclpy.init()
    node_type = create_perception_node()
    node = node_type()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
