"""//Compatible: original camera construction over the execution-side ROS subscriber."""

from senior_robot.ros_video import RosCameraSource as _RosCameraSource
from senior_robot.ros_video import logger as logger

from vehicle_core.image_frame import ImageFrame


class RosCameraSource(_RosCameraSource[ImageFrame]):
    """//Compatible: pass the same original shared frame constructor, no alternate frame type."""

    def __init__(self, topic: str) -> None:
        super().__init__(topic, frame_factory=ImageFrame)
