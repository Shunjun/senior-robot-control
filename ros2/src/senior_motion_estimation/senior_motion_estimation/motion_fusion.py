"""Publish admitted native motion observations for the shared estimator."""

from collections.abc import Callable
from typing import Any

from rclpy.node import Node
from senior_sensor_ingress.ros_lifecycle import MotionExecutor, run

from senior_motion_estimation.motion_observation_adapter import NativeObservationRosBridge


class MotionFusion(Node):
    def __init__(self, *, profile_factory: Callable[..., Any], parameter_overrides=None) -> None:
        super().__init__("motion_fusion", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("wheel_frame", "odom")
        self.declare_parameter("hardware_motion_profile", "")
        profile = profile_factory(
            hardware_path=str(self.get_parameter("hardware_motion_profile").value),
        )
        from tf2_ros import Buffer, TransformListener

        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._native_motion_bridge = NativeObservationRosBridge(self, profile)


def main(profile_factory: Callable[..., Any]) -> None:
    run(
        lambda: MotionFusion(profile_factory=profile_factory),
        cli=True,
        executor_factory=MotionExecutor,
    )
