"""Publish admitted native motion observations for the shared estimator."""

import os

from rclpy.node import Node

from senior_robot.motion_lifecycle import MotionExecutor, run
from senior_robot.motion_observation_adapter import NativeObservationRosBridge
from senior_robot.motion_profile import select_motion_profile


class MotionFusion(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("motion_fusion", parameter_overrides=parameter_overrides or [])
        self.declare_parameter("wheel_frame", "odom")
        self.declare_parameter("motion_core_profile", "")
        self.declare_parameter("hardware_motion_profile", "")
        profile = select_motion_profile(
            str(self.get_parameter("motion_core_profile").value),
            hardware_path=str(self.get_parameter("hardware_motion_profile").value),
            simulated_clock=bool(self.get_parameter("use_sim_time").value),
            localhost_only=os.environ.get("ROS_LOCALHOST_ONLY", ""),
            domain_id=os.environ.get("ROS_DOMAIN_ID", ""),
        )
        from tf2_ros import Buffer, TransformListener

        self._tf = Buffer()
        self._listener = TransformListener(self._tf, self)
        self._native_motion_bridge = NativeObservationRosBridge(self, profile)


def main() -> None:
    run(MotionFusion, cli=True, executor_factory=MotionExecutor)
