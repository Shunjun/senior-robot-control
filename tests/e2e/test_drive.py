"""Live ROS graph checks; these stay skipped when the simulator is absent."""

from __future__ import annotations

import importlib.util
import platform
import time

import pytest
from conftest import SimulationProfile


def test_only_safety_node_publishes_cmd_vel(
    simulation_profile: SimulationProfile,
) -> None:
    if platform.system() != "Linux" or importlib.util.find_spec("rclpy") is None:
        pytest.skip("requires Linux, sourced ROS 2 Humble, and the running simulation stack")

    import rclpy  # type: ignore[import-not-found]

    rclpy.init()
    node = rclpy.create_node(f"acceptance_drive_{simulation_profile.scene_seed}")
    try:
        deadline = time.monotonic() + 10.0
        publishers = []
        while time.monotonic() < deadline:
            publishers = node.get_publishers_info_by_topic("/cmd_vel")
            if publishers:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        assert publishers, "no /cmd_vel publisher discovered"
        assert {publisher.node_name for publisher in publishers} == {"safety_node"}
    finally:
        node.destroy_node()
        rclpy.shutdown()
