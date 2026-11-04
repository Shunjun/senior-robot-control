"""Nav2 process wiring. Skipped without rclpy; a skip is not a Linux pass.

Do not treat a local skip as the Humble navigation check.
"""

from pathlib import Path

import pytest
import yaml


def test_nav2_costmap_nodes_receive_footprint_and_layers():
    config_path = Path(__file__).resolve().parents[2] / "ros2/src/senior_robot/config/nav2.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    # The controller and planner create separate, namespaced costmap nodes.
    local = config["local_costmap"]["local_costmap"]["ros__parameters"]
    global_ = config["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["global_frame"] == "odom"
    assert global_["global_frame"] == "map"
    assert local["footprint"] == global_["footprint"]
    assert len(yaml.safe_load(local["footprint"])) == 4
    assert "obstacle_layer" in local["plugins"]
    assert "static_layer" in global_["plugins"]
    controller = config["controller_server"]["ros__parameters"]
    assert controller["progress_checker_plugin"] == "progress_checker"
    assert controller["FollowPath"]["CostCritic"]["consider_footprint"]


def test_nav2_launch_config_keeps_recovery_output_before_arbitration():
    config_path = Path(__file__).resolve().parents[2] / "ros2/src/senior_robot/config/nav2.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    assert (
        config["navigation_adapter"]["ros__parameters"]["command_topic"] == "/control/recovery_cmd"
    )


def test_navigation_node_does_not_publish_cmd_vel():
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.navigation_adapter import SAFETY_COMMAND_TOPIC, create_navigation_node

    assert SAFETY_COMMAND_TOPIC == "/control/recovery_cmd"
    rclpy.init()
    node = None
    try:
        node = create_navigation_node()()
        publishers = getattr(node, "publishers", None)
        if publishers is None or callable(publishers):
            publishers = getattr(node, "_publishers", ())
        topics = {publisher.topic_name for publisher in publishers}
        assert topics - {"/parameter_events", "/rosout"} == {"/control/recovery_cmd"}
        assert "/cmd_vel" not in topics
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
