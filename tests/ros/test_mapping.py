"""Mapping process wiring. Skipped without rclpy; a skip is not a Linux pass.

Do not treat a local skip as the Humble SLAM check. This host does not load slam_toolbox.
"""

import pytest


def test_mapping_node_does_not_publish_cmd_vel():
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.slam_adapter import create_mapping_node

    rclpy.init()
    node = None
    try:
        node = create_mapping_node()()
        publishers = getattr(node, "publishers", None)
        if publishers is None or callable(publishers):
            publishers = getattr(node, "_publishers", ())
        topics = {publisher.topic_name for publisher in publishers}
        assert "/cmd_vel" not in topics
        assert all("cmd_vel" not in topic for topic in topics)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
