"""Nav2 process wiring. Skipped without rclpy; a skip is not a Linux pass.

Do not treat a local skip as the Humble navigation check.
"""

import pytest


def test_navigation_node_does_not_publish_cmd_vel():
    rclpy = pytest.importorskip("rclpy")
    from senior_robot.navigation_adapter import SAFETY_COMMAND_TOPIC, create_navigation_node

    assert SAFETY_COMMAND_TOPIC == "/safety/command_in"
    rclpy.init()
    node = None
    try:
        node = create_navigation_node()()
        publishers = getattr(node, "publishers", None)
        if publishers is None or callable(publishers):
            publishers = getattr(node, "_publishers", ())
        topics = {publisher.topic_name for publisher in publishers}
        assert topics == {"/safety/command_in"}
        assert "/cmd_vel" not in topics
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
