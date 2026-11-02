"""ROS egress envelopes retain both relay ownership and original bodies."""

import pytest


def test_egress_envelopes_include_depth_receipt_and_original_capture_headers():
    pytest.importorskip("rclpy")
    from senior_motion_msgs.msg import FencedDepthEvidence, FencedSemanticEvidence

    depth = FencedDepthEvidence()
    semantic = FencedSemanticEvidence()
    assert depth.owner.relay_gid == ""
    assert depth.body.depth_capture.bridge_boot_id == ""
    assert semantic.owner.relay_gid == ""
    assert semantic.body.depth_owner.relay_gid == ""
    assert semantic.body.depth.camera_info_capture.bridge_boot_id == ""


def test_gate_and_semantic_receive_fenced_depth_only():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.parameter import Parameter
    from senior_motion_msgs.msg import FencedDepthEvidence
    from senior_robot.safety_node import SafetyGate
    from senior_robot.semantic_ingress import SemanticIngress

    rclpy.init()
    gate = SafetyGate(parameter_overrides=[Parameter("depth_policy", value="required")])
    semantic = SemanticIngress()
    try:
        gate_types = {item.topic_name: item.msg_type for item in gate._subscriptions}
        semantic_types = {item.topic_name: item.msg_type for item in semantic._subscriptions}
        assert gate_types["/safety/bounded_depth"] is FencedDepthEvidence
        assert semantic_types["/safety/bounded_depth"] is FencedDepthEvidence
    finally:
        semantic.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()
