"""Owned native scan receipt process for actual ROS integration tests."""

import subprocess
from pathlib import Path

import pytest


@pytest.fixture
def native_scan_process():
    pytest.importorskip("rclpy")
    from ament_index_python.packages import get_package_prefix

    executable = (
        Path(get_package_prefix("senior_sensor_ingress"))
        / "lib"
        / ("senior_sensor_ingress/scan_receipt_node")
    )
    process = subprocess.Popen(
        [str(executable)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    try:
        yield process
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


@pytest.fixture
def fenced_scan_fixture(monkeypatch):
    """Controlled ownership metadata for direct Gate callback regressions."""
    import time
    from types import SimpleNamespace

    pytest.importorskip("rclpy")
    from rclpy.time import Time
    from senior_motion_msgs.msg import FencedSafetyScan

    from senior_control.sensors.sample_time import map_sample

    def wrap(gate, body):
        monkeypatch.setattr(gate, "_scan_topology_ready", lambda: True)
        gate._scan_relay_gid = "aa" * 24
        monkeypatch.setattr(
            gate,
            "get_publishers_info_by_topic",
            lambda _: [SimpleNamespace(endpoint_gid=bytes.fromhex("bb" * 24))],
        )
        body.raw_source = "cc" * 24
        received_steady_ns = time.monotonic_ns()
        body.ingress_steady = received_steady_ns / 1e9
        receipt = gate.get_clock().now().nanoseconds
        body.ingress_ros = receipt / 1e9
        capture = body.raw_capture
        capture.stamp, capture.received_ros = body.stamp, Time(nanoseconds=receipt).to_msg()
        mapped = map_sample(
            stamp_ns=body.stamp.sec * 1_000_000_000 + body.stamp.nanosec,
            received_ros_ns=receipt,
            received_steady_ns=received_steady_ns,
            clock_generation=1,
        )
        capture.received_steady_ns = mapped.received_steady_ns
        capture.sample_steady_ns = mapped.sample_steady_ns
        capture.clock_generation = mapped.clock_generation
        capture.frame_id = body.frame_id
        capture.raw_gid, capture.bridge_gid = body.raw_source, "bb" * 24
        capture.source_epoch, capture.bridge_boot_id = "raw-epoch", "bridge-boot"
        capture.capture_generation, capture.sequence, capture.valid = 1, body.sequence, True
        envelope = FencedSafetyScan()
        envelope.body = body
        owner = envelope.owner
        owner.relay_gid, owner.relay_boot = gate._scan_relay_gid, "relay-boot"
        owner.relay_sequence, owner.owner_sequence = body.sequence, body.sequence
        owner.owner_gid, owner.owner_epoch = "dd" * 24, body.source_epoch
        owner.owner_generation, owner.valid = 1, True
        return envelope

    return wrap
