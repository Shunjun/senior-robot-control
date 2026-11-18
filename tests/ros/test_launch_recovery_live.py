"""Opt-in, isolated-domain recovery checks against installed ROS processes.

No test here connects to a vehicle or launches the hardware motion composition.
A local skip is not an installed Humble pass.
"""

import copy
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from collections import deque
from contextlib import contextmanager
from pathlib import Path

import pytest


@pytest.fixture
def isolated_ros():
    rclpy = pytest.importorskip("rclpy", reason="ROS 2 is unavailable on this host")
    if os.environ.get("SENIOR_LAUNCH_RECOVERY_LIVE") != "1":
        pytest.skip("explicit isolated ROS recovery run not selected")
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    assert domain.isdecimal() and 0 <= int(domain) <= 101, "choose an isolated ROS domain"
    assert os.environ.get("ROS_LOCALHOST_ONLY") == "1", "localhost-only ROS required"
    if sys.platform != "linux":
        pytest.skip("installed recovery checks require Linux process semantics")
    rclpy.init()
    try:
        yield rclpy
    finally:
        if rclpy.ok():
            rclpy.shutdown()


@contextmanager
def prefixed_process(command, logfile):
    from senior_control.runtime.launch_recovery import restart_prefix

    with logfile.open("w") as log:
        process = subprocess.Popen(
            [*shlex.split(restart_prefix()), *command],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            yield process
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            assert process.wait(timeout=8) == 0, logfile.read_text()


def latest_node_pid(logfile):
    matches = re.findall(r"started pid=(\d+) attempt=\d+", logfile.read_text())
    assert matches, logfile.read_text()
    return int(matches[-1])


def test_scan_receipt_crash_recovers_new_identity_and_rejects_old_capture(isolated_ros, tmp_path):
    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from senior_motion_msgs.msg import BoundedRawLaserScan, FencedFilteredScan
    from senior_robot.scan_adapter import ScanAdapter
    from sensor_msgs.msg import LaserScan
    from test_scan_epoch_live import _room_scan
    from tf2_ros import StaticTransformBroadcaster

    # Reuse the installed native_scan_process path and the existing real room
    # scan fixture. Keep the actual Python adapter alive across receipt death.
    executable = (
        Path(get_package_prefix("senior_sensor_ingress"))
        / "lib/senior_sensor_ingress/scan_receipt_node"
    )
    logfile = tmp_path / "receipt-recovery.log"
    probe = isolated_ros.create_node("launch_recovery_scan_probe")
    adapter = ScanAdapter()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    executor.add_node(adapter)
    publisher = probe.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
    raw, filtered = deque(maxlen=128), deque(maxlen=128)
    probe.create_subscription(
        BoundedRawLaserScan, "/safety/raw/scan", raw.append, qos_profile_sensor_data
    )
    probe.create_subscription(
        FencedFilteredScan, "/perception/scan_filtered", filtered.append, qos_profile_sensor_data
    )
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = "base_link", "laser_link"
    transform.transform.rotation.w = 1.0
    adapter.buffer.set_transform_static(transform, "fixture")
    broadcaster = StaticTransformBroadcaster(probe)
    broadcaster.sendTransform(transform)
    scan = _room_scan()

    def until(predicate, wrapper):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and not predicate():
            assert wrapper.poll() is None, logfile.read_text()
            scan.header.stamp = probe.get_clock().now().to_msg()
            publisher.publish(scan)
            for _ in range(5):
                executor.spin_once(timeout_sec=0.01)
        assert predicate(), logfile.read_text()

    try:
        with prefixed_process([str(executable)], logfile) as wrapper:
            until(lambda: any(value.owner.valid for value in filtered), wrapper)
            first = copy.deepcopy(next(value for value in reversed(raw) if value.capture.valid))
            old_pid = latest_node_pid(logfile)
            os.kill(old_pid, signal.SIGKILL)
            filtered.clear()
            until(
                lambda: any(
                    value.owner.valid
                    and value.body.raw_capture.bridge_gid != first.capture.bridge_gid
                    for value in filtered
                ),
                wrapper,
            )
            second = next(value for value in reversed(raw) if value.capture.valid)
            assert latest_node_pid(logfile) != old_pid
            # A lingering orphan zombie is a failure, never proof of recovery.
            assert not Path(f"/proc/{old_pid}").exists(), logfile.read_text()
            assert second.capture.source_epoch != first.capture.source_epoch
            assert second.capture.bridge_boot_id != first.capture.bridge_boot_id
            accepted = adapter._admission.latest
            lineage = adapter._lineage
            generation = adapter._admission.generation
            adapter.on_scan(first)
            assert adapter._admission.latest == accepted
            assert adapter._lineage == lineage
            assert adapter._admission.generation == generation
    finally:
        executor.shutdown()
        adapter.destroy_node()
        probe.destroy_node()
