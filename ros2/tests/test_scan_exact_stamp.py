"""Real Humble matcher must preserve the original previous scan timestamp."""

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("SENIOR_SCAN_EPOCH_LIVE") != "1", reason="explicit ROS node integration only"
)
rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from senior_motion_msgs.msg import ScanMotion  # noqa: E402
from test_scan_epoch_live import ControlledFencedScanPublisher, _room_scan  # noqa: E402
from tf2_ros import StaticTransformBroadcaster  # noqa: E402


def _ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def test_native_matcher_interval_start_is_exact_original_scan_header(tmp_path, monkeypatch):
    domain = os.environ.get("SENIOR_MOTION_DOMAIN", "")
    assert domain.isdecimal() and 0 <= int(domain) <= 101
    monkeypatch.setenv("ROS_DOMAIN_ID", domain)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    params = Path(__file__).parents[2] / "ros2/src/senior_robot/config/simulation_motion.yaml"
    log_path = tmp_path / "scan-motion-exact-stamps.log"
    with log_path.open("w", encoding="utf-8") as log:
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "senior_scan_matcher",
                "scan_motion_node",
                "--ros-args",
                "--params-file",
                str(params),
                "-p",
                "use_sim_time:=false",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        producer = listener = executor = None
        try:
            rclpy.init()
            producer = rclpy.create_node("scan_receipt")
            listener = rclpy.create_node("scan_exact_stamp_test")
            executor = SingleThreadedExecutor()
            executor.add_node(producer)
            executor.add_node(listener)
            publisher = ControlledFencedScanPublisher(producer)
            received = []
            listener.create_subscription(ScanMotion, "/motion/scan", received.append, 10)
            broadcaster = StaticTransformBroadcaster(producer)
            transform = TransformStamped()
            transform.header.stamp = producer.get_clock().now().to_msg()
            transform.header.frame_id = "base_link"
            transform.child_frame_id = "laser_link"
            transform.transform.rotation.w = 1.0
            broadcaster.sendTransform(transform)
            scan = _room_scan()
            sent = set()

            def exchange(until, *, bad):
                deadline = time.monotonic() + 20
                while time.monotonic() < deadline and not until():
                    assert process.poll() is None, "matcher exited"
                    scan.header.stamp = producer.get_clock().now().to_msg()
                    scan.time_increment = 0.001 if bad else 0.0
                    current = _ns(scan.header.stamp)
                    sent.add(current)
                    publisher.publish(scan)
                    response_deadline = min(deadline, time.monotonic() + 0.3)
                    while time.monotonic() < response_deadline:
                        executor.spin_once(timeout_sec=0.01)
                        if any(_ns(message.header.stamp) == current for message in received):
                            break
                log.flush()
                assert until(), (
                    [(m.valid, m.reason) for m in received[-10:]],
                    log_path.read_text()[-3000:],
                )

            exchange(lambda: sum(m.valid for m in received) >= 8, bad=False)
            old_epoch = next(m.source_epoch for m in reversed(received) if m.valid)
            exchange(
                lambda: any(
                    not m.valid
                    and m.source_epoch != old_epoch
                    and m.reason == "time_or_deskew_unqualified"
                    for m in received
                ),
                bad=True,
            )
            new_epoch = next(
                m.source_epoch
                for m in reversed(received)
                if not m.valid and m.source_epoch != old_epoch
            )
            exchange(
                lambda: sum(m.valid and m.source_epoch == new_epoch for m in received) >= 8,
                bad=False,
            )
            valid = [message for message in received if message.valid]
            log.flush()
            assert len(valid) >= 8, (
                [(m.valid, m.reason) for m in received[-10:]],
                log_path.read_text()[-3000:],
            )
            missing = [
                (_ns(m.interval_start), _ns(m.header.stamp))
                for m in valid
                if _ns(m.interval_start) not in sent
            ]
            assert not missing, (
                f"interval_start did not equal any original published header: {missing}"
            )
            for epoch in (old_epoch, new_epoch):
                previous = None
                pairs = 0
                for message in received:
                    if message.source_epoch != epoch:
                        continue
                    if message.valid:
                        if previous is not None:
                            assert _ns(message.interval_start) == _ns(previous.header.stamp)
                            pairs += 1
                        previous = message
                    elif message.reason == "initializing":
                        previous = message
                    else:
                        previous = None
                assert pairs >= 3, f"insufficient continuous native sequence in {epoch}"
        finally:
            if executor is not None:
                executor.shutdown()
            if producer is not None:
                producer.destroy_node()
            if listener is not None:
                listener.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait(timeout=5)
