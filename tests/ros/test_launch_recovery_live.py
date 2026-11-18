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


def test_installed_command_guard_core_recovery_keeps_estop_and_retires_old_manual(
    isolated_ros, tmp_path, monkeypatch
):
    """Actual guard/receipt/output processes; isolated domain, no chassis or scan source."""
    import json

    from ament_index_python.packages import get_package_prefix
    from geometry_msgs.msg import Twist
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from senior_robot.command_node import CommandNode
    from std_msgs.msg import String

    from senior_control.protocol import SimpleEnvelope
    from senior_control.settings.models import simulation_settings_profile

    monkeypatch.delenv("SENIOR_STARTUP_LEARNING_CLAIM", raising=False)
    # Learning run preservation is checked by the owner/client production-chain
    # test. This live probe starts no learning service and has no physical inputs.
    executable = Path(get_package_prefix("senior_robot")) / "lib/senior_robot/safety_node"
    profile = Path(__file__).parents[1] / "fixtures/generic_motion_profile.json"
    command = CommandNode(parameter_overrides=[Parameter("robot_id", value="launch-recovery")])
    probe = isolated_ros.create_node("launch_recovery_output_probe")
    observer = SingleThreadedExecutor()
    observer.add_node(probe)
    commands = SingleThreadedExecutor()
    commands.add_node(command)
    runtimes, velocities = deque(maxlen=256), deque(maxlen=256)
    probe.create_subscription(
        String,
        "/motion/runtime",
        lambda msg: runtimes.append(json.loads(msg.data)),
        qos_profile_sensor_data,
    )
    probe.create_subscription(Twist, "/cmd_vel", velocities.append, 10)
    logfile = tmp_path / "critical-recovery.log"
    sequence = 0

    def envelope(kind, payload=None):
        nonlocal sequence
        sequence += 1
        value = {
            "v": 1,
            "robot_id": "launch-recovery",
            "session_id": "operator",
            "connection_epoch": 1,
            "request_id": f"live-{sequence}",
            "seq": sequence,
            "expires_at": time.time() + 30,
            "type": kind,
            "payload": payload or {},
        }
        if not kind.startswith("safety."):
            value["controller_generation"] = 1
        return value

    def manual(kind, payload=None):
        now = time.monotonic()
        return command.manual_request(
            {
                "envelope": json.dumps(envelope(kind, payload)),
                "driving": simulation_settings_profile().driving_defaults.model_dump(),
                "received_steady": now,
                "deadline_steady": now + 0.3,
            }
        )

    def until(predicate, wrapper, *, spin_command=True):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not predicate():
            assert wrapper.poll() is None, logfile.read_text()
            observer.spin_once(timeout_sec=0.01)
            if spin_command:
                commands.spin_once(timeout_sec=0.01)
        assert predicate(), logfile.read_text()

    try:
        with prefixed_process(
            [
                str(executable),
                "--ros-args",
                "-p",
                "robot_id:=launch-recovery",
                "-p",
                f"hardware_motion_profile:={profile}",
                "-p",
                "depth_policy:=laser_only",
            ],
            logfile,
        ) as wrapper:
            until(
                lambda: (
                    runtimes
                    and command._output_incarnation is not None
                    and command._output_handoff_boot is None
                ),
                wrapper,
            )
            authority = manual("control.acquire")["authority_id"]
            old_prepared = command._barrier_frame
            assert manual(
                "drive.intent",
                {
                    "authority_id": authority,
                    "gesture": 1,
                    "sequence": 1,
                    "linear": 0.1,
                    "turn": 0,
                    "gear": "low",
                },
            )["ok"]
            old_boot = runtimes[-1]["boot_id"]
            os.kill(latest_node_pid(logfile), signal.SIGKILL)
            until(
                lambda: runtimes and runtimes[-1]["boot_id"] != old_boot,
                wrapper,
                spin_command=False,
            )
            new_boot = runtimes[-1]["boot_id"]
            # Keep Command's runtime callback queued. Send the original cached
            # zero and an actual newly received, still-valid old-authority intent.
            command._final_publisher.publish(old_prepared)
            assert manual(
                "drive.intent",
                {
                    "authority_id": authority,
                    "gesture": 1,
                    "sequence": 2,
                    "linear": 0.1,
                    "turn": 0,
                    "gear": "low",
                },
            )["ok"]
            until(
                lambda: sum(value["boot_id"] == new_boot for value in runtimes) >= 4,
                wrapper,
                spin_command=False,
            )
            assert all(
                value["mode"] == "idle" for value in runtimes if value["boot_id"] == new_boot
            )
            until(
                lambda: (
                    command._output_incarnation is not None
                    and command._output_incarnation[1] == new_boot
                    and command._output_handoff_boot is None
                ),
                wrapper,
            )
            assert command._manual_target is None
            for _ in range(3):
                request = envelope("safety.estop")
                expected = command._latched_estop_revision + 1
                assert command.submit(SimpleEnvelope.model_validate(request)).accepted
                until(
                    lambda expected=expected: (
                        runtimes[-1]["estop_revision"] == expected
                        and command._pending_estop_request is None
                    ),
                    wrapper,
                )
            assert runtimes[-1]["estop_revision"] == 3
            original = request["request_id"]
            os.kill(latest_node_pid(logfile), signal.SIGKILL)
            until(
                lambda: (
                    runtimes[-1]["boot_id"] != new_boot
                    and runtimes[-1]["confirmed_estop_status"] == "software_latched"
                    and runtimes[-1]["estop_revision"] == 3
                    and command._output_handoff_boot is None
                ),
                wrapper,
            )
            assert command._latched_estop_request.request_id == original
            assert manual("safety.estop.reset", {"revision": 3})["ok"]
            until(
                lambda: (
                    runtimes[-1]["confirmed_estop_status"] == "clear"
                    and command._latched_estop_request is None
                ),
                wrapper,
            )
            assert len(probe.get_publishers_info_by_topic("/cmd_vel")) == 1
            assert velocities and all(
                value.linear.x == value.angular.z == 0 for value in velocities
            )
    finally:
        commands.shutdown()
        observer.shutdown()
        command.destroy_node()
        probe.destroy_node()
