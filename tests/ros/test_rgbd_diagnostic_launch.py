"""Installed launch lifecycle smoke test using a synthetic static TF publisher."""

import os
import re
import signal
import subprocess
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import TransformStamped  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402


def _child_with_argument(parent: int, argument: bytes) -> int | None:
    try:
        tasks = list(Path(f"/proc/{parent}/task").iterdir())
    except FileNotFoundError:
        return None
    for task in tasks:
        # The coordinator thread, rather than the process leader, spawns the worker.
        try:
            children = (task / "children").read_text().split()
        except FileNotFoundError:
            continue
        for child in children:
            try:
                command = Path(f"/proc/{child}/cmdline").read_bytes()
            except FileNotFoundError:
                continue
            if argument in command:
                return int(child)
    return None


def _group_exists(group: int) -> bool:
    try:
        os.killpg(group, 0)
    except ProcessLookupError:
        return False
    return True


@pytest.mark.parametrize("signum", [signal.SIGINT, signal.SIGTERM])
def test_installed_diagnostic_launch_reaps_active_worker(signum, tmp_path):
    if os.environ.get("RGBD_TEST_INSTALLED_LAUNCH") != "1":
        pytest.skip("installed diagnostic launch not selected")
    if not Path("/proc/self/task").is_dir():
        pytest.skip("requires Linux process ancestry")
    rclpy.init()
    source = rclpy.create_node("robot_state_publisher")
    tf = source.create_publisher(
        TFMessage,
        "/tf_static",
        QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        ),
    )
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.header.stamp = source.get_clock().now().to_msg()
    transform.child_frame_id = "camera_color_optical_frame"
    transform.transform.rotation.w = 1.0
    expected = {"static_tf_receipt", "rgbd_candidate_receipt", "rgbd_candidate_producer"}
    logfile = tmp_path / "launch.log"
    try:
        with logfile.open("w") as log:
            process = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "senior_robot",
                    "rgbd_diagnostic.launch.py",
                    "use_sim_time:=false",
                    "static_background_declared:=true",
                ],
                start_new_session=True,
                stdout=log,
                stderr=subprocess.STDOUT,
            )
            try:
                deadline = time.monotonic() + 10
                producer = worker = None
                while time.monotonic() < deadline and process.poll() is None:
                    if tf.get_subscription_count():
                        # Match and graph discovery can happen in either order.
                        tf.publish(TFMessage(transforms=[transform]))
                    producer = _child_with_argument(process.pid, b"/rgbd_candidate_producer")
                    worker = (
                        None
                        if producer is None
                        else _child_with_argument(producer, b"multiprocessing.spawn")
                    )
                    if worker is not None and expected.issubset(source.get_node_names()):
                        break
                    time.sleep(0.05)
                assert producer is not None and worker is not None, logfile.read_text()
                assert expected.issubset(source.get_node_names()), logfile.read_text()
                assert os.getpgid(producer) == os.getpgid(worker) == process.pid

                if signum == signal.SIGTERM:
                    # Exercise SIGTERM on the producer itself. Its owned worker
                    # must disappear before the launcher stops the native nodes.
                    os.kill(producer, signum)
                    deadline = time.monotonic() + 8
                    while time.monotonic() < deadline and Path(f"/proc/{producer}").exists():
                        time.sleep(0.05)
                    assert not Path(f"/proc/{producer}").exists(), logfile.read_text()
                    assert not Path(f"/proc/{worker}").exists(), logfile.read_text()
                os.killpg(process.pid, signal.SIGINT)
                assert process.wait(timeout=10) == 0, logfile.read_text()

                deadline = time.monotonic() + 5
                while time.monotonic() < deadline and (
                    _group_exists(process.pid) or expected.intersection(source.get_node_names())
                ):
                    time.sleep(0.05)
                output = logfile.read_text()
                assert not _group_exists(process.pid), output
                assert not Path(f"/proc/{worker}").exists(), output
                assert not expected.intersection(source.get_node_names()), output
                assert re.search(
                    r"rgbd_candidate_producer[^\n]*process has finished cleanly"
                    rf"[^\n]*\b{producer}\b",
                    output,
                ), output
                assert "Traceback (most recent call last)" not in output, output
                assert "process has died" not in output, output
            finally:
                if _group_exists(process.pid):
                    os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    finally:
        source.destroy_node()
        rclpy.shutdown()
