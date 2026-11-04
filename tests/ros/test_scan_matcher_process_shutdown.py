"""Real scan matcher process shutdown joins the transform listener."""

import os
import signal
import subprocess
import time
from pathlib import Path

import pytest


@pytest.mark.parametrize("signal_name", ["SIGINT", "SIGTERM"])
@pytest.mark.parametrize("repeat", [False, True])
def test_scan_matcher_process_shutdown(signal_name, repeat):
    rclpy = pytest.importorskip("rclpy")
    executable = os.environ.get("SENIOR_SCAN_MATCHER_BINARY")
    if not executable:
        pytest.skip("set SENIOR_SCAN_MATCHER_BINARY to an isolated matcher build")
    binary = Path(executable)
    assert binary.is_file(), binary
    config = (
        Path(__file__).resolve().parents[2]
        / "ros2/src/senior_robot/config"
        / "simulation_motion.yaml"
    )
    proc = subprocess.Popen(
        [str(binary), "--ros-args", "--params-file", str(config)],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    rclpy.init()
    observer = rclpy.create_node("matcher_shutdown_observer")
    try:
        deadline = time.monotonic() + 10
        while "scan_motion_node" not in observer.get_node_names() and time.monotonic() < deadline:
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert "scan_motion_node" in observer.get_node_names(), "matcher did not start"
        os.kill(proc.pid, getattr(signal, signal_name))
        if repeat:
            os.kill(proc.pid, getattr(signal, signal_name))
        stdout, stderr = proc.communicate(timeout=10)
        assert proc.returncode == 0, stdout + stderr
        assert "terminate called" not in stderr and "what():" not in stderr
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate(timeout=5)
        observer.destroy_node()
        rclpy.shutdown()
