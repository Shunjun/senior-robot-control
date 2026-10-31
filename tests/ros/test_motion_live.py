"""Owned Fortress motion acceptance; explicitly opt in on Linux/Humble.

The ground-truth topic is observed solely by this test. Test requests enter the
real CommandNode and Arbiter; the test never publishes directly to /cmd_vel.
"""

import json
import math
import os
import shutil
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import pytest

pytestmark = pytest.mark.skipif(
    os.environ.get("SENIOR_MOTION_LIVE") != "1", reason="explicit Fortress acceptance only"
)
rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from senior_motion_msgs.msg import MotionEvidence, ScanMotion  # noqa: E402
from senior_robot.command_node import CommandNode  # noqa: E402
from sensor_msgs.msg import Imu, JointState, LaserScan  # noqa: E402

from senior_control.protocol import (  # noqa: E402
    EmptyPayload,
    SimpleEnvelope,
    VelocityEnvelope,
    VelocityPayload,
)


class LiveRun:
    def __init__(
        self, tmp_path, monkeypatch, *, rear_attachment=False, scan_relay=False, low_friction=False
    ):
        domain = os.environ.get("SENIOR_MOTION_DOMAIN", "")
        assert domain.isdecimal() and 0 <= int(domain) <= 101, (
            "set an isolated SENIOR_MOTION_DOMAIN"
        )
        monkeypatch.setenv("ROS_DOMAIN_ID", domain)
        monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
        monkeypatch.setenv("IGN_PARTITION", f"senior_motion_acceptance_{os.getpid()}")
        launch = Path(__file__).with_name("motion_qualification.launch.py")
        command = ["ros2", "launch", str(launch)]
        if rear_attachment:
            command.append("rear_attachment:=true")
        if scan_relay:
            command.append("scan_relay:=true")
        if low_friction:
            source_world = Path(__file__).with_name("motion_qualification.sdf")
            model = ET.parse(source_world)
            ground = model.find("./world/model[@name='ground']/link/collision")
            assert ground is not None
            ode = ET.SubElement(ET.SubElement(ET.SubElement(ground, "surface"), "friction"), "ode")
            ET.SubElement(ode, "mu").text = "0.0"
            ET.SubElement(ode, "mu2").text = "0.0"
            slip_world = tmp_path / "low-friction.sdf"
            model.write(slip_world, encoding="utf-8", xml_declaration=True)
            command.append(f"world:={slip_world}")
        if not os.environ.get("DISPLAY") and shutil.which("xvfb-run"):
            command = ["xvfb-run", "-a", *command]
        self.log_path = tmp_path / "motion-launch.log"
        self.capture_path = tmp_path / "motion-coverage-capture.json"
        self.summary_path = tmp_path / "motion-run-summary.json"
        self.metrics = {}
        self.log = self.log_path.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            command, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True
        )
        self.owned_pids = set()
        self.last_inventory = 0.0
        self.metrics["owned_pgid"] = self.process.pid
        self.sequence = 0
        self.last_submit = 0.0
        self.output = deque(maxlen=4000)
        self.requests = deque(maxlen=4000)
        self.status = deque(maxlen=200)
        self.scans = deque(maxlen=4000)
        self.raw_scans = deque(maxlen=100)
        self.source_scans = deque(maxlen=100)
        self.imu = deque(maxlen=4000)
        self.relay_enabled = scan_relay
        self.motion = deque(maxlen=4000)
        self.truth = deque(maxlen=4000)
        self.wheels = deque(maxlen=4000)
        self.joints = deque(maxlen=4000)
        self.clocks = deque(maxlen=4000)
        self.command = self.listener = self.executor = None
        try:
            rclpy.init()
            self.command = CommandNode()
            self.listener = rclpy.create_node("motion_acceptance_listener")
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.command)
            self.executor.add_node(self.listener)
            for kind, topic, destination, qos in (
                (Twist, "/cmd_vel", self.output, 10),
                (Twist, "/safety/command_in", self.requests, 10),
                (DiagnosticArray, "/safety/status", self.status, 10),
                (ScanMotion, "/motion/scan", self.scans, 10),
                (LaserScan, "/scan", self.raw_scans, qos_profile_sensor_data),
                (Imu, "/imu/data", self.imu, qos_profile_sensor_data),
                (MotionEvidence, "/motion/evidence", self.motion, 10),
                (Odometry, "/ground_truth/odom", self.truth, qos_profile_sensor_data),
                (Odometry, "/wheel/odom", self.wheels, qos_profile_sensor_data),
                (JointState, "/joint_states", self.joints, qos_profile_sensor_data),
                (Clock, "/clock", self.clocks, qos_profile_sensor_data),
            ):
                self.listener.create_subscription(
                    kind,
                    topic,
                    lambda msg, values=destination: values.append((time.monotonic(), msg)),
                    qos,
                )
            if scan_relay:
                self.relay_publisher = self.listener.create_publisher(
                    LaserScan, "/scan", qos_profile_sensor_data
                )
                self.listener.create_subscription(
                    LaserScan, "/fault_test/source_scan", self._relay_scan, qos_profile_sensor_data
                )
            self.submit("control.acquire")
        except BaseException:
            self.close()
            raise

    def _relay_scan(self, message):
        self.source_scans.append((time.monotonic(), message))
        if self.relay_enabled:
            self.relay_publisher.publish(message)

    def submit(self, kind, vx=0.0):
        self.sequence += 1
        common = dict(
            v=1,
            robot_id="robot",
            session_id="live-test",
            connection_epoch=1,
            request_id=f"live-{self.sequence}",
            seq=self.sequence,
            expires_at=1.0,
        )
        if kind == "drive.velocity":
            msg = VelocityEnvelope(type=kind, payload=VelocityPayload(vx=vx, wz=0.0), **common)
        else:
            msg = SimpleEnvelope(type=kind, payload=EmptyPayload(), **common)
        result = self.command.submit(msg)
        assert result.accepted, f"arbiter rejected {kind}: {result.error}"
        self.last_submit = time.monotonic()

    def pump(self, condition, *, vx, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.diagnostics("fixture exited")
            if time.monotonic() - self.last_submit >= 0.1:
                self.submit("drive.velocity", vx)
            if time.monotonic() - self.last_inventory >= 0.5:
                self.owned_pids.update(self._active_group())
                self.last_inventory = time.monotonic()
            self.executor.spin_once(timeout_sec=0.03)
            if condition():
                return
        pytest.fail(self.diagnostics(description))

    def diagnostics(self, description):
        self.log.flush()
        tail = self.log_path.read_text(encoding="utf-8", errors="replace")[-6000:]
        recent_scans = [(m.valid, m.reason) for _, m in list(self.scans)[-5:]]
        recent_motion = [(m.valid, m.reason, m.motion_anomaly) for _, m in list(self.motion)[-5:]]
        recent_output = [(m.linear.x, m.angular.z) for _, m in list(self.output)[-5:]]
        recent_requests = [(m.linear.x, m.angular.z) for _, m in list(self.requests)[-5:]]
        recent_status = [
            (m.status[0].message, {v.key: v.value for v in m.status[0].values})
            for _, m in list(self.status)[-5:]
            if m.status
        ]
        nonzero = [
            (t, m) for t, m in self.output if abs(m.linear.x) > 0.001 or abs(m.angular.z) > 0.001
        ]
        durations = []
        pulse_start = None
        for received, message in self.output:
            active = abs(message.linear.x) > 0.001 or abs(message.angular.z) > 0.001
            if active and pulse_start is None:
                pulse_start = received
            elif not active and pulse_start is not None:
                durations.append(received - pulse_start)
                pulse_start = None
        if pulse_start is not None:
            durations.append(time.monotonic() - pulse_start)
        requests = [t for t, m in self.requests if m.linear.x > 0.03]
        start = requests[0] if requests else float("inf")
        truth = [m.pose.pose.position for t, m in self.truth if t >= start]
        qualified = [m for t, m in self.motion if t >= start and m.valid]
        truth_net = (
            math.hypot(truth[-1].x - truth[0].x, truth[-1].y - truth[0].y)
            if len(truth) >= 2
            else None
        )
        truth_path = (
            sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(truth, truth[1:], strict=False))
            if len(truth) >= 2
            else None
        )
        local_net = None
        if len(qualified) >= 2 and qualified[0].source_epoch == qualified[-1].source_epoch:
            a, b = qualified[0].local_pose, qualified[-1].local_pose
            local_net = math.hypot(b.x - a.x, b.y - a.y)
        self.metrics.update(
            observed_nonzero_samples=len(nonzero),
            nonzero_pulses=len(durations),
            max_nonzero_pulse=max(durations, default=0.0),
            total_nonzero_time=sum(durations),
            truth_net_since_request=truth_net,
            truth_path_since_request=truth_path,
            local_net_since_request=local_net,
        )
        self.save_summary()

        def scan_snapshot(item):
            received, scan = item
            return {
                "received_steady": received,
                "stamp": scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9,
                "frame": scan.header.frame_id,
                "angle_min": scan.angle_min,
                "angle_increment": scan.angle_increment,
                "range_min": scan.range_min,
                "range_max": scan.range_max,
                "ranges": [value if math.isfinite(value) else None for value in scan.ranges],
            }

        captured_status = [
            {
                "received_steady": received,
                "reason": message.status[0].message,
                "values": {entry.key: entry.value for entry in message.status[0].values},
            }
            for received, message in list(self.status)[-20:]
            if message.status
        ]
        self.capture_path.write_text(
            json.dumps(
                {
                    "status": captured_status,
                    "raw_scans": [scan_snapshot(item) for item in self.raw_scans],
                }
            ),
            encoding="utf-8",
        )
        summary = [
            (reason, {k: v for k, v in values.items() if not k.endswith("_wkt")})
            for reason, values in recent_status
        ]
        return (
            f"{description}; scan={recent_scans}; evidence={recent_motion}; "
            f"request={recent_requests}; cmd_vel={recent_output}; "
            f"safety={summary}; metrics={self.metrics}; capture={self.capture_path}; "
            f"launch log tail:\n{tail}"
        )

    def save_summary(self):
        text = json.dumps(self.metrics, sort_keys=True)
        self.summary_path.write_text(text + "\n", encoding="utf-8")
        print(f"motion acceptance measurements: {text}; saved={self.summary_path}")

    def kill_owned_node(self, executable):
        matches = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdecimal():
                continue
            pid = int(entry.name)
            try:
                if os.getpgid(pid) != self.process.pid:
                    continue
                arguments = (entry / "cmdline").read_bytes().split(b"\0")
                if any(Path(os.fsdecode(arg)).name == executable for arg in arguments if arg):
                    matches.append(pid)
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
        assert len(matches) == 1, self.diagnostics(
            f"expected one owned {executable}, got {matches}"
        )
        os.kill(matches[0], signal.SIGTERM)

    def _active_group(self):
        members = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdecimal():
                continue
            pid = int(entry.name)
            try:
                if os.getpgid(pid) != self.process.pid:
                    continue
                # Zombies are not active and must be reaped by their own parent.
                state = (entry / "stat").read_text().rsplit(")", 1)[1].strip().split()[0]
                if state != "Z":
                    members.append(pid)
            except (FileNotFoundError, ProcessLookupError, PermissionError, IndexError):
                continue
        return members

    def pause_world(self, paused, *, keep_vx=None):
        process = subprocess.Popen(
            [
                "ign",
                "service",
                "-s",
                "/world/motion_qualification/control",
                "--reqtype",
                "ignition.msgs.WorldControl",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                f"pause: {str(paused).lower()}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 8
        try:
            while process.poll() is None and time.monotonic() < deadline:
                if keep_vx is not None and time.monotonic() - self.last_submit >= 0.1:
                    self.submit("drive.velocity", keep_vx)
                self.executor.spin_once(timeout_sec=0.03)
            assert process.poll() is not None, "world control service timed out"
            stdout, stderr = process.communicate(timeout=1)
            assert process.returncode == 0 and "data: true" in stdout.lower(), stdout + stderr
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)

    def spin_without_requests(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.03)

    def close(self):
        try:
            if self.command is not None and self.executor is not None:
                # Disconnect is idempotent even when a pause test expired the lease.
                self.command.disconnect(1)
                until = time.monotonic() + 0.3
                while time.monotonic() < until:
                    self.executor.spin_once(timeout_sec=0.03)
        finally:
            try:
                if self.executor is not None:
                    self.executor.shutdown()
                if self.command is not None:
                    self.command.destroy_node()
                if self.listener is not None:
                    self.listener.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            finally:
                self.owned_pids.update(self._active_group())
                try:
                    os.killpg(self.process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    pass
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.process.wait(timeout=5)
                deadline = time.monotonic() + 3.0
                active = self._active_group()
                while active and time.monotonic() < deadline:
                    time.sleep(0.02)
                    active = self._active_group()
                self.metrics.update(
                    owned_pids_seen=sorted(self.owned_pids), active_owned_after_cleanup=active
                )
                self.save_summary()
                self.log.close()
                assert not active, f"owned simulation process group still active: {active}"


@pytest.fixture
def live(tmp_path, monkeypatch):
    run = LiveRun(tmp_path, monkeypatch)
    try:
        yield run
    finally:
        run.close()


@pytest.fixture
def live_rear(tmp_path, monkeypatch):
    run = LiveRun(tmp_path, monkeypatch, rear_attachment=True)
    try:
        yield run
    finally:
        run.close()


@pytest.fixture
def live_scan_drop(tmp_path, monkeypatch):
    run = LiveRun(tmp_path, monkeypatch, scan_relay=True)
    try:
        yield run
    finally:
        run.close()


@pytest.fixture
def live_low_friction(tmp_path, monkeypatch):
    run = LiveRun(tmp_path, monkeypatch, low_friction=True)
    try:
        yield run
    finally:
        run.close()


def _await_healthy(live):
    def healthy():
        recent = time.monotonic() - 1.0
        valid_scan = [m for t, m in live.scans if t >= recent and m.valid]
        valid_motion = [
            m for t, m in live.motion if t >= recent and m.valid and not m.motion_anomaly
        ]
        publishers = live.listener.get_publishers_info_by_topic("/cmd_vel")
        return (
            len(valid_scan) >= 2
            and len(valid_motion) >= 2
            and live.truth
            and live.output
            and live.clocks
            and len(publishers) == 1
            and publishers[0].node_name == "safety_gate"
        )

    live.pump(healthy, vx=0.0, timeout=90, description="healthy stationary evidence unavailable")
    noise_start = time.monotonic()
    live.pump(
        lambda: time.monotonic() - noise_start >= 1.2,
        vx=0.0,
        timeout=3,
        description="stationary baseline unavailable",
    )
    stationary = [m for t, m in live.truth if t >= noise_start]
    assert len(stationary) >= 5, live.diagnostics("insufficient stationary truth samples")
    noise = max(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) for m in stationary)
    first = stationary[0].pose.pose.position
    last = stationary[-1].pose.pose.position
    stationary_drift = math.hypot(last.x - first.x, last.y - first.y)
    assert noise <= 0.01 and stationary_drift <= 0.01, live.diagnostics(
        f"stationary baseline exceeded: speed={noise}, drift={stationary_drift}"
    )
    live.metrics.update(stationary_max_speed=noise, stationary_net_drift=stationary_drift)
    _, start_truth = live.truth[-1]
    start_motion = next(m for _, m in reversed(live.motion) if m.valid and not m.motion_anomaly)
    assert start_motion.valid and all(start_motion.covariance[i] > 0 for i in (0, 4, 8))
    assert start_motion.imu_valid and start_motion.wheel_valid
    return start_truth, start_motion


def _drive_and_verify(live):
    start_truth, start_motion = _await_healthy(live)
    start_position = start_truth.pose.pose.position
    start_local = start_motion.local_pose

    def moved():
        if not live.truth or not live.motion:
            return False
        truth = live.truth[-1][1].pose.pose.position
        candidates = [
            m for _, m in live.motion if m.valid and m.source_epoch == start_motion.source_epoch
        ]
        if not candidates:
            return False
        local = candidates[-1].local_pose
        actual_distance = math.hypot(truth.x - start_position.x, truth.y - start_position.y)
        local_distance = math.hypot(local.x - start_local.x, local.y - start_local.y)
        moving_output = any(m.linear.x > 0.03 for _, m in live.output)
        verified = (
            moving_output
            and actual_distance >= 0.05
            and local_distance >= 0.025
            and abs(actual_distance - local_distance) <= 0.04
        )
        if verified:
            live.metrics.update(
                truth_distance=actual_distance,
                local_distance=local_distance,
                distance_disagreement=abs(actual_distance - local_distance),
            )
        return verified

    live.pump(moved, vx=0.05, timeout=30, description="no verified independent forward motion")


def _wait_for_stop_with_active_request(live, fault_time):
    def stopped_after_loss():
        after = [(t, m) for t, m in live.output if t >= fault_time + 2.0]
        requests = [(t, m) for t, m in live.requests if t >= fault_time + 2.0]
        truth = [(t, m) for t, m in live.truth if t >= fault_time + 2.0]
        if len(truth) < 5:
            return False
        speeds = [math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) for _, m in truth]
        positions = [m.pose.pose.position for _, m in truth]
        net_drift = math.hypot(positions[-1].x - positions[0].x, positions[-1].y - positions[0].y)
        path_drift = sum(
            math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(positions, positions[1:], strict=False)
        )
        stop_limit = min(0.015, max(0.005, live.metrics["stationary_max_speed"] * 2 + 0.002))
        stopped = (
            after
            and after[-1][0] - after[0][0] >= 1.0
            and len(after) >= 5
            and all(m.linear.x == 0.0 and m.angular.z == 0.0 for _, m in after)
            and len(requests) >= 5
            and time.monotonic() - requests[-1][0] < 0.3
            and all(abs(m.linear.x - 0.05) < 1e-9 for _, m in requests)
            and not any(t >= fault_time + 1.0 and m.valid for t, m in live.motion)
            and truth[-1][0] - truth[0][0] >= 1.0
            and max(speeds) <= stop_limit
            and net_drift <= 0.01
            and path_drift <= 0.02
            and all(abs(m.twist.twist.angular.z) < 0.05 for _, m in truth)
        )
        if stopped:
            post_fault = [(t, m) for t, m in live.output if t >= fault_time]
            first_stable_zero = next(
                t
                for index, (t, m) in enumerate(post_fault)
                if m.linear.x == 0
                and m.angular.z == 0
                and all(x.linear.x == 0 and x.angular.z == 0 for _, x in post_fault[index:])
            )
            live.metrics.update(
                zero_delay=first_stable_zero - fault_time,
                stopped_max_truth_speed=max(speeds),
                stopped_speed_limit=stop_limit,
                stopped_net_drift=net_drift,
                stopped_path_drift=path_drift,
                stopped_sample_duration=truth[-1][0] - truth[0][0],
                latest_request_age=time.monotonic() - requests[-1][0],
            )
        return stopped

    live.pump(stopped_after_loss, vx=0.05, timeout=8, description="sensor loss did not hold zero")
    live.save_summary()


@pytest.mark.parametrize("producer", ["scan_motion_node", "scan_adapter"])
def test_qualified_motion_then_producer_loss_stops(live, producer):
    _drive_and_verify(live)
    fault_time = time.monotonic()
    live.kill_owned_node(producer)
    _wait_for_stop_with_active_request(live, fault_time)


def test_raw_scan_loss_stops_while_other_sensors_and_requests_continue(live_scan_drop):
    live = live_scan_drop
    _drive_and_verify(live)
    fault_time = time.monotonic()
    live.relay_enabled = False
    _wait_for_stop_with_active_request(live, fault_time)
    assert sum(t >= fault_time + 2.0 for t, _ in live.source_scans) >= 5
    assert sum(t >= fault_time + 2.0 for t, _ in live.imu) >= 5
    clock_values = [m.clock.sec + m.clock.nanosec / 1e9 for t, m in live.clocks if t >= fault_time]
    assert clock_values and max(clock_values) > min(clock_values)


def test_paused_clock_stops_and_expired_request_does_not_resume(live):
    _drive_and_verify(live)
    live.pause_world(True, keep_vx=0.05)
    fault_time = time.monotonic()

    def paused_and_zero():
        after = [(t, m) for t, m in live.output if t >= fault_time + 2.0]
        requests = [(t, m) for t, m in live.requests if t >= fault_time + 2.0]
        recent_clocks = [
            m.clock.sec + m.clock.nanosec / 1e9 for t, m in live.clocks if t >= fault_time + 0.5
        ]
        frozen = not recent_clocks or max(recent_clocks) == min(recent_clocks)
        return (
            after
            and after[-1][0] - after[0][0] >= 1
            and len(after) >= 5
            and all(m.linear.x == 0 and m.angular.z == 0 for _, m in after)
            and len(requests) >= 5
            and time.monotonic() - requests[-1][0] < 0.3
            and all(abs(m.linear.x - 0.05) < 1e-9 for _, m in requests)
            and frozen
        )

    live.pump(paused_and_zero, vx=0.05, timeout=8, description="paused clock did not hold zero")
    frozen_stamp = live.clocks[-1][1].clock.sec + live.clocks[-1][1].clock.nanosec / 1e9
    live.spin_without_requests(1.5)
    resume = time.monotonic()
    live.pause_world(False)
    live.spin_without_requests(3.0)
    resumed_clocks = [m.clock.sec + m.clock.nanosec / 1e9 for t, m in live.clocks if t >= resume]
    resumed_output = [m for t, m in live.output if t >= resume]
    assert resumed_clocks and max(resumed_clocks) > frozen_stamp, live.diagnostics(
        "clock did not resume"
    )
    assert len(resumed_output) >= 5 and all(
        m.linear.x == 0 and m.angular.z == 0 for m in resumed_output
    ), live.diagnostics("expired request revived after clock resume")
    resumed_truth = [(t, m) for t, m in live.truth if t >= resume + 1.0]
    assert len(resumed_truth) >= 5 and resumed_truth[-1][0] - resumed_truth[0][0] >= 1.0
    positions = [m.pose.pose.position for _, m in resumed_truth]
    speed = max(
        math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) for _, m in resumed_truth
    )
    net = math.hypot(positions[-1].x - positions[0].x, positions[-1].y - positions[0].y)
    path = sum(
        math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(positions, positions[1:], strict=False)
    )
    limit = min(0.015, max(0.005, live.metrics["stationary_max_speed"] * 2 + 0.002))
    assert speed <= limit and net <= 0.01 and path <= 0.02
    live.metrics.update(
        resumed_max_truth_speed=speed,
        resumed_net_drift=net,
        resumed_path_drift=path,
        resumed_speed_limit=limit,
    )
    live.save_summary()


def test_real_rear_attachment_keeps_unknown_side_coverage_closed(live_rear):
    start_truth, _ = _await_healthy(live_rear)
    start = start_truth.pose.pose.position
    reason_key = "count.required area contains unknown space"

    def entries():
        return [
            (received, {entry.key: entry.value for entry in message.status[0].values})
            for received, message in live_rear.status
            if message.status
        ]

    baseline = max((int(values.get(reason_key, "0")) for _, values in entries()), default=0)
    started = time.monotonic()

    def unknown_decision_observed():
        if time.monotonic() - started < 3.0:
            return False
        for received, values in entries():
            if received < started:
                continue
            try:
                count = int(values.get(reason_key, "0"))
                ambiguous = int(values.get("ambiguous_self_overlap_rays", "0"))
                request = float(values.get("request_vx", "nan"))
                command_age = float(values.get("command_steady_age", "nan"))
            except ValueError:
                continue
            if (
                count > baseline
                and ambiguous > 0
                and abs(request - 0.05) < 1e-9
                and (0 <= command_age <= 0.3)
            ):
                live_rear.metrics.update(
                    rear_unknown_count_before=baseline,
                    rear_unknown_count_after=count,
                    rear_ambiguous_rays=ambiguous,
                    rear_request_age=command_age,
                )
                return True
        return False

    live_rear.pump(
        unknown_decision_observed,
        vx=0.05,
        timeout=15,
        description="rear fixture did not reach an unknown-coverage decision",
    )
    outputs = [m for t, m in live_rear.output if t >= started + 1.0]
    requests = [m for t, m in live_rear.requests if t >= started + 1.0]
    qualified = [
        m for t, m in live_rear.motion if t >= started + 1.0 and m.valid and not m.motion_anomaly
    ]
    truth = live_rear.truth[-1][1].pose.pose.position
    assert len(outputs) >= 5 and all(m.linear.x == 0 and m.angular.z == 0 for m in outputs)
    assert len(requests) >= 5 and all(abs(m.linear.x - 0.05) < 1e-9 for m in requests)
    assert qualified, live_rear.diagnostics("rear fixture did not retain qualified motion")
    truth_distance = math.hypot(truth.x - start.x, truth.y - start.y)
    live_rear.metrics["rear_truth_distance"] = truth_distance
    live_rear.save_summary()
    assert truth_distance < 0.02


def test_physical_wheel_spin_without_body_progress_latches_stop(live_low_friction):
    live = live_low_friction
    start_truth, start_motion = _await_healthy(live)
    live.pump(
        lambda: bool(live.wheels) and bool(live.joints),
        vx=0.0,
        timeout=5,
        description="wheel odometry or joint states absent",
    )
    wheel_start = live.wheels[-1][1].pose.pose.position
    joint_message = live.joints[-1][1]
    start_joints = dict(zip(joint_message.name, joint_message.position, strict=True))
    wheel_names = {"left_front_joint", "left_back_joint", "right_front_joint", "right_back_joint"}
    assert wheel_names <= start_joints.keys()
    truth_start = start_truth.pose.pose.position
    local_start = start_motion.local_pose
    started = time.monotonic()

    def spun_and_tripped():
        if not live.wheels or not live.truth or not live.joints:
            return False
        wheel_now = live.wheels[-1][1].pose.pose.position
        truth_now = live.truth[-1][1].pose.pose.position
        motion = [
            m
            for t, m in live.motion
            if t >= started and m.valid and m.source_epoch == start_motion.source_epoch
        ]
        if not motion:
            return False
        local = motion[-1].local_pose
        wheel_distance = math.hypot(wheel_now.x - wheel_start.x, wheel_now.y - wheel_start.y)
        truth_distance = math.hypot(truth_now.x - truth_start.x, truth_now.y - truth_start.y)
        local_distance = math.hypot(local.x - local_start.x, local.y - local_start.y)
        joint_message = live.joints[-1][1]
        joints = dict(zip(joint_message.name, joint_message.position, strict=True))
        if not wheel_names <= joints.keys():
            return False
        turns = [abs(joints[name] - start_joints[name]) for name in wheel_names]
        anomalies = [t for t, m in live.motion if t >= started and m.valid and m.motion_anomaly]
        live.metrics.update(
            wheel_distance=wheel_distance,
            truth_distance=truth_distance,
            local_distance=local_distance,
            anomaly_count=len(anomalies),
            wheel_joint_rotations=turns,
        )
        if (
            wheel_distance >= 0.03
            and truth_distance < 0.01
            and local_distance < 0.02
            and sum(turn >= 0.3 for turn in turns) >= 2
            and anomalies
        ):
            live.metrics["trip_steady"] = anomalies[0]
            return True
        return False

    live.pump(
        spun_and_tripped,
        vx=0.05,
        timeout=45,
        description="no physical wheel spin with a latched independent-body anomaly",
    )
    trip = live.metrics["trip_steady"]

    def held_stopped():
        outputs = [(t, m) for t, m in live.output if t >= trip + 1.0]
        requests = [(t, m) for t, m in live.requests if t >= trip + 1.0]
        anomalies = [m for t, m in live.motion if t >= trip + 1.0 and m.valid and m.motion_anomaly]
        return (
            len(outputs) >= 5
            and outputs[-1][0] - outputs[0][0] >= 1.0
            and all(m.linear.x == 0 and m.angular.z == 0 for _, m in outputs)
            and len(requests) >= 5
            and time.monotonic() - requests[-1][0] < 0.3
            and all(abs(m.linear.x - 0.05) < 1e-9 for _, m in requests)
            and bool(anomalies)
        )

    live.pump(
        held_stopped, vx=0.05, timeout=8, description="wheel-spin anomaly did not stay latched"
    )
    live.metrics["request_age_after_trip"] = time.monotonic() - live.requests[-1][0]
    live.save_summary()
