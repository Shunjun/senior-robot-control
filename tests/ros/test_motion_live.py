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
from motion_anomaly_trace import AnomalyTrace
from motion_ingress_loss import owner_loss_observed
from motion_low_friction_window import stopped_with_fresh_request
from motion_stand_geometry import load_wheel_geometry, wheel_ground_clearance

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
from senior_motion_msgs.msg import FencedSafetyScan, MotionEvidence, ScanMotion  # noqa: E402
from senior_robot.command_node import CommandNode  # noqa: E402
from sensor_msgs.msg import Imu, JointState, LaserScan  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from senior_control.protocol import (  # noqa: E402
    EmptyPayload,
    SimpleEnvelope,
    VelocityEnvelope,
    VelocityPayload,
)


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _vector(vector):
    return [vector.x, vector.y, vector.z]


def _quaternion(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]


def _planar(pose):
    return [pose.x, pose.y, pose.theta]


def _trace_message(topic, message):
    if topic in ("output", "requests"):
        return {"linear": _vector(message.linear), "angular": _vector(message.angular)}
    if topic == "clocks":
        return {"stamp_ns": _stamp_ns(message.clock)}
    record = {
        "stamp_ns": _stamp_ns(message.header.stamp),
        "frame_id": message.header.frame_id,
    }
    if topic in ("wheels", "truth"):
        record.update(
            child_frame_id=message.child_frame_id,
            position=_vector(message.pose.pose.position),
            orientation=_quaternion(message.pose.pose.orientation),
            linear_velocity=_vector(message.twist.twist.linear),
            angular_velocity=_vector(message.twist.twist.angular),
            pose_covariance=list(message.pose.covariance),
            twist_covariance=list(message.twist.covariance),
        )
    elif topic == "imu":
        record.update(
            orientation=_quaternion(message.orientation),
            angular_velocity=_vector(message.angular_velocity),
            linear_acceleration=_vector(message.linear_acceleration),
            angular_velocity_covariance=list(message.angular_velocity_covariance),
        )
    elif topic in ("scans", "motion"):
        record.update(
            interval_start_ns=_stamp_ns(message.interval_start),
            source_epoch=message.source_epoch,
            valid=message.valid,
            reason=message.reason,
            body_delta=_planar(message.body_delta),
            local_pose=_planar(message.local_pose),
            covariance=list(message.covariance),
        )
        if topic == "motion":
            record.update(
                motion_anomaly=message.motion_anomaly,
                quality=message.quality,
                scan_received_ros_ns=_stamp_ns(message.scan_received_ros),
                scan_received_steady=message.scan_received_steady,
                body_linear_velocity=_vector(message.body_velocity.linear),
                body_angular_velocity=_vector(message.body_velocity.angular),
            )
        else:
            record.update(response=message.response, residual=message.residual)
    return record


class LiveRun:
    def __init__(
        self,
        tmp_path,
        monkeypatch,
        *,
        rear_attachment=False,
        scan_relay=False,
        low_friction=False,
        fixed_stand=False,
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
        if fixed_stand:
            assert not low_friction and not rear_attachment and not scan_relay
            command.append("fixed_stand:=true")
            command.append(f"world:={Path(__file__).with_name('motion_fixed_stand.sdf')}")
            command.append(
                f"safety_overlay:={Path(__file__).with_name('motion_fixed_stand_profile.yaml')}"
            )
        elif low_friction:
            source_world = (
                Path(__file__).resolve().parents[2]
                / "ros2/src/senior_sim/worlds/motion_qualification.sdf"
            )
            model = ET.parse(source_world)
            ground = model.find("./world/model[@name='ground']/link/collision")
            assert ground is not None
            ode = ET.SubElement(ET.SubElement(ET.SubElement(ground, "surface"), "friction"), "ode")
            ET.SubElement(ode, "mu").text = "0.0"
            ET.SubElement(ode, "mu2").text = "0.0"
            slip_world = tmp_path / "low-friction.sdf"
            model.write(slip_world, encoding="utf-8", xml_declaration=True)
            command.append(f"world:={slip_world}")
        else:
            # Explicitly opt in only for the measured normal-friction flat fixture.
            overlay = (
                Path(__file__).parents[2]
                / "ros2/src/senior_robot/config/simulation_motion_flat_lateral.yaml"
            )
            command.append(f"safety_overlay:={overlay}")
        if not os.environ.get("DISPLAY") and shutil.which("xvfb-run"):
            command = ["xvfb-run", "-a", *command]
        self.log_path = tmp_path / "motion-launch.log"
        self.capture_path = tmp_path / "motion-coverage-capture.json"
        self.anomaly_path = tmp_path / "motion-first-anomaly-trace.json"
        self.progress_break_path = tmp_path / "motion-first-progress-break-trace.json"
        self.summary_path = tmp_path / "motion-run-summary.json"
        self.metrics = {}
        self.anomaly_trace = AnomalyTrace()
        self.first_trace_kind = None
        self.seen_qualified_motion = False
        self.progress_break_at = None
        self.progress_break_events = deque(maxlen=128)
        self.motion_progress_unqualified = False
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
        self.fenced_scans = deque(maxlen=4000)
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
        self.stand_states = deque(maxlen=100)
        self.command = self.listener = self.executor = None
        try:
            rclpy.init()
            self.command = CommandNode()
            self.listener = rclpy.create_node("motion_acceptance_listener")
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.command)
            self.executor.add_node(self.listener)
            for kind, topic, destination, label, qos in (
                (Twist, "/cmd_vel", self.output, "output", 10),
                (Twist, "/safety/command_in", self.requests, "requests", 10),
                (FencedSafetyScan, "/safety/bounded_scan", self.fenced_scans, "fenced_scans", 10),
                (DiagnosticArray, "/safety/status", self.status, "status", 10),
                (ScanMotion, "/motion/scan", self.scans, "scans", 10),
                (LaserScan, "/scan", self.raw_scans, "raw", qos_profile_sensor_data),
                (Imu, "/imu/data", self.imu, "imu", qos_profile_sensor_data),
                (MotionEvidence, "/motion/evidence", self.motion, "motion", 10),
                (Odometry, "/ground_truth/odom", self.truth, "truth", qos_profile_sensor_data),
                (Odometry, "/wheel/odom", self.wheels, "wheels", qos_profile_sensor_data),
                (JointState, "/joint_states", self.joints, "joints", qos_profile_sensor_data),
                (Clock, "/clock", self.clocks, "clocks", qos_profile_sensor_data),
            ):
                self.listener.create_subscription(
                    kind,
                    topic,
                    lambda msg, values=destination, key=label: self._record_message(
                        key, values, msg
                    ),
                    qos,
                )
            if scan_relay:
                self.relay_publisher = self.listener.create_publisher(
                    LaserScan, "/scan", qos_profile_sensor_data
                )
                self.listener.create_subscription(
                    LaserScan, "/fault_test/source_scan", self._relay_scan, qos_profile_sensor_data
                )
            if fixed_stand:
                self.listener.create_subscription(
                    String,
                    "/qualification/stand_state",
                    lambda msg: self.stand_states.append((time.monotonic(), msg.data)),
                    10,
                )
                self._qualify_fixed_stand()
            self.submit("control.acquire")
        except BaseException:
            self.close()
            raise

    def _record_message(self, topic, destination, message):
        received = time.monotonic()
        destination.append((received, message))
        if topic not in {
            "output",
            "requests",
            "scans",
            "imu",
            "motion",
            "truth",
            "wheels",
            "clocks",
        }:
            return
        anomaly = topic == "motion" and message.valid and message.motion_anomaly
        progress_break = (
            topic == "motion"
            and self.seen_qualified_motion
            and not message.valid
            and message.reason == "progress_history_unqualified"
        )
        if topic == "motion" and message.valid and not message.motion_anomaly:
            self.seen_qualified_motion = True
            self.motion_progress_unqualified = False
        if progress_break and not self.motion_progress_unqualified:
            self.progress_break_events.append(received)
            self.motion_progress_unqualified = True
        if progress_break and self.progress_break_at is None:
            self.progress_break_at = received
        if (anomaly or progress_break) and self.first_trace_kind is None:
            self.first_trace_kind = "motion_anomaly" if anomaly else "progress_break"
        observed_clock = _stamp_ns(self.clocks[-1][1].clock) if self.clocks else None
        self.anomaly_trace.record(
            topic,
            received,
            observed_clock,
            message,
            anomaly=anomaly or progress_break,
        )

    def _save_anomaly_trace(self):
        document = self.anomaly_trace.document(_trace_message)
        if document is not None:
            document["trigger"] = self.first_trace_kind
            path = (
                self.progress_break_path
                if self.first_trace_kind == "progress_break"
                else self.anomaly_path
            )
            path.write_text(json.dumps(document), encoding="utf-8")
            self.metrics[
                "first_progress_break_trace"
                if self.first_trace_kind == "progress_break"
                else "first_anomaly_trace"
            ] = str(path)

    def _relay_scan(self, message):
        self.source_scans.append((time.monotonic(), message))
        if self.relay_enabled:
            self.relay_publisher.publish(message)

    def _stand_control(self, action, expected):
        started = time.monotonic()
        process = subprocess.Popen(
            [
                "ign",
                "topic",
                "-t",
                f"/qualification/stand_{action}",
                "-m",
                "ignition.msgs.Empty",
                "-p",
                "unused: true",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            deadline = started + 5.0
            while process.poll() is None and time.monotonic() < deadline:
                self.executor.spin_once(timeout_sec=0.03)
            assert process.poll() is not None, f"stand {action} publisher timed out"
            output = process.communicate(timeout=1)[0]
            assert process.returncode == 0, output
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                if any(
                    received >= started and state == expected
                    for received, state in self.stand_states
                ):
                    return True
                self.executor.spin_once(timeout_sec=0.03)
            return False
        finally:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5)
            active = self._active_group(process.pid)
            if active:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                deadline = time.monotonic() + 3.0
                while active and time.monotonic() < deadline:
                    time.sleep(0.02)
                    active = self._active_group(process.pid)
            self.metrics.setdefault("stand_control_groups", []).append(
                {"pgid": process.pid, "active_after_cleanup": active}
            )
            assert not active, f"stand control process group still active: {active}"

    def _qualify_fixed_stand(self):
        # No drive request is submitted until both fresh state transitions are
        # observed. Retry the pair if a one-shot transport event was missed.
        deadline = time.monotonic() + 40.0
        while time.monotonic() < deadline:
            if self._stand_control("detach", "detached"):
                if self._stand_control("attach", "attached"):
                    assert not any(
                        msg.linear.x != 0.0 or msg.angular.z != 0.0 for _, msg in self.output
                    ), "nonzero output preceded stand qualification"
                    self.metrics["stand_state_transitions"] = list(self.stand_states)
                    return
            else:
                self._stand_control("attach", "attached")
        raise AssertionError("fixed stand did not demonstrate fresh detach and attach")

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
        self._save_anomaly_trace()
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

    def _active_group(self, pgid=None):
        if pgid is None:
            pgid = self.process.pid
        members = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdecimal():
                continue
            pid = int(entry.name)
            try:
                if os.getpgid(pid) != pgid:
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
                self._save_anomaly_trace()
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


@pytest.fixture
def live_fixed_stand(tmp_path, monkeypatch):
    run = LiveRun(tmp_path, monkeypatch, fixed_stand=True)
    try:
        yield run
    finally:
        run.close()


def _fail_on_progress_break(live, since=None):
    if any(since is None or event >= since for event in live.progress_break_events):
        pytest.fail(live.diagnostics("qualified motion progress became unqualified"))


def _fail_on_anomaly_since(live, since):
    _fail_on_progress_break(live, since)
    if any(t >= since and m.valid and m.motion_anomaly for t, m in live.motion):
        pytest.fail(live.diagnostics("first motion anomaly while healthy motion was required"))


def _await_healthy(live, *, reject_anomaly=False):
    started = time.monotonic()

    def healthy():
        _fail_on_progress_break(live)
        if reject_anomaly:
            _fail_on_anomaly_since(live, started)
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

    def baseline_elapsed():
        _fail_on_progress_break(live)
        if reject_anomaly:
            _fail_on_anomaly_since(live, started)
        return time.monotonic() - noise_start >= 1.2

    live.pump(
        baseline_elapsed,
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

    def latest_qualified():
        _fail_on_progress_break(live)
        if not live.motion:
            return False
        received, message = live.motion[-1]
        return (
            0 <= time.monotonic() - received <= 0.8 and message.valid and not message.motion_anomaly
        )

    live.pump(
        latest_qualified,
        vx=0.0,
        timeout=3,
        description="fresh stationary progress evidence unavailable",
    )
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
        _fail_on_progress_break(live)
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


def test_healthy_forward_across_multiple_motion_windows(live):
    start_truth, start_motion = _await_healthy(live, reject_anomaly=True)
    start_position, start_local = start_truth.pose.pose.position, start_motion.local_pose
    started = time.monotonic()
    window_started = started
    milestones = (0.04, 0.08, 0.12, 0.16, 0.20)
    windows = []

    def progressing_without_anomaly():
        nonlocal window_started
        _fail_on_anomaly_since(live, started)
        if not live.truth:
            return False
        motion = [
            (t, m)
            for t, m in live.motion
            if t >= window_started and m.valid and not m.motion_anomaly
        ]
        if any(m.source_epoch != start_motion.source_epoch for _, m in motion):
            pytest.fail(live.diagnostics("independent motion epoch changed during healthy drive"))
        if not motion:
            return False
        truth = live.truth[-1][1].pose.pose.position
        local = motion[-1][1].local_pose
        truth_distance = math.hypot(truth.x - start_position.x, truth.y - start_position.y)
        local_distance = math.hypot(local.x - start_local.x, local.y - start_local.y)
        if len(windows) == len(milestones):
            return local_distance >= 0.16 and abs(truth_distance - local_distance) <= 0.04
        if truth_distance < milestones[len(windows)] or abs(truth_distance - local_distance) > 0.04:
            return False
        windows.append(
            {
                "milestone": milestones[len(windows)],
                "truth_distance": truth_distance,
                "local_distance": local_distance,
                "qualified_intervals": len(motion),
            }
        )
        window_started = time.monotonic()
        live.metrics["healthy_forward_windows"] = windows
        return len(windows) == len(milestones) and local_distance >= 0.16

    live.pump(
        progressing_without_anomaly,
        vx=0.05,
        timeout=90,
        description="healthy forward progress across independent-motion windows unavailable",
    )
    live.metrics["independent_crossed_0_12m"] = windows[-1]["local_distance"] >= 0.12
    live.save_summary()


def _wait_for_stop_with_active_request(live, fault_time, *, motion_must_expire=True):
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
            and (
                not any(t >= fault_time + 1.0 and m.valid for t, m in live.motion)
                if motion_must_expire
                else sum(t >= fault_time + 1.0 and m.valid for t, m in live.motion) >= 5
            )
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


def test_progress_requalification_after_real_scan_gap(live_scan_drop):
    from motion_progress_test_helpers import first_zero_acknowledgements
    from std_srvs.srv import Trigger

    live = live_scan_drop
    _drive_and_verify(live)
    fault_time = time.monotonic()
    live.relay_enabled = False
    _wait_for_stop_with_active_request(live, fault_time)
    live.relay_enabled = True
    restored = time.monotonic()

    def source_returned_unqualified():
        scans = [m for t, m in live.scans if t >= restored and m.valid]
        invalid = [
            m
            for t, m in live.motion
            if t >= restored and not m.valid and m.reason == "progress_history_unqualified"
        ]
        return len(scans) >= 3 and bool(invalid)

    live.pump(
        source_returned_unqualified,
        vx=0.05,
        timeout=15,
        description="restored scan source did not remain progress-unqualified",
    )
    client = live.listener.create_client(Trigger, "/motion_fusion/requalify_progress")
    live.pump(
        lambda: client.service_is_ready(),
        vx=0.05,
        timeout=5,
        description="progress requalification service unavailable",
    )
    future = client.call_async(Trigger.Request())
    live.pump(
        future.done,
        vx=0.05,
        timeout=5,
        description="progress requalification request timed out",
    )
    assert future.result() is not None and future.result().success
    requested = time.monotonic()
    live.pump(
        lambda: time.monotonic() - requested >= 1.0,
        vx=0.05,
        timeout=3,
        description="continued nonzero request observation unavailable",
    )
    assert not any(m.valid for t, m in live.motion if t >= requested)
    assert all(m.linear.x == 0 and m.angular.z == 0 for t, m in live.output if t >= requested)
    assert any(abs(m.linear.x - 0.05) < 1e-9 for t, m in live.requests if t >= requested)

    # Fence old nonzero messages from the same CommandNode publisher before
    # sending zero. No history is erased, and a protective Gate zero alone is
    # not an explicit requalification request.
    marker_start_truth = live.truth[-1][1]
    marker_intent = time.monotonic()
    live.submit("drive.velocity", 0.04)

    def observed_marker():
        return any(
            received >= marker_intent
            and abs(message.linear.x - 0.04) < 1e-9
            and message.angular.z == 0
            for received, message in live.requests
        ) and any(received >= marker_intent for received, _ in live.truth)

    live.pump(observed_marker, vx=0.04, timeout=5, description="nonzero marker not observed")
    marker_ack = next(
        received
        for received, message in live.requests
        if received >= marker_intent
        and abs(message.linear.x - 0.04) < 1e-9
        and message.angular.z == 0
    )
    zero_requested = time.monotonic()
    marker_truth = [marker_start_truth] + [
        message for received, message in live.truth if received >= marker_intent
    ]
    marker_positions = [message.pose.pose.position for message in marker_truth]
    marker_path = sum(
        math.hypot(b.x - a.x, b.y - a.y)
        for a, b in zip(marker_positions, marker_positions[1:], strict=False)
    )
    marker_speed = max(
        math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        for message in marker_truth
    )
    marker_speed_limit = min(0.015, max(0.005, live.metrics["stationary_max_speed"] * 2 + 0.002))
    live.metrics.update(
        zero_marker_intent_steady=marker_intent,
        zero_marker_observed_steady=marker_ack,
        zero_intent_steady=zero_requested,
        zero_marker_truth_path=marker_path,
        zero_marker_truth_max_speed=marker_speed,
        zero_marker_truth_speed_limit=marker_speed_limit,
    )
    assert not any(message.valid for received, message in live.motion if received >= requested)
    assert all(
        message.linear.x == 0 and message.angular.z == 0
        for received, message in live.output
        if received >= requested
    )
    assert marker_path <= 0.02 and marker_speed <= marker_speed_limit
    live.submit("drive.velocity", 0.0)
    live.metrics["zero_submit_completed_steady"] = live.last_submit
    live.pump(
        lambda: (
            first_zero_acknowledgements(live.requests, live.output, intent_steady=zero_requested)
            is not None
        ),
        vx=0.0,
        timeout=5,
        description="explicit zero request and safety output not observed",
    )
    acknowledgements = first_zero_acknowledgements(
        live.requests, live.output, intent_steady=zero_requested
    )
    assert acknowledgements is not None
    request_ack, output_ack = acknowledgements
    live.metrics.update(
        zero_request_observed_steady=request_ack,
        zero_output_observed_steady=output_ack,
    )
    boundary = max(request_ack, output_ack)

    def requalified_stopped():
        qualifying = [
            m
            for t, m in live.motion
            if t >= boundary
            and m.scan_received_steady >= boundary
            and m.valid
            and not m.motion_anomaly
        ]
        outputs = [m for t, m in live.output if t >= output_ack]
        requests = [m for t, m in live.requests if t >= request_ack]
        assert all(m.linear.x == 0 and m.angular.z == 0 for t, m in live.output if t >= requested)
        return (
            len(qualifying) >= 2
            and len(outputs) >= 3
            and len(requests) >= 3
            and all(m.linear.x == 0 and m.angular.z == 0 for m in outputs + requests)
        )

    live.pump(
        requalified_stopped,
        vx=0.0,
        timeout=15,
        description="explicit zero and stationary evidence did not requalify progress",
    )
    start_truth = live.truth[-1][1].pose.pose.position
    start_motion = next(m for _, m in reversed(live.motion) if m.valid and not m.motion_anomaly)
    started = time.monotonic()

    def moved_after_requalification():
        _fail_on_anomaly_since(live, started)
        truth = live.truth[-1][1].pose.pose.position
        qualifying = [
            m
            for t, m in live.motion
            if t >= started and m.valid and m.source_epoch == start_motion.source_epoch
        ]
        if not qualifying:
            return False
        local = qualifying[-1].local_pose
        truth_distance = math.hypot(truth.x - start_truth.x, truth.y - start_truth.y)
        local_distance = math.hypot(
            local.x - start_motion.local_pose.x, local.y - start_motion.local_pose.y
        )
        live.metrics.update(
            requalified_truth_distance=truth_distance, requalified_local_distance=local_distance
        )
        return (
            truth_distance >= 0.03
            and local_distance >= 0.02
            and abs(truth_distance - local_distance) <= 0.04
        )

    live.pump(
        moved_after_requalification,
        vx=0.05,
        timeout=30,
        description="drive did not resume after explicit stationary requalification",
    )
    live.save_summary()


def test_ingress_loss_stops_while_raw_scan_motion_and_requests_continue(live):
    _drive_and_verify(live)

    def endpoints(topic):
        return [
            (entry.node_name, entry.node_namespace, bytes(entry.endpoint_gid).hex())
            for entry in live.listener.get_publishers_info_by_topic(topic)
        ]

    def valid_owner_observed():
        owners = endpoints("/safety/scan_candidate")
        relays = endpoints("/safety/bounded_scan")
        return any(
            m.owner.valid
            and owners == [("safety_scan_ingress", "/", m.owner.owner_gid)]
            and relays == [("scan_receipt", "/", m.owner.relay_gid)]
            for _, m in live.fenced_scans
        )

    live.pump(valid_owner_observed, vx=0.05, timeout=5, description="scan owner not observed")
    owners = endpoints("/safety/scan_candidate")
    relays = endpoints("/safety/bounded_scan")
    before = next(
        m.owner
        for _, m in reversed(live.fenced_scans)
        if m.owner.valid
        and owners == [("safety_scan_ingress", "/", m.owner.owner_gid)]
        and relays == [("scan_receipt", "/", m.owner.relay_gid)]
    )
    fault_time = time.monotonic()
    live.kill_owned_node("safety_scan_ingress")

    def loss_observed():
        owners = endpoints("/safety/scan_candidate")
        relays = endpoints("/safety/bounded_scan")
        return any(
            received >= fault_time and owner_loss_observed(before, m.owner, owners, relays)
            for received, m in live.fenced_scans
        )

    live.pump(loss_observed, vx=0.05, timeout=8, description="scan owner loss not observed")
    _wait_for_stop_with_active_request(live, fault_time, motion_must_expire=False)
    assert sum(t >= fault_time + 2.0 for t, _ in live.raw_scans) >= 5
    assert any(
        message.status and message.status[0].message == "sensor_freshness_unqualified"
        for received, message in live.status
        if received >= fault_time
    )


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


def test_unqualified_low_friction_context_stays_stopped_with_fresh_request(live_low_friction):
    live = live_low_friction
    start_truth, _ = _await_healthy(live)
    start = start_truth.pose.pose.position
    started = time.monotonic()
    reason_key = "count.physical_motion_cap_unqualified"

    def stopped_for_missing_cap():
        reason_times = [
            received
            for received, message in live.status
            if received >= started
            and message.status
            and int(
                {entry.key: entry.value for entry in message.status[0].values}.get(reason_key, "0")
            )
            > 0
        ]
        return stopped_with_fresh_request(
            started,
            time.monotonic(),
            [(t, m.linear.x, m.angular.z) for t, m in live.output],
            [(t, m.linear.x) for t, m in live.requests],
            reason_times,
            [t for t, m in live.motion if m.valid and not m.motion_anomaly],
        )

    live.pump(
        stopped_for_missing_cap,
        vx=0.05,
        timeout=10,
        description="unqualified low-friction physical cap did not hold zero",
    )
    last = live.truth[-1][1].pose.pose.position
    drift = math.hypot(last.x - start.x, last.y - start.y)
    live.metrics.update(unqualified_low_friction_truth_drift=drift)
    live.save_summary()
    assert drift < 0.01


def _stand_truth_values(initial, current, geometry, ground_z):
    initial_position, position = initial.pose.pose.position, current.pose.pose.position
    translation = math.sqrt(
        (position.x - initial_position.x) ** 2
        + (position.y - initial_position.y) ** 2
        + (position.z - initial_position.z) ** 2
    )
    initial_q, q = initial.pose.pose.orientation, current.pose.pose.orientation
    initial_norm = math.sqrt(initial_q.x**2 + initial_q.y**2 + initial_q.z**2 + initial_q.w**2)
    norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if abs(initial_norm - 1) > 0.001 or abs(norm - 1) > 0.001:
        raise ValueError("invalid stand truth orientation")
    dot = abs(q.x * initial_q.x + q.y * initial_q.y + q.z * initial_q.z + q.w * initial_q.w)
    rotation = 2 * math.acos(min(1.0, dot / (norm * initial_norm)))
    linear, angular = current.twist.twist.linear, current.twist.twist.angular
    linear_speed = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
    angular_speed = math.sqrt(angular.x**2 + angular.y**2 + angular.z**2)
    clearance = wheel_ground_clearance(position, q, geometry, ground_z)
    values = (translation, rotation, linear_speed, angular_speed, clearance)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("nonfinite stand truth")
    return values


def _qualify_stationary_stand(live):
    started = time.monotonic()

    def feedback_ready():
        _fail_on_anomaly_since(live, started)
        return bool(live.wheels) and bool(live.joints) and bool(live.truth)

    live.pump(
        feedback_ready,
        vx=0.0,
        timeout=5,
        description="wheel odometry or joint states absent",
    )
    geometry = load_wheel_geometry(
        Path(__file__).parents[2] / "ros2/src/senior_sim/urdf/senior.urdf.xacro"
    )
    world = ET.parse(Path(__file__).with_name("motion_fixed_stand.sdf"))
    ground_pose = world.findtext("./world/model[@name='ground']/pose")
    assert ground_pose is not None
    ground_z = float(ground_pose.split()[2])
    wheel_start = live.wheels[-1][1].pose.pose.position
    joint_message = live.joints[-1][1]
    start_joints = dict(zip(joint_message.name, joint_message.position, strict=True))
    wheel_names = {"left_front_joint", "left_back_joint", "right_front_joint", "right_back_joint"}
    assert wheel_names <= start_joints.keys()
    baseline_at = time.monotonic()
    initial_truth = live.truth[-1][1]

    def still_and_clear():
        _fail_on_anomaly_since(live, started)
        if any(t >= started and state == "detached" for t, state in live.stand_states):
            pytest.fail(live.diagnostics("fixed stand detached before drive"))
        joints = [m for t, m in live.joints if t >= baseline_at]
        wheels = [m for t, m in live.wheels if t >= baseline_at]
        truth = [m for t, m in live.truth if t >= baseline_at]
        motions = [
            m for t, m in live.motion if t >= baseline_at and m.valid and not m.motion_anomaly
        ]
        outputs = [m for t, m in live.output if t >= baseline_at]
        requests = [m for t, m in live.requests if t >= baseline_at]
        if any(
            not math.isfinite(value)
            for message in wheels
            for value in (message.pose.pose.position.x, message.pose.pose.position.y)
        ):
            pytest.fail(live.diagnostics("nonfinite wheel odometry"))
        joint_drift, joint_speed = 0.0, 0.0
        for message in joints:
            positions = dict(zip(message.name, message.position, strict=True))
            velocities = dict(zip(message.name, message.velocity, strict=True))
            if not wheel_names <= positions.keys() or not wheel_names <= velocities.keys():
                pytest.fail(live.diagnostics("complete wheel positions and velocities required"))
            if not all(
                math.isfinite(positions[n]) and math.isfinite(velocities[n]) for n in wheel_names
            ):
                pytest.fail(live.diagnostics("nonfinite wheel feedback"))
            joint_drift = max(
                joint_drift, *(abs(positions[n] - start_joints[n]) for n in wheel_names)
            )
            joint_speed = max(joint_speed, *(abs(velocities[n]) for n in wheel_names))
        wheel_drift = max(
            (
                math.hypot(
                    message.pose.pose.position.x - wheel_start.x,
                    message.pose.pose.position.y - wheel_start.y,
                )
                for message in wheels
            ),
            default=0.0,
        )
        body_values = [
            _stand_truth_values(initial_truth, message, geometry, ground_z) for message in truth
        ]
        clearance = min((values[4] for values in body_values), default=float("inf"))
        if (
            joint_drift > 0.01
            or joint_speed > 0.02
            or wheel_drift > 0.001
            or clearance < 0.1
            or any(
                translation > 0.01 or rotation > 0.01 or speed > 0.01 or yaw_rate > 0.02
                for translation, rotation, speed, yaw_rate, _ in body_values
            )
            or any(
                value != 0
                for m in outputs + requests
                for value in (
                    m.linear.x,
                    m.linear.y,
                    m.linear.z,
                    m.angular.x,
                    m.angular.y,
                    m.angular.z,
                )
            )
        ):
            pytest.fail(
                live.diagnostics(
                    "stand not stationary and clear before drive: "
                    f"joint_drift={joint_drift}, joint_speed={joint_speed}, "
                    f"wheel_drift={wheel_drift}, clearance={clearance}"
                )
            )
        live.metrics.update(
            stand_baseline_joint_drift=joint_drift,
            stand_baseline_joint_speed=joint_speed,
            stand_baseline_wheel_drift=wheel_drift,
            stand_baseline_min_wheel_clearance=clearance,
        )
        return (
            time.monotonic() - baseline_at >= 1.2
            and min(len(joints), len(wheels), len(truth), len(motions), len(outputs), len(requests))
            >= 3
            and all(
                max(_stamp_ns(m.header.stamp) for m in samples)
                > min(_stamp_ns(m.header.stamp) for m in samples)
                for samples in (joints, wheels, truth)
            )
        )

    live.pump(still_and_clear, vx=0.0, timeout=5, description="stand baseline unavailable")
    return geometry, ground_z


def test_physical_wheel_spin_without_body_progress_latches_stop(live_fixed_stand):
    live = live_fixed_stand
    _await_healthy(live, reject_anomaly=True)
    geometry, ground_z = _qualify_stationary_stand(live)
    start_truth = live.truth[-1][1]
    start_motion = next(m for _, m in reversed(live.motion) if m.valid and not m.motion_anomaly)
    wheel_start = live.wheels[-1][1].pose.pose.position
    joint_message = live.joints[-1][1]
    start_joints = dict(zip(joint_message.name, joint_message.position, strict=True))
    wheel_names = {"left_front_joint", "left_back_joint", "right_front_joint", "right_back_joint"}
    assert wheel_names <= start_joints.keys()
    truth_start = start_truth.pose.pose.position
    local_start = start_motion.local_pose
    started = time.monotonic()
    wheel_motion_proven_at = None
    wheel_motion_proven_stamp_ns = None

    def spun_and_tripped():
        nonlocal wheel_motion_proven_at, wheel_motion_proven_stamp_ns
        if not live.wheels or not live.truth or not live.joints:
            return False
        wheel_now = live.wheels[-1][1].pose.pose.position
        truth_now = live.truth[-1][1].pose.pose.position
        translation, rotation, speed, yaw_rate, clearance = _stand_truth_values(
            start_truth, live.truth[-1][1], geometry, ground_z
        )
        live.metrics["stand_min_driving_wheel_clearance"] = min(
            clearance, live.metrics.get("stand_min_driving_wheel_clearance", float("inf"))
        )
        if (
            translation > 0.01
            or rotation > 0.01
            or speed > 0.01
            or yaw_rate > 0.02
            or clearance < 0.1
            or any(t >= started and s == "detached" for t, s in live.stand_states)
        ):
            pytest.fail(live.diagnostics("fixed stand lost attachment or wheel clearance"))
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
        anomalies = [
            (t, m) for t, m in live.motion if t >= started and m.valid and m.motion_anomaly
        ]
        live.metrics.update(
            wheel_distance=wheel_distance,
            truth_distance=truth_distance,
            local_distance=local_distance,
            anomaly_count=len(anomalies),
            wheel_joint_rotations=turns,
        )
        if (
            wheel_motion_proven_at is None
            and wheel_distance >= 0.03
            and all(turn >= 0.3 for turn in turns)
            and any(t >= started and m.linear.x > 0.03 for t, m in live.output)
            and not anomalies
        ):
            wheel_motion_proven_at = time.monotonic()
            wheel_stamp = live.wheels[-1][1].header.stamp
            joint_stamp = joint_message.header.stamp
            wheel_motion_proven_stamp_ns = max(
                wheel_stamp.sec * 1_000_000_000 + wheel_stamp.nanosec,
                joint_stamp.sec * 1_000_000_000 + joint_stamp.nanosec,
            )
            live.metrics["wheel_motion_proven_at"] = wheel_motion_proven_at
            live.metrics["wheel_motion_proven_stamp_ns"] = wheel_motion_proven_stamp_ns
        if (
            wheel_motion_proven_at is not None
            and wheel_motion_proven_stamp_ns is not None
            and wheel_distance >= 0.03
            and truth_distance < 0.01
            and local_distance < 0.02
            and all(turn >= 0.3 for turn in turns)
            and any(t >= started and m.linear.x > 0.03 for t, m in live.output)
            and anomalies
            and anomalies[0][0] > wheel_motion_proven_at
            and (
                anomalies[0][1].header.stamp.sec * 1_000_000_000
                + anomalies[0][1].header.stamp.nanosec
                > wheel_motion_proven_stamp_ns
            )
        ):
            live.metrics["trip_steady"] = anomalies[0][0]
            live.metrics["trip_stamp_ns"] = (
                anomalies[0][1].header.stamp.sec * 1_000_000_000
                + anomalies[0][1].header.stamp.nanosec
            )
            return True
        return False

    live.pump(
        spun_and_tripped,
        vx=0.05,
        timeout=45,
        description="no physical wheel spin with a latched independent-body anomaly",
    )
    assert live.stand_states and live.stand_states[-1][1] == "attached"
    assert not any(t >= started and state == "detached" for t, state in live.stand_states)
    body_samples = [m for t, m in live.truth if t >= started]
    assert body_samples
    body_drift = max(
        math.hypot(
            message.pose.pose.position.x - truth_start.x,
            message.pose.pose.position.y - truth_start.y,
        )
        for message in body_samples
    )
    body_speed = max(
        math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        for message in body_samples
    )

    def heading(message):
        q = message.pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    initial_yaw = heading(start_truth)
    yaw_drift = max(
        abs(
            math.atan2(
                math.sin(heading(message) - initial_yaw), math.cos(heading(message) - initial_yaw)
            )
        )
        for message in body_samples
    )
    angular_speed = max(abs(message.twist.twist.angular.z) for message in body_samples)

    def vector_norm(vector):
        return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)

    start_position = start_truth.pose.pose.position
    initial_q = start_truth.pose.pose.orientation
    initial_q_norm = math.sqrt(initial_q.x**2 + initial_q.y**2 + initial_q.z**2 + initial_q.w**2)
    assert abs(initial_q_norm - 1.0) < 0.001
    translation_3d = max(
        math.sqrt(
            (message.pose.pose.position.x - start_position.x) ** 2
            + (message.pose.pose.position.y - start_position.y) ** 2
            + (message.pose.pose.position.z - start_position.z) ** 2
        )
        for message in body_samples
    )
    linear_speed_3d = max(vector_norm(message.twist.twist.linear) for message in body_samples)
    angular_speed_3d = max(vector_norm(message.twist.twist.angular) for message in body_samples)
    rotations = []
    for message in body_samples:
        q = message.pose.pose.orientation
        q_norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        assert abs(q_norm - 1.0) < 0.001
        dot = abs(q.x * initial_q.x + q.y * initial_q.y + q.z * initial_q.z + q.w * initial_q.w)
        rotations.append(2 * math.acos(min(1.0, dot / (q_norm * initial_q_norm))))
    rotation_3d = max(rotations)
    live.metrics.update(
        stand_max_body_drift=body_drift,
        stand_max_body_speed=body_speed,
        stand_max_yaw_drift=yaw_drift,
        stand_max_yaw_rate=angular_speed,
        stand_max_translation_3d=translation_3d,
        stand_max_linear_speed_3d=linear_speed_3d,
        stand_max_rotation_3d=rotation_3d,
        stand_max_angular_speed_3d=angular_speed_3d,
    )
    assert body_drift < 0.01 and body_speed < 0.01
    assert yaw_drift < 0.01 and angular_speed < 0.02
    assert translation_3d < 0.01 and linear_speed_3d < 0.01
    assert rotation_3d < 0.01 and angular_speed_3d < 0.02
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
