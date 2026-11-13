"""Calibration must be bounded, simulation-only, and always stop on exit."""

import copy

import pytest

from tools.vehicle_motion_calibrate import CalibrationPlan, execute_plan, validate_graph


def plan_data():
    return {
        "schema": 1,
        "kind": "isolated_gazebo_calibration",
        "run_id": "review1",
        "robot_id": "senior_4wd_dl",
        "ros_domain_id": 173,
        "target_topic": "/vehicle_motion_calibration/review1/cmd_vel",
        "world": "vehicle_motion_calibration",
        "field_bounds_m": [-20, -20, 20, 20],
        "max_linear_mps": 0.3,
        "max_angular_rps": 0.5,
        "max_accel_mps2": 0.3,
        "max_yaw_accel_rps2": 0.5,
        "segments": [
            {"seconds": 1.0, "vx": 0.0, "wz": 0.0},
            {"seconds": 2.0, "vx": 0.3, "wz": 0.0},
            {"seconds": 2.0, "vx": 0.0, "wz": 0.0},
        ],
    }


@pytest.mark.parametrize(
    "change",
    [
        {"kind": "real"},
        {"ros_domain_id": 0},
        {"ros_domain_id": 232},
        {"target_topic": "/cmd_vel"},
        {"segments": []},
        {"max_linear_mps": 2},
        {"robot_id": "hardware"},
        {"field_bounds_m": [-0.5, -0.5, 0.5, 0.5]},
    ],
)
def test_calibration_refuses_shared_domain_or_unbounded_plan(change):
    data = plan_data() | change
    with pytest.raises(ValueError):
        CalibrationPlan.from_dict(data, hard_seconds=8)


def test_rejects_nan_excess_duration_unknown_fields_and_nonzero_last_segment():
    for change in (
        {"max_linear_mps": float("nan")},
        {"unknown": True},
        {"segments": [{"seconds": 1, "vx": 0.3, "wz": 0}]},
    ):
        with pytest.raises(ValueError):
            CalibrationPlan.from_dict(plan_data() | change, hard_seconds=8)
    with pytest.raises(ValueError):
        CalibrationPlan.from_dict(plan_data(), hard_seconds=3)


def graph():
    return {
        "domain": 173,
        "sim_time": True,
        "watchdog_seconds": 0.3,
        "unstamped_command": True,
        "clock_source": "senior_sim_bridge",
        "truth_source": "senior_sim_bridge",
        "target_publishers": 0,
        "target_subscribers": ["drive"],
        "nodes": [
            "drive",
            "senior_sim_bridge",
            "controller_manager",
            "robot_state_publisher",
            "joint_state_broadcaster",
        ],
        "gazebo_worlds": ["vehicle_motion_calibration"],
    }


def test_graph_refuses_hardware_unknown_nodes_or_second_publisher():
    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    validate_graph(plan, graph())
    for change in (
        {"domain": 0},
        {"sim_time": False},
        {"target_publishers": 1},
        {"nodes": ["safety_gate"]},
        {"truth_source": "wheel"},
        {"target_subscribers": ["serial_driver"]},
        {"gazebo_worlds": []},
    ):
        with pytest.raises(ValueError):
            validate_graph(plan, graph() | change)


class Port:
    def __init__(self, fail=False):
        self.time = 0.0
        self.commands = []
        self.fail = fail
        self.closed = False

    def monotonic(self):
        return self.time

    def wait(self, seconds):
        self.time += seconds

    def check(self):
        if self.fail and self.time > 1.5:
            raise RuntimeError("lost simulation evidence")

    def publish(self, vx, wz):
        self.commands.append((self.time, vx, wz))

    def close(self):
        self.closed = True


@pytest.mark.parametrize("fail", [False, True])
def test_execution_ramps_and_cleans_up_even_after_preflight_loss(fail):
    plan = CalibrationPlan.from_dict(copy.deepcopy(plan_data()), hard_seconds=8)
    port = Port(fail)
    if fail:
        with pytest.raises(RuntimeError):
            execute_plan(plan, port, lambda event: None)
    else:
        execute_plan(plan, port, lambda event: None)
    assert port.closed
    assert port.commands[-1][1:] == (0.0, 0.0)
    assert port.time <= 8
    # Exclude emergency final zeros; ordinary ramp never jumps to target.
    active = [(t, vx) for t, vx, _ in port.commands if vx > 0]
    assert active and active[0][1] <= 0.3 * 0.05 + 1e-9


def test_graph_requires_controller_watchdog_and_unstamped_twist():
    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    for change in (
        {"watchdog_seconds": None},
        {"watchdog_seconds": 10.0},
        {"unstamped_command": False},
    ):
        with pytest.raises(ValueError):
            validate_graph(plan, graph() | change)


def test_default_cli_only_validates_and_preserves_existing_output(tmp_path):
    import json
    import subprocess
    import sys

    source, output = tmp_path / "plan.json", tmp_path / "review.jsonl"
    source.write_text(json.dumps(plan_data()))
    command = [
        sys.executable,
        "tools/vehicle_motion_calibrate.py",
        "--plan",
        str(source),
        "--seconds",
        "8",
        "--output",
        str(output),
    ]
    completed = subprocess.run(command, capture_output=True, timeout=3)
    assert completed.returncode == 0, completed.stderr
    event = json.loads(output.read_text())
    assert event["execute"] is False
    assert event["emitted_wall_ns"] > 0 and event["emitted_steady"] > 0
    assert "observed_clock_ros" not in event
    before = output.read_bytes()
    assert subprocess.run(command, capture_output=True, timeout=3).returncode != 0
    assert output.read_bytes() == before


def test_execute_requires_exact_reviewed_digest_before_ros_import(tmp_path):
    import json
    import subprocess
    import sys

    source, output = tmp_path / "plan.json", tmp_path / "run.jsonl"
    source.write_text(json.dumps(plan_data()))
    completed = subprocess.run(
        [
            sys.executable,
            "tools/vehicle_motion_calibrate.py",
            "--plan",
            str(source),
            "--seconds",
            "8",
            "--output",
            str(output),
            "--execute",
            "--reviewed-sha256",
            "wrong",
        ],
        capture_output=True,
        timeout=3,
    )
    assert completed.returncode == 2
    assert not output.exists()


def test_successful_cleanup_allows_final_zero_delivery():
    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    port = Port()
    execute_plan(plan, port, lambda event: None)
    assert port.time - port.commands[-1][0] >= 0.05 - 1e-9


def test_missing_graph_nodes_and_delayed_preflight_fail_closed():
    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    with pytest.raises(ValueError):
        validate_graph(plan, graph() | {"nodes": []})

    class Delayed(Port):
        def check(self):
            if self.time > 1.05:
                self.time += 0.3

    port = Delayed()
    with pytest.raises(RuntimeError):
        execute_plan(plan, port, lambda event: None)
    assert port.closed
    assert all(vx == wz == 0 for t, vx, wz in port.commands if t > 1.05)


def test_motion_evidence_rejects_replayed_truth_and_spinning_start():
    import tools.vehicle_motion_calibrate as calibration

    assert hasattr(calibration, "SimulationSample"), "typed simulation evidence is required"
    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    sample = calibration.SimulationSample(1.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    calibration.validate_motion_evidence(
        plan, clock=(10.0, 1.0), clock_advanced=10.0, truth=sample, now=10.0, require_origin=True
    )
    # Fresh receipt of old truth cannot keep the calibration alive.
    stale = calibration.SimulationSample(0.1, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    spinning = calibration.SimulationSample(1.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.5)
    for truth in (stale, spinning):
        with pytest.raises(RuntimeError):
            calibration.validate_motion_evidence(
                plan,
                clock=(10.0, 1.0),
                clock_advanced=10.0,
                truth=truth,
                now=10.0,
                require_origin=True,
            )


@pytest.mark.parametrize("axis", ["vx", "wz"])
def test_terminal_zero_accounts_for_entire_earlier_ramp(axis):
    data = plan_data()
    moving = {"seconds": 2.0, "vx": 0.0, "wz": 0.0}
    moving[axis] = 0.3
    data["segments"] = [
        {"seconds": 1.0, "vx": 0.0, "wz": 0.0},
        moving,
        {"seconds": 0.01, "vx": 0.0, "wz": 0.0},
        {"seconds": 0.05, "vx": 0.0, "wz": 0.0},
    ]
    with pytest.raises(ValueError, match="zero"):
        CalibrationPlan.from_dict(data, hard_seconds=8)
    data["segments"][-1]["seconds"] = 2.0
    plan = CalibrationPlan.from_dict(data, hard_seconds=8)
    port = Port()
    events = []
    execute_plan(plan, port, events.append)
    regular = [event for event in events if event["kind"] == "stimulus"]
    assert regular[-1]["vx_mps"] == regular[-1]["wz_rps"] == 0
    # No ordinary ending needs the emergency cleanup to cross to zero.
    assert port.commands[-2][1:] == (0.0, 0.0)


def test_reviewed_read_only_observers_are_allowed_but_actuation_is_still_exclusive():
    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    observed = graph() | {
        "nodes": graph()["nodes"] + ["vehicle_motion_capture", "scan_motion_node"]
    }
    validate_graph(plan, observed)
    for change in (
        {"target_publishers": 1},
        {"target_subscribers": ["drive", "vehicle_motion_capture"]},
        {"nodes": observed["nodes"] + ["ros2cli_daemon"]},
    ):
        with pytest.raises(ValueError):
            validate_graph(plan, observed | change)


def test_tf_helper_requires_verified_read_only_role_and_daemon_never_matches():
    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    helper = "transform_listener_impl_ab12"
    observed = graph() | {"nodes": graph()["nodes"] + [helper]}
    with pytest.raises(ValueError):
        validate_graph(plan, observed)
    validate_graph(plan, observed | {"verified_tf_listeners": [helper]})
    with pytest.raises(ValueError):
        validate_graph(
            plan,
            graph()
            | {
                "nodes": graph()["nodes"] + ["ros2cli_daemon"],
                "verified_tf_listeners": ["ros2cli_daemon"],
            },
        )


def test_schedule_normal_zero_and_slew_survive_bounded_wait_jitter():
    class Jittered(Port):
        def wait(self, seconds):
            self.time += seconds + 0.001

    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    port = Jittered()
    events = []
    execute_plan(plan, port, events.append)
    regular = [row for row in events if row["kind"] == "stimulus"]
    assert regular[-1]["vx_mps"] == regular[-1]["wz_rps"] == 0.0
    for left, right in zip(regular, regular[1:], strict=False):
        dt = right["steady"] - left["steady"]
        assert abs(right["vx_mps"] - left["vx_mps"]) <= 0.3 * dt + 1e-9
        assert abs(right["wz_rps"] - left["wz_rps"]) <= 0.5 * dt + 1e-9


def test_only_endpoint_proved_fixed_recorder_and_control_plugin_are_admitted():
    from tools.vehicle_motion_calibrate import verified_passive_roles

    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    recorder = "vehicle_motion_recorder_review1"
    endpoints = {
        recorder: ("/", {"/rosout", "/events/write_split"}, {"/clock", plan.target_topic}),
        "gz_ros2_control": ("/", {"/rosout", "/parameter_events"}, {"/parameter_events"}),
    }
    proved = verified_passive_roles(plan, endpoints)
    assert proved == {recorder, "gz_ros2_control"}
    base = graph() | {
        "nodes": graph()["nodes"] + list(proved),
        "verified_passive_roles": list(proved),
        "target_subscribers": ["drive", recorder],
    }
    validate_graph(plan, base)
    for bad in (
        {recorder: ("/", {"/rosout", plan.target_topic}, {"/clock"})},
        {recorder: ("/", {"/rosout"}, {"/unknown"})},
        {"vehicle_motion_recorder_other": endpoints[recorder]},
        {recorder: ("/other", {"/rosout"}, {"/clock"})},
        {"gz_ros2_control": ("/", {plan.target_topic}, set())},
    ):
        assert not verified_passive_roles(plan, bad)
    with pytest.raises(ValueError):
        validate_graph(plan, base | {"verified_passive_roles": []})
    for subscribers in (["drive", "drive", recorder], ["drive", recorder, "serial_driver"]):
        with pytest.raises(ValueError):
            validate_graph(plan, base | {"target_subscribers": subscribers})


def test_fortress_world_probe_uses_read_only_binary_and_preserves_timeout(monkeypatch):
    import subprocess
    from types import SimpleNamespace

    import tools.vehicle_motion_calibrate as calibration

    calls = []

    def run(command, **options):
        calls.append((command, options))
        return SimpleNamespace(stdout="/world/vehicle_motion_calibration/pose/info\n/other\n")

    monkeypatch.setattr(calibration.subprocess, "run", run)
    monkeypatch.setattr(calibration.Path, "is_file", lambda self: True)
    assert calibration.gazebo_worlds() == ["vehicle_motion_calibration"]
    assert calls[0][0][-1] == "-l" and "--pub" not in calls[0][0]
    assert calls[0][1]["timeout"] == 2.5

    def stalled(*args, **kwargs):
        raise subprocess.TimeoutExpired(args[0], 2)

    monkeypatch.setattr(calibration.subprocess, "run", stalled)
    with pytest.raises(subprocess.TimeoutExpired):
        calibration.gazebo_worlds()


@pytest.mark.parametrize("reject_world", [False, True])
def test_ros_readiness_probes_overlap_and_failure_never_creates_actuator(monkeypatch, reject_world):
    import sys
    import threading
    from types import SimpleNamespace as NS

    import tools.vehicle_motion_calibrate as calibration

    progressed = threading.Event()
    events = []
    callbacks = {}

    class Node:
        def create_subscription(self, cls, topic, callback, qos):
            callbacks[topic] = callback

        def create_client(self, *args):
            events.append("client_created")
            return object()

        def destroy_client(self, client):
            events.append("client_destroyed")

        def create_publisher(self, *args):
            events.append("actuator_created")
            return object()

        def destroy_node(self):
            events.append("node_destroyed")

    class Executor:
        ticks = 0

        def __init__(self, **kwargs):
            pass

        def add_node(self, node):
            pass

        def spin_once(self, **kwargs):
            self.ticks += 1
            callbacks["/clock"](NS(clock=NS(sec=self.ticks, nanosec=0)))
            callbacks["/ground_truth/odom"](
                NS(
                    header=NS(stamp=NS(sec=self.ticks, nanosec=0)),
                    pose=NS(pose=NS(position=NS(x=0.0, y=0.0))),
                    twist=NS(twist=NS(linear=NS(x=0.0, y=0.0), angular=NS(z=0.0))),
                )
            )
            progressed.set()

        def shutdown(self):
            events.append("executor_closed")

    ros = NS(
        Context=object,
        init=lambda **kwargs: None,
        create_node=lambda *args, **kwargs: Node(),
        shutdown=lambda **kwargs: events.append("context_closed"),
    )
    modules = {
        "rclpy": ros,
        "rclpy.executors": NS(SingleThreadedExecutor=Executor),
        "rclpy.qos": NS(qos_profile_sensor_data=object()),
        "geometry_msgs.msg": NS(Twist=object),
        "nav_msgs.msg": NS(Odometry=object),
        "rcl_interfaces.srv": NS(GetParameters=object),
        "rosgraph_msgs.msg": NS(Clock=object),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    def world_probe():
        assert progressed.wait(0.5), "ROS must progress while transport discovery waits"
        assert "client_created" in events
        return ["foreign" if reject_world else "vehicle_motion_calibration"]

    def preflight(self, client, worlds):
        assert self.publisher is None
        if worlds != ["vehicle_motion_calibration"]:
            raise ValueError("foreign world")

    monkeypatch.setattr(calibration, "gazebo_worlds", world_probe)
    monkeypatch.setattr(calibration.RosCalibrationPort, "_preflight", preflight)
    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    if reject_world:
        with pytest.raises(ValueError, match="foreign world"):
            calibration.RosCalibrationPort(plan)
        assert "actuator_created" not in events
    else:
        port = calibration.RosCalibrationPort(plan)
        assert events.index("client_destroyed") < events.index("actuator_created")
        port.close()
    assert {"client_destroyed", "executor_closed", "node_destroyed", "context_closed"} <= set(
        events
    )


def test_reviewed_scan_diagnostics_recorder_remains_read_only():
    from tools.vehicle_motion_calibrate import verified_passive_roles

    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    name = "vehicle_motion_recorder_review1"
    diagnostics = {
        "/safety/raw/scan",
        "/perception/scan_candidate",
        "/perception/scan_filtered",
        "/perception/scan_status",
    }
    endpoint = ("/", {"/rosout", "/events/write_split"}, diagnostics | {plan.target_topic})
    assert verified_passive_roles(plan, {name: endpoint}) == {name}
    # Diagnostic publication or arbitrary introspection are not observation rights.
    for publication in diagnostics | {plan.target_topic}:
        assert not verified_passive_roles(plan, {name: ("/", {publication}, diagnostics)})
    assert not verified_passive_roles(plan, {name: ("/", set(), diagnostics | {"/unreviewed"})})


def test_rejected_graph_preserves_exact_preflight_evidence():
    from tools.vehicle_motion_calibrate import GraphValidationError

    plan = CalibrationPlan.from_dict(plan_data(), hard_seconds=8)
    rejected = graph() | {"nodes": graph()["nodes"] + ["unexpected_node"]}
    with pytest.raises(GraphValidationError) as failure:
        validate_graph(plan, rejected)
    assert failure.value.graph == rejected
    assert "unexpected_node" in str(failure.value)
    # Diagnosis never changes the fail-closed result or mutates the observed graph.
    assert rejected["target_publishers"] == 0


def test_closed_bag_receipts_require_finalization_and_actual_samples(tmp_path):
    import hashlib
    import sqlite3

    from tools.vehicle_motion_calibrate import native_bag_receipts

    required = ("/clock", "/perception/scan_filtered")
    assert native_bag_receipts(tmp_path, required).missing == required
    assert list(tmp_path.iterdir()) == []  # Readiness never creates a database.
    db = tmp_path / "bag_0.db3"
    with sqlite3.connect(db) as connection:
        connection.executescript(
            "CREATE TABLE topics(id INTEGER,name TEXT); "
            "CREATE TABLE messages(id INTEGER,topic_id INTEGER); "
            "INSERT INTO topics VALUES(1,'/clock'),(2,'/perception/scan_filtered'); "
            "INSERT INTO messages VALUES(1,1);"
        )
    before = hashlib.sha256(db.read_bytes()).hexdigest()
    # A live recorder uses SQLite rollback locks: never query it for readiness.
    assert native_bag_receipts(tmp_path, required).storage_errors == ("bag_not_finalized",)
    (tmp_path / "metadata.yaml").write_text("closed bag fixture\n")
    state = native_bag_receipts(tmp_path, required)
    assert state.counts == {"/clock": 1, "/perception/scan_filtered": 0}
    assert state.missing == ("/perception/scan_filtered",)
    assert hashlib.sha256(db.read_bytes()).hexdigest() == before
    with sqlite3.connect(db) as connection:
        connection.execute("INSERT INTO messages VALUES(2,2)")
    assert native_bag_receipts(tmp_path, required).missing == ()
