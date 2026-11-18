"""Current startup lifecycle through the real output core; ROS I/O stays local."""

import json
import math
import sys
from collections import deque
from types import ModuleType, SimpleNamespace

import pytest
from control_process_test_support import generic_motion_profile
from learning_test_support import TestRun, learning_point
from senior_robot.motion_core_output import MotionCoreOutput
from senior_robot.raw_safety_observation import RawSafetyObservation
from sensor_time_test_support import capture_fields, sample_time
from test_motion_core_handoff import barrier

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionMode
from senior_control.motion.observations import Source


class LearningRig:
    def __init__(self, monkeypatch, tmp_path):
        self.now = 10.0
        self.sequence = 0
        self.cores = []
        self.run = TestRun(tmp_path / "core-learning")
        messages = ModuleType("std_msgs.msg")
        messages.String = lambda: SimpleNamespace(data="")
        monkeypatch.setitem(sys.modules, "std_msgs.msg", messages)
        monkeypatch.setenv("SENIOR_STARTUP_LEARNING_CLAIM", str(self.run.path))
        monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: self.now)
        monkeypatch.setattr(
            "senior_robot.motion_core_output.LearningClient", lambda _: self.run.client()
        )
        monkeypatch.setattr(
            "senior_control.motion.diagnostic_pump.DiagnosticPump",
            lambda _: SimpleNamespace(close=lambda: None),
        )
        monkeypatch.setattr(
            "senior_robot.runtime_reports.RuntimeReports",
            lambda node: SimpleNamespace(
                publish=lambda message: node.runtime.append(json.loads(message.data)),
                status=lambda _: None,
                close=lambda: None,
            ),
        )
        self.core = self.new_core()

    def new_core(self):
        node = SimpleNamespace(
            _body="base_link",
            _scans=deque(maxlen=8),
            emitted=[],
            runtime=[],
            get_parameter=lambda _: SimpleNamespace(value="robot-test"),
            _command_topology_ready=lambda: True,
        )
        core = MotionCoreOutput(node, generic_motion_profile())
        self.run.profile = core.profile

        def emit(velocity, reason, deadline=0.0, *, computed_steady):
            # Model a successful private submission after calculation entry.
            if velocity != Velocity(0, 0) and core.startup.started is None:
                assert self.run.owner.record.action.consumed
                self.now += 0.007
            node.emitted.append((self.now, velocity, reason, deadline, core.startup.started))
            return self.now

        node._emit_motion_core = emit
        self.cores.append(core)
        return core

    def step(self, now, *, wheel=0.0, new_scan=True, rear_occluded=False):
        self.now = now
        self.sequence += 1
        core = self.core
        ns = round(now * 1e9)
        for source, values in ((Source.WHEEL, (wheel, 0.0)), (Source.IMU, (0.0,))):
            point = learning_point(source, ns, ns, ns, values, sequence=self.sequence)
            assert core.sensors.ingest(point, now, now)
        core.sensors.input_reference = core.sensors.estimate(now, now, now_ros_ns=ns)
        if new_scan:
            ranges = [2.0] * 72
            ranges[18] = 0.15  # Body-connected stable side return, outside front/rear sectors.
            if rear_occluded:
                ranges[30:43] = [math.nan] * 13
            capture = SimpleNamespace(
                frame_id="laser_link",
                raw_gid="01",
                bridge_gid="02",
                source_epoch="scan",
                bridge_boot_id="scan-boot",
                capture_generation=1,
                sequence=self.sequence,
                valid=True,
                reason="",
                **capture_fields(
                    time=sample_time(
                        stamp_ns=ns,
                        received_ros_ns=ns,
                        received_steady_ns=ns,
                    )
                ),
            )
            message = SimpleNamespace(
                source_epoch="scan",
                sequence=self.sequence,
                frame_id="laser_link",
                ranges=ranges,
                angle_min=0.0,
                angle_increment=math.tau / 72,
                range_min=0.05,
                range_max=20.0,
                sensor_x=0.0,
                sensor_y=0.0,
                sensor_yaw=0.0,
                transform_stamp=now,
                raw_capture=capture,
            )
            raw = RawSafetyObservation(message, now, now, now, now)
            self.run.offer(raw)
            core.node._scans.append(raw)
        core.tick()
        return core.node.emitted[-1][1]

    def manual_attempt(self):
        from senior_control.protocol import LearningStartPayload
        from senior_control.runtime.learning_wire import BeginAttempt

        self.step(self.now + 0.05)
        prepared = barrier("prepared")
        prepared.owner_boot_id = "d" * 32
        assert self.core.command(prepared)
        self.run.pump()
        request = BeginAttempt(
            start_request_id="manual-round",
            fingerprint="a" * 64,
            payload=LearningStartPayload(expected_active_revision=0, expected_authority_id=None),
            owner_boot_id=self.core._input_boot,
            generation=self.core.output_generation,
            deadline_steady=self.now + 5,
            admitted=True,
        )
        assert self.run.owner.new_attempt("manual-round", "a" * 64, 0) == "accepted"
        self.run.service._reset_stream()
        self.run.service.proposal = request
        self.run.owner.record = self.run.owner.record.model_copy(update={"created_steady_ns": 0})
        self.run.pump()

    def start(self):
        for _ in range(8):
            if self.step(self.now + 0.05).vx > 0:
                return self.core.startup.started
        raise AssertionError(self.core.startup.summary())

    def close(self):
        for core in self.cores:
            core.close()
        self.run.close()


@pytest.fixture
def rig(monkeypatch, tmp_path):
    value = LearningRig(monkeypatch, tmp_path)
    yield value
    value.close()


def test_real_tick_collects_once_and_commits_three_second_action_after_idle_heartbeat(rig):
    heartbeat = barrier("prepared")
    heartbeat.source_kind, heartbeat.source_id = "idle", ""
    assert rig.core.command(heartbeat)
    assert rig.core.startup.active
    rig.step(rig.now + 0.05)
    frame_count = rig.run.service.accepted_frames
    rig.step(rig.now + 0.05, new_scan=False)
    assert rig.run.service.accepted_frames == frame_count
    start = rig.start()
    first = next(row for row in rig.core.node.emitted if row[1].vx > 0)
    assert first[4] is None  # The owner did not start timing before submission succeeded.
    assert start == first[0]
    assert rig.core.startup.deadline == start + 3.0
    assert rig.core.command(heartbeat) and rig.core.startup.started == start
    deadline = rig.core.startup.deadline
    while rig.now + 0.05 < deadline:
        rig.step(rig.now + 0.05, wheel=0.05)
    assert rig.step(deadline) == Velocity(0, 0)
    for _ in range(6):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    record = rig.run.owner.public_record
    assert record.phase == "completed" and record.model.anchors
    assert record.action.consumed and record.action.duration_ns == 3_000_000_000
    assert record.action.first_submitted_steady_ns == round(start * 1e9)
    assert record.action.deadline_steady_ns == round((start + 3) * 1e9)
    assert record.action.reported_nonzero_interval_ns == 3_000_000_000
    assert all(
        0 < v.vx <= 0.05 and v.wz == 0 and t < deadline
        for t, v, *_ in rig.core.node.emitted
        if v.vx
    )
    assert rig.core.startup.state == "completed" and rig.core.machine.state.task_id is None
    assert rig.core._learning_client.view.catalog == record.model
    rig.core.startup.hold_motion("learning_rear_occluded")
    assert rig.step(rig.now + 1.1, new_scan=False) == Velocity(0, 0)
    assert rig.core.startup.state == "completed"
    assert rig.core.machine.state.task_id is None
    assert rig.run.owner.public_record == record


def test_reserved_action_without_submission_is_terminal_when_rear_becomes_occluded(
    rig, monkeypatch
):
    client = rig.core._learning_client
    reserve = client.reserve_action
    requests = []

    def pending_reservation():
        requests.append(reserve())
        return False  # Durable reservation, with its acknowledgement still in flight.

    monkeypatch.setattr(client, "reserve_action", pending_reservation)
    for _ in range(8):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
        if requests:
            break
    assert requests == [True]
    action = rig.run.owner.public_record.action
    assert action.consumed and action.stage == "reserved_may_have_output"
    assert action.first_submitted_steady_ns is None and action.deadline_steady_ns is None
    assert action.reported_nonzero_interval_ns == 0
    assert rig.core.startup.started is None

    assert rig.step(rig.now + 0.05, rear_occluded=True) == Velocity(0, 0)
    assert rig.core.startup.state == "interrupted"
    assert rig.core.startup.reason == "learning_rear_occluded"
    assert rig.core.machine.state.task_id is None
    assert rig.core.target.source == "idle"
    record = rig.run.owner.public_record
    assert record.phase == "failed" and record.reason == "learning_rear_occluded"
    assert record.action == action and record.model is None

    rig.core = rig.new_core()
    for _ in range(8):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.core.startup.state == "interrupted"
    assert rig.core.machine.state.task_id is None
    assert not rig.core.startup.prepare_action()
    assert rig.run.owner.public_record.action == action
    assert requests == [True]


def test_manual_prepared_stops_learning_and_same_run_does_not_resume(rig):
    rig.start()
    assert rig.core.command(barrier("prepared"))
    assert rig.core.startup.state == "interrupted"
    assert rig.core.target.velocity == Velocity(0, 0)
    for _ in range(8):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.run.owner.record.phase == "failed"
    rig.core = rig.new_core()
    for _ in range(8):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.core.startup.state == "interrupted"
    assert rig.core.machine.state.mode is MotionMode.IDLE


def test_manual_prepared_before_first_output_is_terminal_on_same_run_reconstruction(rig):
    assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.core.startup.active and not rig.run.owner.record.action.consumed
    assert rig.core.command(barrier("prepared"))
    assert rig.run.owner.record.phase == "failed"
    rig.core = rig.new_core()
    for _ in range(8):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.core.startup.state == "interrupted"
    assert not rig.run.owner.record.action.consumed


def test_consumed_running_action_survives_component_reconstruction_without_replay(rig):
    start = rig.start()
    action = rig.run.owner.record.action
    assert action.consumed and action.first_submitted_steady_ns == round(start * 1e9)
    rig.core._learning_client.close()  # Consumer loss, without an orderly phase rewrite.
    rig.core = rig.new_core()
    for _ in range(8):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.core.startup.motion_withdrawn
    assert rig.core.startup.state == "interrupted"
    assert rig.core.startup.reason == "consumed_action_no_replay"
    assert rig.core.startup.started is None
    assert rig.core.machine.state.task_id is None
    assert not rig.core.startup.prepare_action()
    assert rig.run.owner.record.phase == "failed"
    assert rig.run.owner.record.action == action


def test_explicit_estop_interrupts_action_and_replayed_close_does_not_reset_latch(rig):
    rig.start()
    close = barrier("estop", request="operator-stop")
    assert rig.core.command(close)
    revision = rig.core.machine.state.estop_revision
    assert rig.core.command(close)
    assert rig.core.machine.state.estop_revision == revision
    for _ in range(8):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.core.startup.state == "interrupted"
    assert rig.core.machine.state.mode is MotionMode.ESTOP
    assert rig.core.node.runtime[-1]["confirmed_estop_status"] == "software_latched"
    assert "acquire" not in rig.core.node.runtime[-1]["allowed_actions"]


def test_slow_compute_does_not_extend_motion_and_complete_capture_survives_takeover(rig):
    rig.run.service.compute.held = True
    start = rig.start()
    while rig.now + 0.05 < start + 3:
        rig.step(rig.now + 0.05, wheel=0.05)
    assert rig.step(start + 3) == Velocity(0, 0)
    assert rig.core.machine.state.task_id is None
    for _ in range(8):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.run.owner.record.capture_summary.state == "complete"
    assert rig.run.owner.record.model is None
    assert rig.core.command(barrier("prepared"))
    assert rig.core.target.velocity == Velocity(0, 0)
    action = rig.run.owner.record.action
    rig.run.service.compute.held = False
    for _ in range(16):
        rig.step(rig.now + 0.05)
    assert rig.run.owner.active_record.model is not None
    assert rig.run.owner.record.action == action
    assert rig.core.machine.state.task_id is None
    assert rig.core.target.source == "idle"


def test_new_calibration_request_is_rejected_if_core_source_generation_changed(rig):
    from senior_control.protocol import LearningStartPayload
    from senior_control.runtime.learning_wire import BeginAttempt

    rig.step(rig.now + 0.05)
    prepared = barrier("prepared")
    prepared.owner_boot_id = "d" * 32
    assert rig.core.command(prepared)
    rig.run.pump()
    assert rig.run.owner.new_attempt("manual-new", "a" * 64, 0) == "accepted"
    rig.run.owner.record = rig.run.owner.record.model_copy(update={"created_steady_ns": 0})
    rig.run.pump()
    rig.run.service._reset_stream()
    rig.run.service.proposal = BeginAttempt(
        start_request_id="manual-new",
        fingerprint="a" * 64,
        payload=LearningStartPayload(expected_active_revision=0, expected_authority_id=None),
        owner_boot_id=rig.core._input_boot,
        generation=rig.core.output_generation + 1,
        deadline_steady=rig.now + 1,
        admitted=True,
    )
    assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.run.owner.record.phase == "failed"
    assert rig.run.owner.record.reason == "control_source_changed"
    assert not rig.run.owner.record.action.consumed


def test_consumed_manual_attempt_reconstruction_does_not_require_a_new_proposal_or_replay(rig):
    rig.manual_attempt()
    start = rig.start()
    action = rig.run.owner.record.action
    assert rig.run.owner.record.trigger == "manual" and action.consumed
    assert rig.run.service.proposal is None
    rig.core._learning_client.close()
    rig.core = rig.new_core()
    for _ in range(8):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert rig.core.startup.state == "interrupted"
    assert rig.core.startup.started is None and not rig.core.startup.prepare_action()
    assert rig.run.owner.record.action == action
    assert action.first_submitted_steady_ns == round(start * 1e9)
