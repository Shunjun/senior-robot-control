"""Current startup lifecycle through the real output core; ROS I/O stays local."""

import json
import math
import pickle
import sys
from collections import deque
from dataclasses import asdict, replace
from types import ModuleType, SimpleNamespace

import pytest
from control_process_test_support import generic_motion_profile
from learning_test_support import TestRun, learning_point
from senior_motion_control.motion_core_output import MotionCoreOutput
from senior_motion_control.vehicle_speed import limit_velocity as reference_limit_velocity
from senior_motion_estimation.observations import Source
from senior_perception.raw_safety_observation import RawSafetyObservation
from sensor_time_test_support import capture_fields, sample_time
from test_motion_core_handoff import barrier
from test_native_output_protocol import output_module  # noqa: F401

from vehicle_core.motion_state import MotionMode
from vehicle_core.values import Velocity


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
        monkeypatch.setattr(
            "senior_motion_control.motion_core_output.time.monotonic", lambda: self.now
        )
        monkeypatch.setattr(
            "senior_perception.learning_runtime.LearningClient",
            lambda _, **kwargs: self.run.client(**kwargs),
        )
        monkeypatch.setattr(
            "senior_motion_control.diagnostic_pump.DiagnosticPump",
            lambda _: SimpleNamespace(close=lambda: None),
        )
        monkeypatch.setattr(
            "senior_motion_control.runtime_reports.RuntimeReports",
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
                raw_source="01",
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
        from senior_contracts.wire import LearningStartPayload
        from senior_perception.learning_wire import BeginAttempt

        self.step(self.now + 0.05)
        prepared = barrier("prepared")
        prepared.owner_boot_id = "d" * 32
        prepared.generation = max(prepared.generation, self.core.output_generation + 1)
        assert self.core.command(prepared)
        self.run.pump()
        request = BeginAttempt(
            start_request_id="manual-round",
            fingerprint="a" * 64,
            payload=LearningStartPayload(expected_active_revision=0, expected_authority_id=None),
            owner_boot_id=self.core._input_boot,
            output_boot_id=self.core.boot,
            generation=self.core.output_generation,
            deadline_steady=self.now + 5,
            admitted=True,
        )
        assert (
            self.run.owner.new_attempt("manual-round", "a" * 64, 0, output_boot_id=self.core.boot)
            == "accepted"
        )
        self.run.service._reset_capture()
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
    # The learning owner returns a retirement request without changing arbitration.
    snapshot = rig.core._learning_control_input()
    previous_target = rig.core.target
    retirement = rig.core.learning._interrupt_learning("control_takeover", rig.now, snapshot)
    assert rig.core.target is previous_target
    assert rig.core.command(barrier("prepared"))
    current_target, sequence = rig.core.target, rig.core.event_sequence
    # A delayed predecessor cannot retire the newer prepared authority.
    assert not rig.core.arbitration.apply_learning_request(retirement, rig.now)
    assert rig.core.target is current_target and rig.core.event_sequence == sequence
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


def test_expired_candidate_after_first_submission_keeps_original_grant_and_window(rig, monkeypatch):
    from senior_motion_control import vehicle_speed

    reserve = rig.run.owner.reserve_action
    reservations = []

    def reserve_once(session):
        reservations.append(session)
        return reserve(session)

    monkeypatch.setattr(rig.run.owner, "reserve_action", reserve_once)
    start = rig.start()
    deadline = rig.core.startup.deadline
    grant = rig.run.owner._reservation
    assert rig.core.startup.action_permitted and len(reservations) == 1
    original_limit = vehicle_speed.limit_velocity
    entered = rig.now + 0.05

    def exhaust_current_frame(*args, **kwargs):
        output = original_limit(*args, **kwargs)
        rig.now += 0.251  # Exhaust this calculation's 250 ms output budget.
        return output

    with monkeypatch.context() as stalled:
        stalled.setattr(vehicle_speed, "limit_velocity", exhaust_current_frame)
        assert rig.step(entered) == Velocity(0, 0)
    expired = rig.core.node.emitted[-1]
    assert expired[2] == "learning_output_expired"
    assert expired[3] == entered + 0.25 < expired[0] < deadline
    assert rig.core.startup.started == start and rig.core.startup.deadline == deadline
    assert rig.core.startup.action_permitted and not rig.core.startup.motion_withdrawn
    assert rig.run.owner._reservation == grant
    assert len(reservations) == 1

    assert rig.step(rig.now + 0.05).vx > 0
    resumed = rig.core.node.emitted[-1]
    assert expired[3] < resumed[3] <= deadline
    assert rig.core.startup.started == start and rig.core.startup.deadline == deadline
    while rig.now + 0.05 < deadline:
        rig.step(rig.now + 0.05, wheel=0.05)
    assert rig.step(deadline) == Velocity(0, 0)
    assert rig.step(deadline + 0.05) == Velocity(0, 0)
    action = rig.run.owner.record.action
    assert action.stage == "window_finished"
    assert action.first_submitted_steady_ns == round(start * 1e9)
    assert action.deadline_steady_ns == round(deadline * 1e9)
    assert rig.run.owner._reservation == grant and len(reservations) == 1


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
    from senior_contracts.wire import LearningStartPayload
    from senior_perception.learning_wire import BeginAttempt

    rig.step(rig.now + 0.05)
    prepared = barrier("prepared")
    prepared.owner_boot_id = "d" * 32
    assert rig.core.command(prepared)
    rig.run.pump()
    assert (
        rig.run.owner.new_attempt("manual-new", "a" * 64, 0, output_boot_id=rig.core.boot)
        == "accepted"
    )
    rig.run.owner.record = rig.run.owner.record.model_copy(update={"created_steady_ns": 0})
    rig.run.service._reset_capture()
    rig.run.pump()
    rig.run.service.proposal = BeginAttempt(
        start_request_id="manual-new",
        fingerprint="a" * 64,
        payload=LearningStartPayload(expected_active_revision=0, expected_authority_id=None),
        owner_boot_id=rig.core._input_boot,
        output_boot_id=rig.core.boot,
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


def test_recovered_core_stays_read_only_until_handoff_then_can_run_a_new_explicit_attempt(
    rig, monkeypatch
):
    # Existing reservation is durable but no raw stream or successful output exists.
    session = rig.core._learning_client.session
    rig.run.owner.reserve_action(session)
    rig.run.pump()
    old = rig.run.owner.record
    monkeypatch.setenv("SENIOR_LAUNCH_RECOVERED", "1")
    rig.core = rig.new_core()
    assert rig.step(rig.now + 0.05, new_scan=False) == Velocity(0, 0)
    assert rig.core._learning_client.read_only and not rig.core.startup.active
    assert rig.run.owner.record.phase != "failed"
    assert rig.run.owner.record.action == old.action

    echo = barrier("prepared")
    echo.owner_boot_id = "d" * 32
    echo.source_kind, echo.source_id, echo.request_id = "idle", rig.core.boot, rig.core.boot
    echo.source_epoch = 0
    assert rig.core.command(echo)
    assert rig.step(rig.now + 0.05, new_scan=False) == Velocity(0, 0)
    assert rig.core._launch_recovered and not rig.core._recovery_pending
    assert rig.run.owner.record.phase == "failed"
    assert rig.run.owner.record.reason == "motion_owner_replaced"
    assert rig.run.owner.record.action == old.action
    assert rig.core._learning_client.read_only
    rig.manual_attempt()
    start = rig.start()
    assert start is not None and rig.core._launch_recovered
    assert rig.run.owner.record.expected_output_boot_id == rig.core.boot
    assert rig.run.owner.record.attempt_id != old.attempt_id
    assert rig.core._old_learning_owner.attempt_id == old.attempt_id
    assert rig.run.owner.record.action.first_submitted_steady_ns == round(start * 1e9)


def uncached_scan(core, raw, *, avoidance=False):
    import senior_perception.scan_decode as perception
    from senior_perception.learning_view import LearningFilterView
    from senior_perception.self_learning import SelfReturnModel

    decode = perception.avoidance_observation if avoidance else perception.scan_observation
    expected = decode(raw, core.node._body)
    view = core._learning_client.view
    if view is not None:
        expected = LearningFilterView(view.catalog, view.geometry).filter(expected)
    elif core.self_returns is not None:
        expected = SelfReturnModel(core._learning_geometry.physical_polygon).filter(
            expected, provisional=core.startup.provisional
        )
    return expected


def assert_original_scan_result(core, raw, actual, now, *, avoidance=False):
    perception_limit_velocity = reference_limit_velocity
    from senior_perception.collision import (
        directional_clearance as perception_directional_clearance,
    )
    from senior_perception.startup_visibility import startup_visibility

    expected = uncached_scan(core, raw, avoidance=avoidance)
    # JSON gives NaN its original nonfinite category; finite coordinates compare exactly.
    assert json.dumps(asdict(actual), sort_keys=True) == json.dumps(
        asdict(expected), sort_keys=True
    )
    flags = dict(front_stop=True, rear_stop=True)
    assert startup_visibility(actual, **flags) == startup_visibility(expected, **flags)
    clearance = perception_directional_clearance([raw], core.profile, now, filtered=actual)
    reference = perception_directional_clearance([raw], core.profile, now, filtered=expected)
    assert clearance == reference
    if clearance is not None:
        arguments = (Velocity(0.05, 0), Velocity(0.01, 0), 0.0)
        assert perception_limit_velocity(*arguments, clearance, core.profile, 0.05) == (
            perception_limit_velocity(*arguments, reference, core.profile, 0.05)
        )
    return clearance


def checked_current_velocity(core, now, function, *arguments):
    from senior_perception.collision import directional_clearance

    raw = core.node._scans[-1]
    filtered = uncached_scan(core, raw, avoidance=bool(raw.predictions))
    clearance = directional_clearance([raw], core.profile, now, filtered=filtered)
    request, previous, measured, actual_clearance, profile, dt = arguments
    assert actual_clearance == clearance
    actual = function(*arguments)
    assert actual == function(request, previous, measured, clearance, profile, dt)
    return actual


def test_reconstructed_current_scan_reuses_facts_but_updates_output_and_expires(rig, monkeypatch):
    from senior_motion_control import vehicle_speed
    from senior_perception import learning_runtime as motion_core_output

    rig.start()
    calls = []
    decode = motion_core_output.scan_observation
    filter_scan = rig.core.self_returns.filter

    def decoded(*args):
        calls.append("decode")
        return decode(*args)

    def filtered(*args, **kwargs):
        calls.append("filter")
        return filter_scan(*args, **kwargs)

    monkeypatch.setattr(motion_core_output, "scan_observation", decoded)
    monkeypatch.setattr(rig.core.self_returns, "filter", filtered)
    limit = vehicle_speed.limit_velocity
    monkeypatch.setattr(
        vehicle_speed,
        "limit_velocity",
        lambda *args: checked_current_velocity(rig.core, rig.now, limit, *args),
    )
    velocities = [rig.step(rig.now + 0.05)]
    raw = rig.core.node._scans[-1]
    deadlines = [rig.core.node.emitted[-1][3]]
    for _ in range(2):
        rebuilt = pickle.loads(pickle.dumps(raw))  # Same identity, different snapshot objects.
        rig.core.node._scans.clear()
        rig.core.node._scans.append(rebuilt)
        velocities.append(rig.step(rig.now + 0.05, new_scan=False))
        deadlines.append(rig.core.node.emitted[-1][3])
        assert deadlines[-1] == rig.now + 0.25
        actual = rig.core._scan_results[0][3]
        assert_original_scan_result(rig.core, rebuilt, actual, rig.now)
    assert calls == ["decode", "filter"]
    assert velocities[0] != velocities[-1] and all(value.vx > 0 for value in velocities)
    assert deadlines[0] < deadlines[1] < deadlines[2]
    from vehicle_core.sensor_policy import default_policy

    expires = raw.measurement_steady + default_policy().hard_age_s
    assert all(deadline <= expires for deadline in deadlines)
    assert rig.step(expires + 1e-6, new_scan=False) == Velocity(0, 0)
    assert calls == ["decode", "filter"]
    assert rig.core.startup.motion_withdrawn  # Cached facts cannot make an expired scan current.


def test_current_scan_keys_invalidate_for_identity_payload_geometry_and_filter_versions(
    rig, monkeypatch
):

    from senior_perception.learning_record import LearnedCatalog, LearningAnchor
    from senior_perception.learning_view import LearningFilterView
    from senior_perception.self_learning import SelfReturnModel

    rig.start()
    calls = []
    filter_scan = SelfReturnModel.filter

    def filtered(model, *args, **kwargs):
        client = rig.core._learning_client
        if model is rig.core.self_returns or (
            client.view is not None and model is client.view._model
        ):
            calls.append(model)
        return filter_scan(model, *args, **kwargs)

    monkeypatch.setattr(SelfReturnModel, "filter", filtered)
    rig.step(rig.now + 0.05)
    raw = rig.core.node._scans[-1]
    assert len(calls) == 1

    def check(value):
        before = len(calls)
        actual = rig.core._filtered_scan(value)
        assert len(calls) == before + 1
        assert_original_scan_result(rig.core, value, actual, rig.now)
        assert rig.core._filtered_scan(pickle.loads(pickle.dumps(value))) is actual
        assert len(calls) == before + 1
        return actual

    source = pickle.loads(pickle.dumps(raw))
    source.message.source_epoch += "-replacement"
    source.message.raw_capture.source_epoch += "-replacement"  # Same sequence, different source.
    check(source)
    payload = pickle.loads(pickle.dumps(source))
    payload.message.ranges[0] += 0.1  # Same capture identity; the value snapshot still invalidates.
    before_transform = check(payload)
    model = rig.core.self_returns
    transformed = pickle.loads(pickle.dumps(payload))
    transformed.message.sensor_x += 0.01
    transformed.message.transform_stamp += 0.01
    after_transform = check(transformed)
    assert after_transform.rays[0].endpoint != before_transform.rays[0].endpoint
    assert rig.core.startup.provisional and rig.core.self_returns is model
    assert rig.core._learning_geometry.sensor_to_body[0] == transformed.message.sensor_x
    rig.core.startup.hold_motion("scan_temporarily_unavailable")
    assert not rig.core.startup.provisional
    check(transformed)
    layout = pickle.loads(pickle.dumps(transformed))
    layout.message.angle_min += 0.01
    check(layout)
    assert rig.core.startup.reason == "scan_or_body_geometry_changed"

    client = rig.core._learning_client
    for revision, point in ((1, (0.01, 0.15)), (2, (0.2, 0.1))):
        catalog = LearnedCatalog(
            model_id=str(revision) * 32, anchors=(LearningAnchor(id=0, point=point),)
        )
        client.view = LearningFilterView(catalog, rig.core._learning_geometry)
        client.active_revision = revision
        check(transformed)

    incomplete = pickle.loads(pickle.dumps(transformed))
    del incomplete.message.raw_source
    before = len(calls)
    for _ in range(2):
        actual = rig.core._filtered_scan(incomplete)
        assert_original_scan_result(rig.core, incomplete, actual, rig.now)
    assert len(calls) == before + 2 and rig.core._scan_results == [None, None]


def test_prediction_changes_only_invalidate_avoidance_and_keep_original_learning_raw(
    rig, monkeypatch
):
    from senior_motion_control import vehicle_speed
    from senior_perception import learning_runtime as motion_core_output
    from senior_perception.raw_safety_observation import PredictedReturn
    from senior_perception.vision.raw_capture import capture_from_message

    rig.start()
    seed = capture_from_message(rig.core.node._scans[-1].message.raw_capture)
    other_seed = capture_from_message(rig.core.node._scans[-2].message.raw_capture)
    offer = rig.run.offer
    batches = []
    calls = []
    decode, avoid = motion_core_output.scan_observation, motion_core_output.avoidance_observation

    def original_frame(raw):
        raw.message.ranges[0] = math.inf
        batch = offer(raw)
        batches.append(batch)
        return batch

    def decoded(*args):
        calls.append("original")
        return decode(*args)

    def avoided(*args):
        calls.append("avoidance")
        return avoid(*args)

    monkeypatch.setattr(rig.run, "offer", original_frame)
    monkeypatch.setattr(motion_core_output, "scan_observation", decoded)
    monkeypatch.setattr(motion_core_output, "avoidance_observation", avoided)
    limit = vehicle_speed.limit_velocity
    monkeypatch.setattr(
        vehicle_speed,
        "limit_velocity",
        lambda *args: checked_current_velocity(rig.core, rig.now, limit, *args),
    )
    rig.step(rig.now + 0.05)
    raw = rig.core.node._scans[-1]
    original = rig.core._scan_results[0][3]
    boundary = rig.core.profile.length / 2 * 1.10 + rig.core.profile.sensor_error
    prediction = PredictedReturn(0, boundary, 1, seed)

    def step_with(value):
        current = replace(raw, predictions=(value,))
        rig.core.node._scans.clear()
        rig.core.node._scans.append(pickle.loads(pickle.dumps(current)))
        output = rig.step(rig.now + 0.05, new_scan=False)
        assert rig.core._scan_results[0][3] is original
        actual = rig.core._scan_results[1][3]
        clearance = assert_original_scan_result(rig.core, current, actual, rig.now, avoidance=True)
        assert_original_scan_result(rig.core, current, original, rig.now)
        return output, clearance

    output, clearance = step_with(prediction)
    assert output == Velocity(0, 0) and clearance.front == 0.0
    step_with(prediction)  # Rebuilt snapshot, unchanged prediction: both slots hit.
    assert calls == ["original", "avoidance"]
    prediction = replace(prediction, distance=boundary + 0.1, missing_frames=2)
    assert step_with(prediction)[0].vx > 0
    step_with(replace(prediction, seed_capture=other_seed))
    assert calls == ["original", "avoidance", "avoidance", "avoidance"]
    assert raw.message.ranges[0] == math.inf and original.scan.ranges[0] == math.inf
    assert batches[0].frames[0].ranges[0] == "inf"
    assert batches[0].frames[0].capture == capture_from_message(raw.message.raw_capture)


def test_async_incomplete_before_submission_retires_target_without_consuming(rig, monkeypatch):
    reservations = []
    reserve = rig.run.owner.reserve_action
    monkeypatch.setattr(
        rig.run.owner,
        "reserve_action",
        lambda session: (reservations.append(session), reserve(session))[1],
    )
    action = rig.run.owner.record.action
    assert not action.consumed
    from senior_perception.learning_wire import MotionObservedThrough

    rig.run.rpc(
        MotionObservedThrough(
            session=rig.core._learning_client.session,
            attempt_id=rig.run.owner.record.attempt_id,
            segment_id=rig.core.startup.segment_id,
            through_steady_ns=round(rig.now * 1e9),
            stationary=True,
            gap=True,
        )
    )
    rig.run.pump()
    assert rig.run.owner.record.reason == "learning_motion_facts_incomplete"
    for _ in range(3):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
        assert not rig.core.startup.active
        assert rig.core.startup.reason == "learning_motion_facts_incomplete"
        assert rig.core.target.source == "idle" and rig.core.machine.state.task_id is None
    assert rig.run.owner.record.action == action and not action.consumed
    assert reservations == []


def test_async_failure_after_submission_preserves_window_then_projects_terminal(rig):
    start = rig.start()
    deadline = rig.core.startup.deadline
    grant = rig.run.owner._reservation
    from senior_perception.learning_wire import MotionObservedThrough

    rig.run.rpc(
        MotionObservedThrough(
            session=rig.core._learning_client.session,
            attempt_id=rig.run.owner.record.attempt_id,
            segment_id=rig.core.startup.segment_id,
            through_steady_ns=round(rig.now * 1e9),
            stationary=True,
            gap=True,
        )
    )
    rig.run.pump()
    assert rig.run.owner.record.phase == "failed"
    while rig.now + 0.05 < deadline:
        assert rig.step(rig.now + 0.05, wheel=0.05).vx > 0
        assert rig.core.startup.started == start and rig.core.startup.deadline == deadline
        assert rig.run.owner._reservation == grant
    assert rig.step(deadline) == Velocity(0, 0)
    assert rig.core.startup.reason == "learning_motion_facts_incomplete"
    assert not rig.core.startup.active and rig.core.target.source == "idle"
    action = rig.run.owner.record.action
    assert action.stage == "window_finished"
    assert action.first_submitted_steady_ns == round(start * 1e9)
    assert action.deadline_steady_ns == round(deadline * 1e9)
    assert rig.step(deadline + 0.05) == Velocity(0, 0)
    assert rig.run.owner.record.action == action


def test_completed_view_zero_target_skips_filter_but_manual_output_keeps_original_facts(
    rig, monkeypatch
):
    from senior_motion_control import vehicle_speed
    from senior_perception import learning_runtime as motion_core_output

    start = rig.start()
    while rig.now + 0.05 < start + 3:
        rig.step(rig.now + 0.05, wheel=0.05)
    rig.step(start + 3)
    for _ in range(8):
        rig.step(rig.now + 0.05)
    client = rig.core._learning_client
    assert client.view is not None and rig.core.startup.state == "completed"
    assert rig.core.target.source == "idle"
    # The transport fixture lacks the real client's closed-capture observation guard.
    from senior_perception.learning_client import LearningClient

    monkeypatch.setattr(
        client,
        "observed",
        lambda **kwargs: LearningClient.observed(
            SimpleNamespace(
                read_only=client.read_only, _session=client.session, _record=client.record
            ),
            **kwargs,
        ),
    )
    calls = dict(poll=0, advance=0, decode=0, filter=0)

    def counted(name, function):
        def call(*args, **kwargs):
            calls[name] += 1
            return function(*args, **kwargs)

        return call

    monkeypatch.setattr(client, "poll", counted("poll", client.poll))
    monkeypatch.setattr(rig.core.startup, "advance", counted("advance", rig.core.startup.advance))
    monkeypatch.setattr(
        motion_core_output,
        "scan_observation",
        counted("decode", motion_core_output.scan_observation),
    )
    monkeypatch.setattr(client.view, "filter", counted("filter", client.view.filter))
    for _ in range(3):
        assert rig.step(rig.now + 0.05) == Velocity(0, 0)
    assert calls["poll"] >= 3  # The in-process record property also pumps replies.
    assert calls["advance"] == 3 and calls["decode"] == calls["filter"] == 0
    assert rig.core.startup.state == "completed" and rig.core.machine.state.task_id is None

    assert rig.core.command(barrier("prepared"))
    intent = barrier("intent")
    intent.received_steady, intent.steady_deadline = rig.now, rig.now + 5
    intent.manual_linear, intent.manual_linear_limit, intent.degraded_linear_limit = 1, 0.05, 0.05
    intent.vx = 0.05
    assert rig.core.command(intent)
    limit = vehicle_speed.limit_velocity
    monkeypatch.setattr(
        vehicle_speed,
        "limit_velocity",
        lambda *args: checked_current_velocity(rig.core, rig.now, limit, *args),
    )
    assert rig.step(rig.now + 0.05).vx > 0
    assert calls["decode"] == calls["filter"] == 1
    raw = rig.core.node._scans[-1]
    filtered = rig.core._scan_results[0][3]
    clearance = assert_original_scan_result(rig.core, raw, filtered, rig.now)
    assert rig.core.node.emitted[-1][3] == min(
        rig.now + 0.25, intent.steady_deadline, clearance.deadline
    )
    assert rig.step(clearance.deadline + 1e-6, new_scan=False) == Velocity(0, 0)
    assert rig.core.target.source == "manual" and rig.core.startup.state == "completed"


def test_started_filter_failure_is_sticky_in_status_without_changing_hold(rig, monkeypatch):
    from senior_perception.vision.raw_capture import capture_from_message

    start = rig.start()
    deadline, reason = rig.core.startup.deadline, rig.core.startup.reason
    now = rig.now + 0.05

    def fail_filter(*_args, **_kwargs):
        raise ValueError("deliberate filter boundary failure")

    with monkeypatch.context() as failure:
        failure.setattr(rig.core.self_returns, "filter", fail_filter)
        assert rig.step(now) == Velocity(0, 0)
    assert rig.core.startup.state == "braking" and rig.core.startup.motion_withdrawn
    assert rig.core.startup.reason == "learning_scan_unavailable"
    assert rig.core.startup.started == start and rig.core.startup.deadline == deadline
    selected = rig.core.node._scans[-1]
    statuses = []
    monkeypatch.setattr(rig.core._runtime, "status", statuses.append)
    value = dict(
        body="base_link",
        reason="learning_scan_unavailable",
        values={"timing.sampled_steady": str(now - 0.05), "timing.tick_ms": "1.2"},
    )
    rig.core.report_status(value)
    assert "learning_failure" not in value["values"]
    fact = json.loads(statuses[-1]["values"]["learning_failure"])
    assert fact["boot"] == rig.core.boot and fact["sequence"] == rig.core.sequence
    assert fact["now"] == now and fact["stage"] == "filter"
    assert fact["exception"] == dict(
        type="ValueError", message="deliberate filter boundary failure"
    )
    assert fact["selected_count"] == len(rig.core.node._scans)
    assert fact["scan_generation"] is None
    assert fact["selected"]["capture"] == json.loads(
        json.dumps(asdict(capture_from_message(selected.message.raw_capture)))
    )
    assert fact["selected"]["bounded_sequence"] == selected.message.sequence
    assert fact["selected"]["bounded_source"] == selected.message.source_epoch
    assert fact["selected"]["measurement"] == selected.measurement_steady
    assert fact["selected"]["receipt"] == selected.received
    assert fact["startup"] == dict(state="moving", reason=reason, start=start, deadline=deadline)
    assert fact["age"] == dict(band="normal", seconds=0.0)
    assert fact["client_error"] is None and fact["record_phase"] == "moving"
    assert len(statuses[-1]["values"]["learning_failure"].encode()) < 4096
    original = statuses[-1]["values"]["learning_failure"]
    assert rig.step(now + 0.05) == Velocity(0, 0)
    assert rig.core.startup.state == "validating" and rig.core.target.source == "idle"
    rig.core.report_status(
        dict(body="base_link", reason="standby", values={"timing.sampled_steady": str(now + 0.05)})
    )
    assert statuses[-1]["values"]["learning_failure"] == original
    assert statuses[-1]["values"]["timing.sampled_steady"] == str(now + 0.05)


def test_core_trace_preserves_velocities_deadlines_ties_and_scan_expiry(
    monkeypatch, tmp_path, request
):
    import gc
    import os
    import socket

    from senior_motion_control import vehicle_speed
    from senior_perception.scan_decode import scan_observation
    from senior_perception.startup_learning import StartupLearning

    gc_settings = gc.isenabled(), gc.get_threshold()
    module = request.getfixturevalue("output_module")
    results, traced_rows = [], []
    limit = vehicle_speed.limit_velocity

    def limit_with_real_clearance(*args):
        # Infinity is a valid unobstructed direction, never a diagnostic string in control.
        assert isinstance(args[3].rear, float) and args[3].rear == math.inf
        return limit(*args)

    monkeypatch.setattr(vehicle_speed, "limit_velocity", limit_with_real_clearance)
    for enabled in (False, True):
        label = tmp_path / str(enabled)
        label.mkdir()
        r = LearningRig(monkeypatch, label)
        offer = r.run.offer

        def production_identity(raw, offer=offer):
            # Original identity values from deadline-trace-1795060825; clocks stay controlled.
            m, capture = raw.message, raw.message.raw_capture
            m.source_epoch = "a27af8563d13499fa332b41651a1bc0a"
            capture.raw_gid = "010fb8ff7d38fc2201000000000013030000000000000000"
            capture.bridge_gid = "010fb8ff43387b4d01000000000012030000000000000000"
            capture.source_epoch = "0ebbf0f147719304bccce28a8e43078c-3"
            capture.bridge_boot_id = "679a993742619426ac726741b8239be6-1"
            m.frame_id = capture.frame_id = "laser"
            m.raw_source = capture.raw_gid
            m.ranges[19:54] = [math.inf] * 35
            return offer(raw)

        monkeypatch.setattr(r.run, "offer", production_identity)
        reader, writer = os.pipe()
        parent, peer = socket.socketpair()
        os.set_blocking(reader, False)
        open_fd = os.open
        if sys.platform != "linux":
            os.set_blocking(writer, False)

        def open_trace(path, flags, open_fd=open_fd, writer=writer):
            if path == "/proc/self/fd/2":
                return (
                    open_fd(f"/proc/self/fd/{writer}", flags)
                    if sys.platform == "linux"
                    else os.dup(writer)
                )
            return open_fd(path, flags)

        monkeypatch.setattr("time.monotonic_ns", lambda r=r: round(r.now * 1e9))
        with monkeypatch.context() as mode:
            if enabled:
                mode.setenv("SENIOR_MOTION_TRACE_ID", "core-test")
                mode.setenv("SENIOR_MOTION_TRACE_START_NS", "9000000000")
                mode.setenv("SENIOR_MOTION_TRACE_END_NS", "20000000000")
            else:
                mode.delenv("SENIOR_MOTION_TRACE_ID", raising=False)
            mode.setattr(os, "open", open_trace)
            output = module.MotionOutput.__new__(module.MotionOutput)
            output._init_trace()
        if enabled and sys.platform != "linux":
            # C1 models the target Linux 4096-byte atomic envelope; P1 uses the real host limit.
            output._trace_size = 4096
        output.channel, output.sequence = parent, 0
        output.process = SimpleNamespace(poll=lambda: None)
        r.core.node._output_transport = output
        r.core.learning.startup = StartupLearning(None)
        r.core.learning._learning_attempt = r.run.owner.record.attempt_id
        packets, emissions, rays = [], [], []

        def emit(velocity, reason, deadline, *, computed_steady, r=r, output=output):
            output.publish(velocity, deadline, computed_steady=computed_steady)
            r.core.node.emitted.append((r.now, velocity, reason, deadline, r.core.startup.started))
            output.flush_send_trace()
            return r.now

        r.core.node._emit_motion_core = emit
        try:
            assert r.core.command(barrier("prepared"))
            intent = barrier("intent")
            intent.owner_boot_id = "f4aba0660bc8419ebc36d1c94eb38c13"
            intent.session_id = "679a993742619426ac726741b8239be6"
            intent.request_id = "0ebbf0f147719304bccce28a8e43078c"
            intent.received_steady, intent.steady_deadline = r.now, 10.05 + 0.25
            intent.manual_linear, intent.manual_linear_limit, intent.degraded_linear_limit = (
                1,
                0.05,
                0.05,
            )
            intent.vx = 0.05
            assert r.core.command(intent)
            for now in (10.05, 10.1, intent.steady_deadline):
                r.step(now)
                packets.append(peer.recv(64))
                emissions.append(r.core.node.emitted[-1][1:4])
                rays.append(asdict(scan_observation(r.core.node._scans[-1], "base_link")))
                if enabled:
                    traced_rows.extend(
                        json.loads(line) for line in os.read(reader, 65536).splitlines()
                    )
            intent.received_steady, intent.steady_deadline = r.now, 15.0
            intent.sequence += 1
            assert r.core.command(intent)
            r.step(10.35)
            packets.append(peer.recv(64))
            emissions.append(r.core.node.emitted[-1][1:4])
            if enabled:
                traced_rows.extend(json.loads(line) for line in os.read(reader, 65536).splitlines())
            r.step(11.350001, new_scan=False)
            packets.append(peer.recv(64))
            emissions.append(r.core.node.emitted[-1][1:4])
            if enabled:
                traced_rows.extend(json.loads(line) for line in os.read(reader, 65536).splitlines())
            assert r.core.node.emitted[-1][1] == Velocity(0, 0)
            assert r.core.node.emitted[-1][2] == "scan_temporarily_unavailable"
            assert r.core.target.source == "manual"
            results.append((packets, emissions, rays))
        finally:
            output.finish_trace("test_end")
            parent.close()
            peer.close()
            os.close(reader)
            os.close(writer)
            r.close()
    assert results[0] == results[1]
    assert (gc.isenabled(), gc.get_threshold()) == gc_settings
    ticks = [row for row in traced_rows if row["event"] == "core_tick"]
    assert len(ticks) == 5
    assert all(row["lost"] == 0 for row in traced_rows)
    assert all(
        len((json.dumps(row, separators=(",", ":"), allow_nan=False) + "\n").encode()) <= 4096
        for row in traced_rows
    )
    sends = [row for row in traced_rows if row["event"] == "transport_send"]
    moving = [row for row in sends if row["vx"] != 0]
    assert moving
    for submitted in moving:
        tick = next(row for row in ticks if row["transport_sequence"] == submitted["sequence"])
        assert tick["computed_ns"] == submitted["computed_ns"]
        assert tick["decision"]["clearance"]["rear"] == "Inf"
        assert float(tick["decision"]["clearance"]["rear"]) == math.inf
    first = ticks[0]
    assert first["argmin"] == ["budget", "target", "startup"]
    assert first["deadline"] == 10.05 + 0.25 and first["bounds"]["startup"] is None
    assert first["selected"]["capture"]["time"]["sample_steady_ns"] == 10_050_000_000
    assert first["selected"]["capture"]["raw_gid"] == (
        "010fb8ff7d38fc2201000000000013030000000000000000"
    )
    assert first["computed_ns"] == 10_050_000_000 and first["transport_sequence"] == 1
    stages = [row for row in traced_rows if row["event"] == "core_stages"]
    assert len(stages) == 5 and stages[0]["computed_ns"] == first["computed_ns"]
    assert any(mark[0] == "clearance_begin" for mark in stages[0]["marks"])
    assert ticks[2]["decision"]["target_expired"] and ticks[2]["submitted"] == [0, 0]
    assert first["last_installed_command"]["sequence"] == 0
    assert first["remaining_ms_at_send"] == first["remaining_ms_at_report"]
    assert ticks[2]["decision"]["zero_stage"] == "target_zero"
    assert ticks[-1]["decision"]["clearance"] is None
    assert ticks[-1]["decision"]["zero_stage"] == "clearance"
    assert ticks[-1]["selected"]["age"]["band"] == "invalid"
