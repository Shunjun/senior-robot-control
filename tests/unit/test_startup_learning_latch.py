"""Learning keeps sampling; software motion authorization remains separate."""

from types import SimpleNamespace as N

import pytest
from control_process_test_support import publish_inputs
from test_motion_core_handoff import barrier, core  # noqa: F401
from test_startup_learning import observation
from test_startup_learning_feasibility import BOUNDARY
from test_startup_learning_review import captured, estimate

from senior_control.models import Velocity
from senior_control.motion.contracts import MotionEventKind as K
from senior_control.motion.contracts import MotionMode
from senior_control.motion.startup_learning import StartupLearning
from senior_control.motion.state_machine import RecoveryObservation
from senior_control.safety.self_learning import SelfReturnModel


@pytest.mark.parametrize("event", [K.SOFTWARE_ESTOP, K.EXECUTION_FAULT])
def test_latched_unmoved_learning_survives_wait_and_static_sampling(core, tmp_path, event):  # noqa: F811
    core.startup = StartupLearning(tmp_path / "claim")
    core.self_returns = SelfReturnModel(BOUNDARY, error_m=0.005)
    core._event(event, 10.0)
    for i in range(1, 16):
        t = 10 + i * 0.1
        core.startup.advance(estimate(t, 0), 0.0, now=t, stationary=True)
        core.startup.frame(core.self_returns, captured(observation(0.15), t))
        core._startup_target(t)
    assert core.startup.active and core.startup.provisional
    assert len(core.self_returns.frames) >= 5
    assert core.target.velocity == Velocity(0, 0)
    assert core.machine.state.mode in {MotionMode.ESTOP, MotionMode.EXECUTION_FAULT}
    assert core.startup.started is None
    # Waiting longer than 30s is not an attempted drive or a failed learning.
    core.startup.advance(estimate(60, 0), 0.0, now=60, stationary=True)
    assert core.startup.active


def test_explicit_reset_continues_only_unmoved_attempt(core, tmp_path):  # noqa: F811
    core.startup = StartupLearning(tmp_path / "claim")
    core.startup.state = "moving"
    core._event(K.SOFTWARE_ESTOP, 10.0)
    core._startup_target(10.0)
    core.machine.observe(RecoveryObservation("body", 1, 10.0, 11.0, True, True), 10.0)
    core.revoke("estop_reset", now=10.0)
    core._event(K.ESTOP_RESET, 10.0, revision=core.machine.state.estop_revision)
    core._startup_target(10.0)
    assert core.startup.active and core.target.velocity.vx > 0
    assert StartupLearning(tmp_path / "claim").state == "interrupted"


def test_lock_after_emitted_motion_never_replays_forward(core, tmp_path):  # noqa: F811
    core.startup = StartupLearning(tmp_path / "claim")
    core.startup.state = "moving"
    core.startup.motion_emitted(10.0)
    core.startup.motion_emitted(10.0)
    core.revoke("estop", now=10.0)
    assert core.startup.active and core.startup.velocity == 0
    core.machine.observe(RecoveryObservation("body", 1, 10.0, 11.0, True, True), 10.0)
    core._startup_target(10.0)
    assert core.target.velocity == Velocity(0, 0)
    assert not core._startup_permission_lost(N(permit=None, brake=None), 10.0)


@pytest.mark.parametrize("event", [K.SOFTWARE_ESTOP, K.EXECUTION_FAULT])
def test_actual_tick_samples_under_latch_and_never_drives(core, tmp_path, monkeypatch, event):  # noqa: F811
    import math

    from test_unified_motion_runtime import point

    from senior_control.config import DeviceProfile
    from senior_control.motion.estimator import MotionObservation, Origin
    from senior_control.motion.observations import Source

    core.startup = StartupLearning(tmp_path / "claim")
    core.node._profile = lambda: DeviceProfile(footprint=BOUNDARY)
    core.node._depth_policy = "laser_only"
    core.node._scan_topology_ready = lambda: True
    now = [10.0]
    monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: now[0])
    core._event(event, 10.0)
    revision = (
        core.machine.state.estop_revision
        if event is K.SOFTWARE_ESTOP
        else core.machine.state.fault_revision
    )
    for i in range(1, 31):
        t = now[0] = (10.0 if i < 16 else 50.0) + i * 0.1
        for source, values in ((Source.WHEEL, (0.0, 0.0)), (Source.IMU, (0.0,))):
            core.node._reception_test.sensors.ingest(point(source, t, t, values, i), t, t, t)
        core.node._reception_test.sensors.ingest_observation(
            MotionObservation(
                "laser",
                (0.0, 0.0, 0.0),
                t,
                (Origin("laser", "a", i, t, t, t),),
                independent=(True, True, True),
            ),
            t,
            t,
        )
        ranges = [10.0] * 720
        ranges[0] = 0.15
        core.node._scans = [
            N(
                stamp=t,
                received=t,
                received_ros=t,
                message=N(
                    source_epoch="scan",
                    frame_id="laser_link",
                    sequence=i,
                    ranges=ranges,
                    angle_min=-math.pi,
                    angle_increment=2 * math.pi / 719,
                    range_min=0.08,
                    range_max=20.0,
                    sensor_x=0.0,
                    sensor_y=0.0,
                    sensor_yaw=0.0,
                    transform_stamp=t,
                ),
            )
        ]
        publish_inputs(core)
        core.prefetch_scan()
        core.tick()
    assert core.startup.active and core.startup.provisional
    assert len(core.self_returns.frames) >= 5
    assert not core.startup.motion_started
    assert all(v == Velocity(0, 0) for v, _ in core.node.emitted)
    assert core.machine.state.mode is (
        MotionMode.ESTOP if event is K.SOFTWARE_ESTOP else MotionMode.EXECUTION_FAULT
    )
    assert revision == (
        core.machine.state.estop_revision
        if event is K.SOFTWARE_ESTOP
        else core.machine.state.fault_revision
    )


def test_withdrawn_move_validates_delayed_frames_without_restarting_budget(tmp_path):
    owner = StartupLearning(tmp_path / "claim")
    owner.state = "moving"
    owner.motion_emitted(10.0)
    owner.hold_motion()
    model = SelfReturnModel(BOUNDARY, error_m=0.005)
    for i in range(7):
        t = 50 + i * 0.1
        owner.advance(estimate(t, 0), 0.0, now=t, stationary=True)
        owner.frame(model, captured(observation(0.15), t - 0.05))
    assert owner.state == "ambiguous"
    assert owner.velocity == 0 and not model.learned
    assert owner.started == 10.0


def test_pending_motion_keeps_a_fresh_bounded_static_window(tmp_path):
    owner = StartupLearning(tmp_path / "claim")
    model = SelfReturnModel(BOUNDARY, error_m=0.005)
    for i in range(1, 101):
        t = i * 0.1
        owner.advance(estimate(t, 0), 0.0, now=t, stationary=True)
        owner.frame(model, captured(observation(0.15), t))
    assert owner.state == "moving" and not owner.motion_started
    assert len(model.frames) == 5
    assert model.frame_stamps[-1] == 10.0
    assert len(owner._frame_stamps) <= 5
