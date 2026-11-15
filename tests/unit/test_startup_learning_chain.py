import pytest
from test_motion_core_handoff import barrier, core  # noqa: F401

from senior_control.models import Velocity
from senior_control.motion.startup_learning import StartupLearning


def test_core_owns_startup_and_revoke_interrupts_without_rearming(core, tmp_path):  # noqa: F811
    core.startup = StartupLearning(tmp_path / "claim")
    core.startup.state = "moving"
    core._startup_target(10)
    assert core.target.source == "startup_learning"
    assert core.target.velocity == Velocity(0.05, 0)
    assert core.command(barrier("estop"))
    from senior_control.motion.contracts import MotionMode

    assert core.machine.state.mode is MotionMode.ESTOP
    assert core.startup.state == "interrupted"
    core._startup_target(10)
    assert core.target.velocity == Velocity(0, 0)


def test_initial_interior_stop_does_not_latch_execution_fault(core, tmp_path):  # noqa: F811
    from types import SimpleNamespace as N

    from senior_control.motion.contracts import MotionMode

    core.startup = StartupLearning(tmp_path / "claim")
    core._startup_target(10)
    assert core.target.velocity == Velocity(0, 0)
    result = core._decision_output(
        N(reason="stop_envelope_blocked", command=None, emergency=True), None, None, "epoch", 10
    )
    assert result == Velocity(0, 0)
    assert core.machine.state.mode != MotionMode.EXECUTION_FAULT


@pytest.mark.parametrize("interior_occlusion", [False, True, "front"])
def test_startup_flows_through_real_tick_and_safety_checks(
    core,  # noqa: F811
    tmp_path,
    monkeypatch,
    interior_occlusion,
):
    import math
    from types import SimpleNamespace as N

    from senior_robot.motion_observation_adapter import NativeObservations, strict_reference_json
    from test_motion_core_chain import InlineWorker, feed

    from senior_control.config import DeviceProfile
    from senior_control.motion.constraint_worker import ConstraintScheduler

    a = NativeObservations(core.profile.admission(), core.profile.temporal(), core.profile.fusion())
    core.scheduler = ConstraintScheduler(core.engine, InlineWorker(), worker_budget_s=0.25)
    core.startup = StartupLearning(tmp_path / "claim")
    node = core.node
    node._depth_policy = "laser_only"
    node._scan_topology_ready = lambda: True
    node._profile = lambda: DeviceProfile(
        footprint=[(-0.337, -0.276), (0.337, -0.276), (0.337, 0.276), (-0.337, 0.276)]
    )
    node.get_publishers_info_by_topic = lambda _: [
        N(node_name="motion_fusion", node_namespace="/", endpoint_gid=b"fusion")
    ]
    now = [10.0]
    monkeypatch.setattr("senior_robot.motion_core_output.time.monotonic", lambda: now[0])
    for i in range(45):
        t = now[0] = 10 + i * 0.025
        feed(a, t)
        core._on_reference(
            N(data=strict_reference_json(a.snapshot(t, t))), {"publisher_gid": b"fusion"}
        )
        ranges = [10.0] * 720
        if interior_occlusion is True:
            ranges[0] = 0.15  # occlusion behind rear equipment remains unknown
        if interior_occlusion == "front":
            ranges[359] = 0.4
        node._scans = [
            N(
                received=t,
                received_ros=t,
                stamp=t,
                message=N(
                    source_epoch="scan",
                    frame_id="laser_link",
                    sequence=i + 1,
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
        core.tick()
    assert core.startup.state == "moving", core.startup.summary()
    if interior_occlusion:
        assert not any(v.vx > 0 for v, _ in node.emitted)
        if interior_occlusion is True:
            assert "sweep_unknown" in node.emitted[-1][1]
    else:
        assert any(v.vx > 0 for v, _ in node.emitted), (core.startup.summary(), node.emitted[-3:])
    assert core.self_returns.learned == ()  # prior has never become learned at rest
    core.revoke("estop", now=now[0])
    assert not core.startup.provisional
    assert core.self_returns.learned == ()


def test_permission_loss_after_first_output_interrupts_and_clears_prior(core, tmp_path):  # noqa: F811
    from types import SimpleNamespace as N

    core.startup = StartupLearning(tmp_path / "claim")
    core.startup.state = "moving"
    core._learning_motion_emitted = True
    assert core._startup_permission_lost(N(permit=None))
    assert core.startup.state == "interrupted"
    assert not core.startup.provisional


def test_component_reconstruction_cannot_repeat_program_start(core, tmp_path, monkeypatch):  # noqa: F811
    from senior_robot.motion_core_output import MotionCoreOutput

    claim = tmp_path / "program-run"
    assert StartupLearning(claim).active
    monkeypatch.setenv("SENIOR_STARTUP_LEARNING_CLAIM", str(claim))
    rebuilt = MotionCoreOutput(core.node, core.profile, ingress_factory=None)
    try:
        assert rebuilt.startup.state == "interrupted"
        rebuilt._startup_target(10)
        assert rebuilt.target.velocity == Velocity(0, 0)
    finally:
        rebuilt.close()
