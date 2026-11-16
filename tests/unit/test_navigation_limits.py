"""Actual callback/arbitration/constraints/controller; synthetic scene and plant only."""

from dataclasses import replace

import pytest
from test_directional_constraints import engine, snapshot
from test_motion_controller import config, feedback, imu_feedback
from test_navigation_small_intent import rig  # noqa: F401

from senior_control.models import Velocity
from senior_control.motion.controller import MotionController


@pytest.mark.parametrize(
    "vx,wz",
    [
        (0.8, 0.0),
        (0.0, 1.8),
        (8.0, 4.0),
        (-8.0, -4.0),
        (0.0049, -0.0037),
        (1e6, 5e5),
    ],
)
def test_navigation_reaches_verified_nonzero_limited_control_output(rig, vx, wz):  # noqa: F811
    r = rig.r
    r.msg.vx, r.msg.wz = vx, wz
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert len(rig.final) == 1
    frame = rig.final[0]
    target = Velocity(frame.vx, frame.wz)
    assert target == Velocity(vx, wz)
    e = engine()
    controller = MotionController(e, config(), Velocity(0, 0), Velocity(0, 0))
    outputs = []
    for i in range(8):
        now = 10.03 + i * 0.025
        s = snapshot(now, target=target)
        s = replace(
            s,
            target=replace(s.target, source="navigation", deadline_steady=frame.steady_deadline),
            envelope=replace(s.envelope, linear_cap_mps=0.2, yaw_cap_rps=0.4),
        )
        permit = e.refine(s, now)
        assert permit.permit is not None
        fastest = permit.permit.velocities[0]
        assert abs(fastest.vx) <= 0.2 and abs(fastest.wz) <= 0.4
        if vx and wz:
            assert fastest.wz / fastest.vx == pytest.approx(wz / vx)
        if abs(vx) <= 0.2 and abs(wz) <= 0.4:
            assert fastest == target
        decision = controller.tick(
            target,
            s,
            permit,
            feedback(now, 0, i),
            feedback(now, 0, i, "wheel"),
            now,
            imu=imu_feedback(feedback(now, 0, i, "wheel")),
            still_wheel=feedback(now, 0, i, "wheel"),
        )
        if decision.command is not None:
            assert decision.verified
            assert abs(decision.command.vx) <= 0.2
            assert abs(decision.command.wz) <= 0.4
            assert decision.command.vx * vx >= 0 and decision.command.wz * wz >= 0
            outputs.append(decision.command)
    assert any(v != Velocity(0, 0) for v in outputs), outputs


@pytest.mark.parametrize(
    "attribute,value",
    [
        ("clock_domain", "foreign-boot"),
        ("run_id", "b" * 32),
        ("produced_steady_ns", 9_600_000_000),
        ("produced_steady_ns", 11_000_000_000),
        ("sequence", 0),
        ("vx", float("nan")),
        ("wz", float("inf")),
    ],
)
def test_invalid_navigation_never_produces_a_final_target(rig, attribute, value):  # noqa: F811
    r = rig.r
    setattr(r.msg, attribute, value)
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert not rig.final and r.node._navigation_source.sequence == 0


def test_writer_identity_replay_old_execution_and_stream_loss(rig):  # noqa: F811
    r = rig.r
    r.info["publisher_gid"] = b"foreign"
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert not rig.final
    r.info["publisher_gid"] = b"controller"
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert len(rig.final) == 1
    r.clock.mono += 0.2
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert len(rig.final) == 1  # replay cannot refresh deadline
    r.msg.sequence += 1
    r.msg.produced_steady_ns = round(r.clock.mono * 1e9)
    r.node._on_navigation_sample(r.topic.replace("a" * 32, "b" * 32), r.msg, r.info)
    assert len(rig.final) == 1
    r.clock.mono += 0.2
    r.node.publish_command()
    assert r.node._arbiter.final_command(r.clock.mono) is None
    assert rig.barriers


def test_humble_missing_sample_gid_keeps_bound_private_run_identity(rig):  # noqa: F811
    from types import SimpleNamespace

    r = rig.r
    r.info.pop("publisher_gid")
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert len(rig.final) == 1
    r.clock.mono += 0.2
    r.msg.sequence += 1
    r.msg.produced_steady_ns = round(r.clock.mono * 1e9)
    r.node.get_publishers_info_by_topic = lambda _: [
        SimpleNamespace(node_name="controller_server", endpoint_gid=b"new-incarnation")
    ]
    r.node._on_navigation_sample(r.topic, r.msg, r.info)
    assert len(rig.final) == 1
