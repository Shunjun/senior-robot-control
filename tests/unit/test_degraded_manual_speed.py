"""Synthetic target/constraint tests; no ROS, physical plant or vehicle acceptance."""

import pytest
from senior_motion_control.manual_target import ManualLinearTarget
from senior_motion_estimation.source_modes import SourceMode
from test_motion_core_handoff import core  # noqa: F401

from vehicle_core.values import Velocity


@pytest.mark.parametrize(
    "field,value",
    [
        ("fraction", 1.1),
        ("fraction", float("nan")),
        ("full_stick_mps", -1),
        ("degraded_mps", float("inf")),
    ],
)
def test_invalid_manual_metadata_rejected(field, value):
    values = dict(fraction=0.5, full_stick_mps=1.0, degraded_mps=0.3)
    values[field] = value
    with pytest.raises(ValueError):
        ManualLinearTarget(**values)


def test_percentage_mapping_mode_switches_keep_controller_acceleration_bounded():
    from senior_motion_control.controller import MotionController
    from test_motion_controller import config, context_for, engine, feedback

    metadata = ManualLinearTarget(0.5, 0.8, 0.2)
    controller = MotionController(engine(), config(), Velocity(0.4, 0), Velocity(0, 0))
    body = 0.4
    commands = []
    for i in range(250):
        now = i * 0.02
        degraded = i < 140
        cap = 0.2 if degraded else 0.8
        target = Velocity(metadata.velocity(degraded=degraded, capability_mps=cap), 0)
        ctx = context_for(
            controller,
            now,
            body,
            budget=target.vx,
            mode=SourceMode.WHEEL_IMU_LIMITED if degraded else SourceMode.LASER_REFERENCE,
            cap=cap,
        )
        decision = controller.tick(
            target,
            ctx,
            engine().refine(ctx, now),
            None if degraded else feedback(now, body, i),
            feedback(now, body, i, "wheel"),
            now,
        )
        if i > 0:
            assert decision.command is not None, decision.reason
        if decision.command is not None:
            commands.append(decision.command.vx)
            body += (decision.command.vx - body) * 0.2
    assert commands[130] < 0.12 and commands[-1] > 0.35
    changes = [abs(b - a) for a, b in zip(commands, commands[1:], strict=False)]
    assert max(changes) <= 0.5 * 0.02 + 1e-9


@pytest.mark.parametrize(
    "field,value",
    [
        ("manual_linear", 2),
        ("manual_linear_limit", -1),
        ("degraded_linear_limit", float("nan")),
        ("vx", 0.1),
    ],
)
def test_output_owner_rejects_inconsistent_or_invalid_manual_wire_metadata(core, field, value):  # noqa: F811
    from test_motion_core_handoff import barrier

    message = barrier("intent")
    setattr(message, field, value)
    assert not core.command(message)
    assert core.target.source == "idle"
