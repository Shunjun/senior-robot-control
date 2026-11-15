"""Synthetic target/constraint tests; no ROS, physical plant or vehicle acceptance."""

from dataclasses import replace

import pytest
from test_directional_constraints import scene
from test_motion_core_handoff import core  # noqa: F401
from test_unified_motion_runtime import point

from senior_control.agent.manual_control import ManualIngress
from senior_control.models import Pose2D, Velocity
from senior_control.motion.contracts import ManualIntent, MotionTarget
from senior_control.motion.estimator import MotionObservation, Origin
from senior_control.motion.manual_target import ManualLinearTarget
from senior_control.motion.observations import Source
from senior_control.motion.source_modes import SourceMode
from senior_control.settings.models import vehicle_settings_profile


def context(
    core,  # noqa: F811
    fraction,
    gear,
    reverse_limit=0.3,
    degraded_limit=0.3,
    low_limit=5 / 9,
    source="manual",
):
    settings = vehicle_settings_profile().driving_defaults.model_copy(
        update={
            "reverse_linear_mps": reverse_limit,
            "degraded_linear_mps": degraded_limit,
            "low_linear_mps": low_limit,
        }
    )
    ingress = ManualIngress("robot", "boot", ttl=0.3)
    ingress.bind(1, 1, "session", 10.0)
    receipt = ingress.acquire("acquire", 10.0)
    sample = ingress.accept(
        ManualIntent(receipt.authority_id, 1, 1, fraction, 0, gear),
        settings,
        10.0,
        10.3,
        10.0,
    )
    assert sample is not None
    metadata = ManualLinearTarget(fraction, sample.linear_limit_mps, sample.degraded_linear_mps)
    core.target = MotionTarget(
        sample.velocity, source, "authority:1", 10.3, metadata if source == "manual" else None
    )
    # Exercise the same single owner as production instead of passing a
    # fabricated feedback object with no canonical estimate behind it.
    core.sensors.ingest(point(Source.WHEEL, 10.0, 10.0, (0.0, 0.0)), 10, 10, 10)
    core.sensors.ingest(point(Source.IMU, 10.0, 10.0, (0.0,)), 10, 10, 10)
    ref, _, wheel = core.sensors.estimate(10, 10)
    return core._snapshot(
        ref, wheel, 10.0, replace(scene(10.0), frame_id="scan_reference"), Pose2D(0, 0, 0)
    )


@pytest.mark.parametrize("gear", ["low", "high"])
@pytest.mark.parametrize("fraction", [0.25, 0.5, 1, -0.25, -0.5, -1])
def test_all_stick_fractions_reach_constraint_candidates(core, gear, fraction):  # noqa: F811
    ctx = context(core, fraction, gear)
    assert ctx.target.velocity.vx == pytest.approx(fraction * 0.3)
    result = core.engine.refine(ctx, 10.0)
    assert result.permit is not None, result.brake
    assert ctx.target.velocity in result.permit.velocities
    assert all(abs(v.vx) <= 0.3 for v in result.permit.velocities)


@pytest.mark.parametrize("fraction", [0.25, 0.5, 1, -0.25, -0.5, -1])
@pytest.mark.parametrize(
    "low,reverse,degraded", [(0.1, 0.3, 0.3), (0.5, 0.08, 0.3), (0.5, 0.3, 0.12)]
)
def test_lower_gear_reverse_and_custom_preference_apply_to_whole_range(
    core,  # noqa: F811
    fraction,
    low,
    reverse,
    degraded,  # noqa: F811
):
    ctx = context(core, fraction, "low", reverse, degraded, low)
    full = min(low, degraded, reverse if fraction < 0 else low)
    assert ctx.target.velocity.vx == pytest.approx(fraction * full)


def test_normal_cap_still_bounds_manual_degraded_speed(core):  # noqa: F811
    core.profile = replace(core.profile, cap=0.1)
    ctx = context(core, 0.5, "high", degraded_limit=0.6)
    assert ctx.envelope.linear_cap_mps == 0.1
    assert ctx.target.velocity.vx == 0.05


@pytest.mark.parametrize("gear,limit", [("low", 5 / 9), ("high", 25 / 18)])
def test_normal_mode_preserves_gear_percentage(core, gear, limit):  # noqa: F811
    ctx = context(core, 0.5, gear, degraded_limit=0.1)
    core.sensors.ingest_observation(
        MotionObservation(
            "laser",
            (0.0, None, 0.0),
            10.0,
            (Origin("laser", "boot", 1, 10.0, 10.0, 10.0),),
            independent=(True, True, True),
        ),
        10.0,
        10.0,
    )
    ref, body, wheel = core.sensors.estimate(10.0, 10.0)
    assert ref["mode"] == SourceMode.FUSED_REFERENCE.value
    actual = core._snapshot(ref, wheel, 10.0, ctx.perception, ctx.pose)
    assert actual.target.velocity.vx == pytest.approx(0.5 * limit)
    assert actual.envelope.linear_cap_mps == core.profile.cap


def test_obstacle_still_denies_mapped_manual_target(core):  # noqa: F811
    ctx = context(core, 1, "high")
    ctx = replace(ctx, perception=replace(scene(10.0, hits=((0.4, 0),)), frame_id="scan_reference"))
    result = core.engine.refine(ctx, 10.0)
    assert result.permit is None and result.brake is not None
    assert result.brake.reason in {"stop_envelope_blocked", "no_clear_candidate"}


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
    from test_motion_controller import config, context_for, engine, feedback

    from senior_control.motion.controller import MotionController

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


@pytest.mark.parametrize("gear", ["low", "high"])
@pytest.mark.parametrize(
    "fraction,expected",
    [(0.25, 0.075), (0.5, 0.15), (1, 0.3), (-0.25, -0.075), (-0.5, -0.15), (-1, -0.3)],
)
def test_old_strategy_cap_cannot_override_saved_manual_default(core, gear, fraction, expected):  # noqa: F811
    core.profile = replace(core.profile, degraded_cap=0.15)
    ctx = context(core, fraction, gear)
    assert ctx.target.velocity.vx == pytest.approx(expected)
    assert ctx.envelope.linear_cap_mps == 0.3


@pytest.mark.parametrize("fraction,expected", [(0.25, 0.15), (0.5, 0.3), (1, 0.6)])
def test_custom_manual_limit_can_exceed_old_strategy(core, fraction, expected):  # noqa: F811
    core.profile = replace(core.profile, degraded_cap=0.15)
    ctx = context(core, fraction, "high", degraded_limit=0.6)
    assert ctx.target.velocity.vx == pytest.approx(expected)
    assert ctx.envelope.linear_cap_mps == 0.6


def test_navigation_retains_legacy_policy_outside_manual_scope(core):  # noqa: F811
    core.profile = replace(core.profile, degraded_cap=0.15)
    ctx = context(core, 1, "high", degraded_limit=0.6, source="navigation")
    assert ctx.envelope.linear_cap_mps == 0.15
    assert ctx.target.velocity.vx == pytest.approx(25 / 18)
