"""Synthetic plant/scene only: no response or physical cap is frozen."""

from dataclasses import replace

import pytest
from shapely.geometry import GeometryCollection, LineString, MultiPoint, Point, box
from shapely.ops import unary_union

from senior_control.models import BodyTwist, Pose2D, Velocity
from senior_control.motion.constraints import (
    ConstraintConfig,
    ConstraintEngine,
    ConstraintSnapshot,
    Footprint,
    FreePatch,
    NoReturnPolicy,
    PatchKind,
    RiskEnvelope,
    prepare_perception,
    stop_axis,
)
from senior_control.motion.contracts import BodyMotion, BrakeKind, MotionTarget, ResponseBounds
from senior_control.motion.source_modes import SourceMode


def scene(now=0.0, hits=(), rear=True, epoch="scan", revision=1, no_return=False, error_m=0.02):
    patch = FreePatch(
        ((-20 if rear else 0, -20), (20, -20), (20, 20), (-20 if rear else 0, 20)),
        PatchKind.NO_RETURN if no_return else PatchKind.OBSERVED_FREE,
    )
    return prepare_perception(
        (patch,),
        hits,
        error_m=error_m,
        stamp_steady=now,
        deadline_steady=now + 2,
        source_epoch=epoch,
        frame_id="local",
        revision=revision,
        forecast_horizon_s=10,
        no_return_policy=NoReturnPolicy.UNKNOWN,
        basis="synthetic scene",
        max_patches=8,
        max_vertices=64,
        max_hits=64,
    )


def snapshot(
    now=0.0,
    target=None,
    current=0.0,
    hits=(),
    rear=True,
    mode=SourceMode.LASER_REFERENCE,
):
    target = target if target is not None else Velocity(0.6, 0)
    motion = BodyMotion(
        BodyTwist(current, 0, 0), None, now - 0.1, now, now, now + 2, "motion", True, "synthetic"
    )
    response = ResponseBounds(0.4, 0.5, 2.0, 0.5, 0.6, 2.0, 0.1, 20.0, "synthetic-response")
    risk = RiskEnvelope(
        "synthetic-floor",
        "independent synthetic exact plant; not wheel truth",
        "motion",
        (SourceMode.LASER_REFERENCE, SourceMode.WHEEL_IMU_LIMITED),
        "synthetic-response",
        20.0,
        10.0,
        (current, current),
        (0.0, 0.0),
        Velocity(0, 0),
        0.0,
        0.0,
        0.0,
        1.0,
        1.0,
        True,
    )
    return ConstraintSnapshot(
        MotionTarget(target, "manual", "authority", now + 3),
        motion,
        mode,
        response,
        Footprint(0.6, 0.4),
        scene(now, hits, rear),
        Pose2D(0, 0, 0),
        "local",
        "synthetic-floor",
        "config-1",
        risk,
    )


def engine():
    return ConstraintEngine(ConstraintConfig((1.0, 0.5, 0.25, 0.125), 1.0, 0.05, 128, 0.002))


def speed(result):
    return max((abs(v.vx) for v in result.permit.velocities), default=0) if result.permit else 0


def test_distance_reduces_permitted_speed_monotonically():
    values = [speed(engine().refine(snapshot(hits=((d, 0),)), 0)) for d in (2.0, 1.0, 0.5)]
    assert values[0] > values[-1]
    assert values == sorted(values, reverse=True)


def test_front_blocked_reverse_requires_rear_observed():
    e = engine()
    reverse = snapshot(target=Velocity(-0.3, 0), hits=((0.45, 0),))
    assert speed(e.refine(reverse, 0)) > 0
    unknown = replace(reverse, perception=scene(hits=((0.45, 0),), rear=False))
    assert e.refine(unknown, 0).permit is None
    assert (
        e.refine(snapshot(rear=False), 0).permit is not None
    )  # rear unknown does not stop forward


def test_forward_inertia_and_zero_goal_request_brake_not_safe_zero():
    e = engine()
    for target in (Velocity(-0.3, 0), Velocity(0, 0)):
        result = e.refine(snapshot(target=target, current=0.4), 0)
        assert result.permit is None
        assert result.brake.kind is BrakeKind.CONTROLLED
        assert result.brake.stop_verified
    unsafe = e.refine(snapshot(target=Velocity(0, 0), current=0.4, hits=((0.4, 0),)), 0)
    assert unsafe.brake.kind is BrakeKind.EMERGENCY
    assert not unsafe.brake.stop_verified


def test_partial_wheel_reference_needs_independent_scoped_envelope():
    s = snapshot(mode=SourceMode.WHEEL_IMU_LIMITED)
    s = replace(s, motion=replace(s.motion, average=None))
    assert engine().refine(s, 0).permit is not None
    missing = engine().refine(replace(s, envelope=None), 0)
    assert missing.permit is None and missing.brake.reason == "missing_risk_envelope"
    assert not missing.brake.stop_verified


def test_unknown_no_return_never_becomes_free_and_deadlines_are_raw():
    s = snapshot()
    assert engine().refine(replace(s, perception=scene(no_return=True)), 0).permit is None
    expired = engine().refine(s, 2.01)
    assert expired.permit is None and not expired.brake.stop_verified


def test_rotation_and_compound_outer_corner_are_checked():
    e = engine()
    for target in (Velocity(0, 0.7), Velocity(0.3, 0.5)):
        s = snapshot(target=target)
        assert e.refine(s, 0).permit is not None
        # Outside static padded side, within rotating rear-corner sweep.
        blocked = replace(s, perception=scene(hits=((-0.15, -0.32),)))
        result = e.refine(blocked, 0)
        assert result.permit is None or target not in result.permit.velocities


def test_jerk_stop_is_longer_than_instant_deceleration_and_has_zero_end_acceleration():
    bound = stop_axis(0.5, 0.0, 0.5, 2.0, 0.1)
    assert bound.distance > 0.5 * 0.1 + 0.5**2 / (2 * 0.5)
    assert bound.phases[-1].end_velocity == pytest.approx(0, abs=1e-12, rel=0)
    assert bound.phases[-1].end_acceleration == pytest.approx(0, abs=1e-12, rel=0)


def test_residual_acceleration_at_zero_speed_and_nonpositive_intervals_are_not_lost():
    s = snapshot(target=Velocity(0, 0))
    residual = replace(
        s,
        envelope=replace(s.envelope, reaction_accel_upper=Velocity(0.4, 0.5)),
        perception=scene(hits=((-0.34, 0),)),
    )
    assert engine().refine(residual, 0).brake.kind is BrakeKind.EMERGENCY
    negative = replace(
        s,
        envelope=replace(s.envelope, linear_interval=(-0.4, 0.0)),
        perception=scene(hits=((-0.4, 0),)),
    )
    assert engine().refine(negative, 0).brake.kind is BrakeKind.EMERGENCY


def test_model_and_response_must_cover_stop_horizon_not_just_current_time():
    s = snapshot(now=0.5)
    for field in ("model", "response"):
        short = (
            replace(s, envelope=replace(s.envelope, valid_until_steady=0.51))
            if field == "model"
            else replace(s, response=replace(s.response, valid_until_steady=0.51))
        )
        assert engine().refine(short, 0.5).permit is None


def test_fast_recheck_brakes_for_new_definite_reversal_and_accepts_moving_pose():
    e = engine()
    s = snapshot()
    result = e.refine(s, 0)
    moved = replace(snapshot(0.3, current=0.1), pose=Pose2D(0.02, 0, 0))
    assert e.tighten(result, moved, 0.3).permit is not None
    backwards = snapshot(0.3, current=-0.1)
    assert e.tighten(result, backwards, 0.3).permit is None


def test_yaw_uncertainty_rotates_translation_not_only_the_body_radius():
    e = engine()
    s = snapshot(hits=((0.9, 0.37),))
    uncertain = replace(s, envelope=replace(s.envelope, yaw_error_rad=0.2))
    result = e.refine(uncertain, 0)
    assert result.permit is None or s.target.velocity not in result.permit.velocities


def test_fast_reversal_cannot_hide_in_previously_mixed_velocity_domain():
    e = engine()
    s = snapshot()
    mixed = replace(s, envelope=replace(s.envelope, linear_interval=(-0.1, 0.1)))
    result = e.refine(mixed, 0)
    assert result.permit is not None
    latest = snapshot(0.2, current=-0.1)
    tightened = e.tighten(result, latest, 0.2)
    assert tightened.permit is None
    assert tightened.brake.reason == "reverse_requires_stop"


def test_lateral_slip_needs_its_own_supported_settling_horizon():
    s = snapshot(target=Velocity(0, 0))
    s = replace(s, envelope=replace(s.envelope, lateral_drift_mps=0.2))
    result = engine().refine(s, 0)
    assert not result.brake.stop_verified


def test_expired_target_still_checks_known_braking_trajectory():
    s = snapshot(current=0.4)
    s = replace(s, target=replace(s.target, deadline_steady=0.1))
    result = engine().refine(s, 0.2)
    assert result.brake.reason == "target_expired"
    assert result.brake.stop_verified


def test_geometry_and_mode_changes_are_rechecked_not_blindly_versioned():
    e = engine()
    s = snapshot()
    result = e.refine(s, 0)
    degraded = replace(
        s, mode=SourceMode.WHEEL_IMU_LIMITED, envelope=replace(s.envelope, linear_cap_mps=0.2)
    )
    low = e.tighten(result, degraded, 0.1)
    assert low.permit is not None and speed(low) <= 0.2
    larger = replace(s, footprint=Footprint(1.0, 1.0), perception=scene(hits=((0.8, 0.4),)))
    assert e.tighten(result, larger, 0.1).permit is None


def test_malformed_no_return_tags_cannot_turn_unknown_into_free():
    with pytest.raises(ValueError):
        prepare_perception(
            (FreePatch(((-2, -2), (2, -2), (2, 2), (-2, 2)), "no_return"),),
            (),
            error_m=0,
            stamp_steady=0,
            deadline_steady=1,
            source_epoch="s",
            frame_id="local",
            revision=0,
            forecast_horizon_s=5,
            no_return_policy=NoReturnPolicy.UNKNOWN,
            basis="synthetic malformed",
            max_patches=1,
            max_vertices=4,
            max_hits=1,
        )


def test_latest_shorter_deadline_and_old_forecast_never_gain_time():
    e = engine()
    s = snapshot()
    result = e.refine(s, 0)
    short = replace(s, target=replace(s.target, deadline_steady=0.4))
    assert e.tighten(result, short, 0.1).permit.deadline_steady == 0.4
    old = replace(
        snapshot(1.0), perception=replace(scene(0), deadline_steady=5, forecast_horizon_s=0.9)
    )
    assert e.refine(old, 1.0).permit is None


def test_ramp_output_requires_an_explicit_new_check_not_discrete_inheritance():
    e = engine()
    s = snapshot()
    result = e.refine(s, 0)
    proposed = Velocity(0.008, 0)
    assert not result.permit.allows(proposed, 0)
    assert e.validate_output(result, s, proposed, 0)
    assert not e.validate_output(result, s, Velocity(0.008, -0.05), 0)


def test_unrepresentable_stop_model_is_a_brake_request_not_a_crash():
    s = snapshot(current=0.4)
    s = replace(s, response=replace(s.response, decel=1e-300, jerk=1e-300))
    result = engine().refine(s, 0)
    assert result.permit is None and not result.brake.stop_verified


def test_lateral_settling_and_validated_no_return_are_explicit_model_support():
    s = snapshot(target=Velocity(0, 0))
    safe = replace(s, envelope=replace(s.envelope, lateral_drift_mps=0.02, lateral_stop_s=0.5))
    assert engine().refine(safe, 0).brake.stop_verified
    blocked = replace(safe, perception=scene(hits=((0, 0.23),)))
    assert engine().refine(blocked, 0).brake.kind is BrakeKind.EMERGENCY
    patch = FreePatch(((-20, -20), (20, -20), (20, 20), (-20, 20)), PatchKind.NO_RETURN)
    observed = prepare_perception(
        (patch,),
        (),
        error_m=0,
        stamp_steady=0,
        deadline_steady=2,
        source_epoch="scan",
        frame_id="local",
        revision=1,
        forecast_horizon_s=10,
        no_return_policy=NoReturnPolicy.VALIDATED_FREE_TO_RANGE,
        basis="synthetic sensor policy",
        max_patches=1,
        max_vertices=4,
        max_hits=1,
    )
    assert engine().refine(replace(snapshot(), perception=observed), 0).permit is not None


def test_geometry_inflation_is_once_and_stops_outside_sensor_error():
    s = snapshot()
    e = engine()
    result = e.refine(s, 0)
    assert max(x for x, y in s.footprint.vertices) == pytest.approx(0.315, abs=1e-12, rel=0)
    c = next(c for c in result.certificates if c.velocity == s.target.velocity)
    edge = c.sweep.bounds[2]
    # Hit just beyond nominal body reach is still unsafe inside sensor uncertainty.
    near = replace(s, perception=scene(hits=((edge + 0.01, 0),)))
    assert not e.validate_output(result, near, s.target.velocity, 0)
    far = replace(s, perception=scene(hits=((edge + 0.03, 0),)))
    assert e.validate_output(result, far, s.target.velocity, 0)


def test_pure_t3b_partial_motion_to_direction_constraint_example():
    from test_motion_temporal import point

    from senior_control.motion.observations import Source
    from tools.motion_fusion_replay import ReplayChain, replay_policy

    policy = replay_policy()
    policy = replace(
        policy, admission=replace(policy.admission, recovery_samples=1, recovery_span_s=0)
    )
    chain = ReplayChain(policy)
    events = [
        replace(point(Source.IMU, i / 50, i + 1, (0,)), received_steady=i / 50) for i in range(6)
    ]
    events += [
        replace(point(Source.WHEEL, i / 30, i + 1, (0.3, 0)), received_steady=i / 30)
        for i in range(4)
    ]
    for p in sorted(events, key=lambda p: p.end_ros):
        assert chain.ingest(p, p.end_ros, p.received_steady).accepted
    fused = chain.window(0, 0.1, 0.1, 0.1)
    assert fused.average is None and fused.upper is None
    epoch = "|".join(f"{h.source.value}:{h.epoch}" for h in fused.source_health)
    partial = BodyMotion(
        None,
        None,
        0,
        0.1,
        0.1,
        min(h.valid_until_steady for h in fused.source_health),
        epoch,
        True,
        "wheel_imu_limited",
    )
    s = snapshot(0.1, mode=fused.mode)
    # A separate synthetic plant knows actual speed .25, enclosed by [.2,.4];
    # these bounds are NOT obtained by adding an arbitrary error to wheel .3.
    risk = replace(s.envelope, motion_epoch=epoch, linear_interval=(0.2, 0.4))
    s = replace(s, motion=partial, envelope=risk)
    result = engine().refine(s, 0.1)
    assert result.permit is not None
    assert engine().validate_output(result, s, Velocity(0.008, 0), 0.1)


def test_initial_acceleration_release_overshoot_expands_command_envelope():
    e = engine()
    s = snapshot(target=Velocity(0.3, 0), current=0.3)
    initial = e.refine(s, 0)
    pushing = replace(s, envelope=replace(s.envelope, reaction_accel_upper=Velocity(0.4, 0)))
    expanded = e.refine(pushing, 0)
    a = next(c for c in initial.certificates if c.velocity == s.target.velocity)
    b = next(c for c in expanded.certificates if c.velocity == s.target.velocity)
    assert b.sweep.bounds[2] > a.sweep.bounds[2]
    assert e.tighten(initial, pushing, 0.1).certificates[0].linear_domain[1] > 0.3


def test_unknown_space_inside_padding_ring_is_not_assumed_physically_occupied():
    s = snapshot(target=Velocity(-0.01, 0))
    # Rear beyond the padded footprint is observed; the min-range blind strip
    # between physical body (-.3) and padding (-.315) is NOT observed.
    patch = FreePatch(((-20, -20), (-0.315, -20), (-0.315, 20), (-20, 20)), PatchKind.OBSERVED_FREE)
    perception = prepare_perception(
        (patch,),
        (),
        error_m=0,
        stamp_steady=0,
        deadline_steady=2,
        source_epoch="scan",
        frame_id="local",
        revision=1,
        forecast_horizon_s=10,
        no_return_policy=NoReturnPolicy.UNKNOWN,
        basis="synthetic blind strip",
        max_patches=1,
        max_vertices=4,
        max_hits=0,
    )
    assert engine().refine(replace(s, perception=perception), 0).permit is None


def test_stopping_integrals_match_closed_form_triangular_and_plateau_cases():
    triangular = stop_axis(0.2, 0, 1, 2, 0)
    duration = 2 * (0.2 / 2) ** 0.5
    assert triangular.duration == pytest.approx(duration, abs=1e-12, rel=0)
    assert triangular.distance == pytest.approx(0.2 * duration / 2, abs=1e-12, rel=0)
    plateau = stop_axis(0.5, 0, 0.5, 2, 0.1)
    assert plateau.duration == pytest.approx(1.35, abs=1e-12, rel=0)
    assert plateau.distance == pytest.approx(0.3625, abs=1e-12, rel=0)


@pytest.mark.parametrize(
    "hits",
    [
        (),
        ((0.0, 0.0),),
        ((0.0, 0.0), (0.0, 0.0)),  # duplicate
        ((0.0, 0.0), (0.25, 0.0)),  # overlapping error squares
        ((0.0, 0.0), (0.5, 0.0)),  # edge contact at error=.25
        ((0.0, 0.0), (0.5, 0.5)),  # corner contact at error=.25
        ((0.0, 0.0), (2.0, 2.0)),  # disconnected
    ],
)
@pytest.mark.parametrize("error_m", [0.0, 0.02, 0.25])
def test_occupied_intersection_preserves_every_hit_and_closed_error_boundary(hits, error_m):
    occupied = scene(hits=hits, error_m=error_m).occupied
    parts = (
        [box(x - error_m, y - error_m, x + error_m, y + error_m) for x, y in hits]
        if error_m
        else [Point(x, y) for x, y in hits]
    )
    probes = [
        GeometryCollection(),
        Point(10, 10),
        LineString([(-3, -3), (3, 3)]),
        box(-3, -3, 3, 3),
        box(0.3, -0.1, 0.4, 0.1),  # gap when squares/points are disconnected
    ]
    for x, y in hits:
        edge = x + error_m
        corner = y + error_m
        probes.extend(
            [
                Point(x, y),
                Point(edge, y),
                Point(edge, corner),
                Point(edge + 1e-9, corner + 1e-9),
                LineString([(edge, y - 1), (edge, y + 1)]),
                box(edge, corner, edge + 0.1, corner + 0.1),
            ]
        )
    # Independent finite-union membership oracle, including zero-area contacts.
    for probe in probes:
        assert probe.intersects(occupied) == any(probe.intersects(part) for part in parts)
    assert occupied.is_empty == (not hits)
    if not error_m and hits:
        assert isinstance(occupied, MultiPoint)


@pytest.mark.parametrize("target", [Velocity(0.6, 0), Velocity(-0.3, 0), Velocity(0.3, 0.5)])
@pytest.mark.parametrize("current", [0.0, 0.4])
@pytest.mark.parametrize(
    "hits",
    [(), ((0.0, 0.0),), ((0.5, 0.0), (0.52, 0.0)), ((2.0, 2.0), (2.04, 2.04))],
)
def test_occupied_representation_preserves_refine_latest_veto_and_output(target, current, hits):
    e = engine()
    s = snapshot(target=target, current=current, hits=hits)
    # The former normalized union is a comparison oracle, not required topology.
    normalized = replace(
        s, perception=replace(s.perception, occupied=unary_union(s.perception.occupied))
    )
    actual = e.refine(s, 0)
    expected = e.refine(normalized, 0)
    assert (actual.permit, actual.brake) == (expected.permit, expected.brake)
    prior = e.refine(snapshot(target=target, current=current), 0)
    tightened = e.tighten(prior, s, 0.1)
    normalized_tightened = e.tighten(prior, normalized, 0.1)
    assert (tightened.permit, tightened.brake) == (
        normalized_tightened.permit,
        normalized_tightened.brake,
    )
    assert e.validate_output(prior, s, target, 0.1) == e.validate_output(
        prior, normalized, target, 0.1
    )


@pytest.mark.parametrize(
    ("hit", "error_m"),
    [
        ((1e20, 1e20), 0.02),
        ((1e20, 0.0), 0.02),
        ((0.0, 0.0), 5e-324),  # area underflow
        ((0.0, 0.0), 1e154),  # area overflow with finite coordinates
        ((1e308, 0.0), 1e308),  # coordinate overflow
    ],
)
@pytest.mark.filterwarnings("ignore:overflow encountered:RuntimeWarning")
@pytest.mark.filterwarnings("ignore:invalid value encountered:RuntimeWarning")
def test_numerical_error_squares_preserve_existing_union_behavior(hit, error_m):
    # Finite inputs can still round a positive-error square to a point or line.
    # Normalization of duplicate degenerate polygons has legacy GEOS semantics.
    x, y = hit
    square = box(x - error_m, y - error_m, x + error_m, y + error_m)
    expected = unary_union([square, square])
    actual = scene(hits=(hit, hit), error_m=error_m).occupied
    assert actual.is_empty == expected.is_empty
    assert Point(x, y).intersects(actual) == Point(x, y).intersects(expected)


def test_full_scan_occupied_squares_preserve_exact_coordinates_and_evidence():
    """720-return ingress keeps every closed error square and its original age."""
    from math import cos, pi, sin

    hits = tuple(
        (0.15699 + 20 * cos(-pi + i * 2 * pi / 719), 0.00201 + 20 * sin(-pi + i * 2 * pi / 719))
        for i in range(720)
    )
    perception = prepare_perception(
        (),
        hits,
        error_m=0.02,
        stamp_steady=100.0,
        deadline_steady=100.8,
        source_epoch="original-scan",
        frame_id="scan_reference",
        revision=301,
        forecast_horizon_s=12.0,
        no_return_policy=NoReturnPolicy.UNKNOWN,
        basis="synthetic 720-return input",
        max_patches=2048,
        max_vertices=8192,
        max_hits=2048,
    )
    expected = GeometryCollection([box(x - 0.02, y - 0.02, x + 0.02, y + 0.02) for x, y in hits])
    assert perception.occupied.equals_exact(expected, 0)
    assert perception.free_space.is_empty
    assert (perception.stamp_steady, perception.deadline_steady) == (100.0, 100.8)
    assert (perception.source_epoch, perception.revision) == ("original-scan", 301)
