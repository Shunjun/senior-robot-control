import math
from dataclasses import replace

import pytest
from test_motion_sources import config, sample
from test_motion_temporal import feed, owner, point, resolver

from senior_control.models import Pose2D
from senior_control.motion.fusion import FusionConfig, NumericalFusion, covariance_intersection
from senior_control.motion.observations import (
    Covariance,
    CovarianceKind,
    Frame,
    Heading,
    Source,
    SourceStatus,
)
from senior_control.motion.source_admission import AdmissionOwner
from senior_control.motion.source_modes import SourceMode


def test_ci_correlated_and_different_values_without_reciprocal_overflow():
    assert covariance_intersection(0.3, 0.01, 0.5, 0.04) == pytest.approx(
        (0.34, 0.016), abs=1e-12, rel=0
    )
    assert covariance_intersection(0.5, 0.04, 0.5, 0.04) == pytest.approx(
        (0.5, 0.04), abs=1e-12, rel=0
    )
    m, p = covariance_intersection(1.0, 1e-320, 1.0, 1e-320)
    assert math.isfinite(m) and 0 < p < math.inf
    with pytest.raises(ValueError):
        covariance_intersection(1.0, 0.0, 1.0, 0.1)


def fusion():
    return NumericalFusion(FusionConfig(("synthetic",), 1, 0.2, 0.4, 0.15))


def synthetic_window():
    o, r = owner(), resolver()
    covariance = Covariance(
        (0.04, 0.04), CovarianceKind.DIAGONAL_ONLY, "synthetic ENU velocity", None
    )
    events = [
        point(
            Source.IMU,
            i * 0.02,
            i + 1,
            (0,),
            covariance=Covariance((0.01,), CovarianceKind.DIAGONAL_ONLY, "synthetic gyro", None),
            model="synthetic",
        )
        for i in range(6)
    ]
    events += [
        replace(
            point(Source.GPS, i / 30, i + 1, (0.5, 0), covariance=covariance, model="synthetic"),
            frame=Frame.ENU,
            body_from_sensor=Pose2D(0, 0, 0),
        )
        for i in range(4)
    ]
    for p in sorted(events, key=lambda p: p.end_ros):
        h = Heading(0, 0, p.end_ros, p.received_steady, "h", "synthetic known yaw")
        assert o.ingest(
            p,
            now_ros=p.end_ros,
            now_steady=p.received_steady,
            heading=h if p.source is Source.GPS else None,
        ).accepted
    laser = replace(
        sample(seq=1, end=0.1, received=100.1, values=(0.3, 0, 0)),
        start_ros=0,
        model_id="synthetic",
    )
    assert o.ingest(laser, now_ros=0.1, now_steady=100.1).accepted
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    return o, r, s, r.resolve(0, 0.1, s)


def test_complete_asynchronous_chain_ci_and_no_repeated_tick_precision():
    _, r, s, w = synthetic_window()
    f = fusion()
    first = f.update(w, s, r)
    assert first.mode is SourceMode.FUSED_REFERENCE
    assert first.average.vx == pytest.approx(0.34, abs=1e-9, rel=0)
    assert first.components[0].variance == pytest.approx(0.016, abs=1e-9, rel=0)
    assert first.components[1].variance > 0.016  # gyro uncertainty also enters GPS lateral
    assert first.upper is None
    assert first.statistical_covariance[0] == pytest.approx(3 * 0.016, abs=1e-9, rel=0)
    assert first.current_reference.predicted_at_ros is None
    assert first.components[0].new_information
    for _ in range(5):
        repeated = f.update(w, s, r)
        assert repeated.components[0].variance == first.components[0].variance
        assert not any(c.new_information for c in repeated.components)
    # Re-numbering a query cannot invent independent observations.
    renamed = f.update(replace(w, start_ros=0.0001), s, r)
    assert not any(c.new_information for c in renamed.components)
    assert first.original_covariances


def test_missing_models_do_not_invent_variances_and_mask_retains_missing_lateral():
    o, r = owner(), resolver()
    feed(
        o,
        [point(Source.IMU, i * 0.02, i + 1, (0.1,)) for i in range(6)]
        + [point(Source.WHEEL, i / 30, i + 1, (0.3, 0.1)) for i in range(4)],
    )
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    result = fusion().update(r.resolve(0, 0.1, s), s, r)
    assert result.mode is SourceMode.WHEEL_IMU_LIMITED
    assert result.supported_components == (True, False, True)
    assert result.average is None and result.upper is None
    assert result.statistical_covariance is None
    assert result.components[0].value == pytest.approx(0.3)
    assert result.target_cap_mps == 0.2  # injected logic example, not a frozen cap


def test_untrusted_covariance_cannot_enter_ci_even_when_all_components_exist():
    _, r, s, w = synthetic_window()
    bad = replace(w, sources=tuple(replace(x, model_ids=("unverified",)) for x in w.sources))
    result = fusion().update(bad, s, r)
    assert not any(c.numerically_fused for c in result.components)
    assert all(c.variance is None for c in result.components)


def test_zero_statistical_variance_is_not_repaired_with_epsilon():
    _, r, s, w = synthetic_window()
    zero = replace(w, sources=tuple(replace(x, variances=(0.0, 0.0, 0.0)) for x in w.sources))
    result = fusion().update(zero, s, r)
    assert not any(c.numerically_fused for c in result.components)
    assert result.statistical_covariance is None


def test_conflict_reports_no_unfounded_gps_or_slip_attribution():
    _, r, s, w = synthetic_window()
    changed = replace(
        w,
        sources=tuple(
            replace(x, values=(2.0, 0.0, None)) if x.source is Source.GPS else x for x in w.sources
        ),
    )
    result = fusion().update(changed, s, r)
    assert "independent_sources_inconsistent_attribution_unknown" in result.conflicts
    assert result.components[0].value is None
    assert not result.components[0].numerically_fused
    assert result.components[1].numerically_fused
    assert result.components[2].value == pytest.approx(0)


@pytest.mark.parametrize("status", [SourceStatus.UNAVAILABLE, SourceStatus.DRIFT])
def test_explicit_gps_loss_or_drift_uses_same_owner_recovery(status):
    o = AdmissionOwner(config(), event_capacity=128)
    for t, seq in [(0.0, 1), (0.1, 2)]:
        p = replace(
            point(Source.GPS, t, seq, (0.3, 0)), frame=Frame.ENU, body_from_sensor=Pose2D(0, 0, 0)
        )
        h = Heading(0, 0, t, 100 + t, "h", "synthetic")
        assert o.ingest(p, now_ros=t, now_steady=100 + t, heading=h).accepted
    assert dict(o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1).sources)[Source.GPS].ready
    bad = replace(p, stamp_ros=0.12, received_steady=100.12, sequence=3, status=status)
    assert not o.ingest(bad, now_ros=0.12, now_steady=100.12).accepted
    for t, seq, ready in [(0.14, 4, False), (0.26, 5, True)]:
        good = replace(p, stamp_ros=t, received_steady=100 + t, sequence=seq)
        h = Heading(0, 0, t, 100 + t, "h", "synthetic")
        assert o.ingest(good, now_ros=t, now_steady=100 + t, heading=h).accepted
        for _ in range(3):
            assert (
                dict(o.snapshot(after_cursor=0, now_ros=t, now_steady=100 + t).sources)[
                    Source.GPS
                ].ready
                is ready
            )


def test_distinct_query_order_does_not_reconsume_an_older_measurement():
    _, r, s, w = synthetic_window()
    # Per-source watermarks survive arbitrary query order without caching every key.
    f = fusion()
    first = f.update(w, s, r)
    assert first.components[0].new_information
    newer = replace(
        w,
        sources=tuple(
            replace(x, original_keys=tuple((a, b, c + 100) for a, b, c in x.original_keys))
            for x in w.sources
        ),
    )
    assert f.update(newer, s, r).components[0].new_information
    second = f.update(w, s, r)
    assert not any(c.new_information for c in second.components)


def test_lateral_conflict_rejects_only_lateral_pair_and_keeps_longitudinal_ci():
    _, r, s, w = synthetic_window()
    changed = replace(
        w,
        sources=tuple(
            replace(x, values=(0.5, 5.0, None)) if x.source is Source.GPS else x for x in w.sources
        ),
    )
    result = fusion().update(changed, s, r)
    assert result.conflicts
    assert result.components[1].value is None
    assert result.components[0].value == pytest.approx(0.34, abs=1e-9, rel=0)
    assert result.components[0].numerically_fused
    assert result.components[2].value == pytest.approx(0)
    assert result.average is None
    assert result.mode is SourceMode.UNAVAILABLE  # no wheel evidence in this fixture


def test_yaw_conflict_rejects_gyro_dependents_but_keeps_independent_rate_xy():
    _, r, s, w = synthetic_window()
    changed = replace(
        w,
        sources=tuple(
            replace(x, values=(0.3, 0, 1.0)) if x.source is Source.LASER else x for x in w.sources
        ),
    )
    f = fusion()
    result = f.update(changed, s, r)
    assert result.components[2].value is None
    assert result.components[0].value == pytest.approx(0.3)  # GPS x needs disputed gyro
    assert result.components[1].value == pytest.approx(0)
    assert not result.components[0].numerically_fused
    assert result.conflicts
    assert f.update(w, s, r).components[0].numerically_fused  # no permanent conflict latch


@pytest.mark.parametrize(
    "axis,above", [(0, False), (0, True), (1, False), (1, True), (2, False), (2, True)]
)
def test_component_conflict_threshold_is_inclusive_with_normal_boundary(axis, above):
    _, r, s, w = synthetic_window()
    threshold = 0.15 if axis == 2 else 0.4
    distance = math.nextafter(threshold, math.inf) if above else threshold
    # Use a zero first operand so the exact representable threshold is tested.
    sources = []
    for x in w.sources:
        values = list(x.values)
        if x.source is Source.LASER:
            values[axis] = 0.0
        if x.source is (Source.IMU if axis == 2 else Source.GPS):
            values[axis] = distance
        sources.append(replace(x, values=tuple(values)))
    result = fusion().update(replace(w, sources=tuple(sources)), s, r)
    assert bool(result.conflicts) is above
    assert (result.components[axis].value is None) is above


def wheel_window():
    o, r, s, _ = synthetic_window()
    for i in range(4):
        assert o.ingest(
            point(Source.WHEEL, i / 30, i + 1, (0.3, 0)), now_ros=0.1, now_steady=100.1
        ).accepted
    s = o.snapshot(after_cursor=r.cursor, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    return r, s, r.resolve(0, 0.1, s)


def test_disputed_gyro_cannot_return_through_wheel_prediction_fallback():
    r, s, w = wheel_window()
    changed = replace(
        w,
        sources=tuple(
            replace(x, values=(0.3, 0, 1.0))
            if x.source is Source.LASER
            else replace(x, values=(2.0, 0, None))
            if x.source is Source.GPS
            else x
            for x in w.sources
        ),
    )
    result = fusion().update(changed, s, r)
    assert result.components[0].value is None
    assert result.components[1].value == pytest.approx(0)
    assert result.components[2].value is None
    assert ("wheel.vx", "conflicting_component") in result.failures


def test_wheel_fallback_after_x_conflict_does_not_regain_normal_cap_from_healthy_y():
    r, s, w = wheel_window()
    changed = replace(
        w,
        sources=tuple(
            replace(x, values=(2.0, 0, None)) if x.source is Source.GPS else x for x in w.sources
        ),
    )
    result = fusion().update(changed, s, r)
    assert result.components[0].reason == "wheel_kinematic_only"
    assert result.components[1].numerically_fused
    assert result.mode is SourceMode.WHEEL_IMU_LIMITED
    assert result.target_cap_mps == 0.2  # injected example, not frozen


def test_delta_log_translation_does_not_claim_independence_from_disputed_laser_yaw():
    from senior_control.motion.kinematics import exponential
    from senior_control.motion.sample_types import DeltaObservation

    _, _, original, _ = synthetic_window()
    o, r = owner(), resolver()
    for event in original.events:
        if event.sample is not None and event.source is not Source.LASER:
            p = event.sample
            assert o.ingest(
                p.original,
                now_ros=p.original.end_ros,
                now_steady=p.original.received_steady,
                heading=p.heading_context,
            ).accepted
    delta = DeltaObservation(
        Source.LASER,
        Frame.BODY,
        0,
        0.1,
        100.1,
        "delta",
        1,
        1,
        SourceStatus.VALID,
        exponential(0.3, 0, 1, 0.1),
        None,
        None,
        Pose2D(0, 0, 0),
    )
    assert o.ingest(delta, now_ros=0.1, now_steady=100.1).accepted
    s = o.snapshot(after_cursor=0, now_ros=0.1, now_steady=100.1)
    r.consume(s)
    result = fusion().update(r.resolve(0, 0.1, s), s, r)
    # Delta->twist x/y use its yaw; GPS x/y use gyro. Both yaw hypotheses disputed.
    assert result.supported_components == (False, False, False)
    assert ("laser.vx", "conflicting_component") in result.failures


def test_cold_snapshot_does_not_let_gps_borrow_recovering_gyro():
    _, r, s, _ = synthetic_window()
    cold = replace(
        s,
        sources=tuple(
            (source, replace(view, ready=False) if source is Source.IMU else view)
            for source, view in s.sources
        ),
    )
    r.consume(cold)
    w = r.resolve(0, 0.1, cold)
    assert w.by_source(Source.IMU).reason == "recovering"
    assert w.by_source(Source.GPS).reason == "dependency_imu_recovering"
    assert not w.by_source(Source.GPS).original_keys
    assert not any(c.numerically_fused for c in fusion().update(w, cold, r).components)


def test_ready_snapshot_new_imu_identity_cannot_consume_old_cached_gyro():
    _, r, s, _ = synthetic_window()
    views = []
    for src, view in s.sources:
        if src is Source.IMU:
            view = replace(
                view,
                observation=replace(
                    view.observation, original=replace(view.observation.original, epoch="new")
                ),
            )
        views.append((src, view))
    changed = replace(s, sources=tuple(views))
    # New identity is ready, but its accepted events have not reached this resolver.
    result = r.resolve(0, 0.1, changed).by_source(Source.GPS)
    assert result.reason == "dependency_imu_history_missing"
    assert not result.original_keys
