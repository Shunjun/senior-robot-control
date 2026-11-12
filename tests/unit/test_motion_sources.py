"""Synthetic policy values verify pure logic only; no physical bound is frozen."""

import math
from dataclasses import replace

import pytest

from senior_control.motion.observations import (
    AdmissionConfig,
    Covariance,
    CovarianceKind,
    Frame,
    Heading,
    Observation,
    Source,
    SourceRule,
    SourceStatus,
)
from senior_control.motion.source_admission import AdmissionOwner
from senior_control.motion.source_modes import SourceMode


def config():
    return AdmissionConfig(
        rules=tuple(SourceRule(source, 0.5, 0.3, 0.1, 2.0) for source in Source),
        recovery_samples=2,
        recovery_span_s=0.1,
        normal_cap_mps=1.0,
        degraded_cap_mps=0.2,
        heading_max_age_s=0.2,
        alignment_max_skew_s=0.1,
        heading_max_variance_rad2=0.05,
        clock_pause_s=0.3,
        epoch_history_limit=8,
    )


def sample(source=Source.LASER, seq=1, end=10.0, received=20.0, epoch="a", **changes):
    values = {
        Source.LASER: (0.3, 0.01, 0.1),
        Source.WHEEL: (0.31, 0.0, 0.12),
        Source.GPS: (0.0, 0.3),
        Source.IMU: (0.1,),
    }[source]
    observation = Observation(
        source,
        Frame.ENU if source is Source.GPS else Frame.BODY,
        end - 0.1,
        end,
        received,
        epoch,
        seq,
        0.9,
        SourceStatus.VALID,
        values,
        Covariance((0.01,) * len(values), CovarianceKind.DIAGONAL_ONLY, "synthetic variance", None),
    )
    return replace(observation, **changes)


def ingest(owner, observation, heading=None):
    return owner.ingest(
        observation,
        now_ros=observation.end_ros,
        now_steady=observation.received_steady,
        heading=heading,
    )


def test_recovery_counts_only_distinct_samples_and_has_no_invented_upper():
    owner = AdmissionOwner(config())
    first = sample()
    assert ingest(owner, first).accepted
    for _ in range(10):
        assert owner.assess(now_ros=10.0, now_steady=20.0).mode is SourceMode.RECOVERING
    assert not ingest(owner, first).accepted
    assert ingest(owner, sample(seq=2, end=10.1, received=20.1)).accepted
    result = owner.assess(now_ros=10.1, now_steady=20.1)
    assert result.mode is SourceMode.LASER_REFERENCE
    assert result.motion.upper is None
    assert not result.uncertainty_supported
    assert result.target_linear_cap_mps == 1.0


def test_weak_laser_and_missing_gps_allow_imu_wheel_limited_reference():
    owner = AdmissionOwner(replace(config(), recovery_samples=1, recovery_span_s=0))
    assert ingest(owner, sample(Source.WHEEL)).accepted
    assert ingest(owner, sample(Source.IMU)).accepted
    assert not ingest(owner, sample(status=SourceStatus.UNAVAILABLE)).accepted
    result = owner.assess(now_ros=10.0, now_steady=20.0)
    assert result.mode is SourceMode.WHEEL_IMU_LIMITED
    assert result.motion.average.vx == 0.31
    assert result.motion.average.wz == 0.1
    assert result.target_linear_cap_mps == 0.2
    assert result.motion.upper is None
    assert {item.original.source for item in result.contributors} == {Source.WHEEL, Source.IMU}
    assert not hasattr(result, "permit")  # Quality is not a T4 motion permission.


def test_gps_requires_known_fresh_heading_and_does_not_assume_covariance_independence():
    owner = AdmissionOwner(config())
    assert not ingest(owner, sample(Source.GPS)).accepted
    heading = Heading(math.pi / 2, 0.02, 10.0, 20.0, "heading-a", "synthetic heading")
    assert ingest(owner, sample(Source.GPS, seq=2), heading).accepted
    assert ingest(owner, sample(Source.IMU)).accepted
    assert ingest(
        owner,
        sample(Source.GPS, seq=3, end=10.1, received=20.1),
        replace(heading, stamp_ros=10.1, received_steady=20.1),
    ).accepted
    assert ingest(owner, sample(Source.IMU, seq=2, end=10.1, received=20.1)).accepted
    result = owner.assess(now_ros=10.1, now_steady=20.1)
    assert result.mode is SourceMode.GPS_IMU_REFERENCE
    assert result.motion.average.vx == pytest.approx(0.3)
    assert result.motion.average.vy == pytest.approx(0.0, abs=1e-12)
    gps = next(x for x in result.contributors if x.original.source is Source.GPS)
    assert gps.heading_context is not None
    assert gps.heading_context.basis == "synthetic heading"
    assert gps.body_covariance is None  # Heading/covariance correlation not defined in T3a.
    assert gps.original.covariance.kind is CovarianceKind.DIAGONAL_ONLY
    assert result.motion.upper is None


@pytest.mark.parametrize(
    "change",
    [
        {"quality": float("nan")},
        {"quality": 0.01},
        {"values": (float("nan"), 0.0, 0.0)},
        {"frame": Frame.ENU},
        {"covariance": Covariance((-0.1, 0.1, 0.1), CovarianceKind.DIAGONAL_ONLY, "test", None)},
        {
            "covariance": Covariance(
                (1.0, 2.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0), CovarianceKind.FULL, "test", None
            )
        },
    ],
)
def test_invalid_values_quality_frame_and_covariance_are_rejected(change):
    assert not ingest(AdmissionOwner(config()), sample(**change)).accepted


def test_expiry_future_replay_epoch_restart_and_ros_rollback_do_not_refresh_health():
    owner = AdmissionOwner(config())
    assert ingest(owner, sample()).accepted
    assert not owner.ingest(sample(seq=2, end=11.0), now_ros=10.0, now_steady=20.0).accepted
    assert not owner.ingest(sample(seq=2), now_ros=10.6, now_steady=20.6).accepted
    assert owner.assess(now_ros=10.6, now_steady=20.6).motion is None
    assert ingest(owner, sample(seq=1, end=10.7, received=20.7, epoch="b")).accepted
    assert not ingest(owner, sample(seq=99, end=10.8, received=20.8)).accepted
    assert owner.assess(now_ros=1.0, now_steady=21.0).motion is None
    assert not owner.ingest(
        sample(seq=2, end=1.0, received=21.0, epoch="b"), now_ros=1.0, now_steady=21.0
    ).accepted


def test_loss_drift_and_imu_failure_remove_current_reference_without_reviving_old_data():
    owner = AdmissionOwner(replace(config(), recovery_samples=1, recovery_span_s=0))
    ingest(owner, sample(Source.WHEEL))
    ingest(owner, sample(Source.IMU))
    assert not ingest(
        owner, sample(Source.IMU, seq=2, end=10.1, received=20.1, status=SourceStatus.UNAVAILABLE)
    ).accepted
    assert owner.assess(now_ros=10.1, now_steady=20.1).mode is SourceMode.UNAVAILABLE
    assert not ingest(owner, sample(Source.GPS, status=SourceStatus.DRIFT)).accepted
    assert not ingest(
        owner, sample(Source.IMU, values=(0.1, 0.2, 0.3))
    ).accepted  # no accel integration


def test_clock_pause_invalidates_without_recounting_repeated_ticks():
    owner = AdmissionOwner(config())
    ingest(owner, sample(Source.WHEEL))
    ingest(owner, sample(Source.IMU))
    assert owner.assess(now_ros=10.0, now_steady=20.31).motion is None


def test_recovery_after_degradation_uses_configured_sample_span_and_source_identity():
    owner = AdmissionOwner(config())
    for seq, end, received in [(1, 9.8, 19.8), (2, 9.9, 19.9)]:
        for source in Source.WHEEL, Source.IMU:
            ingest(owner, sample(source, seq=seq, end=end, received=received))
    ingest(owner, sample())
    assert owner.assess(now_ros=10.0, now_steady=20.0).mode is SourceMode.WHEEL_IMU_LIMITED
    ingest(owner, sample(seq=2, end=10.05, received=20.05))
    assert owner.assess(now_ros=10.05, now_steady=20.05).mode is SourceMode.WHEEL_IMU_LIMITED
    ingest(owner, sample(seq=3, end=10.11, received=20.11))
    assert owner.assess(now_ros=10.11, now_steady=20.11).mode is SourceMode.LASER_REFERENCE


def test_invalid_fresh_sample_fences_earlier_packets_from_recovery():
    owner = AdmissionOwner(config())
    assert ingest(owner, sample()).accepted
    bad = sample(seq=3, end=10.2, received=20.2, status=SourceStatus.UNAVAILABLE)
    assert not ingest(owner, bad).accepted
    earlier = sample(seq=2, end=10.1, received=20.1)
    assert not owner.ingest(earlier, now_ros=10.2, now_steady=20.2).accepted


def test_huge_finite_off_diagonal_covariance_is_rejected_without_overflow():
    covariance = Covariance(
        (1.0, 1e300, 0.0, 1e300, 1.0, 0.0, 0.0, 0.0, 1.0),
        CovarianceKind.FULL,
        "invalid correlation",
        None,
    )
    assert not ingest(AdmissionOwner(config()), sample(covariance=covariance)).accepted


def test_legacy_motion_estimate_reuses_full_twist_and_requires_velocity_covariance():
    from senior_control.models import BodyTwist, Pose2D, Velocity
    from senior_control.motion.estimate_adapters import from_motion_estimate
    from senior_control.safety.motion_estimator import MotionEstimate

    estimate = MotionEstimate(
        Pose2D(0.03, 0.001, 0.01),
        Velocity(0.3, 0.1),
        (0.01, 0.01, 0.01),
        0.9,
        0.1,
        "scan_match",
        10.0,
        False,
        True,
        body_twist=BodyTwist(0.3, 0.01, 0.1),
    )
    covariance = Covariance(
        (0.02, 0.03, 0.04),
        CovarianceKind.DIAGONAL_ONLY,
        "caller provided velocity variance; cross terms unknown",
        "legacy",
    )
    observation = from_motion_estimate(
        estimate, received_steady=20.0, epoch="legacy", sequence=1, velocity_covariance=covariance
    )
    assert observation.values == (0.3, 0.01, 0.1)
    assert observation.covariance is covariance
    assert ingest(AdmissionOwner(config()), observation).accepted
    unavailable = from_motion_estimate(
        replace(estimate, body_twist=None),
        received_steady=20.0,
        epoch="legacy",
        sequence=2,
        velocity_covariance=covariance,
    )
    assert not ingest(AdmissionOwner(config()), unavailable).accepted


def test_delayed_new_epoch_and_retired_heading_cannot_replace_current_reference():
    owner = AdmissionOwner(config())
    assert ingest(owner, sample()).accepted
    delayed = sample(epoch="late-boot", seq=1, end=10.01, received=19.99)
    assert not owner.ingest(delayed, now_ros=10.1, now_steady=20.1).accepted

    heading = Heading(0.0, 0.01, 10.2, 20.2, "h1", "synthetic heading")
    assert ingest(owner, sample(Source.GPS, end=10.2, received=20.2), heading).accepted
    h2 = replace(heading, stamp_ros=10.3, received_steady=20.3, epoch="h2")
    assert ingest(owner, sample(Source.GPS, seq=2, end=10.3, received=20.3), h2).accepted
    assert not ingest(
        owner,
        sample(Source.GPS, seq=3, end=10.31, received=20.31),
        replace(heading, stamp_ros=10.31, received_steady=20.31),
    ).accepted


def test_heading_out_of_order_and_high_uncertainty_are_rejected():
    owner = AdmissionOwner(config())
    heading = Heading(0.0, 0.01, 10.0, 20.0, "h1", "synthetic heading")
    assert ingest(owner, sample(Source.GPS), heading).accepted
    assert not ingest(
        owner,
        sample(Source.GPS, seq=2, end=10.01, received=20.01),
        replace(heading, stamp_ros=9.99, received_steady=19.99),
    ).accepted
    assert not ingest(
        owner,
        sample(Source.GPS, seq=3, end=10.02, received=20.02),
        replace(heading, stamp_ros=10.02, received_steady=20.02, variance_rad2=10.0),
    ).accepted


def test_no_deterministic_bound_or_deadline_refresh_from_rejected_packet():
    owner = AdmissionOwner(replace(config(), recovery_samples=1, recovery_span_s=0))
    observation = sample()
    ingest(owner, observation)
    first = owner.assess(now_ros=10.0, now_steady=20.0)
    assert first.motion is not None
    deadline = first.motion.valid_until_steady
    assert not owner.ingest(observation, now_ros=10.1, now_steady=20.1).accepted
    current = owner.assess(now_ros=10.1, now_steady=20.1)
    assert current.motion is not None
    assert current.motion.valid_until_steady == deadline
    assert current.motion.upper is None


def test_component_assembly_never_labels_different_intervals_as_one_body_average():
    owner = AdmissionOwner(replace(config(), recovery_samples=1, recovery_span_s=0))
    heading = Heading(0.0, 0.01, 10.0, 20.0, "h1", "synthetic heading")
    ingest(owner, sample(Source.GPS), heading)
    assert owner.ingest(
        sample(Source.IMU, end=10.05, received=20.05), now_ros=10.05, now_steady=20.05
    ).accepted
    assessment = owner.assess(now_ros=10.05, now_steady=20.05)
    assert assessment.motion is None
    assert assessment.reason == "interval_alignment_required"


def test_gps_drift_falls_back_without_turning_covariance_into_a_speed_bound():
    owner = AdmissionOwner(replace(config(), recovery_samples=1, recovery_span_s=0))
    for source in Source.WHEEL, Source.IMU:
        ingest(owner, sample(source))
    heading = Heading(0.0, 0.01, 10.0, 20.0, "h1", "synthetic heading")
    ingest(owner, sample(Source.GPS), heading)
    assert owner.assess(now_ros=10.0, now_steady=20.0).mode is SourceMode.GPS_IMU_REFERENCE
    drift = sample(Source.GPS, seq=2, end=10.1, received=20.1, status=SourceStatus.DRIFT)
    assert not ingest(owner, drift, replace(heading, stamp_ros=10.1, received_steady=20.1)).accepted
    fallback = owner.assess(now_ros=10.1, now_steady=20.1)
    assert fallback.mode is SourceMode.WHEEL_IMU_LIMITED
    assert fallback.motion is not None and fallback.motion.upper is None
    assert fallback.target_linear_cap_mps == 0.2


@pytest.mark.parametrize("source", [Source.IMU, Source.WHEEL])
def test_nonfinite_auxiliary_removes_limited_reference(source):
    owner = AdmissionOwner(replace(config(), recovery_samples=1, recovery_span_s=0))
    for item in Source.WHEEL, Source.IMU:
        ingest(owner, sample(item))
    bad_values = (float("nan"),) if source is Source.IMU else (float("nan"), 0.0, 0.0)
    assert not ingest(
        owner, sample(source, seq=2, end=10.1, received=20.1, values=bad_values)
    ).accepted
    assert owner.assess(now_ros=10.1, now_steady=20.1).motion is None


def test_full_correlated_covariance_is_retained_without_fusing_precision():
    owner = AdmissionOwner(replace(config(), recovery_samples=1, recovery_span_s=0))
    covariance = Covariance(
        (1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        CovarianceKind.FULL,
        "perfectly correlated synthetic axes",
        "shared",
    )
    assert ingest(owner, sample(covariance=covariance)).accepted
    result = owner.assess(now_ros=10.0, now_steady=20.0)
    assert result.contributors[0].body_covariance is covariance
    assert result.motion is not None and result.motion.upper is None


def test_clock_reset_retires_old_heading_but_accepts_fresh_clock_generation():
    owner = AdmissionOwner(replace(config(), recovery_samples=1, recovery_span_s=0))
    heading = Heading(0.0, 0.01, 10.0, 20.0, "old-heading", "synthetic heading")
    assert ingest(owner, sample(Source.GPS), heading).accepted
    assert owner.assess(now_ros=1.0, now_steady=2.0).motion is None
    fresh = sample(Source.GPS, epoch="new-clock-source", end=1.0, received=2.0)
    new_heading = Heading(0.0, 0.01, 1.0, 2.0, "new-heading", "synthetic heading")
    assert owner.ingest(fresh, now_ros=1.0, now_steady=2.0, heading=new_heading).accepted


@pytest.mark.parametrize("sequence", [float("nan"), 1.5, True])
def test_source_sequence_must_be_a_real_positive_integer(sequence):
    assert not ingest(AdmissionOwner(config()), sample(seq=sequence)).accepted


def test_invalid_sequence_cannot_poison_the_replay_watermark():
    owner = AdmissionOwner(config())
    assert ingest(owner, sample()).accepted
    assert not ingest(owner, sample(seq=float("nan"), end=10.1, received=20.1)).accepted
    assert not ingest(owner, sample(seq=1, end=10.2, received=20.2)).accepted


@pytest.mark.parametrize(
    "changes",
    [
        {"sequence": "bad"},
        {"sequence": []},
        {"sequence": None},
        {"epoch": []},
        {"epoch": {}},
        {"source": []},
        {"source": "laser"},
        {"end_ros": "bad"},
        {"start_ros": 10.2},
        {"end_ros": 11.0},
        {"received_steady": 21.0},
        {"quality": {}},
        {"values": ("bad", 0.0, 0.0)},
    ],
)
def test_malformed_structure_after_valid_samples_is_pure_rejection(changes):
    owner = AdmissionOwner(config())
    ingest(owner, sample())
    ingest(owner, sample(seq=2, end=10.1, received=20.1))
    original = owner.assess(now_ros=10.1, now_steady=20.1)
    assert original.mode is SourceMode.LASER_REFERENCE
    for _ in range(3):
        bad = replace(sample(seq=999, end=10.2, received=20.2), **changes)
        assert not owner.ingest(bad, now_ros=10.2, now_steady=20.2).accepted
    current = owner.assess(now_ros=10.2, now_steady=20.2)
    assert current.mode is SourceMode.LASER_REFERENCE
    assert current.motion is not None and original.motion is not None
    assert current.motion.valid_until_steady == original.motion.valid_until_steady
    assert ingest(owner, sample(seq=3, end=10.3, received=20.3)).accepted


@pytest.mark.parametrize("end,received", [(11.0, 20.1), (10.1, 20.6)])
def test_recovery_does_not_bridge_ros_or_steady_expiry_without_assess(end, received):
    owner = AdmissionOwner(config())
    ingest(owner, sample())
    assert ingest(owner, sample(seq=2, end=end, received=received)).accepted
    assert owner.assess(now_ros=end, now_steady=received).mode is SourceMode.RECOVERING
    ingest(owner, sample(seq=3, end=end + 0.1, received=received + 0.1))
    assert (
        owner.assess(now_ros=end + 0.1, now_steady=received + 0.1).mode
        is SourceMode.LASER_REFERENCE
    )


@pytest.mark.parametrize("ros_step,steady_step", [(0.1, 0.1), (0.2, 0.1), (0.1, 0.2)])
def test_continuous_recovery_handles_distinct_real_time_factors(ros_step, steady_step):
    owner = AdmissionOwner(config())
    ingest(owner, sample())
    assert ingest(owner, sample(seq=2, end=10.0 + ros_step, received=20.0 + steady_step)).accepted
    assert (
        owner.assess(now_ros=10.0 + ros_step, now_steady=20.0 + steady_step).mode
        is SourceMode.LASER_REFERENCE
    )


def test_unused_heading_does_not_add_a_laser_admission_gate():
    owner = AdmissionOwner(config())
    assert owner.ingest(sample(), now_ros=10.0, now_steady=20.0, heading=[]).accepted
