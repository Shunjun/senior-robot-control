"""Synthetic policy values verify pure logic only; no physical bound is frozen."""

import math
from dataclasses import replace

import pytest
from source_time_test_support import capture_time, synthetic_domain  # noqa: F401

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
from senior_control.sensors.time_proof import MeasurementInterval


def config():
    return AdmissionConfig(
        rules=tuple(SourceRule(source, 500, 300, 0.1, 2.0) for source in Source),
        recovery_samples=2,
        recovery_span_ms=100,
        normal_cap_mps=1.0,
        degraded_cap_mps=0.2,
        heading_max_age_ms=200,
        alignment_max_skew_ms=100,
        heading_max_variance_rad2=0.05,
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
        start_steady=(received) - ((end) - (end - 0.1)),
        end_steady=received,
        measurement_interval=MeasurementInterval(
            capture_time((received) - ((end) - (end - 0.1)), tick=end - 0.1).measurement,
            capture_time(received, tick=end).measurement,
        ),
    )
    return replace(observation, **changes)


def ingest(owner, observation, heading=None):
    return owner.ingest(
        observation,
        now_ros=observation.end_ros,
        now_steady=observation.received_steady,
        heading=heading,
    )


def views(owner, *, now_ros, now_steady):
    return dict(owner.snapshot(after_cursor=0, now_ros=now_ros, now_steady=now_steady).sources)


def ready(owner, source, *, now_ros, now_steady):
    view = views(owner, now_ros=now_ros, now_steady=now_steady).get(source)
    return view is not None and view.ready


def test_recovery_counts_only_distinct_samples():
    owner = AdmissionOwner(config(), event_capacity=128)
    first = sample()
    assert ingest(owner, first).accepted
    for _ in range(10):
        assert not ready(owner, Source.LASER, now_ros=10.0, now_steady=20.0)
    assert not ingest(owner, first).accepted
    assert ingest(owner, sample(seq=2, end=10.1, received=20.1)).accepted
    assert ready(owner, Source.LASER, now_ros=10.1, now_steady=20.1)


def test_weak_laser_does_not_remove_admitted_wheel_and_imu():
    owner = AdmissionOwner(
        replace(config(), recovery_samples=1, recovery_span_ms=0), event_capacity=128
    )
    assert ingest(owner, sample(Source.WHEEL)).accepted
    assert ingest(owner, sample(Source.IMU)).accepted
    assert not ingest(owner, sample(status=SourceStatus.UNAVAILABLE)).accepted
    current = views(owner, now_ros=10.0, now_steady=20.0)
    assert current[Source.WHEEL].ready and current[Source.IMU].ready
    assert Source.LASER not in current
    assert current[Source.WHEEL].observation.body_values == (0.31, 0.0, 0.12)


def test_gps_requires_known_fresh_heading_and_does_not_assume_covariance_independence():
    owner = AdmissionOwner(config(), event_capacity=128)
    assert not ingest(owner, sample(Source.GPS)).accepted
    heading = Heading(
        math.pi / 2,
        0.02,
        10.0,
        20.0,
        "heading-a",
        "synthetic heading",
        measurement_steady=20.0,
        measurement=capture_time(20.0, tick=10.0).measurement,
    )
    assert ingest(owner, sample(Source.GPS, seq=2), heading).accepted
    assert ingest(owner, sample(Source.IMU)).accepted
    assert ingest(
        owner,
        sample(Source.GPS, seq=3, end=10.1, received=20.1),
        replace(
            heading,
            stamp_ros=10.1,
            received_steady=20.1,
            measurement_steady=20.1,
            measurement=capture_time(20.1, tick=10.1).measurement,
        ),
    ).accepted
    assert ingest(owner, sample(Source.IMU, seq=2, end=10.1, received=20.1)).accepted
    current = views(owner, now_ros=10.1, now_steady=20.1)
    assert current[Source.GPS].ready and current[Source.IMU].ready
    gps = current[Source.GPS].observation
    assert gps.body_values[0] == pytest.approx(0.3)
    assert gps.body_values[1] == pytest.approx(0.0, abs=1e-12)
    assert gps.heading_context is not None
    assert gps.heading_context.basis == "synthetic heading"
    assert gps.body_covariance is None  # Heading/covariance correlation is not defined.
    assert gps.original.covariance.kind is CovarianceKind.DIAGONAL_ONLY


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
    assert not ingest(AdmissionOwner(config(), event_capacity=128), sample(**change)).accepted


def test_expiry_future_replay_epoch_restart_and_ros_rollback_do_not_refresh_health():
    owner = AdmissionOwner(config(), event_capacity=128)
    assert ingest(owner, sample()).accepted
    assert not owner.ingest(sample(seq=2, end=11.0), now_ros=10.0, now_steady=20.0).accepted
    assert not owner.ingest(sample(seq=2), now_ros=10.6, now_steady=20.6).accepted
    assert not views(owner, now_ros=10.6, now_steady=20.6)
    assert ingest(owner, sample(seq=1, end=10.7, received=20.7, epoch="b")).accepted
    assert not ingest(owner, sample(seq=99, end=10.8, received=20.8)).accepted
    assert views(owner, now_ros=1.0, now_steady=21.0)
    assert not owner.ingest(
        sample(seq=2, end=1.0, received=20.9, epoch="b"), now_ros=1.0, now_steady=21.0
    ).accepted


def test_loss_drift_and_imu_failure_remove_current_reference_without_reviving_old_data():
    owner = AdmissionOwner(
        replace(config(), recovery_samples=1, recovery_span_ms=0), event_capacity=128
    )
    ingest(owner, sample(Source.WHEEL))
    ingest(owner, sample(Source.IMU))
    assert not ingest(
        owner, sample(Source.IMU, seq=2, end=10.1, received=20.1, status=SourceStatus.UNAVAILABLE)
    ).accepted
    assert not ready(owner, Source.IMU, now_ros=10.1, now_steady=20.1)
    assert not ingest(owner, sample(Source.GPS, status=SourceStatus.DRIFT)).accepted
    assert not ingest(
        owner, sample(Source.IMU, values=(0.1, 0.2, 0.3))
    ).accepted  # no accel integration


def test_paused_ros_keeps_sources_until_original_host_timeout():
    owner = AdmissionOwner(config(), event_capacity=128)
    ingest(owner, sample(Source.WHEEL))
    ingest(owner, sample(Source.IMU))
    assert views(owner, now_ros=10.0, now_steady=20.31)
    assert not views(owner, now_ros=10.0, now_steady=21.0)


def test_recovery_after_degradation_uses_configured_sample_span_and_source_identity():
    owner = AdmissionOwner(config(), event_capacity=128)
    for seq, end, received in [(1, 9.8, 19.8), (2, 9.9, 19.9)]:
        for source in Source.WHEEL, Source.IMU:
            ingest(owner, sample(source, seq=seq, end=end, received=received))
    ingest(owner, sample())
    assert not ready(owner, Source.LASER, now_ros=10.0, now_steady=20.0)
    ingest(owner, sample(seq=2, end=10.05, received=20.05))
    assert not ready(owner, Source.LASER, now_ros=10.05, now_steady=20.05)
    ingest(owner, sample(seq=3, end=10.11, received=20.11))
    assert ready(owner, Source.LASER, now_ros=10.11, now_steady=20.11)


def test_invalid_fresh_sample_fences_earlier_packets_from_recovery():
    owner = AdmissionOwner(config(), event_capacity=128)
    assert ingest(owner, sample()).accepted
    bad = sample(seq=3, end=10.2, received=20.2, status=SourceStatus.UNAVAILABLE)
    assert not ingest(owner, bad).accepted
    earlier = sample(seq=2, end=10.1, received=20.1)
    assert not owner.ingest(earlier, now_ros=10.2, now_steady=20.2).accepted


def test_delayed_new_epoch_and_retired_heading_cannot_replace_current_reference():
    owner = AdmissionOwner(config(), event_capacity=128)
    assert ingest(owner, sample()).accepted
    delayed = sample(epoch="late-boot", seq=1, end=10.01, received=19.99)
    assert not owner.ingest(delayed, now_ros=10.1, now_steady=20.1).accepted

    heading = Heading(
        0.0,
        0.01,
        10.2,
        20.2,
        "h1",
        "synthetic heading",
        measurement_steady=20.2,
        measurement=capture_time(20.2, tick=10.2).measurement,
    )
    assert ingest(owner, sample(Source.GPS, end=10.2, received=20.2), heading).accepted
    h2 = replace(
        heading,
        stamp_ros=10.3,
        received_steady=20.3,
        epoch="h2",
        measurement_steady=20.3,
        measurement=capture_time(20.3, tick=10.3).measurement,
    )
    assert ingest(owner, sample(Source.GPS, seq=2, end=10.3, received=20.3), h2).accepted
    assert not ingest(
        owner,
        sample(Source.GPS, seq=3, end=10.31, received=20.31),
        replace(
            heading,
            stamp_ros=10.31,
            received_steady=20.31,
            measurement_steady=20.31,
            measurement=capture_time(20.31, tick=10.31).measurement,
        ),
    ).accepted


def test_heading_out_of_order_and_high_uncertainty_are_rejected():
    owner = AdmissionOwner(config(), event_capacity=128)
    heading = Heading(
        0.0,
        0.01,
        10.0,
        20.0,
        "h1",
        "synthetic heading",
        measurement_steady=20.0,
        measurement=capture_time(20.0, tick=10.0).measurement,
    )
    assert ingest(owner, sample(Source.GPS), heading).accepted
    assert not ingest(
        owner,
        sample(Source.GPS, seq=2, end=10.01, received=20.01),
        replace(
            heading,
            stamp_ros=9.99,
            received_steady=19.99,
            measurement_steady=19.99,
            measurement=capture_time(19.99, tick=9.99).measurement,
        ),
    ).accepted
    assert not ingest(
        owner,
        sample(Source.GPS, seq=3, end=10.02, received=20.02),
        replace(
            heading,
            stamp_ros=10.02,
            received_steady=20.02,
            variance_rad2=10.0,
            measurement_steady=20.02,
            measurement=capture_time(20.02, tick=10.02).measurement,
        ),
    ).accepted


def test_rejected_packet_does_not_refresh_source_deadline():
    owner = AdmissionOwner(
        replace(config(), recovery_samples=1, recovery_span_ms=0), event_capacity=128
    )
    observation = sample()
    ingest(owner, observation)
    first = views(owner, now_ros=10.0, now_steady=20.0)[Source.LASER]
    deadline = first.observation.valid_until_steady
    assert not owner.ingest(observation, now_ros=10.1, now_steady=20.1).accepted
    current = views(owner, now_ros=10.1, now_steady=20.1)[Source.LASER]
    assert current.ready
    assert current.observation.valid_until_steady == deadline


def test_gps_drift_does_not_revoke_healthy_wheel_and_imu():
    owner = AdmissionOwner(
        replace(config(), recovery_samples=1, recovery_span_ms=0), event_capacity=128
    )
    for source in Source.WHEEL, Source.IMU:
        ingest(owner, sample(source))
    heading = Heading(
        0.0,
        0.01,
        10.0,
        20.0,
        "h1",
        "synthetic heading",
        measurement_steady=20.0,
        measurement=capture_time(20.0, tick=10.0).measurement,
    )
    ingest(owner, sample(Source.GPS), heading)
    assert ready(owner, Source.GPS, now_ros=10.0, now_steady=20.0)
    drift = sample(Source.GPS, seq=2, end=10.1, received=20.1, status=SourceStatus.DRIFT)
    assert not ingest(
        owner,
        drift,
        replace(
            heading,
            stamp_ros=10.1,
            received_steady=20.1,
            measurement_steady=20.1,
            measurement=capture_time(20.1, tick=10.1).measurement,
        ),
    ).accepted
    current = views(owner, now_ros=10.1, now_steady=20.1)
    assert Source.GPS not in current
    assert current[Source.WHEEL].ready and current[Source.IMU].ready


@pytest.mark.parametrize("source", [Source.IMU, Source.WHEEL])
def test_nonfinite_auxiliary_removes_limited_reference(source):
    owner = AdmissionOwner(
        replace(config(), recovery_samples=1, recovery_span_ms=0), event_capacity=128
    )
    for item in Source.WHEEL, Source.IMU:
        ingest(owner, sample(item))
    bad_values = (float("nan"),) if source is Source.IMU else (float("nan"), 0.0, 0.0)
    assert not ingest(
        owner, sample(source, seq=2, end=10.1, received=20.1, values=bad_values)
    ).accepted
    assert not ready(owner, source, now_ros=10.1, now_steady=20.1)


def test_full_correlated_covariance_is_retained_without_fusing_precision():
    owner = AdmissionOwner(
        replace(config(), recovery_samples=1, recovery_span_ms=0), event_capacity=128
    )
    covariance = Covariance(
        (1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        CovarianceKind.FULL,
        "perfectly correlated synthetic axes",
        "shared",
    )
    assert ingest(owner, sample(covariance=covariance)).accepted
    current = views(owner, now_ros=10.0, now_steady=20.0)[Source.LASER]
    assert current.ready
    assert current.observation.body_covariance is covariance


def test_clock_reset_retires_old_heading_but_accepts_fresh_clock_generation():
    owner = AdmissionOwner(
        replace(config(), recovery_samples=1, recovery_span_ms=0), event_capacity=128
    )
    heading = Heading(
        0.0,
        0.01,
        10.0,
        20.0,
        "old-heading",
        "synthetic heading",
        measurement_steady=20.0,
        measurement=capture_time(20.0, tick=10.0).measurement,
    )
    assert ingest(owner, sample(Source.GPS), heading).accepted
    assert not views(owner, now_ros=1.0, now_steady=2.0)
    fresh = sample(Source.GPS, epoch="new-clock-source", end=1.0, received=2.0)
    new_heading = Heading(
        0.0,
        0.01,
        1.0,
        2.0,
        "new-heading",
        "synthetic heading",
        measurement_steady=2.0,
        measurement=capture_time(2.0, tick=1.0).measurement,
    )
    assert owner.ingest(fresh, now_ros=1.0, now_steady=2.0, heading=new_heading).accepted


@pytest.mark.parametrize("end,received", [(11.0, 20.6), (10.1, 20.6)])
def test_recovery_does_not_bridge_ros_or_steady_expiry_without_snapshot(end, received):
    owner = AdmissionOwner(config(), event_capacity=128)
    ingest(owner, sample())
    assert ingest(owner, sample(seq=2, end=end, received=received)).accepted
    assert not ready(owner, Source.LASER, now_ros=end, now_steady=received)
    ingest(owner, sample(seq=3, end=end + 0.1, received=received + 0.1))
    assert ready(owner, Source.LASER, now_ros=end + 0.1, now_steady=received + 0.1)


@pytest.mark.parametrize("ros_step,steady_step", [(0.1, 0.1), (0.2, 0.1), (0.1, 0.2)])
def test_continuous_recovery_handles_distinct_real_time_factors(ros_step, steady_step):
    owner = AdmissionOwner(config(), event_capacity=128)
    ingest(owner, sample())
    assert ingest(owner, sample(seq=2, end=10.0 + ros_step, received=20.0 + steady_step)).accepted
    assert ready(owner, Source.LASER, now_ros=10.0 + ros_step, now_steady=20.0 + steady_step)


def test_unused_heading_does_not_add_a_laser_admission_gate():
    owner = AdmissionOwner(config(), event_capacity=128)
    assert owner.ingest(sample(), now_ros=10.0, now_steady=20.0, heading=[]).accepted
