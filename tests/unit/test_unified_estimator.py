import pytest
from sensor_time_test_support import sample_time

from senior_control.motion.estimator import MotionEstimation, MotionObservation, Origin


def observation(source, values, stamp=9.9, quality=1.0, root=None, sequence=1):
    return MotionObservation(
        source,
        values,
        stamp,
        (
            Origin(
                root or source,
                "boot",
                sequence,
                sample_time(
                    stamp_ns=round((stamp) * 1e9),
                    received_ros_ns=round((max(10.0, stamp)) * 1e9),
                    received_steady_ns=round((20.0 + max(0, stamp - 10)) * 1e9),
                ),
            ),
        ),
        quality,
        measured_steady=stamp + 10.0,
        stamp_ns=round((stamp) * 1e9),
    )


def test_close_weighted_and_missing_lateral_is_not_missing_motion():
    e = MotionEstimation()
    e.ingest(observation("wheel", (0.4, None, 0.1)))
    e.ingest(observation("laser", (0.44, None, 0.12), quality=0.5))
    r = e.estimate(10.0, 20.0)
    assert r.vx.value == pytest.approx(0.413333333333)
    assert r.vy.value == 0 and not r.vy.observed
    assert r.wz.observed and r.healthy


def test_two_conflict_new_raw_observation_wins_not_quality():
    e = MotionEstimation()
    e.ingest(observation("old", (0.1, None, None), 9.4))
    e.ingest(observation("new", (0.9, None, None), 9.9, 0.1))
    assert e.estimate(10.0, 20.0).vx.value == 0.9


def test_consensus_beats_new_outlier_and_shared_root_not_extra_vote():
    e = MotionEstimation()
    for o in [
        observation("a", (0.4, None, None)),
        observation("b", (0.44, None, None)),
        observation("bad", (1.2, None, None), 9.99),
        observation("copy", (1.2, None, None), 9.99, root="bad"),
    ]:
        e.ingest(o)
    r = e.estimate(10.0, 20.0)
    assert r.vx.value == pytest.approx(0.42)
    assert len(r.vx.sources) == 2


def test_same_time_and_no_consensus_deterministic():
    e = MotionEstimation()
    for s, v in [("b", 0.5), ("a", 0.1), ("c", 1.0)]:
        e.ingest(observation(s, (v, None, None)))
    r = e.estimate(10.0, 20.0)
    assert r.vx.value == 0.1
    assert r.vx.confidence <= 0.2


def test_expiring_contributor_does_not_expire_live_axis():
    e = MotionEstimation()
    e.ingest(observation("old", (0.4, None, 0.1), 9.0))
    e.ingest(observation("live", (0.42, None, 0.12), 9.8))
    r = e.estimate(10.0, 20.0)
    assert r.vx.value == 0.42 and r.healthy


def test_prediction_own_axis_deadline_not_renewed_by_other_axis():
    e = MotionEstimation()
    e.ingest(observation("wheel", (0.4, None, None), 9.5))
    e.estimate(10.0, 20.0)
    e.invalidate("wheel")
    e.ingest(observation("gyro", (None, None, 0.1), 10.2))
    r = e.estimate(10.3, 20.3)
    assert r.vx.value == 0.4 and not r.vx.observed
    r = e.estimate(10.5, 20.5)
    assert r.vx.value is None and r.wz.observed and r.healthy


def test_single_wheel_no_other_sensor_required_and_all_expired_stops():
    e = MotionEstimation()
    e.ingest(observation("wheel", (0.4, None, 0.2)))
    assert e.estimate(10.0, 20.0).healthy
    assert not e.estimate(10.9, 20.9).healthy


def test_bad_quality_and_replay_do_not_replace_valid_sample():
    e = MotionEstimation()
    e.ingest(observation("wheel", (0.4, None, 0.2)))
    with pytest.raises(ValueError):
        e.ingest(observation("wheel", (0.9, None, 0.2), quality=float("nan")))
    assert not e.ingest(observation("wheel", (0.9, None, 0.2)))
    assert e.estimate(10.0, 20.0).vx.value == 0.4


def test_nontransitive_consistency_uses_pairwise_clique_not_chain_average():
    e = MotionEstimation()
    for name, value in [("a", 0.4), ("b", 0.48), ("c", 0.56)]:
        e.ingest(observation(name, (value, None, None)))
    r = e.estimate(10, 20)
    assert r.vx.sources == ("a", "b")
    assert r.vx.value == pytest.approx(0.44)


def test_fully_represented_derived_roots_do_not_join_raw_sources_into_extra_vote():
    from dataclasses import replace

    a = observation("a", (0.4, None, 0.0))
    b = observation("b", (0.44, None, 0.0))
    derived = replace(a, source="derived", values=(1.2, None, 0.0), roots=a.roots + b.roots)
    e = MotionEstimation()
    for value in (a, b, derived):
        e.ingest(value)
    assert e.estimate(10, 20).vx.value == pytest.approx(0.42)


def test_partial_new_sample_preserves_own_other_axis_original_deadline():
    e = MotionEstimation()
    e.ingest(observation("body", (0.4, None, 0.1), 9.5))
    e.estimate(10, 20)
    e.ingest(observation("body", (None, None, 0.2), 10.2, sequence=2))
    r = e.estimate(10.3, 20.3)
    assert r.vx.value == 0.4 and r.vx.observed and r.wz.value > 0.1
    r = e.estimate(10.5, 20.5)
    assert r.vx.value is None and r.wz.observed


def test_alignment_uses_only_same_source_history_and_declared_acceleration_limit():
    e = MotionEstimation(acceleration=(0.3, 0.3, 0.5))
    e.ingest(observation("a", (0.1, None, None), 9.6))
    e.ingest(observation("a", (0.4, None, None), 9.9, sequence=2))
    r = e.estimate(10, 20)
    assert r.vx.value == pytest.approx(0.43)
    assert r.vx.error is None


def test_interval_conflict_uses_latest_original_endpoint_not_predicted_timestamp():
    from dataclasses import replace

    older = observation("interval", (0.4, None, None), 9.8)
    end = replace(
        older.roots[0],
        time=sample_time(
            stamp_ns=9_990_000_000,
            received_ros_ns=10_000_000_000,
            received_steady_ns=20_000_000_000,
        ),
    )
    interval = replace(
        older,
        measured_ros=9.895,
        measured_steady=19.895,
        roots=older.roots + (end,),
        interval_half_s=0.095,
    )
    e = MotionEstimation()
    e.ingest(interval)
    e.ingest(observation("point", (0.9, None, None), 9.95))
    assert e.estimate(10, 20).vx.value == 0.4


@pytest.mark.parametrize("quality", [0.0, None, 0.01])
def test_quality_semantics_do_not_make_missing_covariance_an_unavailable_axis(quality):
    e = MotionEstimation()
    e.ingest(observation("body", (0.4, None, 0.1), quality=quality))
    r = e.estimate(10, 20)
    assert r.healthy is (quality != 0.0)
    if quality != 0.0:
        assert r.vx.error is None


def test_republished_same_acquisition_cannot_renew_paused_clock_lifetime():
    from dataclasses import replace

    e = MotionEstimation()
    original = observation("wheel", (0.4, None, 0.1), stamp=9.9)
    e.ingest(original)
    e.estimate(10, 20)
    for sequence in range(2, 6):
        replay = replace(
            original,
            roots=(replace(original.roots[0], sequence=sequence),),
        )
        assert not e.ingest(replay)
    assert not e.estimate(10, 20.9).healthy


def test_invalid_status_veto_is_immediate_but_cached_recovery_is_not_new_data():
    from dataclasses import replace

    e = MotionEstimation()
    original = observation("wheel", (0.4, None, 0.1))
    e.ingest(original)
    e.estimate(10, 20)
    invalid = replace(original, quality=0.0, roots=(replace(original.roots[0], sequence=2),))
    assert e.ingest(invalid)
    assert not e.estimate(10, 20.1).healthy
    cached = replace(original, roots=(replace(original.roots[0], sequence=3),))
    assert not e.ingest(cached)


def test_local_rollback_revokes_already_consumed_new_producer_evidence():
    e = MotionEstimation()
    e.ingest(observation("wheel", (0.4, None, 0.1)))
    e.estimate(10, 20)
    root = Origin(
        "wheel",
        "new-clock",
        1,
        sample_time(
            stamp_ns=round((4.9) * 1e9),
            received_ros_ns=round((5.0) * 1e9),
            received_steady_ns=round((20.1) * 1e9),
        ),
    )
    e.ingest(
        MotionObservation(
            "wheel",
            (0.3, None, 0.1),
            4.9,
            (root,),
            measured_steady=((root,))[-1].measurement_steady,
            stamp_ns=((root,))[-1].coordinate,
        )
    )
    # This owner used the new producer before observing its own ROS rollback.
    assert e.estimate(10, 20.15).healthy
    result = e.estimate(5.0, 20.2)
    assert not result.healthy and result.generation == 2
    assert not result.vx.roots
    assert result.path is None
