"""A malformed independent scan must invalidate immediately, before alignment."""

import math

import pytest
from test_future_source_clock import _add_stationary_end, _scan
from test_future_source_clock import active_fusion as active_fusion


@pytest.mark.parametrize(
    "fault",
    [
        "reason_overflow",
        "frame_overflow",
        "response_above_one",
        "response_below_minimum",
        "response_nonfinite",
        "covariance_nonfinite",
        "covariance_zero",
        "covariance_over_limit",
        "pose_nonfinite",
        "delta_nonfinite",
        "noncanonical_end",
        "noncanonical_start",
    ],
)
def test_invalid_source_shape_immediately_revokes_progress_and_fences_interval(
    active_fusion, fault
):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp + 10_000_000
    _add_stationary_end(fusion, stamp)
    message = _scan(base, stamp)
    message.reason = "qualified"
    if fault == "reason_overflow":
        message.reason = "r" * 65
    elif fault == "frame_overflow":
        message.header.frame_id = "f" * 129
    elif fault == "response_above_one":
        message.response = 1.5
    elif fault == "response_below_minimum":
        message.response = 0.1
    elif fault == "response_nonfinite":
        message.response = float("nan")
    elif fault == "covariance_nonfinite":
        message.covariance[1] = float("nan")
    elif fault == "covariance_zero":
        message.covariance[0] = 0.0
    elif fault == "covariance_over_limit":
        message.covariance[0] = 0.2
    elif fault == "pose_nonfinite":
        message.local_pose.x = float("nan")
    elif fault == "delta_nonfinite":
        message.body_delta.x = float("nan")
    elif fault == "noncanonical_end":
        message.header.stamp.sec -= 1
        message.header.stamp.nanosec += 1_000_000_000
    else:
        message.interval_start.sec -= 1
        message.interval_start.nanosec += 1_000_000_000
    fusion._on_scan(message)
    assert fusion._progress.unqualified and not fusion._progress_started
    assert fusion._pending is None
    assert emitted and not emitted[-1].valid
    assert emitted[-1].header.stamp.sec == message.header.stamp.sec
    assert emitted[-1].header.stamp.nanosec == message.header.stamp.nanosec
    assert fusion._scan_frontier.interval_after_failure(stamp)
    assert not fusion._scan_frontier.interval_after_failure(stamp - 1)


def test_valid_source_shape_still_qualifies(active_fusion):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp + 10_000_000
    _add_stationary_end(fusion, stamp)
    message = _scan(base, stamp)
    fusion._on_scan(message)
    assert fusion._progress_started and not fusion._progress.unqualified
    assert emitted and emitted[-1].valid
    assert math.isfinite(emitted[-1].quality)


def test_retired_source_with_oversized_reason_cannot_discard_successor(active_fusion):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    fusion._on_scan(_scan(base, stamp, epoch="new-epoch"))
    pending = fusion._deferred_scan.peek()
    assert pending is not None and pending.value.source_epoch == "new-epoch"
    count = len(emitted)
    old = _scan(base, stamp + 1, valid=False)
    old.reason = "r" * 65
    fusion._on_scan(old)
    assert fusion._deferred_scan.peek() is pending
    assert len(emitted) == count


def test_older_same_epoch_frame_overflow_cannot_revoke_newer_qualified_progress(active_fusion):
    fusion, clock, base, emitted = active_fusion
    clock["now"] = base + 10_000_000
    old = _scan(base - 100_000_000, base - 50_000_000, valid=False)
    old.header.frame_id = "f" * 129
    fusion._on_scan(old)
    assert fusion._progress_started and not fusion._progress.unqualified
    assert fusion._scan_epochs.active == "stable-independent-scan"
    assert not emitted


@pytest.mark.parametrize("field", ["frame", "reason"])
def test_future_new_epoch_with_oversized_field_revokes_but_waits_to_activate(active_fusion, field):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    new = _scan(base, stamp, epoch="new-epoch")
    if field == "frame":
        new.header.frame_id = "f" * 129
    else:
        new.reason = "r" * 65
    fusion._on_scan(new)
    assert fusion._scan_epochs.is_retired("stable-independent-scan")
    assert fusion._scan_epochs.active is None
    assert fusion._progress.unqualified and emitted and not emitted[-1].valid
    pending = fusion._deferred_scan.peek()
    assert pending is not None and pending.value.source_epoch == "new-epoch"
    assert not pending.value.valid
    assert pending.stamp_ns == stamp
    clock["now"] = stamp
    fusion._try_pending()
    assert fusion._scan_epochs.active == "new-epoch"
    assert fusion._progress.unqualified and not emitted[-1].valid
