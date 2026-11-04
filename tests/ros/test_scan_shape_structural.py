"""Scan health fields must be structurally consistent with an actual match."""

import pytest
from test_future_source_clock import _add_stationary_end, _scan
from test_future_source_clock import active_fusion as active_fusion


@pytest.mark.parametrize(
    "fault",
    [
        "residual_nan",
        "residual_infinite",
        "residual_negative",
        "no_rays",
        "noncanonical_end_same_ns",
        "noncanonical_start_same_ns",
    ],
)
def test_invalid_match_structure_revokes_without_advancing_valid_progress(active_fusion, fault):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp + 10_000_000
    _add_stationary_end(fusion, stamp)
    message = _scan(base, stamp)
    message.valid_rays = 720
    message.residual = 0.001
    if fault == "residual_nan":
        message.residual = float("nan")
    elif fault == "residual_infinite":
        message.residual = float("inf")
    elif fault == "residual_negative":
        message.residual = -0.001
    elif fault == "no_rays":
        message.valid_rays = 0
    elif fault == "noncanonical_end_same_ns":
        message.header.stamp.sec -= 1
        message.header.stamp.nanosec += 1_000_000_000
        assert message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec == stamp
    else:
        message.interval_start.sec -= 1
        message.interval_start.nanosec += 1_000_000_000
        assert message.interval_start.sec * 1_000_000_000 + message.interval_start.nanosec == base
    fusion._on_scan(message)
    assert fusion._progress.unqualified and not fusion._progress_started
    assert fusion._pending is None
    assert emitted and not emitted[-1].valid
    assert fusion._scan_frontier.interval_after_failure(stamp)
    assert not fusion._scan_frontier.interval_after_failure(stamp - 1)


def test_valid_structural_match_still_qualifies(active_fusion):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp + 10_000_000
    _add_stationary_end(fusion, stamp)
    message = _scan(base, stamp)
    message.valid_rays = 720
    message.residual = 0.001
    fusion._on_scan(message)
    assert fusion._progress_started and not fusion._progress.unqualified
    assert emitted and emitted[-1].valid
