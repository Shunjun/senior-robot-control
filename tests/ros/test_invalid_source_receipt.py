"""Undated failures fence at the original receipt, not a later callback clock."""

from types import SimpleNamespace

from test_future_source_clock import _scan
from test_future_source_clock import active_fusion as active_fusion


def test_undated_invalid_uses_first_ros_receipt_for_its_failure_frontier(
    active_fusion, monkeypatch
):
    fusion, _clock, base, emitted = active_fusion
    received = base + 10_000_000
    later = received + 50_000_000
    readings = iter((received, later))

    def clock_now():
        return SimpleNamespace(nanoseconds=next(readings, later))

    monkeypatch.setattr(fusion, "get_clock", lambda: SimpleNamespace(now=clock_now))
    fusion._on_scan(_scan(base, 0, valid=False))
    assert fusion._progress.unqualified and emitted and not emitted[-1].valid
    assert fusion._scan_frontier.interval_after_failure(received)
    assert not fusion._scan_frontier.interval_after_failure(received - 1)
