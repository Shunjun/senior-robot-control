"""Malformed source generations may not erase a newer pending failure."""

from types import SimpleNamespace

from test_future_source_clock import _scan
from test_future_source_clock import active_fusion as active_fusion


def test_invalid_due_successor_retires_the_unadmitted_failed_epoch(active_fusion):
    fusion, clock, base, emitted = active_fusion
    future = base + 100_000_000
    clock["now"] = future - 2_000_000
    fusion._on_scan(_scan(base, future, epoch="epoch-b", valid=False))
    assert fusion._deferred_scan.peek().value.source_epoch == "epoch-b"
    current = base + 90_000_000
    fusion._on_scan(_scan(base, current, epoch="epoch-c", valid=False))
    assert fusion._scan_epochs.active == "epoch-c"
    assert fusion._scan_epochs.is_retired("epoch-b")
    assert fusion._deferred_scan.peek() is None
    count = len(emitted)
    for valid in (False, True):
        fusion._on_scan(_scan(base, future, epoch="epoch-b", valid=valid))
        assert fusion._scan_epochs.active == "epoch-c"
        assert len(emitted) == count


def test_earlier_due_invalid_cannot_erase_newer_pending_invalid_frontier(active_fusion):
    fusion, clock, base, emitted = active_fusion
    future = base + 100_000_000
    clock["now"] = future - 2_000_000
    fusion._on_scan(_scan(base, future, epoch="epoch-b", valid=False))
    earlier = base + 90_000_000
    fusion._on_scan(_scan(base, earlier, epoch="epoch-b", valid=False))
    assert fusion._scan_epochs.active == "epoch-b"
    assert fusion._progress.unqualified and emitted and not emitted[-1].valid
    assert not fusion._scan_frontier.interval_after_failure(earlier)
    assert fusion._scan_frontier.interval_after_failure(future)


def test_future_noncanonical_new_epoch_is_retired_without_becoming_active(active_fusion):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    message = _scan(base, stamp, epoch="epoch-b")
    message.header.stamp.sec -= 1
    message.header.stamp.nanosec += 1_000_000_000
    fusion._on_scan(message)
    assert fusion._scan_epochs.active is None
    assert fusion._scan_epochs.is_retired("stable-independent-scan")
    assert fusion._scan_epochs.is_retired("epoch-b")
    assert fusion._deferred_scan.peek() is None
    assert fusion._progress.unqualified and emitted and not emitted[-1].valid


def _reserve_before_progress_start(fusion, clock, base):
    fusion._progress_started = False
    fusion._progress.reset_after_requalification()
    fusion._scan_epochs.retire_active()
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    fusion._on_scan(_scan(base, stamp, epoch="epoch-b"))
    assert fusion._scan_epochs.active is None
    assert fusion._deferred_scan.peek().value.source_epoch == "epoch-b"


def test_clock_rollback_retires_reserved_epoch_even_before_progress_start(active_fusion):
    fusion, clock, base, _emitted = active_fusion
    _reserve_before_progress_start(fusion, clock, base)
    clock["now"] -= 1
    fusion._try_pending()
    assert fusion._scan_epochs.is_retired("epoch-b")
    assert fusion._scan_epochs.active is None and fusion._deferred_scan.peek() is None


def test_gid_change_retires_reserved_epoch_even_before_progress_start(active_fusion, monkeypatch):
    from senior_robot.motion_fusion import MotionFusion

    fusion, clock, base, _emitted = active_fusion
    _reserve_before_progress_start(fusion, clock, base)
    fusion._scan_topology.observe([("scan_motion_node", "/", bytes([1]) * 16)])
    monkeypatch.setattr(
        fusion,
        "_scan_topology_ready",
        MotionFusion._scan_topology_ready.__get__(fusion, MotionFusion),
    )
    monkeypatch.setattr(
        fusion,
        "get_publishers_info_by_topic",
        lambda _topic: [
            SimpleNamespace(
                node_name="scan_motion_node", node_namespace="/", endpoint_gid=bytes([2]) * 16
            )
        ],
    )
    fusion._try_pending()
    assert fusion._scan_epochs.is_retired("epoch-b")
    assert fusion._scan_epochs.active is None and fusion._deferred_scan.peek() is None
