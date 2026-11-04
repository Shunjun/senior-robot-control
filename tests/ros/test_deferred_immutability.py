"""The future FIFO retains the original values, even if a caller reuses a message."""

from test_future_source_clock import _scan
from test_future_source_clock import active_fusion as active_fusion


def test_future_scan_is_an_immutable_snapshot_of_original_message(active_fusion):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    message = _scan(base, stamp)
    fusion._on_scan(message)
    item = fusion._deferred_scan.peek()
    assert item is not None and not emitted

    # ROS generated nested objects are mutable; callers must not change a ticket.
    message.header.stamp.sec += 1
    message.interval_start.nanosec += 1
    message.body_delta.x = 0.4
    message.local_pose.y = 0.3
    message.covariance[0] = 0.09
    message.startup_reference_stamp.sec += 1

    original = item.value
    assert original.header.stamp.sec * 1_000_000_000 + original.header.stamp.nanosec == stamp
    assert original.interval_start.sec * 1_000_000_000 + original.interval_start.nanosec == base
    assert original.body_delta.x == 0.0
    assert original.local_pose.y == 0.0
    assert original.covariance[0] == 0.001
    assert original.startup_reference_stamp.sec == 0


def test_retired_source_with_oversized_frame_cannot_discard_new_future_epoch(active_fusion):
    fusion, clock, base, emitted = active_fusion
    stamp = base + 100_000_000
    clock["now"] = stamp - 2_000_000
    fusion._on_scan(_scan(base, stamp, epoch="new-epoch"))
    assert fusion._scan_epochs.is_retired("stable-independent-scan")
    pending = fusion._deferred_scan.peek()
    assert pending is not None and pending.value.source_epoch == "new-epoch"
    count = len(emitted)

    old = _scan(base, stamp + 1, valid=False)
    old.header.frame_id = "x" * 129
    fusion._on_scan(old)
    assert fusion._deferred_scan.peek() is pending
    assert len(emitted) == count
