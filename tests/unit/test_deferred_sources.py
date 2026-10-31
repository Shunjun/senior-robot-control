"""Future source samples retain their source and receipt identities."""

from senior_control.safety.motion_runtime import (
    DeferredSample,
    DeferredSourceQueue,
    SourceEpochFence,
)


def test_future_sample_waits_until_its_exact_source_stamp_without_restamping():
    queue = DeferredSourceQueue[str](max_pending=2)
    sample = DeferredSample(1_220_000_000, 1_218_000_000, 100.0, "wheel")
    assert queue.offer(sample)
    assert (
        queue.status(now_ros_ns=1_219_999_999, now_steady=100.02, max_age=0.8, pending_wait=0.25)
        == "waiting"
    )
    assert queue.peek() == sample
    assert (
        queue.status(now_ros_ns=1_220_000_000, now_steady=100.03, max_age=0.8, pending_wait=0.25)
        == "ready"
    )
    assert queue.consume() == sample
    assert (
        queue.status(now_ros_ns=1_220_000_000, now_steady=100.03, max_age=0.8, pending_wait=0.25)
        == "empty"
    )


def test_deferred_source_fails_closed_on_expiry_rollback_or_overflow():
    queue = DeferredSourceQueue[str](max_pending=2)
    first = DeferredSample(1_220_000_000, 1_218_000_000, 100.0, "first")
    second = DeferredSample(1_240_000_000, 1_219_000_000, 100.1, "second")
    assert queue.offer(first) and queue.offer(second)
    assert not queue.offer(DeferredSample(1_260_000_000, 1_220_000_000, 100.2, "overflow"))
    assert (
        queue.status(now_ros_ns=1_219_000_000, now_steady=100.251, max_age=0.8, pending_wait=0.25)
        == "expired"
    )
    assert (
        queue.status(now_ros_ns=1_217_000_000, now_steady=100.1, max_age=0.8, pending_wait=0.25)
        == "clock_rollback"
    )
    queue.clear()
    assert queue.offer(second)
    assert not queue.offer(first)


def test_deferred_source_rejects_invalid_receipts_and_already_stale_samples():
    queue = DeferredSourceQueue[str](max_pending=2)
    assert not queue.offer(DeferredSample(100, 0, 10.0, "missing-clock"))
    assert not queue.offer(DeferredSample(100, 99, float("nan"), "missing-steady"))
    assert queue.offer(DeferredSample(1_000_000_000, 1_000_000_000, 10.0, "old"))
    assert (
        queue.status(now_ros_ns=1_900_000_001, now_steady=10.1, max_age=0.8, pending_wait=0.25)
        == "expired"
    )


def test_future_source_cannot_outlive_first_ros_receipt_when_clock_accelerates():
    queue = DeferredSourceQueue[str](max_pending=1)
    assert queue.offer(DeferredSample(2_000_000_000, 1_000_000_000, 10.0, "early"))
    assert (
        queue.status(now_ros_ns=1_900_000_001, now_steady=10.1, max_age=0.8, pending_wait=0.25)
        == "expired"
    )


def test_replaced_deferred_epoch_cannot_revive_or_exhaustion_be_forgotten():
    fence = SourceEpochFence(max_retired=2)
    assert fence.accept("active")
    fence.retire_active()
    fence.retire_unadmitted("deferred")
    assert fence.active is None
    assert fence.is_retired("active") and fence.is_retired("deferred")
    assert not fence.accept("active") and not fence.accept("deferred")
    fence.retire_unadmitted("overflow")
    assert not fence.accept("fresh")
