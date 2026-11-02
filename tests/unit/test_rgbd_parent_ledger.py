from dataclasses import replace

from senior_control.vision.raw_capture import RawCaptureHeader
from senior_control.vision.rgbd_parent_ledger import CaptureLedger


def header(seq):
    stamp = 1_000_000_000 + seq * 100_000_000
    return RawCaptureHeader(
        stamp, stamp, 1.0 + seq * 0.1, "optical", "01", "02", "source", "boot", 1, seq, True, ""
    )


def bundle(seq):
    return (header(seq),) * 4


def test_new_frame_keeps_live_earlier_ticket_and_original_deadline():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    first = ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    second = ledger.offer(
        bundle(2),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    assert first is not None and second is not None
    assert ledger.current(first, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2)
    assert not ledger.current(first, binding_generation=4, now_ros_ns=1_400_000_001, now_steady=1.4)


def test_source_replacement_revokes_old_and_retired_replay_does_not_revoke_new():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    first = ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    new = tuple(
        replace(h, raw_gid="03", source_epoch="new", capture_generation=2) for h in bundle(2)
    )
    successor = ledger.offer(
        new, bridges=("02",) * 4, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2
    )
    assert first is not None and successor is not None
    assert not ledger.current(first, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2)
    assert (
        ledger.offer(
            bundle(1),
            bridges=("02",) * 4,
            binding_generation=4,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        is None
    )
    assert ledger.current(successor, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2)


def test_mixed_retired_stream_is_ignored_without_revoking_successor():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    new = tuple(
        replace(h, raw_gid="03", source_epoch="new", capture_generation=2) for h in bundle(2)
    )
    successor = ledger.offer(
        new, bridges=("02",) * 4, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2
    )
    mixed = (bundle(1)[0], *new[1:])
    assert (
        ledger.offer(
            mixed,
            bridges=("02",) * 4,
            binding_generation=4,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        is None
    )
    assert ledger.current(successor, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2)


def test_current_malformed_header_revokes_without_raising():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    first = ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    malformed = tuple(replace(h, source_epoch=[]) for h in bundle(2))
    assert (
        ledger.offer(
            malformed,
            bridges=("02",) * 4,
            binding_generation=4,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        is None
    )
    assert not ledger.current(first, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2)


def test_clock_rollback_revokes_ticket_even_on_replayed_capture():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    first = ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    assert (
        ledger.offer(
            bundle(1),
            bridges=("02",) * 4,
            binding_generation=4,
            now_ros_ns=1_150_000_000,
            now_steady=1.21,
        )
        is None
    )
    assert not ledger.current(
        first, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.22
    )


def test_capacity_eviction_revokes_only_old_ticket():
    ledger = CaptureLedger(capacity=1, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    first = ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    second = ledger.offer(
        bundle(2),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    assert second is not None and second.generation == first.generation
    assert not ledger.current(first, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2)
    assert ledger.current(second, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2)


def test_tf_generation_change_revokes_and_cannot_be_rewound():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    first = ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    assert not ledger.current(first, binding_generation=5, now_ros_ns=1_200_000_000, now_steady=1.2)
    assert not ledger.current(first, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2)


def test_pair_rechecks_both_tickets_and_same_binding_lifetime():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    first = ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    second = ledger.offer(
        bundle(2),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    assert ledger.pair_current(
        first, second, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2
    )
    assert not ledger.pair_current(
        first, second, binding_generation=5, now_ros_ns=1_200_000_000, now_steady=1.2
    )
    assert not ledger.pair_current(
        first, second, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2
    )


def test_bounded_ticket_eviction_supports_sustained_stream_without_epoch_churn():
    ledger = CaptureLedger(capacity=2, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    issued = []
    for seq in range(1, 3001):
        h = header(seq)
        ticket = ledger.offer(
            (h,) * 4,
            bridges=("02",) * 4,
            binding_generation=4,
            now_ros_ns=h.stamp_ns,
            now_steady=h.received_steady,
        )
        assert ticket is not None
        issued.append(ticket)
        assert ticket.generation == 1
    last = header(3000)
    assert not ledger.current(
        issued[0], binding_generation=4, now_ros_ns=last.stamp_ns, now_steady=last.received_steady
    )
    assert ledger.current(
        issued[-1], binding_generation=4, now_ros_ns=last.stamp_ns, now_steady=last.received_steady
    )
    next_capture = header(3001)
    new_source = replace(next_capture, raw_gid="03", source_epoch="new", capture_generation=2)
    successor = ledger.offer(
        (new_source,) * 4,
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=next_capture.stamp_ns,
        now_steady=next_capture.received_steady,
    )
    assert successor is not None
    assert (
        ledger.offer(
            (last,) * 4,
            bridges=("02",) * 4,
            binding_generation=4,
            now_ros_ns=next_capture.stamp_ns,
            now_steady=next_capture.received_steady,
        )
        is None
    )
    assert ledger.current(
        successor,
        binding_generation=4,
        now_ros_ns=next_capture.stamp_ns,
        now_steady=next_capture.received_steady,
    )


def test_restart_capture_floor_rejects_old_original_receipts():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1_150_000_000, fresh_after_steady=1.15)
    assert (
        ledger.offer(
            bundle(1),
            bridges=("02",) * 4,
            binding_generation=4,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        is None
    )
    assert (
        ledger.offer(
            bundle(2),
            bridges=("02",) * 4,
            binding_generation=4,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        is not None
    )


def test_retired_raw_gid_cannot_reappear_under_new_generation():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    assert ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    new = tuple(
        replace(h, raw_gid="03", source_epoch="new", capture_generation=2) for h in bundle(2)
    )
    successor = ledger.offer(
        new, bridges=("02",) * 4, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2
    )
    forged_old = tuple(
        replace(h, raw_gid="01", source_epoch="other", capture_generation=3) for h in bundle(3)
    )
    assert not ledger.offer(
        forged_old,
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_300_000_000,
        now_steady=1.3,
    )
    assert ledger.current(successor, binding_generation=4, now_ros_ns=1_300_000_000, now_steady=1.3)


def test_bridge_restart_can_keep_same_raw_source_with_new_boot_and_capture():
    ledger = CaptureLedger(capacity=4, fresh_after_ros_ns=1, fresh_after_steady=0.1)
    assert ledger.offer(
        bundle(1),
        bridges=("02",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    replacement = tuple(replace(h, bridge_gid="04", bridge_boot_id="newboot") for h in bundle(2))
    successor = ledger.offer(
        replacement,
        bridges=("04",) * 4,
        binding_generation=4,
        now_ros_ns=1_200_000_000,
        now_steady=1.2,
    )
    assert successor is not None
    assert (
        ledger.offer(
            bundle(1),
            bridges=("04",) * 4,
            binding_generation=4,
            now_ros_ns=1_200_000_000,
            now_steady=1.2,
        )
        is None
    )
    assert ledger.current(successor, binding_generation=4, now_ros_ns=1_200_000_000, now_steady=1.2)
