from types import SimpleNamespace as NS

from senior_control.vision.static_tf_binding import StaticBinding
from senior_control.vision.static_tf_wire import observe_verified_tf


def ns(value):
    return NS(sec=value // 1_000_000_000, nanosec=value % 1_000_000_000)


def message(sequence, *, admitted=False):
    edge = NS(
        parent_frame="base_link",
        child_frame="camera",
        stamp=ns(0),
        translation=(0.0, 0.0, 0.0),
        rotation_xyzw=(0.0, 0.0, 0.0, 1.0),
    )
    return NS(
        bridge_boot_id="boot",
        bridge_gid="02",
        raw_gid="01",
        generation=1,
        sequence=sequence,
        first_sequence=1,
        first_received_ros=ns(0),
        first_received_steady=0.0,
        received_ros=ns(0 if sequence == 1 else sequence * 100),
        received_steady=0.0 if sequence == 1 else sequence * 0.1,
        valid=True,
        admitted=admitted,
        admitted_from_ros=ns(200) if admitted else ns(0),
        admitted_from_steady=0.2 if admitted else 0.0,
        reason="",
        edges=(edge,),
    )


def test_late_first_receipt_admission_status_keeps_latest_sequence():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(2), current_relay_gid="02")
    assert binding.first_receipt.received_ros_ns == 0
    assert binding.current.sequence == 2
    assert observe_verified_tf(binding, message(1, admitted=True), current_relay_gid="02")
    assert binding.current.sequence == 2
    assert binding.admitted_from == (200, 0.2)


def test_retired_source_invalid_status_does_not_revoke_successor():
    from dataclasses import replace

    from senior_control.vision.static_tf_binding import Edge, Receipt

    binding = StaticBinding("base_link", "camera")
    edge = Edge("base_link", "camera", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    first = Receipt("boot", "01", 1, 1, 0, 0.0, (edge,), "02")
    assert binding.observe(first, current_gid="01", current_relay_gid="02")
    newer = replace(first, source_gid="03", generation=2)
    assert binding.observe(newer, current_gid="03", current_relay_gid="02")
    assert binding.admit(200, 0.2)
    old_invalid = message(1)
    old_invalid.valid = False
    assert not observe_verified_tf(binding, old_invalid, current_relay_gid="02")
    assert binding.current == newer
    assert binding.admitted_from == (200, 0.2)


def test_malformed_queued_retired_source_does_not_revoke_successor():
    from dataclasses import replace

    from senior_control.vision.static_tf_binding import Edge, Receipt

    binding = StaticBinding("base_link", "camera")
    edge = Edge("base_link", "camera", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    first = Receipt("boot", "01", 1, 1, 0, 0.0, (edge,), "02")
    assert binding.observe(first, current_gid="01", current_relay_gid="02")
    newer = replace(first, source_gid="03", generation=2)
    assert binding.observe(newer, current_gid="03", current_relay_gid="02")
    assert binding.admit(200, 0.2)
    old = message(1)
    old.edges[0].stamp = NS(sec=-1, nanosec=0)
    assert not observe_verified_tf(binding, old, current_relay_gid="02")
    assert binding.current == newer


def test_conflicting_admission_floor_revokes_current_lifetime():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(1, admitted=True), current_relay_gid="02")
    conflicting = message(1, admitted=True)
    conflicting.admitted_from_ros = ns(300)
    conflicting.admitted_from_steady = 0.3
    assert not observe_verified_tf(binding, conflicting, current_relay_gid="02")
    assert binding.current is None


def test_same_current_sequence_with_changed_original_receipt_revokes():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(2, admitted=True), current_relay_gid="02")
    conflict = message(2, admitted=True)
    conflict.received_ros = ns(250)
    conflict.received_steady = 0.25
    assert not observe_verified_tf(binding, conflict, current_relay_gid="02")
    assert binding.current is None
