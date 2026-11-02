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


def test_invalid_generation_cannot_be_revived_by_delayed_valid_status():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(1, admitted=True), current_relay_gid="02")
    invalid = message(1)
    invalid.valid = False
    invalid.generation = 3
    assert not observe_verified_tf(binding, invalid, current_relay_gid="02")
    for generation in (2, 3):
        delayed = message(1, admitted=True)
        delayed.generation = generation
        assert not observe_verified_tf(binding, delayed, current_relay_gid="02")
        assert binding.current is None
    fresh = message(1, admitted=True)
    fresh.generation = 4
    assert observe_verified_tf(binding, fresh, current_relay_gid="02")
    assert not observe_verified_tf(binding, invalid, current_relay_gid="02")
    assert binding.current is not None and binding.current.generation == 4


def test_relay_restart_can_recover_after_invalid_frontier():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(1), current_relay_gid="02")
    invalid = message(1)
    invalid.valid = False
    invalid.generation = 3
    assert not observe_verified_tf(binding, invalid, current_relay_gid="02")
    replacement = message(1, admitted=True)
    replacement.bridge_gid = "04"
    replacement.bridge_boot_id = "newboot"
    assert observe_verified_tf(binding, replacement, current_relay_gid="04")
    assert binding.current is not None and binding.current.generation == 1


def test_foreign_invalid_cannot_advance_authenticated_frontier():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(1, admitted=True), current_relay_gid="02")
    foreign = message(1)
    foreign.valid = False
    foreign.bridge_gid = "foreign"
    foreign.bridge_boot_id = "foreignboot"
    foreign.generation = 100
    assert not observe_verified_tf(binding, foreign, current_relay_gid="02")
    assert binding.current is not None and binding.current.generation == 1
    valid = message(1, admitted=True)
    valid.generation = 2
    assert observe_verified_tf(binding, valid, current_relay_gid="02")
    assert binding.current is not None and binding.current.generation == 2


def test_malformed_stale_status_cannot_revoke_current_generation():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(1), current_relay_gid="02")
    invalid = message(1)
    invalid.valid = False
    invalid.generation = 3
    assert not observe_verified_tf(binding, invalid, current_relay_gid="02")
    fresh = message(1, admitted=True)
    fresh.generation = 4
    assert observe_verified_tf(binding, fresh, current_relay_gid="02")
    stale = message(1)
    stale.generation = 2
    stale.edges[0].stamp = NS(sec=-1, nanosec=0)
    assert not observe_verified_tf(binding, stale, current_relay_gid="02")
    assert binding.current is not None and binding.current.generation == 4


def test_invalid_after_raw_replacement_advances_relay_frontier():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(1, admitted=True), current_relay_gid="02")
    changed = message(1)
    changed.valid = False
    assert not observe_verified_tf(binding, changed, current_relay_gid="02")
    invalid_b = message(1)
    invalid_b.valid = False
    invalid_b.raw_gid = "03"
    invalid_b.generation = 3
    assert not observe_verified_tf(binding, invalid_b, current_relay_gid="02")
    delayed = message(1, admitted=True)
    delayed.generation = 2
    assert not observe_verified_tf(binding, delayed, current_relay_gid="02")
    assert binding.current is None


def test_first_authenticated_invalid_establishes_relay_frontier():
    binding = StaticBinding("base_link", "camera")
    invalid = message(1)
    invalid.valid = False
    invalid.raw_gid = ""
    invalid.generation = 3
    assert not observe_verified_tf(binding, invalid, current_relay_gid="02")
    delayed = message(1, admitted=True)
    delayed.generation = 2
    assert not observe_verified_tf(binding, delayed, current_relay_gid="02")
    assert binding.current is None


def test_new_relay_invalid_revokes_old_owner_and_requalifies():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(1, admitted=True), current_relay_gid="02")
    invalid = message(1)
    invalid.bridge_gid = "04"
    invalid.bridge_boot_id = "newboot"
    invalid.raw_gid = ""
    invalid.valid = False
    invalid.generation = 3
    assert not observe_verified_tf(binding, invalid, current_relay_gid="04")
    assert binding.current is None
    older = message(1, admitted=True)
    older.bridge_gid = "04"
    older.bridge_boot_id = "newboot"
    older.generation = 2
    assert not observe_verified_tf(binding, older, current_relay_gid="04")
    older.generation = 4
    assert observe_verified_tf(binding, older, current_relay_gid="04")
    old = message(1)
    old.valid = False
    old.generation = 100
    assert not observe_verified_tf(binding, old, current_relay_gid="04")
    assert binding.current is not None and binding.current.generation == 4


def test_invalid_wrong_boot_for_current_relay_cannot_advance_frontier():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(1), current_relay_gid="02")
    invalid = message(1)
    invalid.bridge_boot_id = "otherboot"
    invalid.valid = False
    invalid.generation = 100
    assert not observe_verified_tf(binding, invalid, current_relay_gid="02")
    newer = message(1)
    newer.generation = 2
    assert observe_verified_tf(binding, newer, current_relay_gid="02")


def test_external_revoke_cannot_replace_known_boot_for_same_relay():
    binding = StaticBinding("base_link", "camera")
    assert observe_verified_tf(binding, message(1), current_relay_gid="02")
    binding.revoke()
    forged = message(1)
    forged.valid = False
    forged.bridge_boot_id = "otherboot"
    forged.generation = 100
    assert not observe_verified_tf(binding, forged, current_relay_gid="02")
    valid = message(1, admitted=True)
    valid.generation = 2
    assert observe_verified_tf(binding, valid, current_relay_gid="02")
    assert binding.current is not None and binding.current.boot == "boot"


def test_numpy_transform_scalars_are_normalized_to_native_float():
    import numpy as np

    binding = StaticBinding("base_link", "camera")
    value = message(1, admitted=True)
    value.edges[0].translation = np.array([0.1, 0.2, 0.3], dtype=np.float64)
    value.edges[0].rotation_xyzw = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    assert observe_verified_tf(binding, value, current_relay_gid="02")
    assert binding.current is not None
    edge = binding.current.edges[0]
    assert edge.translation == (0.1, 0.2, 0.3)
    assert all(type(item) is float for item in (*edge.translation, *edge.quaternion))


def test_transform_rejects_bool_strings_and_nonfinite_values():
    import numpy as np

    for invalid in (True, np.bool_(True), "0.0", float("nan"), float("inf")):
        binding = StaticBinding("base_link", "camera")
        value = message(1, admitted=True)
        value.edges[0].translation = (invalid, 0.0, 0.0)
        assert not observe_verified_tf(binding, value, current_relay_gid="02"), repr(invalid)
        assert binding.current is None
