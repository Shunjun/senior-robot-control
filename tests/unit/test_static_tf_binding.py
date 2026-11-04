from senior_control.vision.static_tf_binding import Edge, Receipt, StaticBinding


def receipt(gid="01", generation=1, sequence=1, translation=(0.0, 0.0, 0.0)):
    return Receipt(
        "boot",
        gid,
        generation,
        sequence,
        0,
        0.0,
        (Edge("base_link", "camera", 0, translation, (0.0, 0.0, 0.0, 1.0)),),
        relay_gid="aa",
    )


def test_zero_stamp_and_receipt_are_preserved_and_admission_is_separate():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.current.received_ros_ns == 0
    assert state.current.received_steady == 0
    assert not state.pair_valid(
        11,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )
    assert state.admit(10, 1.0)
    assert state.pair_valid(
        11,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )
    assert not state.pair_valid(
        9,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )


def test_replacement_retires_queued_owner_without_revoking_successor():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.observe(receipt("02", 2), current_gid="02", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    assert not state.observe(receipt(sequence=2), current_gid="02", current_relay_gid="aa")
    assert state.pair_valid(
        11,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )


def test_changed_edges_revoke_and_require_new_admission():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    assert state.observe(
        receipt(sequence=2, translation=(1.0, 0.0, 0.0)), current_gid="01", current_relay_gid="aa"
    )
    assert not state.pair_valid(
        11,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )


def test_loss_retires_old_owner_before_successor():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    state.revoke()
    assert state.observe(receipt("02", 2), current_gid="02", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    assert not state.observe(receipt(sequence=2), current_gid="02", current_relay_gid="aa")
    assert state.pair_valid(
        11,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )


def test_current_invalid_revokes_binding_even_with_same_sequence():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    invalid = Receipt("boot", "01", 1, 1, 0, 0.0, (), relay_gid="aa")
    assert not state.observe(invalid, current_gid="01", current_relay_gid="aa")
    assert not state.pair_valid(
        11,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )


def test_result_recheck_rejects_previous_lifetime():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    issued = state.generation
    assert state.observe(
        receipt(sequence=2, translation=(1.0, 0.0, 0.0)), current_gid="01", current_relay_gid="aa"
    )
    assert state.admit(10, 1.0)
    assert not state.pair_valid(
        11, 12, 1.1, 1.2, generation=issued, first_received_ros_ns=11, second_received_ros_ns=12
    )


def test_mutable_edge_storage_cannot_enter_binding():
    state = StaticBinding("base_link", "camera")
    bad = Receipt(
        "boot",
        "01",
        1,
        1,
        0,
        0.0,
        [Edge("base_link", "camera", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))],
        relay_gid="aa",
    )
    assert not state.observe(bad, current_gid="01", current_relay_gid="aa")


def test_re_admission_does_not_refresh_lifetime_start():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    assert state.admit(20, 2.0)
    assert state.pair_valid(
        11,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )


def test_repeat_observation_does_not_replace_first_receipt():
    state = StaticBinding("base_link", "camera")
    first = receipt()
    assert state.observe(first, current_gid="01", current_relay_gid="aa")
    later = Receipt("boot", "01", 1, 2, 100, 3.0, first.edges, relay_gid="aa")
    assert state.observe(later, current_gid="01", current_relay_gid="aa")
    assert state.current == later
    assert state.first_receipt == first


def test_unrelated_cycle_and_duplicate_child_are_rejected():
    state = StaticBinding("base_link", "camera")
    base = receipt()
    bad_cycle = (
        base.edges[0],
        Edge("a", "b", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        Edge("b", "a", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    assert not state.observe(
        Receipt("boot", "01", 1, 1, 0, 0.0, bad_cycle, relay_gid="aa"),
        current_gid="01",
        current_relay_gid="aa",
    )
    duplicate = (base.edges[0], Edge("other", "camera", 0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    assert not state.observe(
        Receipt("boot", "01", 1, 2, 0, 0.0, duplicate, relay_gid="aa"),
        current_gid="01",
        current_relay_gid="aa",
    )


def test_retired_source_gid_cannot_return_under_new_boot():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.observe(receipt("02", 2), current_gid="02", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    old_gid_new_boot = Receipt("newboot", "01", 1, 1, 0, 0.0, receipt().edges, relay_gid="aa")
    assert not state.observe(old_gid_new_boot, current_gid="02", current_relay_gid="aa")
    assert state.pair_valid(
        11,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )


def test_chain_composes_rotated_translation():
    from math import sqrt

    from senior_control.vision.static_tf_binding import body_from_camera

    edges = (
        Edge("base_link", "mount", 0, (1.0, 0.0, 0.0), (0.0, 0.0, sqrt(0.5), sqrt(0.5))),
        Edge("mount", "camera", 0, (1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
    )
    matrix = body_from_camera(edges, "base_link", "camera")
    assert abs(matrix[0][3] - 1.0) < 1e-12
    assert abs(matrix[1][3] - 1.0) < 1e-12
    assert abs(matrix[0][1] + 1.0) < 1e-12


def test_malformed_numbers_fail_closed():
    state = StaticBinding("base_link", "camera")
    invalid = Receipt(
        "boot",
        "01",
        1,
        1,
        0,
        0.0,
        (Edge("base_link", "camera", 0, ("bad", 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),),
        relay_gid="aa",
    )
    assert not state.observe(invalid, current_gid="01", current_relay_gid="aa")


def test_relay_restart_same_raw_requalifies_and_old_relay_is_ignored():
    state = StaticBinding("base_link", "camera")
    edge = receipt().edges
    first = Receipt("native-boot-1", "01", 1, 1, 0, 0.0, edge, relay_gid="a1")
    second = Receipt("native-boot-2", "01", 1, 1, 5, 0.5, edge, relay_gid="a2")
    assert state.observe(first, current_gid="01", current_relay_gid="a1")
    assert state.admit(10, 1.0)
    assert state.observe(second, current_gid="01", current_relay_gid="a2")
    assert not state.pair_valid(
        11,
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )
    assert state.admit(20, 2.0)
    assert not state.observe(first, current_gid="01", current_relay_gid="a2")
    assert state.pair_valid(
        21,
        22,
        2.1,
        2.2,
        generation=state.generation,
        first_received_ros_ns=21,
        second_received_ros_ns=22,
    )


def test_result_must_supply_issued_generation():
    import pytest

    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    with pytest.raises(TypeError):
        state.pair_valid(11, 12, 1.1, 1.2)


def test_malformed_lifetime_metadata_fails_closed():
    state = StaticBinding("base_link", "camera")
    invalid = Receipt("boot", "01", "bad", 1, 0, 0.0, receipt().edges, relay_gid="aa")
    assert not state.observe(invalid, current_gid="01", current_relay_gid="aa")


def test_pair_rejects_receipts_before_admission_and_equal_stamps():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    token = state.generation
    assert not state.pair_valid(
        11, 12, 1.1, 1.2, generation=token, first_received_ros_ns=9, second_received_ros_ns=12
    )
    assert not state.pair_valid(
        11, 11, 1.1, 1.2, generation=token, first_received_ros_ns=11, second_received_ros_ns=12
    )


def test_bad_pair_clock_values_fail_closed():
    state = StaticBinding("base_link", "camera")
    assert state.observe(receipt(), current_gid="01", current_relay_gid="aa")
    assert state.admit(10, 1.0)
    assert not state.pair_valid(
        "bad",
        12,
        1.1,
        1.2,
        generation=state.generation,
        first_received_ros_ns=11,
        second_received_ros_ns=12,
    )


def test_accepted_quaternion_roundoff_is_normalized_only_for_composition():
    from math import sqrt

    from senior_control.vision.static_tf_binding import body_from_camera

    q = sqrt((1.0 + 5e-7) / 2)
    edge = Edge("base_link", "camera", 0, (0.0, 0.0, 0.0), (0.0, 0.0, q, q))
    matrix = body_from_camera((edge,), "base_link", "camera")
    assert abs(matrix[0][0]) < 1e-12
    assert abs(matrix[1][0] - 1.0) < 1e-12
    assert edge.quaternion == (0.0, 0.0, q, q)
