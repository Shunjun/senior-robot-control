"""Raw camera admission binds each bridge/source lifetime and first receipts."""

from dataclasses import replace


def raw_header(**changes):
    from senior_control.vision.raw_capture import RawCaptureHeader

    base = RawCaptureHeader(
        stamp_ns=1_000_000_000, received_ros_ns=1_000_000_001, received_steady=10.0,
        frame_id="camera_optical_frame", raw_gid="01" * 24, bridge_gid="02" * 24,
        source_epoch="epoch-a", bridge_boot_id="boot-a", capture_generation=1,
        sequence=1, valid=True, reason="",
    )
    return replace(base, **changes)


def test_bridge_and_raw_restart_retire_old_data_without_restamping():
    from senior_control.vision.raw_capture import RawCaptureAdmission

    admission = RawCaptureAdmission(max_age_s=0.3)
    old = raw_header()
    assert admission.offer(old, bridge_gid="02" * 24, now_ros_ns=1_100_000_000, now_steady=10.1)
    assert admission.latest is old
    changed = raw_header(stamp_ns=1_010_000_000, received_ros_ns=1_020_000_000,
                         bridge_gid="03" * 24, raw_gid="04" * 24,
                         source_epoch="epoch-b", bridge_boot_id="boot-b", sequence=1)
    assert admission.offer(changed, bridge_gid="03" * 24, now_ros_ns=1_100_000_000,
                           now_steady=10.1)
    generation = admission.generation
    assert not admission.offer(old, bridge_gid="03" * 24, now_ros_ns=1_100_000_000,
                               now_steady=10.1)
    assert admission.generation == generation and admission.latest is changed
    replay = replace(old, bridge_gid="03" * 24, sequence=99)
    assert not admission.offer(replay, bridge_gid="03" * 24, now_ros_ns=1_100_000_000,
                               now_steady=10.1)
    assert admission.latest is changed


def test_failed_frame_revokes_and_old_or_same_sequence_cannot_revive():
    from senior_control.vision.raw_capture import RawCaptureAdmission

    admission = RawCaptureAdmission(max_age_s=0.3)
    good = raw_header()
    assert admission.offer(good, bridge_gid=good.bridge_gid, now_ros_ns=1_100_000_000,
                           now_steady=10.1)
    failed = replace(good, valid=False, sequence=2, reason="raw_gid_mismatch")
    assert not admission.offer(failed, bridge_gid=good.bridge_gid, now_ros_ns=1_100_000_000,
                               now_steady=10.1)
    assert admission.latest is None
    assert not admission.offer(replace(good, sequence=2), bridge_gid=good.bridge_gid,
                               now_ros_ns=1_100_000_000, now_steady=10.1)
    fresh = replace(good, sequence=3)
    assert admission.offer(fresh, bridge_gid=good.bridge_gid, now_ros_ns=1_100_000_000,
                           now_steady=10.1)


def test_source_conflict_invalidates_epoch_and_original_first_receipts_expire():
    from senior_control.vision.raw_capture import RawCaptureAdmission

    admission = RawCaptureAdmission(max_age_s=0.3)
    good = raw_header()
    assert admission.offer(good, bridge_gid=good.bridge_gid, now_ros_ns=1_100_000_000,
                           now_steady=10.1)
    conflict = replace(good, raw_gid="04" * 24, sequence=2)
    assert not admission.offer(conflict, bridge_gid=good.bridge_gid, now_ros_ns=1_100_000_000,
                               now_steady=10.1)
    assert admission.latest is None
    assert not admission.offer(replace(good, sequence=3), bridge_gid=good.bridge_gid,
                               now_ros_ns=1_100_000_000, now_steady=10.1)
    newer = replace(good, source_epoch="epoch-b", sequence=1)
    assert not admission.offer(newer, bridge_gid=good.bridge_gid, now_ros_ns=1_100_000_000,
                               now_steady=10.31)
    assert admission.latest is None


def test_graph_disappearing_revokes_current_source_without_old_queue_revival():
    from senior_control.vision.raw_capture import RawCaptureAdmission

    admission = RawCaptureAdmission(max_age_s=0.3)
    good = raw_header()
    assert admission.offer(good, bridge_gid=good.bridge_gid, now_ros_ns=1_100_000_000,
                           now_steady=10.1)
    assert not admission.observe_bridge("")
    assert admission.latest is None
    assert not admission.offer(good, bridge_gid=good.bridge_gid, now_ros_ns=1_100_000_000,
                               now_steady=10.1)


def test_same_stamp_cannot_pair_across_bridge_boot_or_global_capture_generation():
    from senior_control.vision.raw_capture import compatible_captures

    first = raw_header(bridge_boot_id="boot-a", capture_generation=3)
    aligned = replace(first, raw_gid="09" * 24, bridge_gid="08" * 24)
    assert compatible_captures(first, aligned)
    assert not compatible_captures(first, replace(aligned, bridge_boot_id="boot-b"))
    assert not compatible_captures(first, replace(aligned, capture_generation=4))
    assert not compatible_captures(first, replace(aligned, stamp_ns=first.stamp_ns + 1))


def test_delayed_old_global_generation_cannot_reopen_after_replacement():
    from senior_control.vision.raw_capture import RawCaptureAdmission

    admission = RawCaptureAdmission(max_age_s=0.3)
    old = raw_header(bridge_boot_id="boot-a", capture_generation=1)
    assert admission.offer(old, bridge_gid=old.bridge_gid, now_ros_ns=1_100_000_000,
                           now_steady=10.1)
    current = replace(old, capture_generation=2, sequence=2)
    assert admission.offer(current, bridge_gid=old.bridge_gid, now_ros_ns=1_100_000_000,
                           now_steady=10.1)
    delayed = replace(old, sequence=3)
    assert not admission.offer(delayed, bridge_gid=old.bridge_gid, now_ros_ns=1_100_000_000,
                               now_steady=10.1)
    assert admission.latest is current


def test_capture_header_decoding_preserves_original_metadata_and_rejects_bad_time():
    from types import SimpleNamespace

    import pytest

    from senior_control.vision.raw_capture import capture_from_message

    original = raw_header()
    wire = SimpleNamespace(
        stamp=SimpleNamespace(sec=1, nanosec=0),
        received_ros=SimpleNamespace(sec=1, nanosec=1),
        received_steady=original.received_steady,
        frame_id=original.frame_id, raw_gid=original.raw_gid, bridge_gid=original.bridge_gid,
        source_epoch=original.source_epoch, bridge_boot_id=original.bridge_boot_id,
        capture_generation=original.capture_generation, sequence=original.sequence,
        valid=original.valid, reason=original.reason,
    )
    assert capture_from_message(wire) == original
    wire.received_ros.nanosec = 1_000_000_000
    with pytest.raises(ValueError):
        capture_from_message(wire)


def test_group_revokes_all_streams_on_new_generation_and_rejects_old_packets():
    from senior_control.vision.raw_capture import RawCaptureGroup

    group = RawCaptureGroup(("depth", "info"), max_age_s=0.3)
    depth = raw_header(source_epoch="depth", bridge_gid="02" * 24)
    info = raw_header(source_epoch="info", bridge_gid="03" * 24)
    args = {"now_ros_ns": 1_100_000_000, "now_steady": 10.1}
    assert group.offer("depth", depth, bridge_gid=depth.bridge_gid, **args)
    assert group.offer("info", info, bridge_gid=info.bridge_gid, **args)
    assert group.paired("depth", "info")
    invalid = replace(info, valid=False, sequence=2, capture_generation=2, reason="source_changed")
    assert not group.offer("info", invalid, bridge_gid=info.bridge_gid, **args)
    assert not group.paired("depth", "info")
    delayed = replace(depth, sequence=2)
    assert not group.offer("depth", delayed, bridge_gid=depth.bridge_gid, **args)
    assert not group.paired("depth", "info")
    new_depth = replace(depth, sequence=3, capture_generation=2)
    new_info = replace(info, sequence=3, capture_generation=2)
    assert group.offer("depth", new_depth, bridge_gid=depth.bridge_gid, **args)
    assert group.offer("info", new_info, bridge_gid=info.bridge_gid, **args)
    assert group.paired("depth", "info")


def test_group_rejects_same_stamp_from_different_bridge_boots():
    from senior_control.vision.raw_capture import RawCaptureGroup

    group = RawCaptureGroup(("depth", "info"), max_age_s=0.3)
    first = raw_header(source_epoch="depth")
    other = raw_header(source_epoch="info", bridge_gid="03" * 24, bridge_boot_id="other")
    args = {"now_ros_ns": 1_100_000_000, "now_steady": 10.1}
    assert group.offer("depth", first, bridge_gid=first.bridge_gid, **args)
    assert not group.offer("info", other, bridge_gid=other.bridge_gid, **args)
    assert not group.paired("depth", "info")


def test_group_recovers_bridge_restart_without_reviving_retired_boot():
    from senior_control.vision.raw_capture import RawCaptureGroup

    group = RawCaptureGroup(("depth", "info"), max_age_s=0.3)
    first = raw_header(source_epoch="depth")
    info = raw_header(source_epoch="info", bridge_gid="03" * 24)
    args = {"now_ros_ns": 1_100_000_000, "now_steady": 10.1}
    assert group.offer("depth", first, bridge_gid=first.bridge_gid, **args)
    assert group.offer("info", info, bridge_gid=info.bridge_gid, **args)
    replacement = replace(first, source_epoch="new-depth", bridge_boot_id="boot-b",
                          bridge_gid="04" * 24)
    assert group.offer("depth", replacement, bridge_gid=replacement.bridge_gid, **args)
    assert not group.paired("depth", "info")
    assert not group.offer("info", replace(info, sequence=2), bridge_gid=info.bridge_gid, **args)
    new_info = replace(info, source_epoch="new-info", bridge_boot_id="boot-b",
                       bridge_gid="05" * 24)
    assert group.offer("info", new_info, bridge_gid=new_info.bridge_gid, **args)
    assert group.paired("depth", "info")


def test_group_graph_loss_revokes_all_without_waiting_for_another_message():
    from senior_control.vision.raw_capture import RawCaptureGroup

    group = RawCaptureGroup(("depth", "info"), max_age_s=0.3)
    depth = raw_header(source_epoch="depth")
    info = raw_header(source_epoch="info", bridge_gid="03" * 24)
    args = {"now_ros_ns": 1_100_000_000, "now_steady": 10.1}
    assert group.offer("depth", depth, bridge_gid=depth.bridge_gid, **args)
    assert group.offer("info", info, bridge_gid=info.bridge_gid, **args)
    assert group.paired("depth", "info")
    assert not group.observe_bridge("info", "")
    assert not group.paired("depth", "info")
    assert not group.offer("info", info, bridge_gid=info.bridge_gid, **args)


def test_scan_profile_can_select_bounded_longer_receipt_age_without_changing_default():
    import pytest

    from senior_control.vision.raw_capture import RawCaptureAdmission

    with pytest.raises(ValueError):
        RawCaptureAdmission(max_age_s=3.0)
    admission = RawCaptureAdmission(max_age_s=3.0, hard_limit_s=5.0)
    good = raw_header()
    assert admission.offer(good, bridge_gid=good.bridge_gid, now_ros_ns=3_000_000_000,
                           now_steady=12.0)
    assert not admission.offer(replace(good, sequence=2), bridge_gid=good.bridge_gid,
                               now_ros_ns=4_100_000_000, now_steady=13.1)
