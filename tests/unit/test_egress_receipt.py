"""Consumer admission for native, per-message publisher ownership."""

from dataclasses import replace


def test_retired_owner_packet_does_not_revoke_successor():
    from senior_control.vision.egress_receipt import EgressAdmission, EgressHeader

    admission = EgressAdmission(max_age_s=0.3)
    old = EgressHeader(relay_gid="01" * 24, relay_boot="relay-a", owner_gid="11" * 24,
                       owner_epoch="owner-a", owner_generation=1, owner_sequence=1,
                       relay_sequence=1,
                       valid=True, reason="")
    successor = replace(old, owner_gid="22" * 24, owner_epoch="owner-b", owner_generation=2,
                        relay_sequence=2)
    assert admission.offer(old, relay_gid=old.relay_gid)
    assert admission.offer(successor, relay_gid=successor.relay_gid)
    generation = admission.generation
    assert not admission.offer(replace(old, owner_sequence=99, relay_sequence=3),
                               relay_gid=old.relay_gid)
    assert admission.generation == generation
    assert admission.latest == successor
    invalid = replace(successor, owner_sequence=2, relay_sequence=4, valid=False,
                      reason="depth_invalid")
    assert not admission.offer(invalid, relay_gid=successor.relay_gid)
    assert admission.latest is None
    assert admission.generation > generation


def test_current_invalid_is_idempotent_and_a_later_valid_message_recovers():
    from senior_control.vision.egress_receipt import EgressAdmission, EgressHeader

    admission = EgressAdmission(max_age_s=0.3)
    good = EgressHeader(relay_gid="01" * 24, relay_boot="relay-a", owner_gid="11" * 24,
                        owner_epoch="owner-a", owner_generation=1, owner_sequence=1,
                        relay_sequence=1, valid=True, reason="")
    assert admission.offer(good, relay_gid=good.relay_gid)
    failure = replace(good, owner_sequence=2, relay_sequence=2, valid=False, reason="invalid")
    assert not admission.offer(failure, relay_gid=good.relay_gid)
    generation = admission.generation
    assert not admission.offer(replace(failure, owner_sequence=3, relay_sequence=3),
                               relay_gid=good.relay_gid)
    assert admission.generation == generation
    recovered = replace(good, owner_sequence=4, relay_sequence=4)
    assert admission.offer(recovered, relay_gid=good.relay_gid)
    assert admission.latest == recovered


def test_wrong_relay_gid_and_old_generation_cannot_replace_current_owner():
    from senior_control.vision.egress_receipt import EgressAdmission, EgressHeader

    admission = EgressAdmission(max_age_s=0.3)
    good = EgressHeader(relay_gid="01" * 24, relay_boot="relay-a", owner_gid="11" * 24,
                        owner_epoch="owner-a", owner_generation=2, owner_sequence=1,
                        relay_sequence=1, valid=True, reason="")
    assert admission.offer(good, relay_gid=good.relay_gid)
    generation = admission.generation
    assert not admission.offer(replace(good, relay_gid="02" * 24, relay_sequence=2),
                               relay_gid=good.relay_gid)
    assert not admission.offer(replace(good, owner_generation=1, relay_sequence=3),
                               relay_gid=good.relay_gid)
    assert admission.latest == good
    assert admission.generation == generation


def test_topology_loss_invalidates_once_and_new_owner_can_recover():
    from senior_control.vision.egress_receipt import EgressAdmission, EgressHeader

    admission = EgressAdmission(max_age_s=0.3)
    good = EgressHeader(relay_gid="01" * 24, relay_boot="relay-a", owner_gid="11" * 24,
                        owner_epoch="owner-a", owner_generation=1, owner_sequence=1,
                        relay_sequence=1, valid=True, reason="")
    assert admission.offer(good, relay_gid=good.relay_gid)
    lost = replace(good, owner_generation=2, owner_sequence=0, relay_sequence=2,
                   valid=False, owner_lost=True, reason="owner_lost")
    assert not admission.offer(lost, relay_gid=good.relay_gid)
    assert admission.latest is None
    generation = admission.generation
    assert not admission.offer(replace(lost, relay_sequence=3), relay_gid=good.relay_gid)
    assert admission.generation == generation
    new = replace(good, owner_gid="22" * 24, owner_epoch="owner-b", owner_generation=3,
                  relay_sequence=4)
    assert admission.offer(new, relay_gid=good.relay_gid)
    assert admission.latest == new


def test_wire_owner_header_conversion_preserves_topology_failure_fields():
    from types import SimpleNamespace

    from senior_control.vision.egress_receipt import EgressHeader, egress_from_message

    wire = SimpleNamespace(
        relay_gid="01" * 24, relay_boot="boot", relay_sequence=4,
        owner_gid="02" * 24, owner_epoch="epoch", owner_generation=3, owner_sequence=0,
        valid=False, owner_lost=True, upstream_invalid=False, reason="owner_lost",
    )
    assert egress_from_message(wire) == EgressHeader(
        relay_gid=wire.relay_gid, relay_boot="boot", relay_sequence=4,
        owner_gid=wire.owner_gid, owner_epoch="epoch", owner_generation=3, owner_sequence=0,
        valid=False, owner_lost=True, reason="owner_lost",
    )


def test_depth_body_capture_check_uses_original_receipts_and_current_bridge_gids():
    from types import SimpleNamespace

    from senior_control.vision.egress_receipt import depth_capture_headers

    def wire(raw_gid, bridge_gid, sequence):
        return SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=0),
            received_ros=SimpleNamespace(sec=1, nanosec=1), received_steady=10.0,
            frame_id="optical", raw_gid=raw_gid, bridge_gid=bridge_gid, source_epoch=raw_gid,
            bridge_boot_id="boot", capture_generation=1, sequence=sequence,
            valid=True, reason="",
        )

    depth = wire("11" * 24, "01" * 24, 1)
    info = wire("22" * 24, "02" * 24, 1)
    body = SimpleNamespace(
        depth_capture=depth, camera_info_capture=info,
        stamp=depth.stamp, camera_info_stamp=info.stamp,
        received_ros=depth.received_ros, camera_info_received_ros=info.received_ros,
        received_steady=10.0, camera_info_received_steady=10.0,
        frame_id="optical", raw_source=depth.raw_gid, camera_info_source=info.raw_gid,
    )
    args = dict(depth_gid=depth.bridge_gid, info_gid=info.bridge_gid,
                now_ros_ns=1_100_000_000, now_steady=10.1, max_age_s=0.3)
    assert depth_capture_headers(body, **args) is not None
    body.received_steady = 10.05
    assert depth_capture_headers(body, **args) is None
    body.received_steady = 10.0
    assert depth_capture_headers(body, **{**args, "now_steady": 10.31}) is None
    assert depth_capture_headers(body, **{**args, "depth_gid": "03" * 24}) is None


def test_upstream_failure_revokes_once_without_retiring_healthy_semantic_owner():
    from senior_control.vision.egress_receipt import EgressAdmission, EgressHeader

    admission = EgressAdmission(max_age_s=0.3)
    good = EgressHeader(relay_gid="01" * 24, relay_boot="relay-a", owner_gid="11" * 24,
                        owner_epoch="semantic-a", owner_generation=1, owner_sequence=5,
                        relay_sequence=7, valid=True, reason="")
    assert admission.offer(good, relay_gid=good.relay_gid)
    failure = replace(good, relay_sequence=8, valid=False, upstream_invalid=True,
                      reason="depth_invalid")
    assert not admission.offer(failure, relay_gid=good.relay_gid)
    assert admission.latest is None
    generation = admission.generation
    assert not admission.offer(replace(failure, relay_sequence=9), relay_gid=good.relay_gid)
    assert admission.generation == generation
    recovered = replace(good, relay_sequence=10, owner_sequence=6)
    assert admission.offer(recovered, relay_gid=good.relay_gid)


def test_scan_egress_can_use_explicit_three_second_observation_profile():
    import pytest

    from senior_control.vision.egress_receipt import EgressAdmission

    with pytest.raises(ValueError):
        EgressAdmission(max_age_s=3.0)
    admission = EgressAdmission(max_age_s=3.0, hard_limit_s=5.0)
    assert admission.max_age_s == 3.0


def test_safety_scan_capture_requires_original_clocks_and_native_endpoint():
    from types import SimpleNamespace

    from senior_control.vision.egress_receipt import safety_scan_capture

    stamp = SimpleNamespace(sec=1, nanosec=0)
    receipt = SimpleNamespace(sec=1, nanosec=100)
    header = SimpleNamespace(
        stamp=stamp, received_ros=receipt, received_steady=10.0, frame_id="laser",
        raw_gid="11" * 24, bridge_gid="22" * 24, source_epoch="scan-epoch",
        bridge_boot_id="scan-boot", capture_generation=1, sequence=1, valid=True, reason="",
    )
    body = SimpleNamespace(raw_capture=header, stamp=stamp, frame_id="laser",
                           raw_source=header.raw_gid, ingress_ros=1.0000001, ingress_steady=10.0)
    args = dict(bridge_gid=header.bridge_gid, now_ros_ns=1_100_000_000,
                now_steady=10.1, max_age_s=0.5)
    assert safety_scan_capture(body, **args) is not None
    body.ingress_steady = 10.01
    assert safety_scan_capture(body, **args) is None
    body.ingress_steady = 10.0
    assert safety_scan_capture(body, **{**args, "now_steady": 10.51}) is None
    assert safety_scan_capture(body, **{**args, "bridge_gid": "33" * 24}) is None
