"""Current scan lifecycle behavior, with the production reference and bridge."""

import time
from types import SimpleNamespace as NS

from senior_localization.live_mapping_reference import LiveMappingReference
from senior_mapping.workspace import WorkspaceGrid

from vehicle_core.values import MapRef, Pose2D


def header(sequence, now=None):
    now = time.monotonic_ns() if now is None else now
    return NS(
        valid=True,
        stamp=NS(sec=1700000000, nanosec=123456789 + sequence),
        received_ros=NS(sec=1700000000, nanosec=123456789 + sequence),
        received_steady_ns=now,
        sample_steady_ns=now,
        clock_generation=1,
        bridge_boot_id="bridge",
        source_epoch="source",
        capture_generation=1,
        raw_gid="01",
        bridge_gid="02",
        sequence=sequence,
        frame_id="laser",
        reason="",
    )


def reference():
    ref = LiveMappingReference(NS(task_motion_allowed=lambda **_: True))
    ref.begin("backend")
    ref.install(
        "backend",
        WorkspaceGrid(
            MapRef("map", 1), "grid", 100, 100, 1.0, Pose2D(-50.0, -50.0, 0.0), bytes(10000)
        ),
        "R1",
        "0",
        Pose2D(10.0, 0.0, 0.0),
        Pose2D(0.0, 0.0, 0.0),
    )
    return ref


def test_late_observation_cannot_invalidate_newer_current_reference():
    ref = reference()
    first, second = header(1), header(2)
    ref.observe("backend", second, Pose2D(2.0, 0.0, 0.0))
    assert ref.pose() == Pose2D(12.0, 0.0, 0.0)
    ref.observe("backend", first, Pose2D(1.0, 0.0, 0.0))
    assert ref.pose() == Pose2D(12.0, 0.0, 0.0)


def message(sequence=1, ranges=(2.0, 3.0, 4.0), sign=1):
    return NS(
        backend_boot="backend",
        status=1,
        static_receipt_valid=True,
        reason="",
        scan=NS(
            capture=header(sequence),
            ranges=ranges,
            angle_min=-0.2,
            angle_increment=0.2,
            range_min=0.1,
            range_max=20.0,
        ),
        odom_base=NS(x=0.0, y=0.0, theta=0.0),
        base_lidar=NS(translation=NS(x=0.0, y=0.0, z=0.1), rotation=NS(x=0.0, y=0.0, z=0.0, w=1.0)),
        raw_ray_angle_sign=sign,
        geometry_id="geometry",
    )


def decode(msg):
    from senior_mapping.shared_scan import decode_qualified_scan

    from vehicle_core.config import DeviceProfile

    return decode_qualified_scan(msg, DeviceProfile(), learning_view=None, host_boot_id="host")


def test_full_scan_projection_keeps_indices_and_original_deadline():
    from senior_mapping.shared_scan import project_hits

    msg = message(ranges=(2.0, float("inf"), 20.0, 3.0), sign=-1)
    scan = decode(msg)
    assert scan.capture.time.stamp_ns == 1700000000123456790
    assert scan.valid_until_steady_ns == msg.scan.capture.sample_steady_ns + 500000000
    assert [beam.index for beam in scan.beams] == [0, 1, 2, 3]
    assert [beam.kind for beam in scan.beams] == ["hit", "invalid", "invalid", "hit"]
    assert scan.beams[0].endpoint_odom[1] > 0  # inverted, raw index retained
    points = project_hits(scan, Pose2D(10.0, 0.0, 0.0))
    assert len(points) == 2 and points[0][0] > 10
    assert len(project_hits(decode(message(ranges=(2.0,) * 2048)), Pose2D(0.0, 0.0, 0.0))) == 2048


def test_slot_retirement_reference_floor_and_late_writes(tmp_path):
    from senior_mapping.shared_scan import LatestScanSlot, ReferenceBasis

    slot = LatestScanSlot(
        tmp_path / "latest-scan.json", nonce="nonce", backend_boot="backend", host_boot_id="host"
    )
    try:
        basis = ReferenceBasis(
            "map", 1, "R1", 1, "0" * 64, "0", Pose2D(10.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0)
        )
        slot.install_reference(basis)
        a, b = decode(message(1)), decode(message(2))
        assert slot.write(a)
        slot.flush()
        row = slot.read_after(0)
        assert row.changed and row.envelope.scan == a
        assert not slot.read_after(row.envelope.revision).changed
        assert not slot.write(a)  # does not extend deadline
        assert slot.write(b)
        slot.retire()
        assert not slot.write(b)
        slot.flush()
        retired = slot.read_after(row.envelope.revision).envelope
        assert retired.status == "retired" and retired.scan is None
        slot.install_reference(
            ReferenceBasis(
                "map", 2, "R2", 2, "1" * 64, "1", Pose2D(20.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0)
            )
        )
        assert not slot.write(b)
        assert slot.write(decode(message(3)))
        slot.flush()
        fresh = slot.read_after(retired.revision).envelope
        assert fresh.basis.reference_generation == "R2"
        assert fresh.scan.key.sequence == 3 and fresh.acceptance_floor == b.key
        assert (tmp_path / "latest-scan.json").stat().st_mode & 0o777 == 0o600
    finally:
        slot.close()


def test_scan_a_b_invalid_updates_reference_tf_without_rpc_or_keyframe(monkeypatch):
    from senior_mapping.navigation_bridge import NativeNavigationBridge

    ref = reference()
    calls, transforms, outputs = [], [], []

    async def forbidden(*args, **kwargs):
        calls.append(args)
        raise AssertionError("scan freshness cannot wait for RPC")

    bridge = NativeNavigationBridge(NS(), ref, forbidden)
    bridge.boot = "backend"
    bridge._publish_transform = transforms.append
    bridge._publish_current = lambda *args: outputs.append(args)
    a = message(1, ranges=(2.0, 3.0))
    b = message(2, ranges=(4.0, 5.0))
    bridge.observe_scan(decode(a), a.scan.capture)
    assert ref.pose() == Pose2D(10.0, 0.0, 0.0)
    assert ref.qualified(MapRef("map", 1), "R1")
    bridge.observe_scan(decode(b), b.scan.capture)
    assert len(transforms) == 2 and transforms[-1] == b.scan.capture.stamp
    assert outputs[-1][2] != outputs[-2][2] and len(outputs[-1][2]) == 2
    # Delayed A does not replace B or renew its original deadline.
    deadline = outputs[-1][3]
    bridge.observe_scan(decode(a), a.scan.capture)
    assert len(transforms) == 2 and outputs[-1][3] == deadline
    monkeypatch.setattr("senior_mapping.navigation_bridge.time.monotonic_ns", lambda: deadline)
    bridge.check_current()
    assert outputs[-1] == (False, "expired", (), 0)
    assert ref.pose() is None and not calls
    assert ref.grid() is not None  # Q withdrawal never erases fixed data


def test_valid_empty_coverage_pending_and_new_reference_floor():
    from senior_mapping.navigation_bridge import NativeNavigationBridge

    ref = reference()
    bridge = NativeNavigationBridge(NS(), ref, None)
    bridge.boot = "backend"
    outputs = []
    bridge._publish_transform = lambda stamp: None
    bridge._publish_current = lambda *args: outputs.append(args)
    a = message(1, ranges=(float("nan"), float("inf")))
    bridge.observe_scan(decode(a), a.scan.capture)
    assert outputs[-1][0] and outputs[-1][2] == ()
    assert ref.qualified(MapRef("map", 1), "R1")
    b = message(2, ranges=(19.0, 19.0))
    b.odom_base.x = 40.0
    bridge.observe_scan(decode(b), b.scan.capture)
    assert outputs[-1][0] and outputs[-1][1] == "coverage_pending"
    assert len(outputs[-1][2]) == 2
    assert not ref.qualified(MapRef("map", 1), "R1")
    ref.install("backend", ref.grid(), "R2", "0", Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0))
    bridge.observe_scan(decode(b), b.scan.capture)
    assert ref.pose() is None
    c = message(3)
    bridge.observe_scan(decode(c), c.scan.capture)
    assert ref.qualified(MapRef("map", 1), "R2")
    bridge.invalidate_current("source_retired", retired=True)
    bridge.observe_scan(decode(message(4)), header(4))
    assert outputs[-1] == (False, "source_retired", (), 0)


def test_slow_slot_write_coalesces_and_retirement_wins(tmp_path, monkeypatch):
    import threading

    from senior_mapping import shared_scan

    entered, release = threading.Event(), threading.Event()
    replace = shared_scan.os.replace

    def slow_replace(*args):
        entered.set()
        assert release.wait(3)
        replace(*args)

    monkeypatch.setattr(shared_scan.os, "replace", slow_replace)
    slot = shared_scan.LatestScanSlot(
        tmp_path / "latest-scan.json", nonce="n", backend_boot="backend", host_boot_id="host"
    )
    try:
        assert entered.wait(2)
        # The one in-flight file operation cannot acquire the scan callback lock.
        from senior_mapping.navigation_bridge import NativeNavigationBridge

        bridge = NativeNavigationBridge(NS(), reference(), None)
        bridge.boot, bridge.slot = "backend", slot
        outputs = []
        bridge._publish_transform = lambda stamp: None
        bridge._publish_current = lambda *args: outputs.append(args)
        for sequence in (1, 2):
            current = message(sequence)
            bridge.observe_scan(decode(current), current.scan.capture)
        assert len(outputs) == 2 and all(row[0] for row in outputs)
        slot.retire()
        assert not slot.write(decode(message(3)))
        release.set()
        slot.flush()
        row = slot.read_after(0).envelope
        assert row.status == "retired" and row.revision == 4
        assert row.scan is None and row.acceptance_floor.sequence == 2
    finally:
        release.set()
        slot.close()
    alias = tmp_path / "alias"
    alias.symlink_to(slot.path)
    import pytest

    with pytest.raises(OSError):
        shared_scan.read_latest_scan(
            alias, 0, nonce="n", backend_boot="backend", host_boot_id="host"
        )


def test_unseen_scan_captured_before_new_reference_cannot_cross_floor(tmp_path):
    from senior_mapping.shared_scan import LatestScanSlot, ReferenceBasis

    slot = LatestScanSlot(
        tmp_path / "latest-scan.json", nonce="n", backend_boot="backend", host_boot_id="host"
    )
    try:
        delayed = decode(message(5))
        slot.install_reference(
            ReferenceBasis(
                "map", 2, "R2", 2, "0" * 64, "0", Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0)
            )
        )
        assert not slot.write(delayed), "newer unseen sequence was captured under old reference"
    finally:
        slot.close()


def test_bridge_install_rejects_scan_sampled_before_new_reference():
    from senior_mapping.navigation_bridge import NativeNavigationBridge

    ref = reference()
    bridge = NativeNavigationBridge(NS(), ref, None)
    bridge.boot = "backend"
    bridge._grid = (1, ref.grid(), object())
    bridge._publisher = NS(publish=lambda _: None)
    bridge.observe_cut(
        dict(
            boot_id="backend",
            sequence=1,
            keyframe_id="0",
            anchors=[dict(id="0", x=10.0, y=0.0, yaw=0.0)],
            odom=[0.0, 0.0, 0.0],
        )
    )
    outputs = []
    bridge._publish_transform = lambda stamp: None
    bridge._publish_current = lambda *args: outputs.append(args)
    delayed = message(2)
    bridge.install(
        dict(
            active=True,
            sequence=1,
            grid_sequence=1,
            map_id="map",
            reference_revision=1,
            generation="R2",
        )
    )
    bridge.observe_scan(decode(delayed), delayed.scan.capture)
    assert not outputs[-1][0] and ref.pose() is None
    fresh = message(3)
    bridge.observe_scan(decode(fresh), fresh.scan.capture)
    assert outputs[-1][0] and ref.qualified(MapRef("map", 1), "R2")


def test_adapter_requires_original_authenticated_native_receipt_metadata():
    from senior_mapping.navigation_bridge import NativeNavigationBridge
    from senior_mapping.ros_runtime import RosMappingBackend

    from vehicle_core.config import DeviceProfile

    owner = NS(
        node_name="slam_toolbox",
        node_namespace="/",
        topic_type="senior_interfaces/msg/QualifiedMappingScan",
        endpoint_gid=b"\x01" * 24,
    )
    node = NS(get_publishers_info_by_topic=lambda _: [owner])
    backend = RosMappingBackend.__new__(RosMappingBackend)
    backend.node, backend._scan_accepting, backend._boot = node, True, "backend"
    backend._qualified_gid, backend._learning_client = None, None
    backend._scan_profile, backend._host_boot_id = DeviceProfile(), "host"
    backend._navigation = NativeNavigationBridge(node, reference(), None)
    backend._navigation.boot = "backend"
    outputs = []
    backend._navigation._publish_current = lambda *args: outputs.append(args)
    backend._navigation._publish_transform = lambda _: None
    current = message(1)
    current.QUALIFIED, current.RETIRED = 1, 2
    backend._on_qualified_scan(current, {"publisher_gid": owner.endpoint_gid})
    assert not outputs  # discovery is not this sample's receipt
    backend._on_qualified_scan(
        current, {"publisher_gid": b"\x02" * 24, "native_receipt_authenticated": True}
    )
    assert not outputs
    backend._on_qualified_scan(
        current, {"publisher_gid": owner.endpoint_gid, "native_receipt_authenticated": True}
    )
    assert outputs[-1][0] and len(outputs[-1][2]) == 3


def test_real_graph_install_uses_c3_digest_independent_of_keyframe_order():
    from senior_mapping.navigation_bridge import NativeNavigationBridge
    from senior_mapping.ros_runtime import RosMappingBackend

    frames = [NS(id=i, x=float(i), y=0.0, yaw=0.0) for i in (0, 1, 2, 10)]
    digests = []
    for ordered in (frames, list(reversed(frames))):
        cut = RosMappingBackend._graph(NS(
            boot_id="backend", graph_commit_sequence=1, odom_frame="odom",
            map_frame="map", keyframes=ordered,
        ))
        original_anchors = [dict(row) for row in cut["anchors"]]
        cut.update(keyframe_id=0, odom=[0.0, 0.0, 0.0])
        ref = reference()
        bridge = NativeNavigationBridge(NS(), ref, None)
        bridge.boot = "backend"
        bridge._grid = (1, ref.grid(), object())
        bridge._publisher = NS(publish=lambda _: None)
        bridge._publish_current = lambda *args: None
        bases = []
        bridge.slot = NS(install_reference=lambda basis, target=bases, **kw: target.append(basis),
                         invalidate=lambda _: None)
        bridge.observe_cut(cut)
        bridge.install(dict(active=True, grid_sequence=1, sequence=1, map_id="map",
                            reference_revision=1, generation="R1"))
        # Independent review's literal C3 result, not an expectation built by our helper.
        assert bases[0].anchor_digest == (
            "df9170d6bdc8571fc6657a6cffa56f545366733859708317fcfe164c991334f9"
        )
        from senior_mapping.evidence import anchor_digest

        records = [dict(anchor_id=str(f.id), available=True,
                        pose=dict(x=f.x, y=f.y, yaw=f.yaw)) for f in ordered]
        assert bases[0].anchor_digest == anchor_digest(records)
        assert cut["anchors"] == original_anchors  # native graph/capture wire stays intact
        digests.append(bases[0].anchor_digest)
    assert digests[0] == digests[1]
