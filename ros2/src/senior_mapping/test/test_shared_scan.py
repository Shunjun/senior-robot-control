"""Current scan lifecycle behavior, with the production reference and bridge."""

import time
from types import SimpleNamespace as NS

from senior_localization.live_mapping_reference import LiveMappingReference
from senior_mapping.workspace import WorkspaceGrid

from vehicle_core.values import MapRef, Pose2D


def install_synthetic_fixed(bridge, fields, *, fixed_revision=None):
    """Synthetic-admitted fixed F baseline replaces the retired raw install fixture."""
    from dataclasses import replace

    from senior_mapping.evidence import FixedWatermark, SourceIdentity, anchor_digest
    from senior_mapping.references import (
        AnchorPose,
        FixedSnapshot,
        LegacyBase,
        fixed_anchor_records,
    )
    from senior_mapping.workspace import fixed_grid_id

    graph = bridge._cut
    original = bridge._grid[1]
    grid = replace(
        original,
        grid_id=fixed_grid_id(
            original.map_ref,
            original.width,
            original.height,
            original.resolution,
            original.origin,
            original.cells,
        ),
        display_only=True,
        waypoint_writes=False,
    )
    mask = bytearray((len(grid.cells) + 7) // 8)
    for i, value in enumerate(grid.cells):
        if value != 255:
            mask[i // 8] |= 1 << (i % 8)
    base = LegacyBase(
        grid.grid_id, grid, "legacy-derived", "0" * 64, "synthetic-admitted", bytes(mask), ()
    )
    anchors = tuple(
        sorted(
            (
                AnchorPose(str(a["id"]), Pose2D(a["x"], a["y"], a["yaw"]), True)
                for a in graph["anchors"]
            ),
            key=lambda a: a.anchor_id.encode(),
        )
    )
    old = bridge._fixed_watermark
    revision = (
        0
        if old is None
        else old.fixed_revision + (old.reference_generation != fields["generation"])
    )
    watermark = FixedWatermark(
        grid.map_ref,
        fields["generation"],
        bridge.boot,
        "host",
        None if bridge._last_key is None else SourceIdentity.from_key(bridge._last_key),
        graph["sequence"],
        anchor_digest(fixed_anchor_records(anchors)),
        revision if fixed_revision is None else fixed_revision,
        None,
        0,
        0,
    )
    cut = FixedSnapshot(watermark, grid, anchors, None, (), (), base, (), ())
    bridge.host_boot_id = "host"
    bridge._publish_fixed_grid = lambda grid: None
    bridge.authorize_reference(
        {
            **fields,
            "boot_id": bridge.boot,
            "acceptance_floor_steady_ns": getattr(bridge, "_owner_floor_ns", 0),
        }
    )
    return bridge.install_fixed(cut, graph_cut=graph)


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
    dense = message(ranges=(2.0,) * 5400)
    dense.scan.angle_increment = 0.001
    complete = decode(dense)
    assert [beam.index for beam in complete.beams] == list(range(5400))
    assert len(project_hits(complete, Pose2D(0.0, 0.0, 0.0))) == 5400


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
    assert ref._coverage and not ref.qualified(MapRef("map", 1), "R1")
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
    assert ref._coverage and not ref.qualified(MapRef("map", 1), "R1")
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
    assert ref._coverage and not ref.qualified(MapRef("map", 1), "R2")
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
    install_synthetic_fixed(
        bridge,
        dict(
            active=True,
            sequence=1,
            grid_sequence=1,
            map_id="map",
            reference_revision=1,
            generation="R2",
        ),
    )
    bridge.observe_scan(decode(delayed), delayed.scan.capture)
    assert not outputs[-1][0] and ref.pose() is None
    fresh = message(3)
    bridge.observe_scan(decode(fresh), fresh.scan.capture)
    assert outputs[-1][0] and ref._coverage
    assert not ref.qualified(MapRef("map", 1), "R2")


def test_adapter_uses_native_channel_without_rechecking_diagnostic_gid():
    from senior_mapping.navigation_bridge import NativeNavigationBridge
    from senior_mapping.ros_runtime import RosMappingBackend

    from vehicle_core.config import DeviceProfile

    owner = NS(
        node_name="slam_toolbox",
        node_namespace="/",
        topic_type="senior_interfaces/msg/QualifiedMappingScan",
        endpoint_gid=b"\x01" * 24,
    )

    def unexpected_graph_query(_topic):
        raise AssertionError("managed receipt must not repeat publisher qualification")

    node = NS(get_publishers_info_by_topic=unexpected_graph_query)
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
    assert outputs[-1][0]  # GID is diagnostic after the receiving entry checks topology.
    backend._on_qualified_scan(
        current, {"publisher_gid": owner.endpoint_gid, "native_receipt_authenticated": True}
    )
    assert outputs[-1][0] and len(outputs[-1][2]) == 3
    backend._scan_slot = None
    backend._poll_scan_channel()  # Housekeeping must reuse receiving-entry availability too.


def test_real_graph_install_uses_c3_digest_independent_of_keyframe_order():
    from senior_mapping.navigation_bridge import NativeNavigationBridge
    from senior_mapping.ros_runtime import RosMappingBackend

    frames = [NS(id=i, x=float(i), y=0.0, yaw=0.0) for i in (0, 1, 2, 10)]
    digests = []
    for ordered in (frames, list(reversed(frames))):
        cut = RosMappingBackend._graph(
            NS(
                boot_id="backend",
                graph_commit_sequence=1,
                odom_frame="odom",
                map_frame="map",
                keyframes=ordered,
            )
        )
        original_anchors = [dict(row) for row in cut["anchors"]]
        cut.update(keyframe_id=0, odom=[0.0, 0.0, 0.0])
        ref = reference()
        bridge = NativeNavigationBridge(NS(), ref, None)
        bridge.boot = "backend"
        bridge._grid = (1, ref.grid(), object())
        bridge._publisher = NS(publish=lambda _: None)
        bridge._publish_current = lambda *args: None
        bases = []
        bridge.slot = NS(
            install_reference=lambda basis, target=bases, **kw: target.append(basis),
            invalidate=lambda _: None,
        )
        bridge.observe_cut(cut)
        install_synthetic_fixed(
            bridge,
            dict(
                active=True,
                grid_sequence=1,
                sequence=1,
                map_id="map",
                reference_revision=1,
                generation="R1",
            ),
        )
        # Independent review's literal C3 result, not an expectation built by our helper.
        assert bases[0].anchor_digest == (
            "df9170d6bdc8571fc6657a6cffa56f545366733859708317fcfe164c991334f9"
        )
        from senior_mapping.evidence import anchor_digest

        records = [
            dict(anchor_id=str(f.id), available=True, pose=dict(x=f.x, y=f.y, yaw=f.yaw))
            for f in ordered
        ]
        assert bases[0].anchor_digest == anchor_digest(records)
        assert cut["anchors"] == original_anchors  # native graph/capture wire stays intact
        digests.append(bases[0].anchor_digest)
    assert digests[0] == digests[1]


def test_bootstrap_selection_two_roles_and_original_expiry(monkeypatch, tmp_path):
    from dataclasses import replace

    from senior_mapping.shared_scan import (
        ContractSlot,
        CostmapGeometry,
        CurrentLayerCoverage,
        ReferenceAuthorization,
        read_contract_record,
    )

    now = time.monotonic_ns()
    monkeypatch.setattr(time, "monotonic_ns", lambda: now)
    ref = reference()
    scan = decode(message(10))
    ref.observe("backend", header(10), Pose2D(0.0, 0.0, 0.0))
    ref.set_current_coverage(True)
    assert not ref.qualified(MapRef("map", 1), "R1")
    geometry = CostmapGeometry(100, 100, 1.0, -50.0, -50.0)
    auth = ReferenceAuthorization(
        run_nonce="run",
        host_boot_id="host",
        owner_instance="owner",
        revision=1,
        authority_epoch=1,
        status="selected",
        map_id="map",
        reference_revision=1,
        reference_generation="R1",
        backend_boot="backend",
        source=scan.key.source_payload,
        selection_token="selection",
        selection_generation=1,
        navigation_floor_steady_ns=now - 1,
        geometry=geometry,
        coverage_domain_epoch=1,
        global_instance_permit="g",
        local_instance_permit="l",
        lease_until_steady_ns=scan.valid_until_steady_ns,
    )
    ref.set_navigation_authorization(auth, accepted_key=scan.key)
    slot = ContractSlot(tmp_path / "reference.json", ReferenceAuthorization)
    try:
        slot.offer(auth)
        slot.flush()
        assert read_contract_record(slot.path, ReferenceAuthorization) == auth
        import pytest

        # The encoding cap rejects the whole candidate and preserves the previous inode.
        overlarge = replace(
            auth,
            revision=2,
            **{
                name: "\0" * 128
                for name in (
                    "run_nonce",
                    "host_boot_id",
                    "owner_instance",
                    "reason",
                    "map_id",
                    "reference_generation",
                    "backend_boot",
                    "selection_token",
                    "anchor_digest",
                    "expected_grid_id",
                    "global_instance_permit",
                    "local_instance_permit",
                )
            },
        )
        slot.offer(overlarge)
        with pytest.raises(OSError, match="8KiB"):
            slot.flush()
        assert read_contract_record(slot.path, ReferenceAuthorization) == auth
        slot.offer(replace(auth, revision=3))
        slot.flush()
        # An older candidate cannot replace a later observed authorization revision.
        slot.offer(auth)
        slot.flush()
        assert read_contract_record(slot.path, ReferenceAuthorization).revision == 3

        def receipt(role, cycle=1, status="ready"):
            return CurrentLayerCoverage(
                run_nonce="run",
                host_boot_id="host",
                owner_instance="owner",
                authority_epoch=1,
                role=role,
                role_instance_permit={"global": "g", "local": "l"}[role],
                plugin_instance=role,
                cycle=cycle,
                status=status,
                map_id="map",
                reference_revision=1,
                reference_generation="R1",
                backend_boot="backend",
                key=scan.key,
                sample_steady_ns=now,
                scan_deadline_steady_ns=scan.valid_until_steady_ns,
                coverage_domain_epoch=1,
                geometry=geometry,
                cycle_started_steady_ns=now,
                cycle_completed_steady_ns=now,
                valid_until_steady_ns=scan.valid_until_steady_ns,
                master_frame="map",
                master_geometry=geometry,
                updated_bounds=(-50.0, -50.0, 50.0, 50.0),
                global_all_hits_covered=True,
                local_in_window_hits_covered=True,
                transform_valid=True,
                cleanup_complete=True,
            )

        ref.observe_costmap_coverage("global", receipt("global"))
        assert not ref.qualified(MapRef("map", 1), "R1")
        ref.observe_costmap_coverage("local", receipt("local"))
        assert ref.qualified(MapRef("map", 1), "R1")
        # A later accepted Q does not invalidate an unexpired completed costmap cycle.
        ref.set_navigation_authorization(
            replace(auth, revision=2), accepted_key=replace(scan.key, sequence=11)
        )
        assert ref.qualified(MapRef("map", 1), "R1")
        ref.observe_costmap_coverage("global", receipt("global", 2, "invalid"))
        ref.set_current_coverage(True)
        ref.observe_costmap_coverage("local", receipt("local", 2))
        ref.observe_costmap_coverage("global", receipt("global", 1))
        assert not ref.qualified(MapRef("map", 1), "R1")
        ref.observe_costmap_coverage("global", receipt("global", 3))
        assert ref.qualified(MapRef("map", 1), "R1")
        # A new plugin instance cannot authorize itself with an existing role permit.
        ref.observe_costmap_coverage(
            "global", replace(receipt("global", 4), plugin_instance="foreign")
        )
        assert not ref.qualified(MapRef("map", 1), "R1")
        ref.observe_costmap_coverage("global", receipt("global", 5))
        # Exercise the actual independent receipt-file reader, including a bad inode.
        from senior_mapping.navigation_bridge import NativeNavigationBridge

        bridge = NativeNavigationBridge(NS(), ref, None)
        bridge.contract = dict(contract_directory=str(tmp_path))
        bridge.authorization_slot = slot
        writers = {
            role: ContractSlot(tmp_path / (role + "-coverage.json"), CurrentLayerCoverage)
            for role in ("global", "local")
        }
        try:
            for role, writer in writers.items():
                writer.offer(receipt(role, 6))
                writer.flush()
            bridge.poll_contract()
            assert ref.qualified(MapRef("map", 1), "R1")
            writers["global"].path.unlink()
            writers["global"].path.symlink_to(slot.path)
            bridge.poll_contract()
            ref.set_current_coverage(True)
            assert not ref.qualified(MapRef("map", 1), "R1")
            writers["global"].offer(receipt("global", 7))
            writers["global"].flush()
            bridge.poll_contract()
            assert ref.qualified(MapRef("map", 1), "R1")
        finally:
            for writer in writers.values():
                writer.close()
        # Only the controller owner can select a new local instance using a rotated permit.
        auth = replace(auth, revision=3, local_instance_permit="l2")
        ref.set_navigation_authorization(auth, accepted_key=scan.key)
        ref.observe_costmap_coverage("local", receipt("local", 20))
        assert not ref.qualified(MapRef("map", 1), "R1")
        ref.observe_costmap_coverage(
            "local",
            replace(
                receipt("local", 1), role_instance_permit="l2", plugin_instance="restarted-local"
            ),
        )
        assert ref.qualified(MapRef("map", 1), "R1")
        # Domain changes clear both completed cycles, even though MapRef/R did not change.
        ref.set_navigation_authorization(
            replace(auth, revision=4, coverage_domain_epoch=2), accepted_key=scan.key
        )
        ref.observe_costmap_coverage("global", receipt("global", 8))
        assert not ref.qualified(MapRef("map", 1), "R1")
        ref.set_navigation_authorization(replace(auth, revision=5), accepted_key=scan.key)
        ref.observe_costmap_coverage("global", receipt("global", 9))
        ref.observe_costmap_coverage(
            "local",
            replace(
                receipt("local", 2), role_instance_permit="l2", plugin_instance="restarted-local"
            ),
        )
        assert ref.qualified(MapRef("map", 1), "R1")
        assert ref.state("idle", navigation_ready=True)["navigation_ready"]
        now = scan.valid_until_steady_ns
        assert not ref.qualified(MapRef("map", 1), "R1")
        assert not ref.state("idle", navigation_ready=True)["navigation_ready"]
    finally:
        slot.close()


def test_static_fallback_remains_qualified_without_navigation_contract():
    fallback = NS(
        qualified=lambda ref, generation: ref == MapRef("static", 1) and generation == "G",
        pose=lambda: Pose2D(1.0, 2.0, 0.0),
        state=lambda *a, **k: dict(navigation_ready=k["navigation_ready"]),
    )
    ref = LiveMappingReference(NS(), fallback)
    assert not ref.enabled
    assert ref.qualified(MapRef("static", 1), "G")
    assert ref.pose() == Pose2D(1.0, 2.0, 0.0)
    assert ref.state("manual", navigation_ready=True)["navigation_ready"]


def test_role_pins_and_cycles_survive_map_withdraw_begin(monkeypatch):
    from dataclasses import replace

    from senior_mapping.navigation_bridge import NativeNavigationBridge
    from senior_mapping.shared_scan import CurrentLayerCoverage

    now = time.monotonic_ns()
    monkeypatch.setattr(time, "monotonic_ns", lambda: now)
    ref = reference()
    grid = ref.grid()
    bridge = NativeNavigationBridge(NS(), ref, None)
    bridge.host_boot_id = "host"
    bridge.contract = dict(run_nonce="run", global_instance_permit="g", local_instance_permit="l")
    bridge._publish_current = lambda *a: None
    bridge._publish_transform = lambda *a: None

    def session(boot, generation):
        nonlocal now
        bridge.begin(boot)
        bridge._grid = (1, grid, object())
        bridge.observe_cut(
            dict(
                boot_id=boot,
                sequence=1,
                keyframe_id="0",
                anchors=[dict(id="0", x=10.0, y=0.0, yaw=0.0)],
                odom=[0.0, 0.0, 0.0],
            )
        )
        now += 1
        first = message(1)
        first.backend_boot = boot
        bridge.observe_scan(decode(first), first.scan.capture)
        fields = dict(sequence=1, map_id="map", reference_revision=1, generation=generation)
        install_synthetic_fixed(bridge, fields)
        bridge.authorize_navigation(
            dict(**fields, selection_token="selection-" + boot, selection_generation=1)
        )
        now += 1
        fresh = message(2)
        fresh.backend_boot = boot
        scan = decode(fresh)
        bridge.observe_scan(scan, fresh.scan.capture)
        return ref._authorization, scan

    def receipt(role, instance, cycle):
        return CurrentLayerCoverage(
            run_nonce=auth.run_nonce,
            host_boot_id="host",
            owner_instance=auth.owner_instance,
            authority_epoch=auth.authority_epoch,
            role=role,
            role_instance_permit=getattr(auth, role + "_instance_permit"),
            plugin_instance=instance,
            cycle=cycle,
            status="ready",
            map_id="map",
            reference_revision=1,
            reference_generation=auth.reference_generation,
            backend_boot=auth.backend_boot,
            key=scan.key,
            sample_steady_ns=now,
            scan_deadline_steady_ns=scan.valid_until_steady_ns,
            coverage_domain_epoch=auth.coverage_domain_epoch,
            geometry=auth.geometry,
            master_geometry=auth.geometry,
            cycle_started_steady_ns=now,
            cycle_completed_steady_ns=now,
            valid_until_steady_ns=scan.valid_until_steady_ns,
            master_frame="map",
            updated_bounds=(-50.0, -50.0, 50.0, 50.0),
            global_all_hits_covered=True,
            local_in_window_hits_covered=True,
            transform_valid=True,
            cleanup_complete=True,
        )

    auth, scan = session("backend", "R1")
    old_auth = auth
    old_rows = {r: receipt(r, r, 10) for r in ("global", "local")}
    for role, row in old_rows.items():
        ref.observe_costmap_coverage(role, row)
    assert ref.qualified(grid.map_ref, "R1")
    for role in ("global", "local"):
        ref.observe_costmap_coverage(role, receipt(role, "replacement", 11))
    assert not ref.qualified(grid.map_ref, "R1")
    bridge.withdraw()
    # Old authorization alone cannot reopen the retired reference or erase its pin.
    ref.set_navigation_authorization(old_auth, accepted_key=scan.key)
    assert not ref.qualified(grid.map_ref, "R1")
    auth, scan = session("backend2", "R2")
    for role in ("global", "local"):
        ref.observe_costmap_coverage(role, receipt(role, "replacement", 12))
    assert not ref.qualified(grid.map_ref, "R2")
    for role in ("global", "local"):
        ref.observe_costmap_coverage(role, old_rows[role])  # old epoch cannot restore
        ref.observe_costmap_coverage(role, receipt(role, role, 10))  # cycle cannot rewind
    assert not ref.qualified(grid.map_ref, "R2")
    for role in ("global", "local"):
        ref.observe_costmap_coverage(role, receipt(role, role, 11))
    assert ref.qualified(grid.map_ref, "R2")
    # Original owner rotates before a replacement spawn; both roles may start new cycles.
    bridge.contract.update(global_instance_permit="g2", local_instance_permit="l2")
    bridge._offer_authorization()
    auth = ref._authorization
    for role in ("global", "local"):
        ref.observe_costmap_coverage(role, receipt(role, "owner-replacement", 1))
    assert ref.qualified(grid.map_ref, "R2")
    auth = replace(auth, run_nonce="new-startup-run", revision=auth.revision + 1)
    ref.set_navigation_authorization(auth, accepted_key=scan.key)
    for role in ("global", "local"):
        ref.observe_costmap_coverage(role, receipt(role, "new-run-instance", 1))
    assert ref.qualified(grid.map_ref, "R2")


def test_real_bridge_authority_epochs_have_immutable_visible_identity(tmp_path, monkeypatch):
    from dataclasses import asdict

    from senior_mapping.navigation_bridge import NativeNavigationBridge
    from senior_mapping.shared_scan import ReferenceAuthorization, read_contract_record

    bridge = NativeNavigationBridge(NS(), reference(), None)
    bridge.host_boot_id = "host"
    bridge._publish_current = lambda *a: None
    bridge._publish_transform = lambda *a: None
    bridge.configure_contract(
        dict(
            contract_directory=str(tmp_path),
            run_nonce="run",
            global_instance_permit="g",
            local_instance_permit="l",
        )
    )
    slot = bridge.authorization_slot
    offer = slot.offer
    visible = []

    def observe_inode(row):
        offer(row)
        slot.flush()  # Force every real offer to become observable before the next call.
        installed = read_contract_record(slot.path, ReferenceAuthorization)
        assert installed == row
        visible.append(installed)

    monkeypatch.setattr(slot, "offer", observe_inode)
    try:
        bridge.begin("backend")
        grid = reference().grid()
        bridge._grid = (1, grid, object())
        bridge.observe_cut(
            dict(
                boot_id="backend",
                sequence=1,
                keyframe_id="0",
                anchors=[dict(id="0", x=10.0, y=0.0, yaw=0.0)],
                odom=[0.0, 0.0, 0.0],
            )
        )
        fields = dict(sequence=1, map_id="map", reference_revision=1, generation="R1")
        install_synthetic_fixed(bridge, fields)
        bridge.authorize_navigation(
            dict(**fields, selection_token="selection", selection_generation=1)
        )
        assert visible[-1].status == "selected" and visible[-1].lease_until_steady_ns == 0
        fresh = message(1)
        bridge.observe_scan(decode(fresh), fresh.scan.capture)
        # First verified source after a source-less bootstrap basis; same R.
        install_synthetic_fixed(bridge, fields, fixed_revision=1)
        fresh = message(2)
        bridge.observe_scan(decode(fresh), fresh.scan.capture)
        assert visible[-1].lease_until_steady_ns == decode(fresh).valid_until_steady_ns
        fields["generation"] = "R2"
        install_synthetic_fixed(bridge, fields)
        assert visible[-1].reference_generation == "R2"
        assert visible[-1].selection_token == "selection"
        assert visible[-1].lease_until_steady_ns == 0
        fresh = message(3)
        bridge.observe_scan(decode(fresh), fresh.scan.capture)
        assert visible[-1].lease_until_steady_ns == decode(fresh).valid_until_steady_ns
        bridge.rotate_local_permit("l2")
        bridge.withdraw()
        assert visible[-1].status == "revoked" and visible[-1].lease_until_steady_ns == 0
        # Frozen sameAuthority identity, plus owner permits which also receive a new epoch.
        names = (
            "owner_instance",
            "map_id",
            "reference_revision",
            "reference_generation",
            "backend_boot",
            "source",
            "acceptance_floor",
            "acceptance_floor_steady_ns",
            "navigation_floor_steady_ns",
            "selection_token",
            "selection_generation",
            "global_instance_permit",
            "local_instance_permit",
        )
        epochs = {}
        for row in visible:
            print(
                "VISIBLE_AUTHORITY",
                row.authority_epoch,
                row.status,
                row.reference_generation,
                row.source,
                "revision",
                row.revision,
            )
        for row in visible:
            identity = {name: asdict(row)[name] for name in names}
            old = epochs.setdefault(row.authority_epoch, identity)
            assert old == identity, f"identity changed inside epoch {row.authority_epoch}"
    finally:
        slot.close()


def test_initial_gate_waits_for_original_first_commit():
    """An owned empty watermark must not release the first commit query."""
    import asyncio

    from senior_mapping.ros_runtime import RosMappingBackend

    async def scenario():
        backend = RosMappingBackend.__new__(RosMappingBackend)
        backend._lock = asyncio.Lock()
        backend._boot = "owned-boot"
        backend._input_deadline = asyncio.get_running_loop().time() + 10
        backend._prepare_services = lambda: None
        snapshots = iter(
            (
                NS(
                    boot_id="owned-boot",
                    status=2,
                    OK=1,
                    graph_commit_watermark_valid=True,
                    graph_commit_sequence=0,
                    last_committed_scan_valid=False,
                ),
                NS(
                    boot_id="owned-boot",
                    status=1,
                    OK=1,
                    graph_commit_watermark_valid=True,
                    graph_commit_sequence=1,
                    last_committed_scan_valid=False,
                ),
                NS(
                    boot_id="owned-boot",
                    status=1,
                    OK=1,
                    graph_commit_watermark_valid=True,
                    graph_commit_sequence=1,
                    last_committed_scan_valid=True,
                ),
            )
        )
        calls = []

        async def call(method, **fields):
            calls.append((method, fields))
            if method == "gate":
                return NS(ok=True, graph_commit_sequence=0)
            assert method == "graph"
            return next(snapshots)

        backend._call = call
        reply = await backend.request("map.backend.gate", boot_id="owned-boot", accepting=True)
        assert reply == {"ok": True, "sequence": 0}
        assert [method for method, _ in calls] == ["gate", "graph", "graph", "graph"]
        assert calls[0][1] == {"boot_id": "owned-boot", "accepting": True}

    asyncio.run(scenario())


def test_initial_gate_does_not_reset_expired_start_budget():
    import asyncio

    from senior_mapping.ros_runtime import RosMappingBackend

    async def scenario():
        backend = RosMappingBackend.__new__(RosMappingBackend)
        backend._lock = asyncio.Lock()
        backend._boot = "owned-boot"
        backend._input_deadline = asyncio.get_running_loop().time() - 1
        deadline = backend._input_deadline
        backend._prepare_services = lambda: None
        calls = []

        async def call(method, **fields):
            calls.append(method)
            assert method == "gate"
            return NS(ok=True, graph_commit_sequence=0)

        backend._call = call
        reply = await backend.request("map.backend.gate", boot_id="owned-boot", accepting=True)
        assert reply == {"ok": False, "error": "unavailable"}
        assert backend._input_deadline == deadline
        assert calls == ["gate"]

    asyncio.run(scenario())


def test_initial_gate_rejects_a_different_native_boot():
    import asyncio

    from senior_mapping.ros_runtime import RosMappingBackend

    async def scenario():
        backend = RosMappingBackend.__new__(RosMappingBackend)
        backend._lock = asyncio.Lock()
        backend._boot = "owned-boot"
        backend._input_deadline = asyncio.get_running_loop().time() + 10
        backend._prepare_services = lambda: None

        async def call(method, **fields):
            if method == "gate":
                return NS(ok=True, graph_commit_sequence=0)
            assert method == "graph"
            return NS(
                boot_id="another-boot",
                status=1,
                OK=1,
                graph_commit_watermark_valid=True,
                graph_commit_sequence=1,
                last_committed_scan_valid=True,
            )

        backend._call = call
        reply = await backend.request("map.backend.gate", boot_id="owned-boot", accepting=True)
        assert reply == {"ok": False, "error": "conflict"}

    asyncio.run(scenario())
