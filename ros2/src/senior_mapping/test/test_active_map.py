"""Pure in-process cuts; source tokens stand for an upstream verifier's result."""

from dataclasses import asdict

import pytest
from senior_mapping.active import ActiveMap
from senior_mapping.evidence import CellEvidence
from senior_mapping.references import AnchorPose, AnchorSnapshot, CellBinding, GridExtent
from senior_mapping.workspace import MAX_GRID_SIDE, WorkspaceMap

from vehicle_core.errors import DomainError
from vehicle_core.values import MapRef, Pose2D

MAP = MapRef("active-test", 1)
POSE = Pose2D(0, 0, 0)


def session() -> tuple[ActiveMap, object]:
    active = ActiveMap(MAP, "wheel_odom", 0.5)
    source = object()
    active.bind_verified_source(source, backend_boot="graph-a", source_epoch="scanner-a")
    return active, source


def anchors(count: int, width: int = 2) -> AnchorSnapshot:
    return AnchorSnapshot(
        MAP,
        "wheel_odom",
        tuple(AnchorPose(f"kf{i}", POSE, True) for i in range(1, count + 1)),
        GridExtent(width, 2, 0, 0),
    )


def hit(x: int, anchor: str) -> CellBinding:
    return CellBinding(
        CellEvidence((x, 0), anchor, 3, 0, 1.0, 3.0, "geometric_confirmation"),
        x * 0.5 + 0.25,
        0.25,
        POSE,
        False,
    )


def test_continuous_cuts_keep_reference_and_version_only_changed_raster() -> None:
    # Catches raw/unknown cells promoted to free and grid IDs tied to reference nonce.
    active, source = session()
    view = WorkspaceMap(active.grid)
    nonce = active.map_generation
    assert view.read("map.active", {}) == {"ok": False, "error": "unavailable"}
    active.commit(source, 1, "kf1", anchors(1), (hit(0, "kf1"),))
    first = active.grid()
    assert first is not None
    assert first.cells == bytes([100, 255, 255, 255])
    assert first.display_only and not first.waypoint_writes
    manifest = view.read("map.manifest", asdict(MAP))
    assert manifest["display_only"] is True
    assert "waypoint_writes" not in manifest and "navigation_ready" not in manifest

    active.commit(source, 2, "kf2", anchors(2, 3), (hit(2, "kf2"),))
    second = active.grid()
    assert second is not None and second.grid_id != first.grid_id
    assert second.map_ref == first.map_ref == MAP
    assert active.map_generation == nonce
    assert (second.width, second.height) == (3, 2)
    assert second.cells == bytes([100, 255, 100, 255, 255, 255])
    request = {**asdict(MAP), "tile_x": 0, "tile_y": 0}
    assert view.read("map.tile", {**request, "grid_id": first.grid_id}) == {
        "ok": False,
        "error": "conflict",
    }
    assert view.read("map.tile", {**request, "grid_id": second.grid_id})["cells"] == [
        100,
        -1,
        100,
        -1,
        -1,
        -1,
    ]
    active.commit(source, 3, "kf3", anchors(3, 3), ())
    assert active.grid() is second
    assert active.map_generation == nonce


def test_duplicate_preserves_cut_but_gap_and_invalid_extent_end_session() -> None:
    # Catches mixed cuts, allocating an unchecked extent, and resurrection after a gap.
    unbound = ActiveMap(MAP, "wheel_odom", 0.5)
    with pytest.raises(DomainError):
        unbound.bind_verified_source("graph-a", backend_boot="graph-a", source_epoch="scanner-a")
    active, source = session()
    active.commit(source, 1, "kf1", anchors(1), (hit(0, "kf1"),))
    first = active.grid()
    with pytest.raises(DomainError):
        active.commit(source, 1, "kf1", anchors(1), ())
    assert active.grid() is first
    with pytest.raises(DomainError):
        active.commit(object(), 2, "kf2", anchors(2), ())
    assert active.grid() is first
    with pytest.raises(DomainError):
        active.commit(source, 3, "kf3", anchors(3), ())
    assert active.grid() is None
    with pytest.raises(DomainError):
        active.commit(source, 2, "kf2", anchors(2), ())
    with pytest.raises(DomainError):
        active.bind_verified_source(source, backend_boot="graph-a", source_epoch="scanner-a")

    bounded, token = session()
    bounded.commit(token, 1, "kf1", anchors(1), ())
    with pytest.raises(DomainError):
        bounded.commit(token, 2, "kf2", anchors(2, MAX_GRID_SIDE + 1), ())
    assert bounded.grid() is None
    with pytest.raises(DomainError):
        bounded.commit(token, 2, "kf2", anchors(2), ())


def test_revoked_view_cannot_read_or_write_recreated_same_reference() -> None:
    # Catches old tiles/source handles surviving a same-MapRef activation.
    old, source = session()
    old_view = WorkspaceMap(old.grid)
    old.commit(source, 1, "kf1", anchors(1), (hit(0, "kf1"),))
    previous = old.grid()
    assert previous is not None
    old.revoke(source)
    assert old_view.read("map.active", {}) == {"ok": False, "error": "unavailable"}
    assert old_view.read(
        "map.tile", {**asdict(MAP), "grid_id": previous.grid_id, "tile_x": 0, "tile_y": 0}
    ) == {"ok": False, "error": "unavailable"}

    new, current_source = session()
    with pytest.raises(DomainError):
        new.commit(source, 1, "kf1", anchors(1), ())
    with pytest.raises(DomainError):
        old.commit(source, 2, "kf2", anchors(2), ())
    assert new.grid() is None
    new.commit(current_source, 1, "kf1", anchors(1), (hit(0, "kf1"),))
    current = new.grid()
    assert current is not None
    assert current.map_ref == previous.map_ref
    assert current.grid_id != previous.grid_id
    assert WorkspaceMap(new.grid).read(
        "map.tile", {**asdict(MAP), "grid_id": previous.grid_id, "tile_x": 0, "tile_y": 0}
    ) == {"ok": False, "error": "conflict"}


def test_home_binding_update_keeps_the_live_observation_stream_and_reference() -> None:
    from senior_mapping.references import AnchoredWaypoint, WaypointBinding

    active, source = session()
    active.commit(source, 1, "kf1", anchors(1), (hit(0, "kf1"),))
    before = active.grid()
    generation = active.map_generation
    home = WaypointBinding(AnchoredWaypoint("home", Pose2D(0.25, 0.25, 0.2), "kf1", False), POSE)
    active.update_waypoints(source, expected_sequence=1, changes=(home,))
    active.commit(source, 2, "kf2", anchors(2), ())
    assert active.grid() is before
    assert active.map_generation == generation
    with pytest.raises(DomainError):
        active.update_waypoints(source, expected_sequence=1, changes=(home,))


def fixed_episode_setup(*, legacy_base=None):
    """Synthetic admitted evidence only: no physical stability classification."""
    from senior_mapping.evidence import ProfileProvenance, SourceIdentity
    from senior_mapping.shared_scan import ScanKey

    active, token = session()
    active.commit(token, 1, "kf1", anchors(1, 8), ())
    identity = SourceIdentity.from_key(
        ScanKey("graph-a", "bridge", "scanner-a", 1, 1, "01", "02", 0)
    )
    state = {"accepting": True, "now": 1_000_000_100}
    active.bind_fixed_context(
        token,
        source_identity=identity,
        host_boot_id="host",
        acceptance_floor=None,
        acceptance_floor_steady_ns=0,
        accepting=lambda: state["accepting"],
        clock=lambda: state["now"],
        profiles=(ProfileProvenance("synthetic-admitted", "a" * 64, {}),),
        legacy_base=legacy_base,
    )
    return active, token, identity, state


def accepted_fixed_scan(active, token, state, seq, anchor="kf1", endpoint=(3.75, 0.75)):
    from senior_mapping.evidence import VerifiedBasis
    from senior_mapping.shared_scan import Beam, QualifiedScan, ScanKey
    from senior_perception.vision.raw_capture import RawCaptureHeader

    from vehicle_core.sample_time import SampleTime

    t = 1_000_000_000 + seq * 1_000_000
    state["now"] = t + 1
    capture = RawCaptureHeader(
        SampleTime(t, t, t, t, 1), "laser", "01", "02", "scanner-a", "bridge", 1, seq, True, ""
    )
    scan = QualifiedScan(
        ScanKey.of("graph-a", capture),
        capture,
        "synthetic",
        POSE,
        (Beam(0, (0.0, 0.0), endpoint, "hit"),),
        t + 500_000_000,
        "host",
    )
    cut = active.fixed_snapshot(token)
    pose = next(a.pose for a in cut.anchors if a.anchor_id == anchor)
    basis = VerifiedBasis(
        anchor,
        pose,
        POSE,
        cut.watermark.graph_sequence,
        cut.watermark.anchor_digest,
        MAP,
        active.map_generation,
    )
    active.accept_scan(token, scan, basis)
    return scan, active.fixed_snapshot(token)


def test_fixed_three_state_lifecycle_episode():
    from dataclasses import replace

    from senior_mapping.evidence import (
        FixedDelta,
        FreeTile,
        StableSupport,
        SupportCellRemoval,
        SupportRevocation,
    )
    from test_map_references import assert_fixed_episode_codec, fixed_episode_legacy_base

    active, token, identity, state = fixed_episode_setup(legacy_base=fixed_episode_legacy_base(MAP))

    def apply(seq, build=lambda key, cut: {}, anchor="kf1"):
        scan, cut = accepted_fixed_scan(active, token, state, seq, anchor)
        delta = FixedDelta(cut.watermark, scan.key, seq, anchor, **build(scan.key, cut))
        from senior_mapping.references import decode_fixed_delta, encode_fixed_delta

        delta = decode_fixed_delta(encode_fixed_delta(delta))
        return active.apply_evidence(
            token,
            expected_graph_sequence=cut.watermark.graph_sequence,
            expected_anchor_digest=cut.watermark.anchor_digest,
            delta=delta,
        )

    def tile(key, revision, cells, anchor="kf1"):
        mask = bytearray(128)
        for x, y in cells:
            bit = 32 * y + x
            mask[bit // 8] |= 1 << (bit % 8)
        return FreeTile(anchor, 0.5, 0, 0, bytes(mask), revision, key, identity, key.sequence)

    names = {name: active.allocate_support_id(token) for name in ("wall", "object")}

    def support(name, cells, key):
        endpoints = tuple((x * 0.5 + 0.25, y * 0.5 + 0.25) for x, y in cells)
        if name == "wall":
            endpoints = (endpoints[0],) * 32 + endpoints[1:]
        return StableSupport(
            names[name],
            "kf1",
            0.5,
            endpoints[:32],
            endpoints,
            cells,
            key,
            key,
            identity,
            "geometric",
            "synthetic-admitted",
            "a" * 64,
            key.sequence,
            key.sequence,
        )

    old_free = apply(1, lambda k, c: {"free_tiles": (tile(k, 1, ((1, 0), (2, 0), (5, 0))),)})
    assert old_free.grid.cells[:8] == bytes([255, 0, 0, 255, 255, 0, 255, 100])
    occupied = apply(
        2,
        lambda k, c: {
            "upsert_supports": (
                support("wall", ((1, 0), (3, 0), (4, 0)), k),
                support("object", ((1, 0), (2, 0)), k),
            )
        },
    )
    assert occupied.grid.cells[:8] == bytes([255, 100, 100, 100, 100, 0, 255, 100])
    moved = apply(
        3,
        lambda k, c: {
            "revocations": (SupportRevocation(names["object"], 2, "motion_evidenced", k, "kf1"),)
        },
    )
    assert moved.grid.cells[:8] == bytes([255, 100, 255, 100, 100, 0, 255, 100])
    assert not moved.legacy_base.live_mask[0] & 4
    assert moved.free_tiles[0].active_mask[0] == 32  # old free cannot revive
    occluded = apply(4)
    assert occluded.grid is moved.grid  # no geometry/bytes copy on evidence-only update
    assert occluded.watermark.graph_sequence == 1
    assert occluded.watermark.fixed_revision > moved.watermark.fixed_revision
    # Full CAS and source token; rejected transactions leave the complete cut unchanged.
    scan, cut = accepted_fixed_scan(active, token, state, 5, endpoint=(4.25, 0.75))
    # Natural scan extent changes fixed/dirty only, and repeating it is a no-op.
    expanded = active.ensure_observed_extent(
        token, expected=cut.watermark, bounds=(0.0, 0.0, 4.25, 0.75)
    )
    assert expanded.grid.width == 9
    assert expanded.watermark.evidence_key == cut.watermark.evidence_key
    assert expanded.watermark.graph_sequence == cut.watermark.graph_sequence
    assert expanded.watermark.change_sequence == cut.watermark.change_sequence + 1
    assert (
        active.ensure_observed_extent(
            token, expected=expanded.watermark, bounds=(0.0, 0.0, 4.25, 0.75)
        )
        is expanded
    )
    cut = expanded
    # A complete over-budget upgrade is rejected after staging, retaining walls,
    # old-free invalidation and every watermark; it does not retire Q intake.
    oversized = tuple(
        StableSupport(
            active.allocate_support_id(token),
            "kf1",
            0.5,
            ((3.25, 0.25),) * 32,
            ((3.25, 0.25),) * 2048,
            ((6, 0),),
            scan.key,
            scan.key,
            identity,
            "geometric",
            "synthetic-admitted",
            "a" * 64,
            5,
            5,
        )
        for _ in range(4)
    )
    with pytest.raises(DomainError, match="fixed_capacity"):
        active.apply_evidence(
            token,
            expected_graph_sequence=1,
            expected_anchor_digest=cut.watermark.anchor_digest,
            delta=FixedDelta(cut.watermark, scan.key, 5, "kf1", upsert_supports=oversized),
        )
    assert active.fixed_snapshot(token) == cut
    stale = FixedDelta(occupied.watermark, scan.key, 5, "kf1")
    with pytest.raises(DomainError):
        active.apply_evidence(
            token,
            expected_graph_sequence=1,
            expected_anchor_digest=cut.watermark.anchor_digest,
            delta=stale,
        )
    assert active.fixed_snapshot(token) == cut
    with pytest.raises(DomainError):
        active.ensure_observed_extent(object(), expected=cut.watermark, bounds=(0, 0, 4, 1))
    with pytest.raises(DomainError):
        active.ensure_observed_extent(token, expected=cut.watermark, bounds=(-999, 0, 4, 1))
    with pytest.raises(DomainError), active._store.freeze_for_capture():
        with pytest.raises(DomainError):
            active.apply_evidence(
                token,
                expected_graph_sequence=1,
                expected_anchor_digest=cut.watermark.anchor_digest,
                delta=FixedDelta(cut.watermark, scan.key, 5, "kf1"),
            )
    # Original graph rule: exactly one new anchor; correction changes R, never graph spoofing.
    corrected = AnchorSnapshot(
        MAP,
        "wheel_odom",
        (AnchorPose("kf1", Pose2D(0.5, 0, 0), True), AnchorPose("kf2", POSE, True)),
        GridExtent(9, 3, 0, 0),
    )
    active.commit(token, 2, "kf2", corrected, ())
    partial = apply(
        6,
        lambda k, c: {
            "support_cell_removals": (
                SupportCellRemoval(names["wall"], 2, ((3, 0),), "observed_free_space", k, "kf2"),
            ),
            "free_tiles": (tile(k, 1, ((4, 0),), "kf2"),),
        },
        "kf2",
    )
    assert partial.grid.cells[2] == 100  # remaining independently supported wall
    assert partial.grid.cells[4] == 0
    assert partial.grid.cells[5] == 100
    wall = next(s for s in partial.supports if s.support_id == names["wall"])
    assert wall.anchor_id == "kf1" and wall.local_cells == ((1, 0), (4, 0))
    assert len(wall.local_endpoints) == 33
    # Rotation of a free tile: target boundaries require full coverage.
    rotated = AnchorSnapshot(
        MAP,
        "wheel_odom",
        corrected.anchors + (AnchorPose("kf3", Pose2D(0, 0, 0.35), True),),
        GridExtent(9, 3, 0, 0),
    )
    active.commit(token, 3, "kf3", rotated, ())
    rotated_cut = apply(7, lambda k, c: {"free_tiles": (tile(k, 1, ((0, 0),), "kf3"),)}, "kf3")
    assert rotated_cut.grid.cells[0] == 255
    assert_fixed_episode_codec(rotated_cut)
    # Explicit V2 opt-in cannot silently pass through the production V1 saver.
    from pathlib import Path

    with pytest.raises(DomainError):
        active._store.snapshot(Path("/unused"), ())
    # Observed retirement permanently fences a previously accepted file fact.
    state["accepting"] = False
    with pytest.raises(DomainError):
        accepted_fixed_scan(active, token, state, 8, "kf3")
    state["accepting"] = True
    with pytest.raises(DomainError):
        active.accept_scan(token, scan, replace(cut.basis))


def repair_graph_correction(active, token, sequence, x):
    from dataclasses import replace

    cut = active.fixed_snapshot(token)
    corrected = tuple(
        replace(a, pose=Pose2D(x, 0.0, 0.0)) if a.anchor_id == "kf1" else a for a in cut.anchors
    ) + (AnchorPose("kf" + str(sequence), POSE, True),)
    active.commit(
        token,
        sequence,
        "kf" + str(sequence),
        AnchorSnapshot(MAP, "wheel_odom", corrected, GridExtent(8, 2, 0, 0)),
        (),
    )


def repair_wall(active, token, identity, key, cells):
    from senior_mapping.evidence import StableSupport

    points = tuple((x * 0.5 + 0.25, y * 0.5 + 0.25) for x, y in cells)
    return StableSupport(
        active.allocate_support_id(token),
        "kf1",
        0.5,
        points,
        points,
        cells,
        key,
        key,
        identity,
        "geometric",
        "synthetic-admitted",
        "a" * 64,
        1,
        1,
    )


def repair_apply(active, token, scan, cut, **parts):
    from senior_mapping.evidence import FixedDelta

    return active.apply_evidence(
        token,
        expected_graph_sequence=cut.watermark.graph_sequence,
        expected_anchor_digest=cut.watermark.anchor_digest,
        delta=FixedDelta(cut.watermark, scan.key, scan.key.sequence, "kf1", **parts),
    )


def repair_assert_materialization(active, token, cut):
    from senior_mapping.references import decode_fixed_snapshot, encode_fixed_snapshot
    from senior_mapping.workspace import FixedRaster

    assert active.grid() is cut.grid
    raster = FixedRaster(MAP, 0.5, cut.grid.origin, cut.grid.width, cut.grid.height, True, False)
    raster.install(cut.anchors, cut.supports, cut.free_tiles, cut.legacy_base)
    assert raster.grid() == cut.grid
    assert decode_fixed_snapshot(encode_fixed_snapshot(cut)) == cut


def test_fixed_bound_legacy_partial_free_never_revives():
    from senior_mapping.evidence import SupportRevocation
    from test_map_references import repair_legacy_free_base

    active, token, identity, state = fixed_episode_setup(legacy_base=repair_legacy_free_base(MAP))
    repair_graph_correction(active, token, 2, 0.25)
    shifted = active.fixed_snapshot(token)
    assert shifted.grid.cells[:2] == bytes([255, 255])  # Partial coverage cannot render F.
    scan, cut = accepted_fixed_scan(active, token, state, 1)
    wall = repair_wall(active, token, identity, scan.key, ((0, 0),))
    occupied = repair_apply(active, token, scan, cut, upsert_supports=(wall,))
    assert occupied.legacy_base.live_mask == b"\xfe\xff"
    assert occupied.grid.cells[:2] == bytes([100, 100])
    repair_assert_materialization(active, token, occupied)
    scan, cut = accepted_fixed_scan(active, token, state, 2)
    cleared = repair_apply(
        active,
        token,
        scan,
        cut,
        revocations=(SupportRevocation(wall.support_id, 1, "motion_evidenced", scan.key, "kf1"),),
    )
    assert cleared.grid.cells[:2] == bytes([255, 255])
    repair_graph_correction(active, token, 3, 0.0)
    end = active.fixed_snapshot(token)
    assert end.legacy_base.live_mask == b"\xfe\xff"
    assert end.grid.cells[:2] == bytes([255, 255])
    repair_assert_materialization(active, token, end)


def test_fixed_unbound_legacy_offset_clears_all_overlaps():
    from test_map_references import repair_legacy_free_base

    active, token, identity, state = fixed_episode_setup(
        legacy_base=repair_legacy_free_base(MAP, offset=True),
    )
    assert active.grid().cells[:8] == bytes([255, 0, 0, 0, 0, 0, 0, 0])
    scan, cut = accepted_fixed_scan(active, token, state, 1)
    wall = repair_wall(active, token, identity, scan.key, ((2, 0),))
    result = repair_apply(active, token, scan, cut, upsert_supports=(wall,))
    assert result.legacy_base.live_mask == b"\xf9\xff"
    assert result.grid.cells[:8] == bytes([255, 255, 100, 255, 0, 0, 0, 0])
    repair_assert_materialization(active, token, result)


def test_fixed_legacy_quad_caller_mutation_cannot_change_cut():
    from senior_mapping.references import encode_fixed_snapshot, fixed_payload_size
    from test_map_references import repair_legacy_free_base

    quad = [0.0, 0.0, 0.5, 0.0, 0.5, 0.5, 0.0, 0.5]
    base = repair_legacy_free_base(MAP, mutable_quad=quad)
    active, token, _, _ = fixed_episode_setup(legacy_base=base)
    cut = active.fixed_snapshot(token)
    cache = {}
    before, size = encode_fixed_snapshot(cut), fixed_payload_size(cut, cache)
    quad[0] = 100.0
    assert active.fixed_snapshot(token) is cut
    assert cut.legacy_base.anchor_bindings[0].local_quads[0] == (
        0.0,
        0.0,
        0.5,
        0.0,
        0.5,
        0.5,
        0.0,
        0.5,
    )
    assert encode_fixed_snapshot(cut) == before
    assert fixed_payload_size(cut, cache) == size == len(before)
    # V1 CellEvidence remains permissive; the new C3 legacy boundary rejects its list alias.
    with pytest.raises(ValueError, match="immutable"):
        repair_legacy_free_base(MAP, mutable_cell=[0, 0])


def test_fixed_three_cell_dirty_union_is_per_projection(monkeypatch):
    from dataclasses import replace

    class Counted(frozenset):
        calls = 0
        copied = 0

        def __or__(self, other):
            Counted.calls += 1
            Counted.copied += len(self) + len(other)
            return super().__or__(other)

    active, token, identity, state = fixed_episode_setup()
    raster = active._store._fixed_raster
    project = raster.support_projection

    def counted(support, pose):
        result = project(support, pose)
        return replace(result, occupied=Counted(result.occupied))

    monkeypatch.setattr(raster, "support_projection", counted)
    scan, cut = accepted_fixed_scan(active, token, state, 1)
    wall = repair_wall(active, token, identity, scan.key, ((1, 0), (3, 0), (4, 0)))
    result = repair_apply(active, token, scan, cut, upsert_supports=(wall,))
    assert result.grid.cells[:8] == bytes([255, 100, 255, 100, 100, 255, 255, 255])
    first_calls, first_members = Counted.calls, Counted.copied

    class Contributors:
        copies = 0

        def __init__(self, members):
            self.members = members

        def __iter__(self):
            Contributors.copies += 1
            return iter(self.members)

    # Two contributors touch the same three cells in one real stage. Reuse the
    # wall geometry; copying the original reverse set is needed only once/cell.
    monkeypatch.setattr(
        raster, "reverse", {cell: Contributors(members) for cell, members in raster.reverse.items()}
    )
    scan, cut = accepted_fixed_scan(active, token, state, 2)
    refreshed = replace(wall, last_key=scan.key, updated_evidence_revision=2)
    independent = replace(
        repair_wall(active, token, identity, scan.key, ((1, 0), (3, 0), (4, 0))),
        created_evidence_revision=2,
        updated_evidence_revision=2,
    )
    result = repair_apply(active, token, scan, cut, upsert_supports=(refreshed, independent))
    assert result.grid is cut.grid
    print(
        f"three-cell union: {first_calls} calls, {first_members} members; "
        f"overlapping stage reverse copies: {Contributors.copies}"
    )
    assert Contributors.copies == 3
    assert first_calls <= 3
    assert first_members <= 9
