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
