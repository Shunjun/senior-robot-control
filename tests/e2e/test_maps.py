"""Map persistence acceptance assertions that do not require ROS."""

from __future__ import annotations

import math
from pathlib import Path

from conftest import SimulationProfile

from senior_control.localization.geo import AlignedPose
from senior_control.localization.relocalization import PoseHint, Relocalizer
from senior_control.mapping import (
    CellEvidence,
    MappingObservation,
    OccupancyMaintainer,
    OccupancyThresholds,
    RangeRay,
    SceneHit,
)
from senior_control.mapping.archive import Georeference
from senior_control.mapping.references import SlamSnapshot
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationStatus, Pose2D

MAP = MapRef("acceptance-map", 1)
POSE = Pose2D(0.25, 0.25, 0.0)
BOX = (1.25, 0.25)


def thresholds() -> OccupancyThresholds:
    return OccupancyThresholds(
        cell_size_m=0.5,
        confirm_hits=3,
        confirm_span_s=0.4,
        clear_free=3,
        clear_span_s=0.4,
        distance_tolerance_m=0.1,
        hit_weight=1.0,
        free_weight=1.0,
        occupied_belief=2.5,
        free_belief=0.5,
        belief_min=0.0,
        belief_max=6.0,
        max_age_s=0.5,
        min_quality=0.5,
    )


def observation(
    stamp: float,
    *,
    hits: tuple[SceneHit, ...] = (),
    ray: RangeRay | None = None,
    dynamic: bool = False,
) -> MappingObservation:
    rays = () if ray is None else (ray,)
    return MappingObservation(
        MAP,
        f"kf-{stamp}",
        POSE,
        stamp,
        rays,
        hits,
        (dynamic,) * len(rays),
        (False,) * len(rays),
        1.0,
        True,
        False,
        0.0,
    )


def test_box_added_then_removed_survives_reload(
    tmp_path: Path,
    simulation_profile: SimulationProfile,
) -> None:
    maintainer = OccupancyMaintainer(thresholds())
    box_ray = RangeRay(0.0, math.dist((POSE.x, POSE.y), BOX), True, False, None)
    for stamp in (0.0, 0.5, 1.0):
        maintainer.update(observation(stamp, hits=(SceneHit(*BOX, "static"),), ray=box_ray))
    added = maintainer.snapshot().cells
    assert len(added) == 1

    store = MapReferenceStore(MAP, "wheel_odom", 0.5)
    store.place(added[0], world=BOX, anchor_pose=POSE)
    free_ray = RangeRay(0.0, 2.5, True, False, None)
    removed: tuple[CellEvidence, ...] = ()
    for stamp in (2.0, 2.5, 3.0):
        removed = maintainer.update(observation(stamp, ray=free_ray)).removed
    assert len(removed) == 1
    store.remove(removed[0])

    destination = tmp_path / "snapshot"
    store.snapshot(Path("graph"), ()).write(destination)
    loaded = SlamSnapshot.read(destination)

    assert loaded.occupancy.cells == ()
    assert len(loaded.removed) == 1
    assert loaded.removed[0].cell == removed[0].cell
    assert loaded.removed[0].change_source == "observed_free_space"
    assert loaded.removed[0].anchor_id == added[0].anchor_id
    assert loaded.map_ref == MAP
    assert simulation_profile.database.parent == tmp_path


def test_person_not_persisted() -> None:
    maintainer = OccupancyMaintainer(thresholds())
    person = (0.75, 0.25)
    ray = RangeRay(0.0, 0.5, True, False, None)
    for stamp in (0.0, 0.5, 1.0, 1.5):
        delta = maintainer.update(
            observation(stamp, hits=(SceneHit(*person, "person"),), ray=ray, dynamic=True)
        )
        assert delta.added == ()
    assert maintainer.snapshot().cells == ()


async def test_map_import_with_and_without_gps() -> None:
    calls: list[Pose2D | AlignedPose | None] = []

    def match(seed: Pose2D | AlignedPose | None) -> tuple[Pose2D, bool]:
        calls.append(seed)
        return Pose2D(3.0, 4.0, 0.2), True

    without_gps = Relocalizer(
        map_ref=MAP,
        georeference=Georeference("unreferenced"),
        alignment=None,
        scan_match=match,
        bundle_has_gps=False,
    )
    manual = await without_gps.begin(MAP, PoseHint(Pose2D(2.9, 4.1, 0.2), "manual"))
    assert manual.status is OperationStatus.SUCCEEDED
    assert calls[-1] == Pose2D(2.9, 4.1, 0.2)

    with_gps = Relocalizer(
        map_ref=MAP,
        georeference=Georeference("unreferenced"),
        alignment=None,
        scan_match=match,
        bundle_has_gps=True,
    )
    assert with_gps.ready is False
    assert with_gps.pose is None
