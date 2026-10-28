"""Stable occupancy is confirmed and cleared only by repeated, explicit evidence."""

import math
from dataclasses import fields

from senior_control.mapping import (
    CellEvidence,
    MapDelta,
    MappingObservation,
    OccupancyMaintainer,
    OccupancySnapshot,
    OccupancyThresholds,
    RangeRay,
    SceneHit,
)
from senior_control.models import MapRef, Pose2D

CELL_SIZE = 0.5
ORIGIN = Pose2D(0.25, 0.25, 0.0)
MAP = MapRef("home", 1)
BOX = (1.25, 0.25)
OUTLIER = (1.375, 0.25)
NEAR_B = (1.3125, 0.25)
NEAR_C = (1.1875, 0.25)
PERSON = (0.75, 0.25)
MOVING = (0.25, 1.25)
WALL = (2.25, 0.25)
FAR_RANGE = 2.5


def cell_of(x: float, y: float) -> tuple[int, int]:
    """Cell containing the point, using floor division by the configured size."""
    return (math.floor(x / CELL_SIZE), math.floor(y / CELL_SIZE))


def thresholds(**overrides: object) -> OccupancyThresholds:
    values: dict[str, object] = {
        "cell_size_m": CELL_SIZE,
        "confirm_hits": 3,
        "confirm_span_s": 0.4,
        "clear_free": 3,
        "clear_span_s": 0.4,
        "distance_tolerance_m": 0.1,
        "hit_weight": 1.0,
        "free_weight": 1.0,
        "occupied_belief": 2.5,
        "free_belief": 0.5,
        "belief_min": 0.0,
        "belief_max": 6.0,
        "max_age_s": 0.5,
        "min_quality": 0.5,
    }
    values.update(overrides)
    return OccupancyThresholds(**values)  # type: ignore[arg-type]


def beam(point: tuple[float, float]) -> RangeRay:
    distance = math.dist((ORIGIN.x, ORIGIN.y), point)
    return RangeRay(0.0, distance, True, False, None)


def observe(
    stamp: float,
    *,
    hits: tuple[SceneHit, ...] = (),
    rays: tuple[RangeRay, ...] = (),
    dynamic_mask: tuple[bool, ...] | None = None,
    occlusion_mask: tuple[bool, ...] | None = None,
    anchor: str = "kf",
    quality: float = 1.0,
    localization_trusted: bool = True,
    stale: bool = False,
    age_s: float | None = 0.0,
    map_ref: MapRef = MAP,
    pose: Pose2D = ORIGIN,
) -> MappingObservation:
    return MappingObservation(
        map_ref,
        anchor,
        pose,
        stamp,
        rays,
        hits,
        tuple(False for _ in rays) if dynamic_mask is None else dynamic_mask,
        tuple(False for _ in rays) if occlusion_mask is None else occlusion_mask,
        quality,
        localization_trusted,
        stale,
        age_s,
    )


def assert_no_velocity(value: object) -> None:
    assert {item.name for item in fields(value)}.isdisjoint({"vx", "wz", "velocity"})


def assert_delta_idle(delta: MapDelta) -> None:
    assert isinstance(delta, MapDelta)
    assert delta.added == ()
    assert delta.removed == ()
    assert_no_velocity(delta)


def only_cell(snapshot: OccupancySnapshot) -> CellEvidence:
    assert isinstance(snapshot, OccupancySnapshot)
    assert_no_velocity(snapshot)
    assert len(snapshot.cells) == 1
    return snapshot.cells[0]


def test_stable_box_added_after_confirmation() -> None:
    once = OccupancyMaintainer(thresholds())
    burst = once.update(
        observe(
            0.0,
            hits=(SceneHit(*BOX, "static"),) * 3,
            rays=(beam(BOX),) * 3,
            anchor="once",
        )
    )
    assert_delta_idle(burst)
    assert once.snapshot().cells == ()
    assert once.snapshot().map_ref is None

    kept = OccupancyMaintainer(thresholds())
    samples = (
        (0.0, BOX, "box-0"),
        (0.25, OUTLIER, "box-outlier"),
        (0.5, NEAR_B, "box-1"),
        (1.0, NEAR_C, "box-2"),
    )
    early = [
        kept.update(
            observe(stamp, hits=(SceneHit(x, y, "static"),), rays=(beam((x, y)),), anchor=anchor)
        )
        for stamp, (x, y), anchor in samples[:3]
    ]
    assert all(delta.added == () and delta.removed == () for delta in early)
    assert kept.snapshot().cells == ()

    stamp, (x, y), anchor = samples[3]
    added = kept.update(
        observe(stamp, hits=(SceneHit(x, y, "static"),), rays=(beam((x, y)),), anchor=anchor)
    )
    assert len(added.added) == 1
    assert added.removed == ()
    assert_no_velocity(added)
    evidence = added.added[0]
    assert isinstance(evidence, CellEvidence)
    assert evidence.cell == cell_of(*BOX)
    assert evidence.anchor_id == "box-2"
    assert evidence.hit_count == 3
    assert evidence.free_count == 0
    assert evidence.last_valid_time == 1.0
    assert evidence.belief == 3.0
    assert evidence.change_source == "geometric_confirmation"

    stored = only_cell(kept.snapshot())
    assert kept.snapshot().map_ref == MAP
    assert stored == evidence

    for index, stamp in enumerate((2.0, 3.0, 4.0, 5.0)):
        extra = kept.update(
            observe(
                stamp,
                hits=(SceneHit(*BOX, "static"),),
                rays=(beam(BOX),),
                anchor=f"box-extra-{index}",
            )
        )
        assert extra.added == ()
        assert extra.removed == ()
    saturated = only_cell(kept.snapshot())
    assert saturated.belief == 6.0
    assert saturated.hit_count == 7
    assert saturated.change_source == "geometric_confirmation"
    assert saturated.anchor_id == "box-extra-3"


def test_removed_box_cleared_after_free_evidence() -> None:
    maintainer = OccupancyMaintainer(thresholds())
    for index, stamp in enumerate((0.0, 0.5, 1.0)):
        maintainer.update(
            observe(
                stamp, hits=(SceneHit(*BOX, "static"),), rays=(beam(BOX),), anchor=f"box-{index}"
            )
        )
    confirmed = only_cell(maintainer.snapshot())
    assert confirmed.cell == cell_of(*BOX)
    assert confirmed.free_count == 0

    aged = maintainer.update(observe(30.0, anchor="time-only"))
    assert_delta_idle(aged)
    assert only_cell(maintainer.snapshot()).last_valid_time == confirmed.last_valid_time
    assert only_cell(maintainer.snapshot()).free_count == 0

    free_ray = RangeRay(0.0, FAR_RANGE, True, False, None)
    burst = maintainer.update(observe(31.0, rays=(free_ray,) * 3, anchor="free-burst"))
    assert_delta_idle(burst)
    after_burst = only_cell(maintainer.snapshot())
    assert after_burst.free_count == 1
    assert after_burst.belief == 2.0
    assert after_burst.change_source == "geometric_confirmation"

    too_soon = maintainer.update(observe(31.2, rays=(free_ray,), anchor="free-1"))
    assert_delta_idle(too_soon)
    held = only_cell(maintainer.snapshot())
    assert held.free_count == 2
    assert held.belief == 1.0

    cleared = maintainer.update(observe(31.5, rays=(free_ray,), anchor="free-2"))
    assert cleared.added == ()
    assert len(cleared.removed) == 1
    assert_no_velocity(cleared)
    removed = cleared.removed[0]
    assert removed.cell == cell_of(*BOX)
    assert removed.anchor_id == "free-2"
    assert removed.hit_count == 3
    assert removed.free_count == 3
    assert removed.last_valid_time == 31.5
    assert removed.belief == 0.0
    assert removed.change_source == "observed_free_space"
    assert maintainer.snapshot().cells == ()
    assert maintainer.snapshot().map_ref == MAP

    bounded = OccupancyMaintainer(
        thresholds(belief_min=-1.0, free_belief=-0.5, clear_free=4, free_weight=2.0)
    )
    for index, stamp in enumerate((0.0, 0.5, 1.0)):
        bounded.update(
            observe(stamp, hits=(SceneHit(*BOX, "static"),), rays=(beam(BOX),), anchor=f"b-{index}")
        )
    for index, stamp in enumerate((2.0, 2.5, 3.0)):
        held_delta = bounded.update(observe(stamp, rays=(free_ray,), anchor=f"low-{index}"))
        assert_delta_idle(held_delta)
    clamped = only_cell(bounded.snapshot())
    assert clamped.belief == -1.0
    assert clamped.free_count == 3
    released = bounded.update(observe(3.5, rays=(free_ray,), anchor="low-3"))
    assert len(released.removed) == 1
    assert released.removed[0].belief == -1.0
    assert released.removed[0].change_source == "observed_free_space"


def test_person_standing_excluded() -> None:
    maintainer = OccupancyMaintainer(thresholds())
    for index, stamp in enumerate((0.0, 0.5, 1.0)):
        maintainer.update(
            observe(
                stamp, hits=(SceneHit(*WALL, "static"),), rays=(beam(WALL),), anchor=f"wall-{index}"
            )
        )
    wall = only_cell(maintainer.snapshot())
    assert wall.cell == cell_of(*WALL)
    assert wall.belief == 3.0

    for index, stamp in enumerate((2.0, 2.5, 3.0, 3.5)):
        delta = maintainer.update(
            observe(
                stamp,
                hits=(SceneHit(*PERSON, "person"), SceneHit(*MOVING, "moving")),
                rays=(beam(PERSON),),
                dynamic_mask=(True,),
                anchor=f"person-{index}",
            )
        )
        assert_delta_idle(delta)

    remaining = only_cell(maintainer.snapshot())
    assert remaining.cell == cell_of(*WALL)
    assert remaining.hit_count == wall.hit_count
    assert remaining.free_count == 0
    assert remaining.belief == wall.belief
    assert remaining.last_valid_time == wall.last_valid_time
    assert remaining.change_source == "geometric_confirmation"
    assert cell_of(*PERSON) != remaining.cell
    assert cell_of(*MOVING) != remaining.cell


def test_occlusion_does_not_clear() -> None:
    maintainer = OccupancyMaintainer(thresholds())
    for index, stamp in enumerate((0.0, 0.5, 1.0)):
        maintainer.update(
            observe(
                stamp, hits=(SceneHit(*WALL, "static"),), rays=(beam(WALL),), anchor=f"wall-{index}"
            )
        )
    wall = only_cell(maintainer.snapshot())
    free_ray = RangeRay(0.0, FAR_RANGE, True, False, None)
    blocked = (
        observe(2.0, rays=(free_ray,), occlusion_mask=(True,), anchor="occluded"),
        observe(2.5, rays=(RangeRay(0.0, math.nan, False, False, None),), anchor="invalid-depth"),
        observe(3.0, rays=(free_ray,), stale=True, anchor="stale"),
        observe(3.5, rays=(free_ray,), localization_trusted=False, anchor="untrusted"),
        observe(4.0, rays=(free_ray,), age_s=10.0, anchor="old"),
        observe(4.5, rays=(free_ray,), age_s=None, anchor="untimed"),
        observe(5.0, rays=(free_ray,), quality=0.49, anchor="poor"),
        observe(5.5, rays=(free_ray,), quality=math.nan, anchor="bad-quality"),
        observe(6.0, rays=(RangeRay(0.0, math.inf, False, False, 3.0),), anchor="no-return"),
        observe(6.5, rays=(free_ray,), map_ref=MapRef("other", 1), anchor="other-map"),
        observe(7.0, rays=(free_ray,), dynamic_mask=(True,), anchor="masked-clear"),
    )
    for sample in blocked:
        assert_delta_idle(maintainer.update(sample))
    preserved = only_cell(maintainer.snapshot())
    assert preserved.cell == wall.cell
    assert preserved.free_count == 0
    assert preserved.belief == wall.belief
    assert preserved.last_valid_time == wall.last_valid_time
    assert maintainer.snapshot().map_ref == MAP

    unsupported = RangeRay(0.0, math.inf, False, False, 3.0)
    for index, stamp in enumerate((8.0, 8.5, 9.0)):
        assert_delta_idle(
            maintainer.update(observe(stamp, rays=(unsupported,), anchor=f"unsupported-{index}"))
        )
    assert only_cell(maintainer.snapshot()).free_count == 0

    supported = RangeRay(0.0, math.inf, False, True, 3.0)
    assert_delta_idle(maintainer.update(observe(10.0, rays=(supported,), anchor="model-0")))
    assert_delta_idle(maintainer.update(observe(10.5, rays=(supported,), anchor="model-1")))
    cleared = maintainer.update(observe(11.0, rays=(supported,), anchor="model-2"))
    assert len(cleared.removed) == 1
    assert cleared.removed[0].cell == wall.cell
    assert cleared.removed[0].change_source == "observed_free_space"
    assert cleared.removed[0].free_count == 3
    assert maintainer.snapshot().cells == ()


def test_single_miss_preserves_wall() -> None:
    maintainer = OccupancyMaintainer(thresholds())
    for index, stamp in enumerate((0.0, 0.5, 1.0)):
        maintainer.update(
            observe(
                stamp, hits=(SceneHit(*WALL, "static"),), rays=(beam(WALL),), anchor=f"wall-{index}"
            )
        )
    confirmed = only_cell(maintainer.snapshot())
    miss = maintainer.update(
        observe(2.0, rays=(RangeRay(0.0, FAR_RANGE, True, False, None),), anchor="miss")
    )
    assert_delta_idle(miss)
    after_miss = only_cell(maintainer.snapshot())
    assert after_miss.cell == confirmed.cell
    assert after_miss.anchor_id == "miss"
    assert after_miss.hit_count == confirmed.hit_count
    assert after_miss.free_count == 1
    assert after_miss.last_valid_time == 2.0
    assert after_miss.belief == 2.0
    assert after_miss.belief > 0.5
    assert after_miss.change_source == "geometric_confirmation"

    later = maintainer.update(observe(80.0, anchor="clock"))
    assert_delta_idle(later)
    kept = only_cell(maintainer.snapshot())
    assert kept.free_count == 1
    assert kept.last_valid_time == 2.0
    assert kept.belief == 2.0
    assert kept.cell == confirmed.cell


def test_unknown_dynamic_tracks_excluded() -> None:
    dynamic = OccupancyMaintainer(thresholds())
    for index, stamp in enumerate((0.0, 0.5, 1.0, 1.5)):
        delta = dynamic.update(
            observe(
                stamp,
                hits=(SceneHit(*BOX, "unknown_dynamic"),),
                rays=(beam(BOX),),
                dynamic_mask=(True,),
                anchor=f"track-{index}",
            )
        )
        assert_delta_idle(delta)
    assert dynamic.snapshot().cells == ()

    static = OccupancyMaintainer(thresholds())
    early = static.update(
        observe(0.0, hits=(SceneHit(*BOX, "unknown_static"),), rays=(beam(BOX),), anchor="static-0")
    )
    assert_delta_idle(early)
    assert static.snapshot().cells == ()
    static.update(
        observe(0.5, hits=(SceneHit(*BOX, "unknown_static"),), rays=(beam(BOX),), anchor="static-1")
    )
    assert static.snapshot().cells == ()
    added = static.update(
        observe(1.0, hits=(SceneHit(*BOX, "unknown_static"),), rays=(beam(BOX),), anchor="static-2")
    )
    assert len(added.added) == 1
    evidence = added.added[0]
    assert evidence.cell == cell_of(*BOX)
    assert evidence.change_source == "geometric_confirmation"
    assert evidence.hit_count == 3
    assert evidence.belief == 3.0
    assert evidence.anchor_id == "static-2"
    assert only_cell(static.snapshot()) == evidence


def test_missing_or_nonfinite_thresholds_fail_closed() -> None:
    sequence = []
    for index, stamp in enumerate((0.0, 0.5, 1.0)):
        sequence.append(
            observe(
                stamp, hits=(SceneHit(*BOX, "static"),), rays=(beam(BOX),), anchor=f"add-{index}"
            )
        )
    free_ray = RangeRay(0.0, FAR_RANGE, True, False, None)
    for index, stamp in enumerate((2.0, 2.5, 3.0)):
        sequence.append(observe(stamp, rays=(free_ray,), anchor=f"clear-{index}"))

    working = OccupancyMaintainer(thresholds())
    working_deltas = [working.update(sample) for sample in sequence]
    assert any(delta.added for delta in working_deltas)
    assert any(delta.removed for delta in working_deltas)

    bad_configs: tuple[OccupancyThresholds | None, ...] = (
        None,
        OccupancyThresholds(),
        thresholds(confirm_hits=None),
        thresholds(clear_free=None),
        thresholds(cell_size_m=float("nan")),
        thresholds(confirm_span_s=float("inf")),
        thresholds(clear_span_s=float("-inf")),
        thresholds(hit_weight=float("nan")),
        thresholds(free_weight=float("nan")),
        thresholds(occupied_belief=float("nan")),
        thresholds(free_belief=float("nan")),
        thresholds(distance_tolerance_m=float("nan")),
        thresholds(max_age_s=float("inf")),
        thresholds(min_quality=float("nan")),
        thresholds(belief_min=float("nan")),
        thresholds(belief_max=float("nan")),
        thresholds(cell_size_m=0.0),
        thresholds(confirm_hits=1),
        thresholds(clear_free=1),
        thresholds(confirm_span_s=0.0),
        thresholds(clear_span_s=0.0),
        thresholds(confirm_hits=True),
        thresholds(occupied_belief=1.0),
        thresholds(free_belief=2.0),
    )
    for config in bad_configs:
        maintainer = OccupancyMaintainer(config)
        for sample in sequence:
            assert_delta_idle(maintainer.update(sample))
        snapshot = maintainer.snapshot()
        assert snapshot.cells == ()
        assert snapshot.map_ref is None
        assert_no_velocity(snapshot)
