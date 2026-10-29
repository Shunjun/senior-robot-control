"""Qualified GPS alignment, relocalization, and passive heading checks."""

import importlib.util
import inspect
import math
from dataclasses import replace
from pathlib import Path

import pytest

from senior_control.clock import Clock
from senior_control.errors import DomainError, ErrorCode
from senior_control.localization.geo import GeoAlignment, NavigationPosture, QualifiedFix
from senior_control.localization.heading import HeadingObserver, MotionSample
from senior_control.localization.relocalization import PoseHint, Relocalizer
from senior_control.mapping.archive import Georeference
from senior_control.models import MapRef, OperationStatus, Pose2D, Velocity

ROOT = Path(__file__).resolve().parents[2]
MAP = MapRef("yard", 3)
ORIGIN = (31.0, 121.0, 4.0)
FIX_LLA = (31.00018, 121.00005, 6.0)
# WGS84 ENU of FIX_LLA at ORIGIN, then +90° about z and translation (10, -3, 0).
MAP_X = -9.956523767
MAP_Y = 1.775208636
YAW_90 = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
COURSE = -math.pi + 0.1
YAW_NEAR_PI = math.pi - 0.1


class FixedClock:
    def __init__(self, now: float) -> None:
        self._now = now

    def monotonic(self) -> float:
        return self._now


def clock() -> Clock:
    return FixedClock(100.0)


def satellite(**overrides: object) -> QualifiedFix:
    fix = QualifiedFix(
        FIX_LLA[0],
        FIX_LLA[1],
        FIX_LLA[2],
        100.0,
        0.3,
        0.5,
        "fixed",
        2,
        True,
        None,
    )
    return replace(fix, **overrides)  # type: ignore[arg-type]


def referenced() -> Georeference:
    return Georeference(
        "referenced",
        datum="WGS84",
        height_datum="ellipsoidal",
        translation_m=(10.0, -3.0, 0.0),
        rotation=YAW_90,
        heading_convention="enu_math_yaw",
        quality="survey",
        source="control",
    )


def alignment(
    georeference: Georeference,
    *,
    uncertainty: tuple[float, float, float] = (0.4, 0.5, 0.05),
) -> GeoAlignment:
    return GeoAlignment(
        georeference,
        origin_latitude_deg=ORIGIN[0],
        origin_longitude_deg=ORIGIN[1],
        origin_altitude_m=ORIGIN[2],
        uncertainty_m=uncertainty,
        clock=clock(),
        max_age_s=0.5,
        max_horizontal_std_m=3.0,
    )


def along(distance: float) -> tuple[float, float]:
    return distance * math.cos(COURSE), distance * math.sin(COURSE)


def straight(**overrides: object) -> MotionSample:
    east, north = along(8.0)
    sample = MotionSample(
        20.0,
        YAW_NEAR_PI,
        east,
        north,
        east,
        north,
        8.0,
        0.01,
        True,
        True,
        True,
        False,
        False,
    )
    return replace(sample, **overrides)  # type: ignore[arg-type]


def relocator(
    georeference: Georeference,
    match: object,
    *,
    aligned: GeoAlignment | None,
    bundle_has_gps: bool = False,
) -> Relocalizer:
    return Relocalizer(
        map_ref=MAP,
        georeference=georeference,
        alignment=aligned,
        scan_match=match,  # type: ignore[arg-type]
        bundle_has_gps=bundle_has_gps,
    )


def localization_module() -> object:
    path = ROOT / "ros2" / "src" / "senior_robot" / "senior_robot" / "localization_node.py"
    spec = importlib.util.spec_from_file_location("localization_node_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_unreferenced_map_rejects_geo_conversion() -> None:
    geo = alignment(Georeference("unreferenced"))
    with pytest.raises(DomainError) as caught:
        geo.to_map(satellite(absolute_heading_rad=0.4, rtk="fixed"))
    assert caught.value.code == ErrorCode.LOCALIZATION_UNAVAILABLE
    assert "unreferenced" in str(caught.value)


def test_single_fix_no_heading() -> None:
    partial = Georeference("referenced", datum="WGS84", height_datum="ellipsoidal")
    geo = alignment(partial)
    estimate = geo.to_map(satellite(rtk=None, navsat_status=2, absolute_heading_rad=None))
    assert estimate.full_transform is False
    assert estimate.heading_rad is None
    assert estimate.pose is None
    assert estimate.rtk is None
    assert math.isinf(estimate.uncertainty[2])
    assert geo.georeference.rotation is None
    assert geo.georeference.translation_m is None
    explicit = geo.to_map(satellite(rtk="float", navsat_status=0, absolute_heading_rad=1.2))
    assert explicit.rtk == "float"
    assert explicit.heading_rad is None
    assert explicit.full_transform is False


def test_referenced_alignment_converts_enu_and_records_uncertainty() -> None:
    geo = alignment(referenced())
    estimate = geo.to_map(satellite())
    assert estimate.accepted is True
    assert estimate.full_transform is True
    assert estimate.datum == "WGS84"
    assert estimate.height_datum == "ellipsoidal"
    assert estimate.translation_m == (10.0, -3.0, 0.0)
    assert estimate.rotation == pytest.approx(YAW_90)
    assert estimate.pose is not None
    assert estimate.pose.x == pytest.approx(MAP_X, abs=1e-3)
    assert estimate.pose.y == pytest.approx(MAP_Y, abs=1e-3)
    assert estimate.heading_rad is None
    assert estimate.pose.yaw is None
    assert math.isinf(estimate.uncertainty[2])
    assert estimate.uncertainty[0] == pytest.approx(math.hypot(0.4, 0.3))
    assert estimate.uncertainty[1] == pytest.approx(math.hypot(0.5, 0.3))
    assert estimate.rtk == "fixed"
    assert geo.georeference == referenced()
    headed = geo.to_map(satellite(rtk=None, navsat_status=2, absolute_heading_rad=0.5))
    expected_yaw = math.atan2(math.sin(0.5 + math.pi / 2), math.cos(0.5 + math.pi / 2))
    assert headed.rtk is None
    assert headed.heading_rad == pytest.approx(expected_yaw)
    assert headed.pose is not None
    assert headed.pose.yaw == pytest.approx(headed.heading_rad)
    assert headed.uncertainty[2] == pytest.approx(0.05)


def test_no_fix_status_is_rejected_and_ordinary_status_is_not_rtk() -> None:
    geo = alignment(referenced())
    no_fix = geo.to_map(satellite(navsat_status=-1, rtk="fixed", fresh=True, horizontal_std_m=0.2))
    assert no_fix.accepted is False
    assert no_fix.pose is None
    for status in (0, 1, 2):
        ordinary = geo.to_map(satellite(navsat_status=status, rtk=None))
        assert ordinary.accepted is True
        assert ordinary.rtk is None
        assert ordinary.pose is not None
        assert ordinary.pose.yaw is None


def test_rotated_map_compares_heading_in_one_frame() -> None:
    observer = HeadingObserver(min_baseline_m=5.0, max_correction_rad=0.35)
    psi = math.pi / 2
    map_dx = math.cos(psi) * 8.0
    map_dy = math.sin(psi) * 8.0
    consistent = straight(
        map_yaw=psi,
        map_dx=map_dx,
        map_dy=map_dy,
        enu_east_m=8.0,
        enu_north_m=0.0,
        alignment_rotation=YAW_90,
    )
    assert observer.observe(consistent) is None
    biased = replace(consistent, map_yaw=psi - 0.2)
    correction = observer.observe(biased)
    assert correction is not None
    assert correction.source == "heading"
    assert correction.ready is True
    assert correction.correction_rad == pytest.approx(0.2, abs=1e-9)


def test_unknown_bias_pauses_map_navigation_without_stopping() -> None:
    observer = HeadingObserver(min_baseline_m=5.0, max_correction_rad=0.35)
    observation = observer.observe(
        straight(
            map_yaw=0.0,
            map_dx=8.0 * math.cos(0.35),
            map_dy=8.0 * math.sin(0.35),
            enu_east_m=8.0 * math.cos(-0.35),
            enu_north_m=8.0 * math.sin(-0.35),
        )
    )
    assert observation is not None
    assert observation.source == "unknown"
    assert observation.ready is False
    assert observation.correction_rad == pytest.approx(-0.7, abs=1e-9)
    geo = alignment(referenced())
    geo.note_heading(source=observation.source, ready=observation.ready)
    paused = geo.posture(fix=satellite(), local_healthy=True, server_connected=False)
    assert paused.map_navigation is False
    assert paused.stop is False
    assert paused.geodetic_targets_paused is False
    assert paused.alignment_updates_paused is False
    geo.note_heading(source="heading", ready=True)
    resumed = geo.posture(fix=satellite(), local_healthy=True, server_connected=False)
    assert resumed.map_navigation is True
    assert resumed.stop is False
    lost = geo.posture(fix=None, local_healthy=True, server_connected=False)
    assert lost.map_navigation is True
    assert lost.stop is False


def test_forward_baseline_corrects_wrapped_angle() -> None:
    observer = HeadingObserver(min_baseline_m=5.0, max_correction_rad=0.35)
    observation = observer.observe(straight())
    assert observation is not None
    assert observation.correction_rad == pytest.approx(0.2, abs=1e-9)
    assert observation.source == "heading"
    assert observation.ready is True
    assert abs(observation.correction_rad) <= 0.35
    clamped = observer.observe(
        straight(map_yaw=-1.0, map_dx=8.0, map_dy=0.0, enu_east_m=8.0, enu_north_m=0.0)
    )
    assert clamped is not None
    assert clamped.source == "heading"
    assert clamped.ready is True
    assert clamped.correction_rad == pytest.approx(0.35, abs=1e-9)
    assert observer.observe(straight(georeferenced=False)) is None
    conflict = straight(map_yaw=0.4, map_dx=8.0 * math.cos(0.4), map_dy=8.0 * math.sin(0.4))
    east = 8.0 * math.cos(0.65)
    north = 8.0 * math.sin(0.65)
    attributed = observer.observe(replace(conflict, enu_east_m=east, enu_north_m=north))
    assert attributed is not None
    assert attributed.source == "georeference"
    assert attributed.ready is True
    assert attributed.correction_rad == pytest.approx(0.25, abs=1e-9)
    ambiguous = observer.observe(
        straight(
            map_yaw=0.0,
            map_dx=8.0 * math.cos(0.35),
            map_dy=8.0 * math.sin(0.35),
            enu_east_m=8.0 * math.cos(-0.35),
            enu_north_m=8.0 * math.sin(-0.35),
        )
    )
    assert ambiguous is not None
    assert ambiguous.source == "unknown"
    assert ambiguous.ready is False
    assert ambiguous.correction_rad == pytest.approx(-0.7, abs=1e-9)


def test_reverse_curve_slip_jump_rejected() -> None:
    observer = HeadingObserver(min_baseline_m=5.0, max_correction_rad=0.35)
    east, north = along(2.5)
    rejected = {
        "reverse": straight(odom_forward_m=-8.0),
        "curve": straight(imu_yaw_delta_rad=0.8),
        "slip": straight(slip=True),
        "jump": straight(jump=True),
        "stationary": straight(
            map_dx=0.0,
            map_dy=0.0,
            enu_east_m=0.0,
            enu_north_m=0.0,
            odom_forward_m=0.0,
            commanded_forward=False,
        ),
        "short": straight(
            map_dx=east,
            map_dy=north,
            enu_east_m=east,
            enu_north_m=north,
            odom_forward_m=2.5,
        ),
        "command_only": straight(
            map_dx=0.0,
            map_dy=0.0,
            enu_east_m=0.0,
            enu_north_m=0.0,
            odom_forward_m=0.0,
            commanded_forward=True,
        ),
    }
    for name, sample in rejected.items():
        assert observer.observe(sample) is None, name
    gnss = observer.observe(straight(gnss_trusted=False))
    assert gnss is not None
    assert gnss.source == "gnss"
    assert gnss.ready is False
    assert gnss.correction_rad == 0.0


def test_heading_observer_never_commands_motion() -> None:
    signature = inspect.signature(HeadingObserver)
    assert "cmd_vel" not in signature.parameters
    assert "publisher" not in signature.parameters
    observer = HeadingObserver(min_baseline_m=5.0, max_correction_rad=0.35)
    result = observer.observe(straight())
    assert not isinstance(result, Velocity)
    assert getattr(result, "velocity", None) is None
    assert not hasattr(observer, "home")
    module = localization_module()
    topics: list[str] = []
    adapter = module.LocalizationAdapter(  # type: ignore[attr-defined]
        alignment=alignment(referenced()),
        relocalizer=relocator(referenced(), lambda _seed: None, aligned=alignment(referenced())),
        observer=observer,
        clock=clock(),
    )
    adapter.ingest_sample(straight())
    adapter.ingest_fix(satellite())
    topics.extend(adapter.published_topics)
    assert topics == []
    assert "/cmd_vel" not in topics


async def test_no_gps_manual_hint_scan_match() -> None:
    seen: list[Pose2D | None] = []

    def match(seed: Pose2D | None) -> tuple[Pose2D, bool] | None:
        seen.append(seed)
        if seed == Pose2D(3.0, 4.0, 0.5):
            return Pose2D(3.2, 4.1, 0.55), True
        if seed is None:
            return Pose2D(8.0, 1.0, -0.2), True
        return None

    manual = relocator(Georeference("unreferenced"), match, aligned=None)
    assert manual.ready is False
    assert manual.pose is None
    result = await manual.begin(MAP, PoseHint(Pose2D(3.0, 4.0, 0.5), "manual"))
    assert result.status == OperationStatus.SUCCEEDED
    assert manual.ready is True
    assert manual.pose == Pose2D(3.2, 4.1, 0.55)
    assert seen == [Pose2D(3.0, 4.0, 0.5)]

    seen.clear()
    scanned = relocator(Georeference("unreferenced"), match, aligned=None)
    searched = await scanned.begin(MAP, None)
    assert searched.status == OperationStatus.SUCCEEDED
    assert scanned.ready is True
    assert scanned.pose == Pose2D(8.0, 1.0, -0.2)
    assert seen == [None]


async def test_last_pose_without_match_is_not_ready() -> None:
    def match(_seed: Pose2D | None) -> tuple[Pose2D, bool] | None:
        return None

    reloc = relocator(referenced(), match, aligned=alignment(referenced()))
    assert reloc.ready is False
    result = await reloc.begin(MAP, PoseHint(Pose2D(1.0, 2.0, 0.3), "last_pose"))
    assert result.status == OperationStatus.FAILED
    assert reloc.ready is False
    assert reloc.pose is None


def test_bundle_gps_does_not_start_ready() -> None:
    reloc = relocator(
        referenced(),
        lambda _seed: (Pose2D(1.0, 1.0, 0.0), True),
        aligned=alignment(referenced()),
        bundle_has_gps=True,
    )
    assert reloc.ready is False
    assert reloc.pose is None


async def test_gps_seed_needs_reliable_match() -> None:
    seen: list[Pose2D | None] = []

    def match(seed: Pose2D | None) -> tuple[Pose2D, bool] | None:
        seen.append(seed)
        return None

    geo = alignment(referenced())
    reloc = relocator(referenced(), match, aligned=geo, bundle_has_gps=True)
    result = await reloc.begin(MAP, PoseHint(None, "gps", satellite()))
    assert result.status == OperationStatus.FAILED
    assert reloc.ready is False
    assert reloc.pose is None
    assert len(seen) == 1
    assert seen[0] is not None
    assert seen[0].x == pytest.approx(MAP_X, abs=1e-2)
    assert seen[0].y == pytest.approx(MAP_Y, abs=1e-2)
    assert seen[0].yaw is None

    expected_yaw = math.atan2(math.sin(0.5 + math.pi / 2), math.cos(0.5 + math.pi / 2))
    headed_yaw: list[float | None] = []

    def headed(seed: Pose2D | None) -> tuple[Pose2D, bool] | None:
        headed_yaw.append(None if seed is None else seed.yaw)
        if seed is None or seed.yaw is None:
            return None
        return Pose2D(seed.x, seed.y, seed.yaw), True

    with_heading = relocator(referenced(), headed, aligned=geo)
    headed_fix = satellite(rtk=None, navsat_status=0, absolute_heading_rad=0.5)
    headed_result = await with_heading.begin(MAP, PoseHint(None, "gps", headed_fix))
    assert headed_result.status == OperationStatus.SUCCEEDED
    assert headed_yaw[0] == pytest.approx(expected_yaw)

    def confirm(seed: Pose2D | None) -> tuple[Pose2D, bool] | None:
        if seed is None:
            return None
        return Pose2D(seed.x + 0.1, seed.y - 0.2, 0.4), True

    confirmed = relocator(referenced(), confirm, aligned=geo)
    done = await confirmed.begin(MAP, PoseHint(None, "gps", satellite()))
    assert done.status == OperationStatus.SUCCEEDED
    assert confirmed.ready is True
    assert confirmed.pose is not None
    assert confirmed.pose.x == pytest.approx(MAP_X + 0.1, abs=1e-2)
    assert confirmed.pose.y == pytest.approx(MAP_Y - 0.2, abs=1e-2)


def test_bad_gps_does_not_kill_healthy_local_navigation() -> None:
    geo = alignment(referenced())
    stale = satellite(stamp=90.0, fresh=False, horizontal_std_m=20.0, rtk=None)
    posture = geo.posture(fix=stale, local_healthy=True, server_connected=False)
    assert posture == NavigationPosture(True, True, True, False)
    estimate = geo.to_map(stale)
    assert estimate.accepted is False
    assert estimate.pose is None
    assert geo.georeference == referenced()
    lost = geo.posture(fix=None, local_healthy=True, server_connected=True)
    assert lost.map_navigation is True
    assert lost.geodetic_targets_paused is True
    assert lost.alignment_updates_paused is True
    assert lost.stop is False
    healthy = geo.posture(fix=satellite(), local_healthy=True, server_connected=False)
    assert healthy.stop is False
    assert healthy.map_navigation is True
    assert healthy.geodetic_targets_paused is False
    assert healthy.alignment_updates_paused is False


def test_localization_console_script_is_registered() -> None:
    text = (ROOT / "ros2" / "src" / "senior_robot" / "setup.py").read_text()
    assert "localization_node = senior_robot.localization_node:main" in text

