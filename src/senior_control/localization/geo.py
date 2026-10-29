"""Loose GPS-to-map alignment. One fix does not solve a transform or a heading.

RTK Fixed and Float come only from the caller's explicit ``rtk`` field. A
NavSat status is not a substitute. Uncertainty stays on the position estimate.
This does not insert GPS into the SLAM pose graph.
"""

import math
from dataclasses import dataclass

from senior_control.clock import Clock
from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.archive import Georeference

_WGS84_A = 6_378_137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)
_EXPLICIT_RTK = frozenset({"fixed", "float"})


@dataclass(frozen=True)
class QualifiedFix:
    """A caller-qualified GNSS fix. ``rtk`` is never inferred from ``navsat_status``."""

    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    stamp: float
    horizontal_std_m: float
    vertical_std_m: float
    rtk: str | None
    navsat_status: int
    fresh: bool
    absolute_heading_rad: float | None


@dataclass(frozen=True)
class AlignedPose:
    """Map position. ``yaw`` stays empty until an absolute heading or a scan provides one."""

    x: float
    y: float
    yaw: float | None


@dataclass(frozen=True)
class PositionEstimate:
    """Map position from one alignment. ``uncertainty`` is easting, northing, yaw."""

    pose: AlignedPose | None
    uncertainty: tuple[float, float, float]
    heading_rad: float | None
    full_transform: bool
    accepted: bool
    reason: str
    rtk: str | None
    datum: str | None
    height_datum: str | None
    translation_m: tuple[float, float, float] | None
    rotation: tuple[float, float, float, float] | None


@dataclass(frozen=True)
class NavigationPosture:
    """What this module will allow. It never commands a stop."""

    map_navigation: bool
    geodetic_targets_paused: bool
    alignment_updates_paused: bool
    stop: bool


class GeoAlignment:
    """Convert a qualified fix with a stored Georeference. Unreferenced maps refuse."""

    def __init__(
        self,
        georeference: Georeference,
        *,
        origin_latitude_deg: float | None,
        origin_longitude_deg: float | None,
        origin_altitude_m: float | None,
        uncertainty_m: tuple[float, float, float],
        clock: Clock,
        max_age_s: float,
        max_horizontal_std_m: float,
    ) -> None:
        self.georeference = georeference
        self._origin_latitude_deg = origin_latitude_deg
        self._origin_longitude_deg = origin_longitude_deg
        self._origin_altitude_m = origin_altitude_m
        self._uncertainty_m = uncertainty_m
        self._clock = clock
        self._max_age_s = max_age_s
        self._max_horizontal_std_m = max_horizontal_std_m
        self._unverified_heading = False

    def note_heading(self, *, source: str, ready: bool) -> None:
        self._unverified_heading = source == "unknown" and not ready

    def to_map(self, fix: QualifiedFix) -> PositionEstimate:
        if self.georeference.status == "unreferenced":
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "map is unreferenced")
        if not self._complete():
            return self._estimate(
                fix,
                pose=None,
                uncertainty=(self._uncertainty_m[0], self._uncertainty_m[1], math.inf),
                heading_rad=None,
                full_transform=False,
                accepted=False,
                reason="single fix does not determine the transform",
            )
        if not self._usable(fix):
            return self._estimate(
                fix,
                pose=None,
                uncertainty=(math.inf, math.inf, math.inf),
                heading_rad=None,
                full_transform=True,
                accepted=False,
                reason="gps untrusted",
            )
        east, north, _up = _enu(
            fix.latitude_deg,
            fix.longitude_deg,
            fix.altitude_m,
            self._origin_latitude_deg,
            self._origin_longitude_deg,
            self._origin_altitude_m,
        )
        rotation = self.georeference.rotation
        translation = self.georeference.translation_m
        assert rotation is not None and translation is not None
        rotated = _rotate(rotation, (east, north, _up))
        heading = _map_heading(fix.absolute_heading_rad, rotation)
        sigma_yaw = math.inf if heading is None else self._uncertainty_m[2]
        return self._estimate(
            fix,
            pose=AlignedPose(rotated[0] + translation[0], rotated[1] + translation[1], heading),
            uncertainty=(
                math.hypot(self._uncertainty_m[0], fix.horizontal_std_m),
                math.hypot(self._uncertainty_m[1], fix.horizontal_std_m),
                sigma_yaw,
            ),
            heading_rad=heading,
            full_transform=True,
            accepted=True,
            reason="aligned",
        )

    def posture(
        self,
        *,
        fix: QualifiedFix | None,
        local_healthy: bool,
        server_connected: bool,
    ) -> NavigationPosture:
        # A server drop is not a stop. An unverified heading bias pauses map
        # navigation without stopping. GPS loss only pauses geodetic updates.
        del server_connected
        trusted = fix is not None and self._usable(fix) and self._complete()
        return NavigationPosture(
            map_navigation=local_healthy and not self._unverified_heading,
            geodetic_targets_paused=not trusted,
            alignment_updates_paused=not trusted,
            stop=False,
        )

    def _estimate(
        self,
        fix: QualifiedFix,
        *,
        pose: AlignedPose | None,
        uncertainty: tuple[float, float, float],
        heading_rad: float | None,
        full_transform: bool,
        accepted: bool,
        reason: str,
    ) -> PositionEstimate:
        geo = self.georeference
        return PositionEstimate(
            pose,
            uncertainty,
            heading_rad,
            full_transform,
            accepted,
            reason,
            _explicit_rtk(fix),
            geo.datum,
            geo.height_datum,
            geo.translation_m,
            geo.rotation,
        )

    def _complete(self) -> bool:
        geo = self.georeference
        if geo.status != "referenced" or geo.datum is None or geo.height_datum is None:
            return False
        if not geo.datum.strip() or not geo.height_datum.strip():
            return False
        translation = geo.translation_m
        rotation = geo.rotation
        latitude = self._origin_latitude_deg
        longitude = self._origin_longitude_deg
        altitude = self._origin_altitude_m
        if (
            translation is None
            or rotation is None
            or latitude is None
            or longitude is None
            or altitude is None
        ):
            return False
        return (
            _finite_triple(translation)
            and _unit_quaternion(rotation)
            and _finite_triple((latitude, longitude, altitude))
        )

    def _usable(self, fix: QualifiedFix) -> bool:
        if fix.navsat_status < 0:
            return False
        if not fix.fresh or not math.isfinite(fix.stamp):
            return False
        if abs(self._clock.monotonic() - fix.stamp) > self._max_age_s:
            return False
        if not _geographic(fix.latitude_deg, fix.longitude_deg, fix.altitude_m):
            return False
        horizontal = fix.horizontal_std_m
        if not _positive_std(horizontal) or horizontal > self._max_horizontal_std_m:
            return False
        return _positive_std(fix.vertical_std_m)


def _explicit_rtk(fix: QualifiedFix) -> str | None:
    if fix.rtk in _EXPLICIT_RTK:
        return fix.rtk
    return None


def _positive_std(value: float) -> bool:
    return math.isfinite(value) and value >= 0.0


def _geographic(latitude: float, longitude: float, altitude: float) -> bool:
    return (
        math.isfinite(latitude)
        and math.isfinite(longitude)
        and math.isfinite(altitude)
        and abs(latitude) <= 90.0
        and abs(longitude) <= 180.0
    )


def _finite_triple(values: tuple[float, float, float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _unit_quaternion(rotation: tuple[float, float, float, float]) -> bool:
    if not all(math.isfinite(part) for part in rotation):
        return False
    norm = math.sqrt(sum(part * part for part in rotation))
    return abs(norm - 1.0) <= 1e-3


def _map_heading(
    absolute_heading_rad: float | None,
    rotation: tuple[float, float, float, float],
) -> float | None:
    if absolute_heading_rad is None or not math.isfinite(absolute_heading_rad):
        return None
    return _wrap(absolute_heading_rad + quaternion_yaw(rotation))


def quaternion_yaw(rotation: tuple[float, float, float, float]) -> float:
    x, y, z, w = rotation
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def _wrap(delta: float) -> float:
    return math.atan2(math.sin(delta), math.cos(delta))


def _rotate(
    rotation: tuple[float, float, float, float], vector: tuple[float, float, float]
) -> tuple[float, float, float]:
    x, y, z, w = rotation
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _enu(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    origin_latitude_deg: float | None,
    origin_longitude_deg: float | None,
    origin_altitude_m: float | None,
) -> tuple[float, float, float]:
    assert origin_latitude_deg is not None
    assert origin_longitude_deg is not None
    assert origin_altitude_m is not None
    x, y, z = _ecef(latitude_deg, longitude_deg, altitude_m)
    x0, y0, z0 = _ecef(origin_latitude_deg, origin_longitude_deg, origin_altitude_m)
    return _ecef_to_enu(x - x0, y - y0, z - z0, origin_latitude_deg, origin_longitude_deg)


def _ecef(
    latitude_deg: float, longitude_deg: float, altitude_m: float
) -> tuple[float, float, float]:
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    radius = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    return (
        (radius + altitude_m) * cos_lat * math.cos(lon),
        (radius + altitude_m) * cos_lat * math.sin(lon),
        (radius * (1.0 - _WGS84_E2) + altitude_m) * sin_lat,
    )


def _ecef_to_enu(
    dx: float, dy: float, dz: float, latitude_deg: float, longitude_deg: float
) -> tuple[float, float, float]:
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    return (
        -sin_lon * dx + cos_lon * dy,
        -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
        cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,
    )
