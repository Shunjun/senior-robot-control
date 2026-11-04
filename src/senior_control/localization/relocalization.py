"""Restart localization. A packaged GPS fix is not evidence the robot is localized."""

from collections.abc import Callable
from dataclasses import dataclass

from senior_control.errors import DomainError
from senior_control.localization.geo import AlignedPose, GeoAlignment, QualifiedFix
from senior_control.mapping.archive import Georeference
from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D

ScanMatch = Callable[[Pose2D | AlignedPose | None], tuple[Pose2D, bool] | None]


@dataclass(frozen=True)
class PoseHint:
    """Initial guess. ``last_pose`` is only a seed for scan matching."""

    pose: Pose2D | None
    source: str
    fix: QualifiedFix | None = None


class Relocalizer:
    """GPS seed, static scan match, last pose, or a manual pose. Ready needs a match."""

    def __init__(
        self,
        *,
        map_ref: MapRef,
        georeference: Georeference,
        alignment: GeoAlignment | None,
        scan_match: ScanMatch,
        bundle_has_gps: bool,
    ) -> None:
        self._map_ref = map_ref
        self._georeference = georeference
        self._alignment = alignment
        self._scan_match = scan_match
        self.ready = False
        self.pose: Pose2D | None = None
        self.reason = "bundle gps is not a pose" if bundle_has_gps else "not localized"

    async def begin(self, map_ref: MapRef, hint: PoseHint | None) -> OperationResult:
        if map_ref != self._map_ref:
            return self._fail("map reference mismatch")
        matched = self._scan_match(self._seed(hint))
        if matched is None or not matched[1]:
            return self._fail("no reliable match")
        self.ready = True
        self.pose = matched[0]
        self.reason = "scan_match" if hint is None else hint.source
        return OperationResult("relocalize", OperationStatus.SUCCEEDED)

    def _seed(self, hint: PoseHint | None) -> Pose2D | AlignedPose | None:
        if hint is None:
            return None
        if hint.source == "gps":
            return self._gps_seed(hint)
        return hint.pose

    def _gps_seed(self, hint: PoseHint) -> AlignedPose | None:
        unreferenced = self._georeference.status == "unreferenced"
        if self._alignment is None or hint.fix is None or unreferenced:
            return None
        try:
            estimate = self._alignment.to_map(hint.fix)
        except DomainError:
            return None
        pose = estimate.pose
        if not estimate.accepted or pose is None or not estimate.full_transform:
            return None
        return AlignedPose(pose.x, pose.y, estimate.heading_rad)

    def _fail(self, reason: str) -> OperationResult:
        self.ready = False
        self.pose = None
        self.reason = reason
        return OperationResult("relocalize", OperationStatus.FAILED)
