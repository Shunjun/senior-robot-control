"""//Compatible: original Home values and construction over the sole manager service."""

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, cast

from senior_mapping.archive import BundleWaypoint as BundleWaypoint
from senior_mapping.evidence import CellEvidence as CellEvidence
from senior_mapping.operations import HomeLink
from senior_mapping.references import AnchoredWaypoint
from senior_mapping.references import AnchorSnapshot as AnchorSnapshot
from senior_mapping.store import MapReferenceStore
from vehicle_manager.waypoints.home_ports import HomeServicePorts
from vehicle_manager.waypoints.home_service import HomeService as _HomeService

from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D
from senior_control.storage.database import Database
from senior_control.waypoints.store import StoredWaypoint as StoredWaypoint
from senior_control.waypoints.store import Waypoint, WaypointStore


@dataclass(frozen=True)
class StartupReference:
    """Motion from power-on until the first valid localization.

    ``relative_pose`` is the current body expressed in the power-on frame.
    It recovers the start only when that chain is continuous and trusted.
    """

    moved: bool
    continuous: bool
    trusted: bool
    relative_pose: Pose2D | None = None
    latitude: float | None = None
    longitude: float | None = None


@dataclass(frozen=True)
class LocalizedPose:
    """Localization must be produced and certified for the supplied reference nonce."""

    pose: Pose2D
    map_ref: MapRef
    anchor_id: str
    anchor_pose: Pose2D
    healthy: bool
    latitude: float | None = None
    longitude: float | None = None
    # //Compatible: legacy poses omit this nonce and remain unqualified for motion.
    map_generation: str | None = None

    def __post_init__(self) -> None:
        if not self.anchor_id.strip():
            raise ValueError("anchor is required")
        if (self.latitude is None) != (self.longitude is None):
            raise ValueError("gps display needs both coordinates")


@dataclass(frozen=True)
class HomeGoal:
    map_ref: MapRef
    pose: Pose2D
    map_generation: str
    intent_token: str
    on_accepted: Callable[[str], None] | None = None


@dataclass(frozen=True)
class HomeResult:
    initialized: bool
    waypoint: Waypoint | None
    reason: str

    def __post_init__(self) -> None:
        if self.initialized and self.waypoint is None:
            raise ValueError("initialized home is missing")


class NavigationPort(Protocol):
    """Injected cancellation port. A missing qualified goal port must not open Nav2."""

    async def cancel(self, execution_id: str) -> OperationResult:
        """Report success only when the accepted goal is confirmed terminal."""
        ...


class QualifiedHomeNavigationPort(Protocol):
    """Atomically fence admission on the reference nonce and fresh healthy localization.

    The receiver must reject stale or unknown qualification before accepting a goal;
    local verification immediately before an await is not a receiver-side fence.
    A successful return identifies the accepted goal; callbacks belong to the same
    intent and must be delivered before this call returns or raises.
    """

    async def navigate_qualified(self, goal: HomeGoal) -> str: ...


def _error_code(error: Exception) -> ErrorCode:
    # //Compatible: called only inside the original concrete DomainError catch.
    return cast(DomainError, error).code


class HomeService(_HomeService[Waypoint, Pose2D, HomeResult, HomeLink, HomeGoal, OperationResult]):
    """//Compatible: original constructor; inherited business/guard methods, original map owner."""

    def __init__(
        self,
        database: Database,
        reference: MapReferenceStore,
        *,
        robot_id: str,
        navigation: NavigationPort | None = None,
    ) -> None:
        def bind_waypoint(waypoint: Waypoint, anchor_pose: Pose2D) -> None:
            reference.place_waypoint(
                AnchoredWaypoint(waypoint.id, waypoint.pose, waypoint.anchor_id, False),
                anchor_pose=anchor_pose,
            )

        super().__init__(
            database,
            WaypointStore(database, robot_id=robot_id),
            robot_id=robot_id,
            navigation=navigation,
            coordination=database.home_execution(robot_id),
            target_lock=database.home_target_lock(robot_id),
            ports=HomeServicePorts(
                reference=reference,
                snapshot=lambda: reference.snapshot(Path("pose_graph"), ()),
                place=reference.place,
                apply_anchor_update=reference.apply_anchor_update,
                bind_waypoint=bind_waypoint,
                pose=Pose2D,
                result=HomeResult,
                link=HomeLink,
                goal=HomeGoal,
                error=DomainError,
                error_code=_error_code,
                succeeded_status=OperationStatus.SUCCEEDED,
            ),
        )
