"""Home decisions. This module does not open a Nav2 client."""

import asyncio
import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.archive import BundleWaypoint
from senior_control.mapping.evidence import CellEvidence
from senior_control.mapping.operations import HomeLink
from senior_control.mapping.references import AnchoredWaypoint, AnchorSnapshot
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D
from senior_control.storage.database import Database
from senior_control.waypoints.store import StoredWaypoint, Waypoint, WaypointStore


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
    pose: Pose2D
    map_ref: MapRef
    anchor_id: str
    anchor_pose: Pose2D
    healthy: bool
    latitude: float | None = None
    longitude: float | None = None

    def __post_init__(self) -> None:
        if not self.anchor_id.strip():
            raise ValueError("anchor is required")
        if (self.latitude is None) != (self.longitude is None):
            raise ValueError("gps display needs both coordinates")


@dataclass(frozen=True)
class HomeGoal:
    map_ref: MapRef
    pose: Pose2D
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
    """Injected goal port. A missing port must not open Nav2."""

    async def navigate(self, goal: HomeGoal) -> str: ...

    async def cancel(self, execution_id: str) -> OperationResult: ...


class HomeService:
    def __init__(
        self,
        database: Database,
        reference: MapReferenceStore,
        *,
        robot_id: str,
        navigation: NavigationPort | None = None,
    ) -> None:
        self._reference = reference
        self._robot_id = robot_id
        self._navigation = navigation
        self._store = WaypointStore(database, robot_id=robot_id)
        self._localized: LocalizedPose | None = None
        self._connected = True
        self._execution_id: str | None = None
        self._accepted_id: str | None = None
        self._cancel_id: str | None = None
        self._in_send = False
        self._return_lock = asyncio.Lock()

    async def prepare(self) -> None:
        await self._store.prepare()
        for item in await self._store.points():
            self._bind(item.waypoint, item.anchor_pose)

    async def initialize(self, start: StartupReference, localized: LocalizedPose) -> HomeResult:
        existing = await self._store.home()
        if existing is not None:
            self._bind(existing.waypoint, existing.anchor_pose)
            if _kept(existing):
                self._remember(localized)
                return HomeResult(True, existing.waypoint, "")
            return HomeResult(False, None, "home is not adopted")
        if not localized.healthy:
            return HomeResult(False, None, "localization is unhealthy")
        if not _recoverable(start):
            return HomeResult(False, None, "home is uninitialized")
        pose = localized.pose
        latitude = localized.latitude
        longitude = localized.longitude
        if start.moved:
            assert start.relative_pose is not None
            pose = _recover_start(localized.pose, start.relative_pose)
            latitude, longitude = _pair(start.latitude, start.longitude)
        waypoint = await self._store.save_home(
            waypoint_id=None,
            pose=pose,
            map_ref=localized.map_ref,
            anchor_id=localized.anchor_id,
            anchor_pose=localized.anchor_pose,
            source_robot_id=self._robot_id,
            status="confirmed",
            adopted=True,
            latitude=latitude,
            longitude=longitude,
        )
        self._remember(localized)
        self._bind(waypoint, localized.anchor_pose)
        return HomeResult(True, waypoint, "")

    async def set_current(self, pose: LocalizedPose) -> Waypoint:
        if not pose.healthy:
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "localization is unhealthy")
        waypoint = await self._store.save_home(
            waypoint_id=None,
            pose=pose.pose,
            map_ref=pose.map_ref,
            anchor_id=pose.anchor_id,
            anchor_pose=pose.anchor_pose,
            source_robot_id=self._robot_id,
            status="confirmed",
            adopted=True,
            latitude=pose.latitude,
            longitude=pose.longitude,
        )
        self._remember(pose)
        self._bind(waypoint, pose.anchor_pose)
        return waypoint

    async def return_home(self, expected: MapRef) -> str:
        async with self._return_lock:
            if self._execution_id is not None:
                await self._cancel_owed_goal()
            waypoint = await self._ready(expected)
            navigation = self._navigation
            assert navigation is not None
            self._accepted_id = None
            self._in_send = True
            cancel_this = False
            try:
                try:
                    execution_id = await navigation.navigate(
                        HomeGoal(expected, waypoint.pose, self._note_accepted)
                    )
                except Exception:
                    self._cancel_id = None
                    self._execution_id = None
                    raise
                else:
                    self._execution_id = execution_id
                    cancel_this = self._cancel_id == execution_id
            finally:
                self._in_send = False
                self._accepted_id = None
            if cancel_this:
                await self._cancel_owed_goal()
            return execution_id

    async def home(self) -> Waypoint | None:
        stored = await self._store.home()
        if stored is None:
            return None
        return stored.waypoint

    async def home_link(self) -> HomeLink | None:
        stored = await self._store.home()
        if stored is None or stored.adopted is None or stored.link_status is None:
            return None
        waypoint = stored.waypoint
        return HomeLink(
            self._robot_id,
            waypoint.source_robot_id,
            waypoint.map_ref,
            waypoint.id,
            stored.link_status,
            stored.adopted,
        )

    async def observe_obstacle(
        self,
        evidence: CellEvidence,
        *,
        world: tuple[float, float],
        anchor_pose: Pose2D,
    ) -> None:
        self._reference.place(evidence, world=world, anchor_pose=anchor_pose)

    async def apply_anchor_update(self, update: AnchorSnapshot) -> Waypoint:
        self._reference.apply_anchor_update(update)
        snapshot = self._reference.snapshot(Path("pose_graph"), ())
        await self._store.sync_poses(snapshot.waypoint_bindings)
        stored = await self._store.home()
        if stored is None:
            raise DomainError(ErrorCode.VALIDATION, "home is not set")
        return stored.waypoint

    async def import_home(self, source: BundleWaypoint, *, anchor_pose: Pose2D) -> Waypoint:
        if source.kind != "home":
            raise DomainError(ErrorCode.VALIDATION, "imported home kind is invalid")
        existing = await self._store.home()
        pending = source.source_robot_id != self._robot_id
        if pending and _kept(existing):
            assert existing is not None
            return existing.waypoint
        status = "pending_confirmation" if pending else "confirmed"
        waypoint = await self._store.save_home(
            waypoint_id=source.waypoint_id,
            pose=source.pose,
            map_ref=source.map_ref,
            anchor_id=source.anchor_id,
            anchor_pose=anchor_pose,
            source_robot_id=source.source_robot_id,
            status=status,
            adopted=not pending,
            latitude=None,
            longitude=None,
        )
        self._bind(waypoint, anchor_pose)
        return waypoint

    async def save_waypoint(self, pose: LocalizedPose, *, waypoint_id: str) -> Waypoint:
        waypoint = await self._store.save_waypoint(
            waypoint_id=waypoint_id,
            pose=pose.pose,
            map_ref=pose.map_ref,
            anchor_id=pose.anchor_id,
            anchor_pose=pose.anchor_pose,
            source_robot_id=self._robot_id,
            status="confirmed",
            latitude=pose.latitude,
            longitude=pose.longitude,
        )
        self._bind(waypoint, pose.anchor_pose)
        return waypoint

    async def waypoint(self, waypoint_id: str) -> Waypoint | None:
        stored = await self._store.waypoint(waypoint_id)
        if stored is None:
            return None
        return stored.waypoint

    async def note_localization(self, pose: LocalizedPose) -> None:
        self._localized = pose

    async def note_connection(self, connected: bool) -> None:
        self._connected = connected
        if not connected:
            accepted = self._accepted_id if self._in_send else self._execution_id
            if accepted is not None:
                self._cancel_id = accepted
        if self._navigation is None or self._return_lock.locked():
            return
        async with self._return_lock:
            if self._cancel_id is None or self._execution_id != self._cancel_id:
                if not self._in_send:
                    self._cancel_id = None
                return
            await self._cancel_owed_goal()

    def _note_accepted(self, execution_id: str) -> None:
        if execution_id.strip():
            self._accepted_id = execution_id

    async def note_terminal(self, execution_id: str) -> None:
        async with self._return_lock:
            if self._execution_id != execution_id:
                return
            navigation = self._navigation
            if navigation is not None:
                result = await navigation.cancel(execution_id)
                if result.status != OperationStatus.SUCCEEDED:
                    raise DomainError(ErrorCode.CONFLICT, "navigation cancel was rejected")
            self._execution_id = None
            if self._cancel_id == execution_id:
                self._cancel_id = None

    async def _cancel_owed_goal(self) -> None:
        execution_id = self._execution_id
        assert execution_id is not None
        result = await self._finish_execution(execution_id)
        if result.status != OperationStatus.SUCCEEDED:
            raise DomainError(ErrorCode.CONFLICT, "navigation cancel was rejected")
        self._execution_id = None
        if self._cancel_id == execution_id:
            self._cancel_id = None

    async def _finish_execution(self, execution_id: str) -> OperationResult:
        navigation = self._navigation
        assert navigation is not None
        return await navigation.cancel(execution_id)

    async def _ready(self, expected: MapRef) -> Waypoint:
        stored = await self._store.home()
        if stored is None:
            raise DomainError(ErrorCode.VALIDATION, "home is not set")
        if not stored.adopted or stored.link_status != "confirmed":
            raise DomainError(ErrorCode.CONFLICT, "home is not adopted")
        waypoint = stored.waypoint
        if waypoint.map_ref != expected or self._reference.map_ref != expected:
            raise DomainError(ErrorCode.CONFLICT, "map reference does not match")
        if self._reference.review_required:
            raise DomainError(ErrorCode.CONFLICT, "coordinate reference needs review")
        if self._localized is None or not self._localized.healthy:
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "localization is unhealthy")
        if self._localized.map_ref != expected:
            raise DomainError(ErrorCode.CONFLICT, "map reference does not match")
        if not self._connected:
            raise DomainError(ErrorCode.OFFLINE, "connection is offline")
        if self._navigation is None:
            raise DomainError(ErrorCode.OFFLINE, "navigation port is unavailable")
        return waypoint

    def _remember(self, localized: LocalizedPose) -> None:
        if localized.healthy:
            self._localized = localized

    def _bind(self, waypoint: Waypoint, anchor_pose: Pose2D) -> None:
        if waypoint.map_ref != self._reference.map_ref:
            return
        self._reference.place_waypoint(
            AnchoredWaypoint(waypoint.id, waypoint.pose, waypoint.anchor_id, False),
            anchor_pose=anchor_pose,
        )


def _kept(stored: StoredWaypoint | None) -> bool:
    return (
        stored is not None
        and stored.adopted is True
        and stored.link_status == "confirmed"
        and stored.waypoint.status == "confirmed"
    )


def _recoverable(start: StartupReference) -> bool:
    if not start.moved:
        return True
    return start.continuous and start.trusted and start.relative_pose is not None


def _pair(latitude: float | None, longitude: float | None) -> tuple[float | None, float | None]:
    if latitude is None or longitude is None:
        return None, None
    return latitude, longitude


def _recover_start(localized: Pose2D, relative: Pose2D) -> Pose2D:
    """Compose the localized pose with the inverse of the power-on-frame motion."""

    return _compose(localized, _invert(relative))


def _invert(pose: Pose2D) -> Pose2D:
    yaw = -pose.yaw
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return Pose2D(
        cosine * -pose.x - sine * -pose.y,
        sine * -pose.x + cosine * -pose.y,
        _wrap(yaw),
    )


def _compose(origin: Pose2D, local: Pose2D) -> Pose2D:
    cosine = math.cos(origin.yaw)
    sine = math.sin(origin.yaw)
    return Pose2D(
        origin.x + cosine * local.x - sine * local.y,
        origin.y + sine * local.x + cosine * local.y,
        _wrap(origin.yaw + local.yaw),
    )


def _wrap(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))
