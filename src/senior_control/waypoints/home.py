"""Home decisions. This module does not open a Nav2 client."""

import asyncio
import math
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, cast

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
    """

    async def navigate_qualified(self, goal: HomeGoal) -> str: ...


class HomeService:
    def __init__(
        self,
        database: Database,
        reference: MapReferenceStore,
        *,
        robot_id: str,
        navigation: NavigationPort | None = None,
    ) -> None:
        self._database = database
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
        self._send_interrupted = False
        self._send_map_generation: str | None = None
        self._guard_unresolved = False
        self._return_lock = asyncio.Lock()
        self._home_target_lock = database.home_target_lock(robot_id)

    async def prepare(self) -> None:
        from senior_control.storage.core_schema import migrate_core

        await self._store.prepare()
        await migrate_core(self._database)
        async with self._database.write_lock:
            async with self._database.connection.execute(
                "SELECT execution_id FROM home_execution_guard WHERE robot_id = ?",
                (self._robot_id,),
            ) as cursor:
                row = await cursor.fetchone()
        if row is not None:
            self._guard_unresolved = True
            self._execution_id = row[0]
            self._cancel_id = row[0]

    async def initialize(self, start: StartupReference, localized: LocalizedPose) -> HomeResult:
        async with self._home_target_lock:
            return await self._initialize(start, localized)

    async def _initialize(self, start: StartupReference, localized: LocalizedPose) -> HomeResult:
        existing = await self._store.home()
        if existing is not None:
            if _kept(existing):
                self._remember(localized)
                return HomeResult(True, existing.waypoint, "")
            return HomeResult(False, None, "home is not adopted")
        if not localized.healthy:
            return HomeResult(False, None, "localization is unhealthy")
        if not self._pose_qualified(localized):
            return HomeResult(False, None, "localization is unavailable")
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
        if not self._pose_qualified(localized):
            return HomeResult(False, None, "localization is unavailable")
        self._remember(localized)
        self._bind(waypoint, localized.anchor_pose)
        return HomeResult(True, waypoint, "")

    async def set_current(
        self, pose: LocalizedPose, *, expected_revision: int | None = None
    ) -> Waypoint:
        async with self._home_target_lock:
            return await self._set_current(pose, expected_revision=expected_revision)

    async def _set_current(
        self, pose: LocalizedPose, *, expected_revision: int | None = None
    ) -> Waypoint:
        if not pose.healthy:
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "localization is unhealthy")
        if not self._pose_qualified(pose):
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "localization is unavailable")
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
            expected_revision=expected_revision,
        )
        if not self._pose_qualified(pose):
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "localization is unavailable")
        self._remember(pose)
        self._bind(waypoint, pose.anchor_pose)
        return waypoint

    async def home_revision(self) -> int:
        return await self._store.home_revision()

    async def return_home(self, expected: MapRef) -> str:
        async with self._return_lock:
            if self._execution_id is not None:
                await self._cancel_owed_goal()
            elif self._guard_unresolved:
                raise DomainError(ErrorCode.CONFLICT, "navigation outcome needs review")
            async with self._home_target_lock:
                execution_id, cancel_this = await self._send_home(expected)
            if cancel_this:
                await self._cancel_owed_goal()
            return execution_id

    async def _send_home(self, expected: MapRef) -> tuple[str, bool]:
        waypoint = await self._ready(expected)
        navigation = self._navigation
        assert navigation is not None
        if not callable(getattr(navigation, "navigate_qualified", None)):
            raise DomainError(ErrorCode.UNAVAILABLE, "qualified home navigation is unavailable")
        qualified = cast(QualifiedHomeNavigationPort, navigation)
        map_generation = self._reference.map_generation
        self._accepted_id = None
        self._in_send = True
        self._send_interrupted = False
        self._send_map_generation = map_generation
        cancel_this = False
        try:
            # Persist an unknown outcome before the external operation can accept a goal.
            self._guard_unresolved = True
            await self._write_guard(None)
            if not self._generation_ready(map_generation) or not self._connected:
                await self._clear_guard()
                raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "localization is unavailable")
            try:
                execution_id = await qualified.navigate_qualified(
                    HomeGoal(expected, waypoint.pose, map_generation, self._note_accepted)
                )
                self._execution_id = execution_id
                if self._send_interrupted or not self._generation_ready(map_generation):
                    self._cancel_id = execution_id
                await self._write_guard(execution_id)
                cancel_this = self._cancel_id == execution_id
            except asyncio.CancelledError:
                self._execution_id = self._accepted_id or self._execution_id
                self._cancel_id = self._execution_id
                await self._write_guard(self._execution_id)
                raise
            except Exception:
                if self._accepted_id is not None:
                    self._execution_id = self._accepted_id
                    self._cancel_id = self._accepted_id
                    await self._write_guard(self._accepted_id)
                else:
                    self._cancel_id = None
                    self._execution_id = None
                    await self._write_guard(None)
                raise
        finally:
            self._in_send = False
            self._accepted_id = None
            self._send_map_generation = None
        return execution_id, cancel_this

    async def _write_guard(self, execution_id: str | None) -> None:
        async def write() -> None:
            async with self._database.write_lock:
                connection = self._database.connection
                await connection.execute(
                    "INSERT INTO home_execution_guard (robot_id, execution_id) VALUES (?, ?) "
                    "ON CONFLICT(robot_id) DO UPDATE SET execution_id=excluded.execution_id",
                    (self._robot_id, execution_id),
                )
                await connection.commit()

        await _finish_guard_write(write())

    async def _clear_guard(self) -> None:
        async def clear() -> None:
            async with self._database.write_lock:
                connection = self._database.connection
                await connection.execute(
                    "DELETE FROM home_execution_guard WHERE robot_id = ?", (self._robot_id,)
                )
                await connection.commit()

        await _finish_guard_write(clear())
        self._guard_unresolved = False

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
        async with self._home_target_lock:
            snapshot = self._reference.snapshot(Path("pose_graph"), ())
            await self._store.sync_poses(snapshot.waypoint_bindings)
            stored = await self._store.home()
            if stored is None:
                raise DomainError(ErrorCode.VALIDATION, "home is not set")
            return stored.waypoint

    async def import_home(self, source: BundleWaypoint, *, anchor_pose: Pose2D) -> Waypoint:
        async with self._home_target_lock:
            return await self._import_home(source, anchor_pose=anchor_pose)

    async def _import_home(self, source: BundleWaypoint, *, anchor_pose: Pose2D) -> Waypoint:
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
        return waypoint

    async def save_waypoint(self, pose: LocalizedPose, *, waypoint_id: str) -> Waypoint:
        async with self._home_target_lock:
            return await self._save_waypoint(pose, waypoint_id=waypoint_id)

    async def _save_waypoint(self, pose: LocalizedPose, *, waypoint_id: str) -> Waypoint:
        if not self._pose_qualified(pose):
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "localization is unavailable")
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
        if not self._pose_qualified(pose):
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "localization is unavailable")
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
            if self._in_send:
                self._send_interrupted = True
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
            if self._send_interrupted or not self._generation_ready(self._send_map_generation):
                self._cancel_id = execution_id

    async def note_terminal(self, execution_id: str) -> None:
        async with self._return_lock:
            if self._execution_id != execution_id:
                return
            navigation = self._navigation
            if navigation is not None:
                result = await navigation.cancel(execution_id)
                if result.status != OperationStatus.SUCCEEDED:
                    raise DomainError(ErrorCode.CONFLICT, "navigation cancel was rejected")
            await self._clear_guard()
            self._execution_id = None
            if self._cancel_id == execution_id:
                self._cancel_id = None

    async def _cancel_owed_goal(self) -> None:
        execution_id = self._execution_id
        assert execution_id is not None
        result = await self._finish_execution(execution_id)
        if result.status != OperationStatus.SUCCEEDED:
            raise DomainError(ErrorCode.CONFLICT, "navigation cancel was rejected")
        await self._clear_guard()
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
        if self._localized.map_generation != self._reference.map_generation:
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "localization is unavailable")
        if self._localized.map_ref != expected:
            raise DomainError(ErrorCode.CONFLICT, "map reference does not match")
        bindings = self._reference.snapshot(Path("pose_graph"), ()).waypoint_bindings
        binding = next(
            (item for item in bindings if item.waypoint.waypoint_id == waypoint.id), None
        )
        if (
            binding is None
            or binding.waypoint.review_required
            or binding.waypoint.anchor_id != waypoint.anchor_id
            or not _same_pose(binding.waypoint.pose, waypoint.pose)
            or not _same_pose(binding.anchor_pose, stored.anchor_pose)
        ):
            raise DomainError(ErrorCode.CONFLICT, "home binding needs review")
        if not self._connected:
            raise DomainError(ErrorCode.OFFLINE, "connection is offline")
        if self._navigation is None:
            raise DomainError(ErrorCode.OFFLINE, "navigation port is unavailable")
        return waypoint

    def _pose_qualified(self, pose: LocalizedPose) -> bool:
        return (
            pose.healthy
            and not self._reference.review_required
            and pose.map_ref == self._reference.map_ref
            and pose.map_generation is not None
            and pose.map_generation == self._reference.map_generation
        )

    def _generation_ready(self, generation: str | None) -> bool:
        return (
            generation is not None
            and not self._reference.review_required
            and self._reference.map_generation == generation
            and self._localized is not None
            and self._localized.healthy
            and self._localized.map_ref == self._reference.map_ref
            and self._localized.map_generation == generation
        )

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


def _same_pose(left: Pose2D, right: Pose2D) -> bool:
    # //Compatible: tolerate floating-point serialization round trips.
    return all(
        math.isclose(a, b, rel_tol=0.0, abs_tol=1e-9)
        for a, b in ((left.x, right.x), (left.y, right.y), (left.yaw, right.yaw))
    )


def _kept(stored: StoredWaypoint | None) -> bool:
    return (
        stored is not None
        and stored.adopted is True
        and stored.link_status == "confirmed"
        and stored.waypoint.status == "confirmed"
    )


async def _finish_guard_write(operation: Awaitable[None]) -> None:
    task = asyncio.ensure_future(operation)
    try:
        await asyncio.shield(task)
    except asyncio.CancelledError:
        await task
        raise


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
