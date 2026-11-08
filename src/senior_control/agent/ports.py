"""Bounded IPC adapters for ROS runtime consumers."""

import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Protocol

from pydantic import BaseModel, ConfigDict, ValidationError

from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef, Pose2D
from senior_control.navigation import RoutePlan, RouteProgress
from senior_control.runtime.ipc import IpcError
from senior_control.tasks.models import TaskDefinition
from senior_control.waypoints.store import WaypointStore


class Requester(Protocol):
    async def request(self, kind: str, **fields: Any) -> dict[str, Any]: ...


class IpcMapPort:
    def __init__(self, client: Requester) -> None:
        self._client = client

    async def _call(self, kind: str, **fields: Any) -> None:
        try:
            reply = await self._client.request(kind, **fields)
        except (IpcError, OSError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS map service unavailable") from exc
        if reply.get("ok") is not True:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS map service unavailable")

    async def export(self, operation_id: str, map_ref: MapRef, destination: Path) -> None:
        await self._call(
            "map.export",
            operation_id=operation_id,
            map_id=map_ref.map_id,
            reference_revision=map_ref.reference_revision,
            path=str(destination),
        )

    async def import_(self, operation_id: str, path: Path) -> None:
        await self._call("map.import", operation_id=operation_id, path=str(path))

    async def clear(self, operation_id: str, expected: MapRef) -> None:
        await self._call(
            "map.clear",
            operation_id=operation_id,
            map_id=expected.map_id,
            reference_revision=expected.reference_revision,
        )


class IpcNavigation:
    """Only a matching explicit acknowledgement counts as an accepted action."""

    def __init__(self, client: Requester, waypoints: WaypointStore | None = None) -> None:
        self._client = client
        self._waypoints = waypoints

    async def _call(self, kind: str, execution_id: str, **fields: Any) -> dict[str, Any]:
        try:
            reply = await self._client.request(kind, execution_id=execution_id, **fields)
        except (IpcError, OSError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable") from exc
        if reply.get("ok") is not True or reply.get("execution_id") != execution_id:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable")
        return reply

    async def start(self, execution_id: str, task: TaskDefinition) -> None:
        # //Compatible: legacy task IPC cannot fence an accepted reference generation.
        del execution_id, task
        raise DomainError(ErrorCode.UNAVAILABLE, "qualified ROS navigation unavailable")

    async def start_qualified(
        self, execution_id: str, task: TaskDefinition, map_generation: str
    ) -> None:
        if self._waypoints is None:
            raise DomainError(ErrorCode.UNAVAILABLE, "route waypoints unavailable")
        poses = await self._waypoints.route_poses(task.map_ref, task.waypoint_ids)
        await self.start_poses(
            execution_id, task.map_ref, map_generation, poses, task_id=task.task_id
        )

    async def start_poses(
        self, execution_id: str, map_ref: MapRef, map_generation: str, poses: tuple[Pose2D, ...],
        *, task_id: str | None = None,
    ) -> None:
        plan = RoutePlan(
            execution_id=execution_id, map_id=map_ref.map_id, task_id=task_id,
            reference_revision=map_ref.reference_revision, map_generation=map_generation,
            poses=[[pose.x, pose.y, pose.yaw] for pose in poses],
        )
        reply = await self._call(
            "navigation.task_start", execution_id, plan=plan.model_dump()
        )
        if reply.get("map_generation") != map_generation:
            raise DomainError(ErrorCode.CONFLICT, "ROS accepted a different reference generation")

    async def cancel(self, execution_id: str) -> None:
        await self._call("navigation.task_cancel", execution_id)

    async def status(self, execution_id: str) -> RouteProgress | None:
        reply = await self._call("navigation.task_status", execution_id)
        return self._progress(reply)

    async def current(self) -> RouteProgress | None:
        try:
            reply = await self._client.request("navigation.current")
        except (IpcError, OSError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable") from exc
        if reply.get("ok") is not True:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS navigation unavailable")
        return self._progress(reply)

    async def cancel_current(self) -> None:
        current = await self.current()
        if current is not None:
            await self.cancel(current.execution_id)

    @staticmethod
    def _progress(reply: dict[str, Any]) -> RouteProgress | None:
        data = reply.get("progress")
        if data is None:
            return None
        try:
            return RouteProgress.model_validate(data)
        except ValidationError as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "invalid ROS navigation status") from exc


@dataclass(frozen=True)
class RuntimeState:
    localized: bool
    safety_clear: bool
    manual_driving: bool
    map_ref: MapRef | None
    execution_status: str
    map_generation: str | None = None
    localization_generation: str | None = None
    navigation_ready: bool = False
    route_progress: RouteProgress | None = None


class StateReply(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    v: Literal[1]
    ok: Literal[True]
    sampled_at: float
    localized: bool
    safety_clear: bool
    manual_driving: bool
    map_id: str | None
    reference_revision: int | None
    execution_status: str
    # //Compatible: older state replies have no qualified reference or navigation fields.
    map_generation: str | None = None
    localization_generation: str | None = None
    navigation_ready: bool = False
    route_progress: RouteProgress | None = None


class IpcState:
    def __init__(self, client: Requester, *, clock: Callable[[], float] = time.monotonic) -> None:
        self._client = client
        self._clock = clock

    async def read(self) -> RuntimeState:
        try:
            raw = await self._client.request("state.get")
            state = StateReply.model_validate(raw)
            age = self._clock() - state.sampled_at
            if not 0 <= age <= 0.5:
                raise ValueError("state is stale")
            if (state.map_id is None) != (state.reference_revision is None):
                raise ValueError("incomplete map reference")
            map_ref = (
                None
                if state.map_id is None or state.reference_revision is None
                else MapRef(state.map_id, state.reference_revision)
            )
            return RuntimeState(
                state.localized,
                state.safety_clear,
                state.manual_driving,
                map_ref,
                state.execution_status,
                state.map_generation,
                state.localization_generation,
                state.navigation_ready,
                state.route_progress,
            )
        except (IpcError, OSError, ValueError, ValidationError) as exc:
            raise DomainError(ErrorCode.UNAVAILABLE, "ROS state unavailable") from exc


class PoseReply(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, allow_inf_nan=False)
    v: Literal[1]
    ok: Literal[True]
    sampled_at: float
    healthy: Literal[True]
    map_id: str
    reference_revision: int
    anchor_id: str
    x: float
    y: float
    yaw: float
    anchor_x: float
    anchor_y: float
    anchor_yaw: float
    # //Compatible: legacy pose IPC remains readable but cannot authorize Home.
    map_generation: str | None = None
    simulation: bool = False


class IpcHomePort:
    def __init__(self, client: Requester, store: WaypointStore, *, robot_id: str) -> None:
        self._client = client
        self._store = store
        self._robot_id = robot_id
        self._state = IpcState(client)

    async def home_revision(self) -> int:
        return await self._store.home_revision()

    async def set_current(self, expected_revision: int) -> int:
        try:
            pose = PoseReply.model_validate(await self._client.request("pose.get"))
            map_ref = MapRef(pose.map_id, pose.reference_revision)
            state = await self._state.read()
        except (IpcError, OSError, ValueError, ValidationError) as exc:
            raise DomainError(
                ErrorCode.UNAVAILABLE, "qualified ROS localization unavailable"
            ) from exc
        if (
            not pose.simulation
            or not 0 <= time.monotonic() - pose.sampled_at <= 0.5
            or not state.localized
            or state.map_ref != map_ref
            or not pose.map_generation
            or pose.map_generation != state.map_generation
            or pose.map_generation != state.localization_generation
        ):
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "qualified pose unavailable")
        await self._store.save_home(
            waypoint_id=None,
            pose=Pose2D(pose.x, pose.y, pose.yaw),
            map_ref=map_ref,
            anchor_id=pose.anchor_id,
            anchor_pose=Pose2D(pose.anchor_x, pose.anchor_y, pose.anchor_yaw),
            source_robot_id=self._robot_id,
            status="confirmed",
            adopted=True,
            latitude=None,
            longitude=None,
            expected_revision=expected_revision,
        )
        return expected_revision + 1
