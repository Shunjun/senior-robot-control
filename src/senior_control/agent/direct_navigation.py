"""Qualify a browser-selected navigation target against the vehicle's live map."""

from collections.abc import Mapping
from typing import Any, Protocol

from pydantic import ValidationError

from senior_control.agent.ports import IpcState
from senior_control.agent.workspace import WorkspaceReader
from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef, Pose2D
from senior_control.protocol import MapRefPayload, ShortText
from senior_control.waypoints.store import WaypointStore


class _Navigation(Protocol):
    async def start_poses(
        self, execution_id: str, map_ref: MapRef, map_generation: str, poses: tuple[Pose2D, ...]
    ) -> None: ...

    async def cancel_current(self) -> None: ...


class _Reference(MapRefPayload):
    grid_id: ShortText


class _Goal(_Reference):
    x: float
    y: float
    yaw: float


class DirectNavigation:
    def __init__(
        self,
        state: IpcState,
        workspace: WorkspaceReader,
        waypoints: WaypointStore,
        navigation: _Navigation,
    ) -> None:
        self._state = state
        self._workspace = workspace
        self._waypoints = waypoints
        self._navigation = navigation

    async def __call__(self, kind: str, execution_id: str, body: Mapping[str, Any]) -> None:
        if kind == "navigation.cancel":
            await self._navigation.cancel_current()
            return
        try:
            if kind == "navigation.goal":
                goal = _Goal.model_validate(body)
                target: _Reference = goal
                pose = Pose2D(goal.x, goal.y, goal.yaw)
            elif kind == "navigation.return_home":
                target = _Reference.model_validate(body)
                pose = None
            else:
                raise ValueError("unsupported navigation operation")
            map_ref = MapRef(target.map_id, target.reference_revision)
        except (ValidationError, ValueError) as exc:
            raise DomainError(ErrorCode.VALIDATION, "invalid navigation target") from exc

        if pose is None:
            home = await self._waypoints.home()
            if (
                home is None
                or not home.adopted
                or home.link_status != "confirmed"
                or home.waypoint.status != "confirmed"
                or home.waypoint.map_ref != map_ref
            ):
                raise DomainError(ErrorCode.CONFLICT, "no confirmed Home on the current map")
            pose = home.waypoint.pose

        await self._workspace.verify_free_target(map_ref, target.grid_id, pose)
        live = await self._state.read()
        if live.map_ref != map_ref or live.map_generation != target.grid_id:
            raise DomainError(ErrorCode.CONFLICT, "navigation target belongs to an older map")
        if not live.localized or live.localization_generation != live.map_generation:
            raise DomainError(ErrorCode.LOCALIZATION_UNAVAILABLE, "live localization unavailable")
        if not live.navigation_ready:
            raise DomainError(ErrorCode.UNAVAILABLE, "qualified navigation unavailable")
        if live.manual_driving or not live.safety_clear:
            raise DomainError(ErrorCode.SAFETY_REJECTED, "navigation is not safe to start")

        # The ROS-side receiver checks the same generation again before accepting the goal.
        await self._navigation.start_poses(execution_id, map_ref, target.grid_id, (pose,))
