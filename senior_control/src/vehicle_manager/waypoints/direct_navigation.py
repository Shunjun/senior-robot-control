"""Original management target validation, before the existing durable task receiver."""

from collections.abc import Mapping
from typing import Any, Generic

from pydantic import ValidationError
from senior_contracts.errors import ErrorCode
from senior_contracts.wire import MapRefPayload, ShortText

from vehicle_manager.waypoints.ports import (
    HomeReader,
    MapT,
    NavigationStatePort,
    NavigationTargetPort,
    PoseT,
    WaypointValuePorts,
    WorkspaceTargetPort,
)


class _Reference(MapRefPayload):
    grid_id: ShortText


class _Goal(_Reference):
    x: float
    y: float
    yaw: float


class DirectNavigation(Generic[MapT, PoseT]):
    def __init__(
        self,
        state: NavigationStatePort,
        workspace: WorkspaceTargetPort[MapT, PoseT],
        waypoints: HomeReader[PoseT],
        navigation: NavigationTargetPort[MapT, PoseT],
        *,
        values: WaypointValuePorts[MapT, PoseT],
    ) -> None:
        self._state = state
        self._workspace = workspace
        self._waypoints = waypoints
        self._navigation = navigation
        self._values = values

    async def __call__(self, kind: str, execution_id: str, body: Mapping[str, Any]) -> None:
        if kind == "navigation.cancel":
            await self._navigation.cancel_current()
            return
        try:
            if kind == "navigation.goal":
                goal = _Goal.model_validate(body)
                target: _Reference = goal
                pose = self._values.pose(goal.x, goal.y, goal.yaw)
            elif kind == "navigation.return_home":
                target = _Reference.model_validate(body)
                pose = None
            else:
                raise ValueError("unsupported navigation operation")
            map_ref = self._values.map_reference(target.map_id, target.reference_revision)
        except (ValidationError, ValueError) as exc:
            raise self._values.error(ErrorCode.VALIDATION, "invalid navigation target") from exc

        if pose is None:
            home = await self._waypoints.home()
            if (
                home is None
                or not home.adopted
                or home.link_status != "confirmed"
                or home.waypoint.status != "confirmed"
                or home.waypoint.map_ref != map_ref
            ):
                raise self._values.error(ErrorCode.CONFLICT, "no confirmed Home on the current map")
            pose = home.waypoint.pose

        await self._workspace.verify_navigation_target(map_ref, target.grid_id, pose)
        live = await self._state.read()
        if live.map_ref != map_ref or live.map_grid_id != target.grid_id:
            raise self._values.error(
                ErrorCode.CONFLICT, "navigation target belongs to an older map"
            )
        if (
            not live.map_generation
            or not live.localized
            or live.localization_generation != live.map_generation
        ):
            raise self._values.error(
                ErrorCode.LOCALIZATION_UNAVAILABLE, "live localization unavailable"
            )
        if not live.navigation_ready:
            raise self._values.error(ErrorCode.UNAVAILABLE, "qualified navigation unavailable")
        if live.manual_driving or not live.safety_clear:
            raise self._values.error(ErrorCode.SAFETY_REJECTED, "navigation is not safe to start")

        # The ROS-side receiver checks the same generation again before accepting the goal.
        await self._navigation.start_poses(execution_id, map_ref, live.map_generation, (pose,))
