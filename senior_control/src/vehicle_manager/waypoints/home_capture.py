"""Original qualified Home capture and revision-CAS request; transport owners are injected."""

import time
from collections.abc import Awaitable, Callable
from typing import Generic

from senior_contracts.errors import ErrorCode

from vehicle_manager.waypoints.ports import (
    HomePoseReading,
    HomeStatePort,
    HomeWriter,
    MapT,
    PoseT,
    WaypointValuePorts,
)


class HomeCapture(Generic[MapT, PoseT]):
    def __init__(
        self,
        request_pose: Callable[[], Awaitable[object]],
        store: HomeWriter[PoseT],
        *,
        robot_id: str,
        state: HomeStatePort,
        parse_pose: Callable[[object], HomePoseReading],
        values: WaypointValuePorts[MapT, PoseT],
        transport_errors: tuple[type[Exception], ...],
    ) -> None:
        self._request_pose = request_pose
        self._store = store
        self._robot_id = robot_id
        self._state = state
        self._parse_pose = parse_pose
        self._values = values
        self._transport_errors = transport_errors

    async def home_revision(self) -> int:
        return await self._store.home_revision()

    async def set_current(self, expected_revision: int) -> int:
        try:
            pose = self._parse_pose(await self._request_pose())
            map_ref = self._values.map_reference(pose.map_id, pose.reference_revision)
            state = await self._state.read()
        except self._transport_errors as exc:
            raise self._values.error(
                ErrorCode.UNAVAILABLE, "qualified ROS localization unavailable"
            ) from exc
        if (
            not 0 <= time.monotonic() - pose.sampled_at <= 0.5
            or not state.localized
            or state.map_ref != map_ref
            or not pose.map_generation
            or pose.map_generation != state.map_generation
            or pose.map_generation != state.localization_generation
        ):
            raise self._values.error(
                ErrorCode.LOCALIZATION_UNAVAILABLE, "qualified pose unavailable"
            )
        await self._store.save_home(
            waypoint_id=None,
            pose=self._values.pose(pose.x, pose.y, pose.yaw),
            map_ref=map_ref,
            anchor_id=pose.anchor_id,
            anchor_pose=self._values.pose(pose.anchor_x, pose.anchor_y, pose.anchor_yaw),
            source_robot_id=self._robot_id,
            status="confirmed",
            adopted=True,
            latitude=None,
            longitude=None,
            expected_revision=expected_revision,
        )
        return expected_revision + 1
