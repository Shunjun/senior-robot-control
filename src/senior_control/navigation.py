"""Qualified route execution; public DTOs live in independent contracts."""

from collections.abc import Awaitable, Callable, Sequence

# //Compatible: old navigation DTO imports retain type identity.
from senior_contracts.navigation import MAX_ROUTE_WAYPOINTS as MAX_ROUTE_WAYPOINTS
from senior_contracts.navigation import RoutePlan as RoutePlan
from senior_contracts.navigation import RouteProgress as RouteProgress
from senior_contracts.navigation import RouteRejected as RouteRejected
from senior_contracts.navigation import RouteStatus as RouteStatus

from senior_control.models import MapRef, Pose2D


class QualifiedRoute:
    def __init__(
        self,
        navigate: Callable[[Pose2D], Awaitable[bool]],
        qualified: Callable[[MapRef, str], bool],
    ) -> None:
        self._navigate = navigate
        self._qualified = qualified

    async def run(self, ref: MapRef, generation: str, poses: Sequence[Pose2D]) -> bool:
        route = tuple(poses)
        if not route or not generation or not self._qualified(ref, generation):
            return False
        for pose in route:
            if not await self._navigate(pose) or not self._qualified(ref, generation):
                return False
        return True
