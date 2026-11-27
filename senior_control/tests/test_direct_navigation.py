from pathlib import Path

import pytest
from vehicle_manager.agent.local_state import RuntimeState
from vehicle_manager.agent.request_bindings import DirectNavigation
from vehicle_manager.storage.bindings import Database
from vehicle_manager.storage.domain_bindings import WaypointStore

from vehicle_core.errors import DomainError, ErrorCode
from vehicle_core.values import MapRef, Pose2D

REF = MapRef("sim", 1)
BODY = {"map_id": "sim", "reference_revision": 1, "grid_id": "current"}


class Navigation:
    def __init__(self) -> None:
        self.starts: list[tuple[str, MapRef, str, tuple[Pose2D, ...]]] = []
        self.cancels = 0

    async def start_poses(
        self, execution_id: str, ref: MapRef, generation: str, poses: tuple[Pose2D, ...]
    ) -> None:
        self.starts.append((execution_id, ref, generation, poses))

    async def cancel_current(self) -> None:
        self.cancels += 1


class Workspace:
    async def verify_navigation_target(self, ref: MapRef, grid_id: str, pose: Pose2D) -> None:
        if ref != REF or grid_id != "current" or pose.x > 1:
            raise DomainError(ErrorCode.CONFLICT, "not current free space")


class State:
    def __init__(self) -> None:
        self.generation: str | None = "reference-generation"
        self.grid_id: str | None = "current"

    async def read(self) -> RuntimeState:
        return RuntimeState(
            True,
            True,
            False,
            REF,
            "idle",
            self.generation,
            self.generation,
            True,
            map_grid_id=self.grid_id,
        )


async def test_direct_goal_requires_current_free_map_and_matching_generation(
    tmp_path: Path,
) -> None:
    async with Database(tmp_path / "robot.db") as database:
        store = WaypointStore(database, robot_id="r1")
        await store.prepare()
        state, navigation = State(), Navigation()
        direct = DirectNavigation(state, Workspace(), store, navigation)
        valid = {**BODY, "x": 0.5, "y": 0.25, "yaw": 0.0}
        await direct("navigation.goal", "direct", valid)
        assert navigation.starts == [
            ("direct", REF, "reference-generation", (Pose2D(0.5, 0.25, 0),))
        ]

        with pytest.raises(DomainError) as blocked:
            await direct("navigation.goal", "blocked", {**valid, "x": 2})
        assert blocked.value.code is ErrorCode.CONFLICT

        state.grid_id = "replacement"
        with pytest.raises(DomainError) as stale:
            await direct("navigation.goal", "old", valid)
        assert stale.value.code is ErrorCode.CONFLICT
        assert len(navigation.starts) == 1

        state.grid_id = None
        with pytest.raises(DomainError) as missing_grid:
            await direct("navigation.goal", "missing-grid", valid)
        assert missing_grid.value.code is ErrorCode.CONFLICT
        state.grid_id = "current"
        state.generation = None
        with pytest.raises(DomainError) as missing_reference:
            await direct("navigation.goal", "missing-reference", valid)
        assert missing_reference.value.code is ErrorCode.LOCALIZATION_UNAVAILABLE
        assert len(navigation.starts) == 1

        await direct("navigation.cancel", "cancel", {})
        assert navigation.cancels == 1


async def test_return_home_requires_adopted_confirmed_waypoint(tmp_path: Path) -> None:
    async with Database(tmp_path / "robot.db") as database:
        store = WaypointStore(database, robot_id="r1")
        await store.prepare()
        navigation = Navigation()
        direct = DirectNavigation(State(), Workspace(), store, navigation)
        home = {
            "waypoint_id": "home",
            "pose": Pose2D(0.3, 0.1, 0.0),
            "map_ref": REF,
            "anchor_id": "origin",
            "anchor_pose": Pose2D(0, 0, 0),
            "source_robot_id": "r1",
            "latitude": None,
            "longitude": None,
        }
        await store.save_home(**home, adopted=False, status="pending_confirmation")
        with pytest.raises(DomainError) as pending:
            await direct("navigation.return_home", "home", BODY)
        assert pending.value.code is ErrorCode.CONFLICT
        await store.save_home(**home, adopted=True, status="confirmed")
        await direct("navigation.return_home", "home", BODY)
        assert navigation.starts == [("home", REF, "reference-generation", (home["pose"],))]
