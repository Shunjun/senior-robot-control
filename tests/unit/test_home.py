"""Home stays at the recoverable power-on pose and rejects a bad return."""

import asyncio
import importlib.util
from pathlib import Path

import pytest

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.evidence import CellEvidence
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D
from senior_control.storage.database import Database
from senior_control.waypoints import HomeService, LocalizedPose, StartupReference

ROOT = Path(__file__).resolve().parents[2]
MAP = MapRef("yard", 3)
ROBOT = "robot-a"
CELL = 0.5
STILL = StartupReference(False, True, True, None)


def localized(
    pose: Pose2D,
    *,
    healthy: bool = True,
    latitude: float | None = None,
    longitude: float | None = None,
    map_ref: MapRef = MAP,
) -> LocalizedPose:
    return LocalizedPose(
        pose,
        map_ref,
        "node-1",
        Pose2D(0.0, 0.0, 0.0),
        healthy,
        latitude,
        longitude,
    )


def navigation_module() -> object:
    path = ROOT / "ros2" / "src" / "senior_robot" / "senior_robot" / "navigation_adapter.py"
    spec = importlib.util.spec_from_file_location("navigation_adapter_under_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RecordingNavigation:
    def __init__(self) -> None:
        self.goals: list[object] = []

    async def navigate(self, goal: object) -> str:
        self.goals.append(goal)
        return "exec-1"

    async def cancel(self, execution_id: str) -> OperationResult:
        return OperationResult(execution_id, OperationStatus.SUCCEEDED)


async def test_home_persists_restart(tmp_path: Path) -> None:
    path = tmp_path / "home.db"
    async with Database(path) as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        first = await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        assert first.initialized
        assert first.waypoint is not None
        assert first.waypoint.pose == Pose2D(1.0, 2.0, 0.4)
        original_id = first.waypoint.id
    async with Database(path) as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        again = await service.initialize(STILL, localized(Pose2D(9.0, 9.0, 1.0)))
        assert again.initialized
        assert again.waypoint is not None
        assert again.waypoint.pose == Pose2D(1.0, 2.0, 0.4)
        assert again.waypoint.id == original_id


async def test_moved_before_localization_not_silently_home(tmp_path: Path) -> None:
    async with Database(tmp_path / "moved.db") as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        later = localized(Pose2D(5.0, 6.0, 0.2))
        for start in (
            StartupReference(True, False, False, None),
            StartupReference(True, True, False, Pose2D(1.0, 0.0, 0.0)),
            StartupReference(True, False, True, Pose2D(1.0, 0.0, 0.0)),
        ):
            result = await service.initialize(start, later)
            assert result.initialized is False
            assert result.waypoint is None
            assert result.reason == "home is uninitialized"
            assert await service.home() is None


async def test_trusted_start_transform_recovers_start(tmp_path: Path) -> None:
    async with Database(tmp_path / "trusted.db") as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        current = Pose2D(5.0, 5.0, 1.5707963267948966)
        result = await service.initialize(
            StartupReference(True, True, True, Pose2D(2.0, 0.0, 1.5707963267948966)),
            localized(current, latitude=31.2, longitude=121.4),
        )
        assert result.initialized
        assert result.reason == ""
        assert result.waypoint is not None
        assert result.waypoint.kind == "home"
        assert result.waypoint.map_ref == MAP
        assert result.waypoint.anchor_id == "node-1"
        assert result.waypoint.source_robot_id == ROBOT
        assert result.waypoint.status == "confirmed"
        assert result.waypoint.pose.x == pytest.approx(3.0)
        assert result.waypoint.pose.y == pytest.approx(5.0)
        assert result.waypoint.pose.yaw == pytest.approx(0.0)
        assert result.waypoint.pose != current
        assert result.waypoint.latitude is None
        assert result.waypoint.longitude is None


async def test_obstacle_does_not_move_home(tmp_path: Path) -> None:
    async with Database(tmp_path / "obstacle.db") as database:
        reference = MapReferenceStore(MAP, "odom", CELL)
        service = HomeService(database, reference, robot_id=ROBOT)
        await service.prepare()
        created = await service.initialize(STILL, localized(Pose2D(4.0, 5.0, 0.2)))
        assert created.waypoint is not None
        original = created.waypoint.pose
        await service.observe_obstacle(
            CellEvidence((0, 0), "node-1", 4, 0, 1.0, 0.95, "geometric_confirmation"),
            world=(0.1, 0.1),
            anchor_pose=Pose2D(0.0, 0.0, 0.0),
        )
        current = await service.home()
        assert current is not None
        assert current.pose == original
        assert current.id == created.waypoint.id


async def test_reference_mismatch_rejected(tmp_path: Path) -> None:
    navigation = RecordingNavigation()
    async with Database(tmp_path / "mismatch.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        created = await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        assert created.initialized
        with pytest.raises(DomainError) as caught:
            await service.return_home(MapRef("other-map", MAP.reference_revision))
        assert caught.value.code == ErrorCode.CONFLICT
        assert str(caught.value) == "map reference does not match"
        with pytest.raises(DomainError) as revision:
            await service.return_home(MapRef(MAP.map_id, MAP.reference_revision + 1))
        assert revision.value.code == ErrorCode.CONFLICT
        assert str(revision.value) == "map reference does not match"
        assert navigation.goals == []


class RejectingExecutor:
    def __init__(self) -> None:
        self.started: list[object] = []

    async def start(self, goal: object) -> str:
        self.started.append(goal)
        return "nav-1"

    async def cancel(self, execution_id: str) -> OperationResult:
        return OperationResult(execution_id, OperationStatus.FAILED)


class AdapterPort:
    def __init__(self, executor: RejectingExecutor) -> None:
        module = navigation_module()
        self._module = module
        self._adapter = module.NavigationAdapter(executor)

    async def navigate(self, goal: object) -> str:
        map_goal = self._module.MapGoal(goal.map_ref, goal.pose)
        return await self._adapter.navigate(map_goal)

    async def cancel(self, execution_id: str) -> OperationResult:
        return await self._adapter.cancel(execution_id)


async def test_rejected_cancel_does_not_start_replacement(tmp_path: Path) -> None:
    executor = RejectingExecutor()
    async with Database(tmp_path / "cancel.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=AdapterPort(executor),
        )
        await service.prepare()
        created = await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        assert created.waypoint is not None
        first = await service.return_home(MAP)
        assert first == "nav-1"
        assert len(executor.started) == 1
        caught: Exception | None = None
        try:
            await service.return_home(MAP)
        except Exception as exc:
            caught = exc
        assert isinstance(caught, DomainError)
        assert caught.code == ErrorCode.CONFLICT
        assert str(caught) == "navigation cancel was rejected"
        assert len(executor.started) == 1


async def test_unhealthy_localization_rejects_return(tmp_path: Path) -> None:
    navigation = RecordingNavigation()
    async with Database(tmp_path / "unhealthy.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4), healthy=False))
        with pytest.raises(DomainError) as caught:
            await service.return_home(MAP)
        assert caught.value.code == ErrorCode.LOCALIZATION_UNAVAILABLE
        assert str(caught.value) == "localization is unhealthy"
        assert navigation.goals == []


async def test_missing_port_rejects_without_navigation(tmp_path: Path) -> None:
    async with Database(tmp_path / "noport.db") as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        with pytest.raises(DomainError) as caught:
            await service.return_home(MAP)
        assert caught.value.code == ErrorCode.OFFLINE
        assert str(caught.value) == "navigation port is unavailable"


def test_cancel_status_waits_for_a_terminal_goal() -> None:
    module = navigation_module()
    assert module.cancel_status(0, False) is OperationStatus.FAILED
    assert module.cancel_status(0, True) is OperationStatus.SUCCEEDED
    assert module.cancel_status(3, False) is OperationStatus.SUCCEEDED
    assert module.cancel_status(1, True) is OperationStatus.FAILED


class TerminalCodeExecutor:
    def __init__(self, module: object) -> None:
        self._module = module
        self.started: list[object] = []

    async def start(self, goal: object) -> str:
        self.started.append(goal)
        return f"nav-{len(self.started)}"

    async def cancel(self, execution_id: str) -> OperationResult:
        status = self._module.cancel_status(3, False)
        return OperationResult(execution_id, status)


async def test_finished_goal_does_not_block_the_next_return(tmp_path: Path) -> None:
    module = navigation_module()
    executor = TerminalCodeExecutor(module)
    async with Database(tmp_path / "finished.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=AdapterPort(executor),
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        first = await service.return_home(MAP)
        assert first == "nav-1"
        started = False
        try:
            started = await service.return_home(MAP) == "nav-2"
        except DomainError:
            started = False
        assert started
        assert len(executor.started) == 2


class SlowNavigation:
    def __init__(self) -> None:
        self.started = 0
        self.release = asyncio.Event()

    async def navigate(self, goal: object) -> str:
        del goal
        self.started += 1
        entered = self.started
        await self.release.wait()
        return f"exec-{entered}"

    async def cancel(self, execution_id: str) -> OperationResult:
        return OperationResult(execution_id, OperationStatus.SUCCEEDED)


async def test_overlapping_returns_do_not_both_navigate(tmp_path: Path) -> None:
    navigation = SlowNavigation()
    async with Database(tmp_path / "overlap.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        first = asyncio.create_task(service.return_home(MAP))
        second = asyncio.create_task(service.return_home(MAP))
        try:
            for _ in range(50):
                if navigation.started:
                    break
                await asyncio.sleep(0.01)
            await asyncio.sleep(0.05)
            assert navigation.started == 1
        finally:
            navigation.release.set()
        assert await first == "exec-1"
        assert await second == "exec-2"


class FailingSecondNavigation:
    def __init__(self) -> None:
        self.starts = 0
        self.cancelled: list[str] = []

    async def navigate(self, goal: object) -> str:
        del goal
        self.starts += 1
        if self.starts == 2:
            raise RuntimeError("send failed")
        return f"exec-{self.starts}"

    async def cancel(self, execution_id: str) -> OperationResult:
        self.cancelled.append(execution_id)
        return OperationResult(execution_id, OperationStatus.FAILED)


async def test_failed_send_clears_a_finished_execution(tmp_path: Path) -> None:
    navigation = FailingSecondNavigation()
    async with Database(tmp_path / "send-fail.db") as database:
        service = HomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        assert await service.return_home(MAP) == "exec-1"
        await service.note_terminal("exec-1")
        sent = False
        try:
            await service.return_home(MAP)
        except RuntimeError as exc:
            sent = str(exc) == "send failed"
        assert sent
        again = False
        try:
            again = await service.return_home(MAP) == "exec-3"
        except DomainError:
            again = False
        assert again
        assert navigation.starts == 3
        assert navigation.cancelled == []


async def test_recovered_start_keeps_its_own_fix(tmp_path: Path) -> None:
    async with Database(tmp_path / "fix.db") as database:
        service = HomeService(database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT)
        await service.prepare()
        result = await service.initialize(
            StartupReference(
                True,
                True,
                True,
                Pose2D(2.0, 0.0, 1.5707963267948966),
                30.0,
                120.0,
            ),
            localized(
                Pose2D(5.0, 5.0, 1.5707963267948966),
                latitude=31.2,
                longitude=121.4,
            ),
        )
        assert result.waypoint is not None
        assert result.waypoint.latitude == pytest.approx(30.0)
        assert result.waypoint.longitude == pytest.approx(120.0)
