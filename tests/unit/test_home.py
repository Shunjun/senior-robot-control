"""Home stays at the recoverable power-on pose and rejects a bad return."""

import asyncio
import importlib.util
from dataclasses import replace
from pathlib import Path

import pytest

from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.evidence import CellEvidence
from senior_control.mapping.store import MapReferenceStore
from senior_control.models import MapRef, OperationResult, OperationStatus, Pose2D
from senior_control.storage.database import Database
from senior_control.waypoints import HomeService, LocalizedPose, StartupReference


class SyntheticHomeService(HomeService):
    """A local fixture supplies a fresh generation with each synthetic observation."""

    async def initialize(self, start: StartupReference, localized: LocalizedPose):
        if localized.map_generation is None:
            localized = replace(localized, map_generation=self._reference.map_generation)
        return await super().initialize(start, localized)

    async def set_current(self, pose: LocalizedPose, *, expected_revision: int | None = None):
        if pose.map_generation is None:
            pose = replace(pose, map_generation=self._reference.map_generation)
        return await super().set_current(pose, expected_revision=expected_revision)

    def _remember(self, localized: LocalizedPose) -> None:
        if localized.map_generation is None:
            localized = replace(localized, map_generation=self._reference.map_generation)
        super()._remember(localized)

    async def note_localization(self, pose: LocalizedPose) -> None:
        if pose.map_generation is None:
            pose = replace(pose, map_generation=self._reference.map_generation)
        await super().note_localization(pose)


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


class SyntheticQualifiedNavigation:
    """Single-loop fake; real receivers must independently enforce the generation."""

    async def navigate_qualified(self, goal: object) -> str:
        return await self.navigate(goal)  # type: ignore[attr-defined,no-any-return]


class RecordingNavigation(SyntheticQualifiedNavigation):
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
        service = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT
        )
        await service.prepare()
        first = await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        assert first.initialized
        assert first.waypoint is not None
        assert first.waypoint.pose == Pose2D(1.0, 2.0, 0.4)
        original_id = first.waypoint.id
    async with Database(path) as database:
        service = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT
        )
        await service.prepare()
        again = await service.initialize(STILL, localized(Pose2D(9.0, 9.0, 1.0)))
        assert again.initialized
        assert again.waypoint is not None
        assert again.waypoint.pose == Pose2D(1.0, 2.0, 0.4)
        assert again.waypoint.id == original_id


async def test_moved_before_localization_not_silently_home(tmp_path: Path) -> None:
    async with Database(tmp_path / "moved.db") as database:
        service = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT
        )
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
        service = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT
        )
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
        service = SyntheticHomeService(database, reference, robot_id=ROBOT)
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
        service = SyntheticHomeService(
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


class AdapterPort(SyntheticQualifiedNavigation):
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
        service = SyntheticHomeService(
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
        service = SyntheticHomeService(
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
        service = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT
        )
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
        service = SyntheticHomeService(
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


class SlowNavigation(SyntheticQualifiedNavigation):
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
        service = SyntheticHomeService(
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


class FailingSecondNavigation(SyntheticQualifiedNavigation):
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
        return OperationResult(execution_id, OperationStatus.SUCCEEDED)


async def test_failed_send_keeps_unknown_separate_from_finished_execution(tmp_path: Path) -> None:
    navigation = FailingSecondNavigation()
    async with Database(tmp_path / "send-fail.db") as database:
        service = SyntheticHomeService(
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
        with pytest.raises(DomainError) as error:
            await service.return_home(MAP)
        assert error.value.code is ErrorCode.CONFLICT
        assert navigation.starts == 2
        assert navigation.cancelled == ["exec-1"]


async def test_recovered_start_keeps_its_own_fix(tmp_path: Path) -> None:
    async with Database(tmp_path / "fix.db") as database:
        service = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT
        )
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


class AcceptThenWait(SyntheticQualifiedNavigation):
    """Goal is accepted before navigate returns, so disconnect can land in that window."""

    def __init__(self) -> None:
        self.events: list[str] = []
        self.accepted = asyncio.Event()
        self.release = asyncio.Event()
        self.active: str | None = None

    async def navigate(self, goal: object) -> str:
        self.active = "nav-1"
        self.events.append("navigate")
        notify = getattr(goal, "on_accepted", None)
        if notify is not None:
            notify(self.active)
        self.accepted.set()
        await self.release.wait()
        return "nav-1"

    async def cancel(self, execution_id: str) -> OperationResult:
        self.events.append("cancel")
        self.active = None
        return OperationResult(execution_id, OperationStatus.SUCCEEDED)


async def test_disconnect_during_navigate_cancels_the_accepted_goal(tmp_path: Path) -> None:
    navigation = AcceptThenWait()
    async with Database(tmp_path / "inflight.db") as database:
        service = SyntheticHomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        task = asyncio.create_task(service.return_home(MAP))
        try:
            await asyncio.wait_for(navigation.accepted.wait(), timeout=1)
            await service.note_connection(False)
            assert navigation.events == ["navigate"]
            await service.note_connection(True)
            assert navigation.events == ["navigate"]
        finally:
            navigation.release.set()
        await task
        assert navigation.events == ["navigate", "cancel"]
        assert navigation.active is None


class ReleasingExecutor:
    def __init__(self) -> None:
        self.started: list[object] = []

    async def start(self, goal: object) -> str:
        self.started.append(goal)
        return f"nav-{len(self.started)}"

    async def cancel(self, execution_id: str) -> OperationResult:
        return OperationResult(execution_id, OperationStatus.SUCCEEDED)


async def test_terminal_note_releases_the_adapter_goal(tmp_path: Path) -> None:
    executor = ReleasingExecutor()
    async with Database(tmp_path / "terminal.db") as database:
        service = SyntheticHomeService(
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
        await service.note_terminal(first)
        second: str | None = None
        caught: Exception | None = None
        try:
            second = await service.return_home(MAP)
        except Exception as exc:
            caught = exc
        assert caught is None
        assert second == "nav-2"
        assert len(executor.started) == 2


class KeepingNavigation(SyntheticQualifiedNavigation):
    def __init__(self) -> None:
        self.events: list[str] = []
        self.active: str | None = None

    async def navigate(self, goal: object) -> str:
        del goal
        self.active = "nav-1"
        self.events.append("navigate")
        return "nav-1"

    async def cancel(self, execution_id: str) -> OperationResult:
        self.events.append("cancel")
        self.active = None
        return OperationResult(execution_id, OperationStatus.SUCCEEDED)


async def test_disconnect_before_the_goal_is_accepted_keeps_the_later_goal(
    tmp_path: Path,
) -> None:
    navigation = KeepingNavigation()
    async with Database(tmp_path / "before-accept.db") as database:
        service = SyntheticHomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        await database.write_lock.acquire()
        task = asyncio.create_task(service.return_home(MAP))
        try:
            entered = False
            for _ in range(50):
                waiters = getattr(database.write_lock, "_waiters", None)
                if waiters:
                    entered = True
                    break
                await asyncio.sleep(0)
            assert entered
            await service.note_connection(False)
            await service.note_connection(True)
            assert navigation.events == []
        finally:
            database.write_lock.release()
        assert await task == "nav-1"
        assert navigation.events == ["navigate"]
        assert navigation.active == "nav-1"


class FailThenAccept(SyntheticQualifiedNavigation):
    def __init__(self) -> None:
        self.events: list[str] = []
        self.started = asyncio.Event()
        self.release = asyncio.Event()
        self.active: str | None = None

    async def navigate(self, goal: object) -> str:
        del goal
        self.events.append("navigate")
        if self.events.count("navigate") == 1:
            self.started.set()
            await self.release.wait()
            raise RuntimeError("send failed")
        self.active = "nav-2"
        return "nav-2"

    async def cancel(self, execution_id: str) -> OperationResult:
        self.events.append("cancel")
        self.active = None
        return OperationResult(execution_id, OperationStatus.SUCCEEDED)


async def test_failed_send_without_acceptance_stays_unknown(tmp_path: Path) -> None:
    navigation = FailThenAccept()
    async with Database(tmp_path / "stale-debt.db") as database:
        service = SyntheticHomeService(
            database,
            MapReferenceStore(MAP, "odom", CELL),
            robot_id=ROBOT,
            navigation=navigation,
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        task = asyncio.create_task(service.return_home(MAP))
        await asyncio.wait_for(navigation.started.wait(), timeout=1)
        await service.note_connection(False)
        await service.note_connection(True)
        navigation.release.set()
        with pytest.raises(RuntimeError, match="send failed"):
            await task
        assert navigation.events == ["navigate"]
        with pytest.raises(DomainError) as error:
            await service.return_home(MAP)
        assert error.value.code is ErrorCode.CONFLICT
        assert navigation.events == ["navigate"]
        assert navigation.active is None


async def test_cancelled_accepted_home_goal_survives_disconnect_and_restart(tmp_path: Path) -> None:
    class AcceptedUnknown(AcceptThenWait):
        def __init__(self) -> None:
            super().__init__()
            self.cancel_status = OperationStatus.FAILED

        async def cancel(self, execution_id: str) -> OperationResult:
            self.events.append("cancel:" + execution_id)
            if self.cancel_status is OperationStatus.SUCCEEDED:
                self.active = None
            return OperationResult(execution_id, self.cancel_status)

    navigation = AcceptedUnknown()
    path = tmp_path / "accepted.db"
    first_ref = MapReferenceStore(MAP, "odom", CELL)
    async with Database(path) as database:
        service = SyntheticHomeService(database, first_ref, robot_id=ROBOT, navigation=navigation)
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        task = asyncio.create_task(service.return_home(MAP))
        await asyncio.wait_for(navigation.accepted.wait(), 1)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        with pytest.raises(DomainError):
            await service.note_connection(False)
        assert navigation.events == ["navigate", "cancel:nav-1"]
        assert navigation.active == "nav-1"
    saved_snapshot = first_ref.snapshot(Path("pose_graph"), ())
    next_ref = MapReferenceStore(MAP, "odom", CELL)
    next_ref.replace(saved_snapshot)
    async with Database(path) as database:
        service = SyntheticHomeService(database, next_ref, robot_id=ROBOT, navigation=navigation)
        await service.prepare()
        await service.note_localization(localized(Pose2D(1.0, 2.0, 0.4)))
        with pytest.raises(DomainError):
            await service.return_home(MAP)
        assert navigation.events == ["navigate", "cancel:nav-1", "cancel:nav-1"]
        navigation.cancel_status = OperationStatus.SUCCEEDED
        navigation.release.set()
        assert await service.return_home(MAP) == "nav-1"
        assert navigation.events == [
            "navigate",
            "cancel:nav-1",
            "cancel:nav-1",
            "cancel:nav-1",
            "navigate",
        ]


@pytest.mark.parametrize("reconnect", [False, True])
async def test_disconnect_before_acceptance_cancels_late_accepted_goal(
    tmp_path: Path, reconnect: bool
) -> None:
    class LateAcceptance(AcceptThenWait):
        async def navigate(self, goal: object) -> str:
            self.events.append("navigate")
            self.accepted.set()
            await self.release.wait()
            self.active = "nav-1"
            notify = goal.on_accepted
            notify(self.active)
            return self.active

    navigation = LateAcceptance()
    async with Database(tmp_path / "late.db") as database:
        service = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT, navigation=navigation
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1.0, 2.0, 0.4)))
        task = asyncio.create_task(service.return_home(MAP))
        await asyncio.wait_for(navigation.accepted.wait(), 1)
        await service.note_connection(False)
        if reconnect:
            await service.note_connection(True)
        navigation.release.set()
        await task
        assert navigation.events == ["navigate", "cancel"]
        assert navigation.active is None


async def test_home_without_generation_fenced_port_is_unavailable(tmp_path: Path) -> None:
    class LegacyNavigation(RecordingNavigation):
        navigate_qualified = None  # type: ignore[assignment]

    navigation = LegacyNavigation()
    async with Database(tmp_path / "legacy.db") as database:
        service = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT, navigation=navigation
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1, 2, 0)))
        with pytest.raises(DomainError) as error:
            await service.return_home(MAP)
        assert error.value.code is ErrorCode.UNAVAILABLE
        assert navigation.goals == []


async def test_map_switch_during_home_send_cancels_accepted_goal(tmp_path: Path) -> None:
    navigation = AcceptThenWait()
    reference = MapReferenceStore(MAP, "odom", CELL)
    async with Database(tmp_path / "switch.db") as database:
        service = SyntheticHomeService(database, reference, robot_id=ROBOT, navigation=navigation)
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1, 2, 0)))
        task = asyncio.create_task(service.return_home(MAP))
        await asyncio.wait_for(navigation.accepted.wait(), 1)
        reference.guard_reference()
        navigation.release.set()
        await task
        assert navigation.events == ["navigate", "cancel"]
        assert navigation.active is None


async def test_home_without_localization_generation_stays_unavailable(tmp_path: Path) -> None:
    navigation = RecordingNavigation()
    reference = MapReferenceStore(MAP, "odom", CELL)
    async with Database(tmp_path / "no-generation.db") as database:
        service = HomeService(database, reference, robot_id=ROBOT, navigation=navigation)
        await service.prepare()
        pose = replace(localized(Pose2D(1, 2, 0)), map_generation=reference.map_generation)
        await service.initialize(STILL, pose)
        await service.note_localization(localized(Pose2D(1, 2, 0)))
        with pytest.raises(DomainError) as error:
            await service.return_home(MAP)
        assert error.value.code is ErrorCode.LOCALIZATION_UNAVAILABLE
        assert navigation.goals == []


async def test_home_reference_nonce_survives_obstacle_and_new_anchor_but_not_moved_anchor(
    tmp_path: Path,
) -> None:
    from senior_control.mapping.references import AnchorPose, AnchorSnapshot, GridExtent

    reference = MapReferenceStore(MAP, "odom", CELL)
    navigation = RecordingNavigation()
    async with Database(tmp_path / "anchor-qualification.db") as database:
        service = HomeService(database, reference, robot_id=ROBOT, navigation=navigation)
        await service.prepare()
        pose = replace(localized(Pose2D(1, 2, 0)), map_generation=reference.map_generation)
        await service.initialize(STILL, pose)
        generation = reference.map_generation
        obstacle = CellEvidence((2, 0), "node-1", 3, 0, 1.0, 3.0, "geometric_confirmation")
        reference.place(obstacle, world=(1, 0), anchor_pose=Pose2D(0, 0, 0))
        reference.apply_anchor_update(
            AnchorSnapshot(
                MAP,
                "odom",
                (
                    AnchorPose("node-1", Pose2D(0, 0, 0), True),
                    AnchorPose("node-new", Pose2D(4, 0, 0), True),
                ),
                GridExtent(8, 8, 0, 0),
            )
        )
        assert reference.map_generation == generation
        assert await service.return_home(MAP) == "exec-1"
        await service.note_terminal("exec-1")
        reference.apply_anchor_update(
            AnchorSnapshot(
                MAP,
                "odom",
                (
                    AnchorPose("node-1", Pose2D(1, 0, 0), True),
                    AnchorPose("node-new", Pose2D(4, 0, 0), True),
                ),
                GridExtent(8, 8, 0, 0),
            )
        )
        with pytest.raises(DomainError) as error:
            await service.return_home(MAP)
        assert error.value.code is ErrorCode.LOCALIZATION_UNAVAILABLE
        assert len(navigation.goals) == 1


@pytest.mark.parametrize("reconnect", [False, True])
async def test_late_acceptance_with_rejected_cancel_blocks_replacement_after_restart(
    tmp_path: Path, reconnect: bool
) -> None:
    class RejectLate(AcceptThenWait):
        async def navigate(self, goal: object) -> str:
            self.events.append("navigate")
            self.accepted.set()
            await self.release.wait()
            self.active = "nav-1"
            goal.on_accepted(self.active)
            return self.active

        async def cancel(self, execution_id: str) -> OperationResult:
            self.events.append("cancel:" + execution_id)
            return OperationResult(execution_id, OperationStatus.NEEDS_REVIEW)

    navigation = RejectLate()
    path = tmp_path / "late-rejected.db"
    async with Database(path) as database:
        service = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT, navigation=navigation
        )
        await service.prepare()
        await service.initialize(STILL, localized(Pose2D(1, 2, 0)))
        send = asyncio.create_task(service.return_home(MAP))
        await asyncio.wait_for(navigation.accepted.wait(), 1)
        await service.note_connection(False)
        if reconnect:
            await service.note_connection(True)
        navigation.release.set()
        with pytest.raises(DomainError):
            await send
        assert navigation.events == ["navigate", "cancel:nav-1"]
        assert navigation.active == "nav-1"
    async with Database(path) as database:
        restarted = SyntheticHomeService(
            database, MapReferenceStore(MAP, "odom", CELL), robot_id=ROBOT, navigation=navigation
        )
        await restarted.prepare()
        await restarted.note_localization(localized(Pose2D(1, 2, 0)))
        with pytest.raises(DomainError):
            await restarted.return_home(MAP)
        assert navigation.events == ["navigate", "cancel:nav-1", "cancel:nav-1"]


async def test_home_cannot_be_set_from_unqualified_localization(tmp_path: Path) -> None:
    reference = MapReferenceStore(MAP, "odom", CELL)
    async with Database(tmp_path / "set-unqualified.db") as database:
        service = HomeService(database, reference, robot_id=ROBOT)
        await service.prepare()
        with pytest.raises(DomainError) as error:
            await service.set_current(localized(Pose2D(1, 2, 0)))
        assert error.value.code is ErrorCode.LOCALIZATION_UNAVAILABLE
        assert await service.home() is None


async def test_home_initialize_without_generation_does_not_store_pose(tmp_path: Path) -> None:
    reference = MapReferenceStore(MAP, "odom", CELL)
    async with Database(tmp_path / "initialize-unqualified.db") as database:
        service = HomeService(database, reference, robot_id=ROBOT)
        await service.prepare()
        result = await service.initialize(STILL, localized(Pose2D(1, 2, 0)))
        assert not result.initialized
        assert await service.home() is None


async def test_home_restart_rejects_old_localization_nonce(tmp_path: Path) -> None:
    path = tmp_path / "restart-localization.db"
    first_ref = MapReferenceStore(MAP, "odom", CELL)
    old = replace(localized(Pose2D(1, 2, 0)), map_generation=first_ref.map_generation)
    async with Database(path) as database:
        service = HomeService(database, first_ref, robot_id=ROBOT)
        await service.prepare()
        result = await service.initialize(STILL, old)
        assert result.initialized
    next_ref = MapReferenceStore(MAP, "odom", CELL)
    next_ref.replace(first_ref.snapshot(Path("pose_graph"), ()))
    navigation = RecordingNavigation()
    async with Database(path) as database:
        restarted = HomeService(database, next_ref, robot_id=ROBOT, navigation=navigation)
        await restarted.prepare()
        await restarted.note_localization(old)
        with pytest.raises(DomainError) as error:
            await restarted.return_home(MAP)
        assert error.value.code is ErrorCode.LOCALIZATION_UNAVAILABLE
        fresh = replace(old, map_generation=next_ref.map_generation)
        await restarted.note_localization(fresh)
        assert await restarted.return_home(MAP) == "exec-1"


async def test_map_switch_during_home_save_never_binds_the_stale_pose(tmp_path: Path) -> None:
    reference = MapReferenceStore(MAP, "odom", CELL)
    navigation = RecordingNavigation()
    async with Database(tmp_path / "save-race.db") as database:
        service = HomeService(database, reference, robot_id=ROBOT, navigation=navigation)
        await service.prepare()
        pose = replace(localized(Pose2D(1, 2, 0)), map_generation=reference.map_generation)
        await database.write_lock.acquire()
        pending = asyncio.create_task(service.set_current(pose))
        try:
            for _ in range(30):
                if getattr(database.write_lock, "_waiters", None):
                    break
                await asyncio.sleep(0)
            reference.guard_reference()
        finally:
            database.write_lock.release()
        with pytest.raises(DomainError) as error:
            await pending
        assert error.value.code is ErrorCode.LOCALIZATION_UNAVAILABLE
        reference.release_reference()
        await service.note_localization(replace(pose, map_generation=reference.map_generation))
        with pytest.raises(DomainError):
            await service.return_home(MAP)
        assert navigation.goals == []
