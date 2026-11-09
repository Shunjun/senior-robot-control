from pathlib import Path
from typing import Any

import pytest

from senior_control.agent.ports import IpcMapPort
from senior_control.errors import DomainError, ErrorCode
from senior_control.models import MapRef


class UnsupportedIpc:
    async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
        return {"v": 1, "ok": False, "error": "unsupported"}


async def test_missing_map_consumer_is_explicitly_unavailable(tmp_path: Path) -> None:
    maps = IpcMapPort(UnsupportedIpc())
    with pytest.raises(DomainError) as caught:
        await maps.export("operation-1", MapRef("yard", 1), tmp_path / "bundle")
    assert caught.value.code is ErrorCode.UNAVAILABLE


async def test_missing_navigation_consumer_is_explicitly_unavailable(tmp_path: Path) -> None:
    from datetime import datetime, timezone

    from senior_control.agent.ports import IpcNavigation
    from senior_control.tasks.models import TaskDefinition
    from senior_control.tasks.schedule import Schedule

    created = datetime(2026, 1, 1, tzinfo=timezone.utc)
    task = TaskDefinition(
        "patrol",
        "robot-a",
        MapRef("yard", 1),
        ("gate",),
        Schedule.interval(created, 60, "UTC"),
        "UTC",
        True,
        0,
        created,
    )
    navigation = IpcNavigation(UnsupportedIpc())
    with pytest.raises(DomainError) as caught:
        await navigation.start("run-1", task)
    assert caught.value.code is ErrorCode.UNAVAILABLE


async def test_missing_state_consumer_reports_unavailable() -> None:
    from senior_control.agent.ports import IpcState

    state = IpcState(UnsupportedIpc())
    with pytest.raises(DomainError) as caught:
        await state.read()
    assert caught.value.code is ErrorCode.UNAVAILABLE


async def test_home_set_requires_real_localization(tmp_path: Path) -> None:
    from senior_control.agent.ports import IpcHomePort
    from senior_control.storage.database import Database
    from senior_control.waypoints.store import WaypointStore

    async with Database(tmp_path / "robot.db") as database:
        store = WaypointStore(database, robot_id="robot-a")
        await store.prepare()
        home = IpcHomePort(UnsupportedIpc(), store, robot_id="robot-a")
        with pytest.raises(DomainError) as caught:
            await home.set_current(0)
        assert caught.value.code is ErrorCode.UNAVAILABLE
        assert await store.home() is None


async def test_legacy_ipc_pose_without_reference_generation_cannot_set_home(tmp_path: Path) -> None:
    import time

    from senior_control.agent.ports import IpcHomePort
    from senior_control.storage.database import Database
    from senior_control.waypoints.store import WaypointStore

    class LegacyPose:
        async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
            return {
                "v": 1,
                "ok": True,
                "sampled_at": time.monotonic(),
                "healthy": True,
                "map_id": "yard",
                "reference_revision": 1,
                "anchor_id": "a",
                "x": 1.0,
                "y": 2.0,
                "yaw": 0.0,
                "anchor_x": 0.0,
                "anchor_y": 0.0,
                "anchor_yaw": 0.0,
            }

    async with Database(tmp_path / "robot.db") as database:
        store = WaypointStore(database, robot_id="robot-a")
        await store.prepare()
        home = IpcHomePort(LegacyPose(), store, robot_id="robot-a")
        with pytest.raises(DomainError) as error:
            await home.set_current(0)
        assert error.value.code is ErrorCode.UNAVAILABLE
        assert await store.home() is None


async def test_legacy_navigation_ipc_cannot_report_unqualified_task_start(tmp_path: Path) -> None:
    from datetime import datetime, timezone

    from senior_control.agent.ports import IpcNavigation
    from senior_control.tasks.models import TaskDefinition
    from senior_control.tasks.schedule import Schedule

    class AckWithoutGeneration:
        def __init__(self) -> None:
            self.calls: list[str] = []

        async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
            self.calls.append(kind)
            return {"v": 1, "ok": True, "execution_id": fields["execution_id"]}

    created = datetime(2026, 1, 1, tzinfo=timezone.utc)
    task = TaskDefinition(
        "patrol",
        "r",
        MapRef("yard", 1),
        ("gate",),
        Schedule.interval(created, 60, "UTC"),
        "UTC",
        True,
        0,
        created,
    )
    client = AckWithoutGeneration()
    with pytest.raises(DomainError) as error:
        await IpcNavigation(client).start("run-1", task)
    assert error.value.code is ErrorCode.UNAVAILABLE
    assert client.calls == []


async def test_stop_uses_an_independent_motion_ipc_client() -> None:
    from unittest.mock import AsyncMock

    from senior_control.agent.connection import IpcMotion
    from senior_control.protocol import SimpleEnvelope

    motion_ipc = AsyncMock()
    safety_ipc = AsyncMock()
    safety_ipc.request.return_value = {"ok": True}
    stop = SimpleEnvelope(
        type="drive.stop",
        robot_id="robot-a",
        session_id="session",
        connection_epoch=1,
        seq=1,
        expires_at=5.0,
        v=1,
        request_id="stop",
        payload={},
    )
    motion = IpcMotion(motion_ipc, safety_client=safety_ipc)
    await motion.stop(stop)
    motion_ipc.request.assert_not_awaited()
    safety_ipc.request.assert_awaited_once()
