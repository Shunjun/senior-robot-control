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
