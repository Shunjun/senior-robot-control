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
            if kind == "state.get":
                return {
                    "v": 1,
                    "ok": True,
                    "sampled_at": time.monotonic(),
                    "localized": True,
                    "safety_clear": False,
                    "manual_driving": False,
                    "execution_status": "idle",
                    "map_id": "yard",
                    "reference_revision": 1,
                    "map_generation": "map-a",
                    "localization_generation": "map-a",
                }
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
        assert error.value.code is ErrorCode.LOCALIZATION_UNAVAILABLE
        assert await store.home() is None


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
    motion = IpcMotion(motion_ipc, safety_client=safety_ipc, intent_client=AsyncMock())
    await motion.stop(stop)
    motion_ipc.request.assert_not_awaited()
    safety_ipc.request.assert_awaited_once()


async def test_fresh_generic_reference_can_set_home_without_environment_permission(tmp_path: Path):
    import time

    from senior_control.agent.ports import IpcHomePort
    from senior_control.models import Pose2D
    from senior_control.storage.database import Database
    from senior_control.waypoints.store import WaypointStore

    sampled = time.monotonic()

    class GenericReference:
        async def request(self, kind: str, **fields: Any) -> dict[str, Any]:
            common = {
                "v": 1,
                "ok": True,
                "sampled_at": sampled,
                "map_id": "yard",
                "reference_revision": 1,
                "map_generation": "map-a",
            }
            if kind == "pose.get":
                return {
                    **common,
                    "healthy": True,
                    "anchor_id": "a",
                    "x": 1.0,
                    "y": 2.0,
                    "yaw": 0.0,
                    "anchor_x": 0.0,
                    "anchor_y": 0.0,
                    "anchor_yaw": 0.0,
                }
            assert kind == "state.get"
            return {
                **common,
                "localized": True,
                "localization_generation": "map-a",
                "safety_clear": False,
                "manual_driving": False,
                "execution_status": "idle",
            }

    async with Database(tmp_path / "robot.db") as database:
        store = WaypointStore(database, robot_id="robot-a")
        await store.prepare()
        home = IpcHomePort(GenericReference(), store, robot_id="robot-a")
        assert await home.set_current(0) == 1
        saved = await store.home()
        assert saved is not None
        assert saved.waypoint.pose == Pose2D(1, 2, 0)
        assert saved.waypoint.map_ref == MapRef("yard", 1)


async def test_learning_uses_regular_request_lane_with_original_deadline_and_typed_receipt():
    from senior_control.agent.connection import IpcMotion
    from senior_control.protocol import LearningStartEnvelope

    calls = []

    class Client:
        async def request(self, kind, **fields):
            calls.append((kind, fields))
            if kind == "learning_capability":
                return {"ok": True, "learning_protocol_version": 1}
            return {
                "ok": True,
                "data": {"start_request_id": "start", "attempt_id": "a" * 32, "state": "accepted"},
            }

    client = Client()
    motion = IpcMotion(client, intent_client=UnsupportedIpc())
    assert await motion.learning_available()
    assert await motion.learning_available()
    envelope = LearningStartEnvelope(
        v=1,
        type="learning.start",
        robot_id="robot",
        session_id="operator",
        connection_epoch=1,
        controller_generation=2,
        request_id="start",
        seq=1,
        expires_at=10.0,
        payload={"expected_active_revision": 0, "expected_authority_id": None},
    )
    receipt = await motion.learning(envelope, received_steady=1.0, deadline_steady=1.2)
    assert receipt["state"] == "accepted"
    assert [kind for kind, _ in calls] == ["learning_capability", "learning"]
    assert calls[-1][1]["deadline_steady"] == 1.2
    assert '"controller_generation":2' in calls[-1][1]["envelope"]


async def test_ipc_grid_id_is_optional_and_does_not_grant_localization():
    from vehicle_manager.agent.local_state import StateReply

    from senior_control.agent.ports import IpcState

    raw = {
        "v": 1,
        "ok": True,
        "sampled_at": 10.0,
        "localized": False,
        "safety_clear": False,
        "manual_driving": False,
        "map_id": "yard",
        "reference_revision": 1,
        "execution_status": "idle",
        "map_generation": "reference-a",
        "localization_generation": None,
    }

    class Client:
        async def request(self, kind):
            assert kind == "state.get"
            return dict(raw)

    ipc = IpcState(Client(), clock=lambda: 10.1)
    assert (await ipc.read()).map_grid_id is None
    for grid_id in ("grid-a", "x" * 128):
        raw["map_grid_id"] = grid_id
        state = await ipc.read()
        assert state.map_grid_id == grid_id
        assert state.map_generation == "reference-a"
        assert state.map_ref == MapRef("yard", 1)
        assert not state.localized and not state.navigation_ready
        assert state.robot_pose is None and state.scan_points == ()
    for grid_id in ("", "x" * 129, 1):
        raw["map_grid_id"] = grid_id
        with pytest.raises(DomainError):
            await ipc.read()
    raw.pop("map_grid_id")
    legacy = StateReply.model_validate(raw)
    assert "map_grid_id" not in legacy.model_dump()
    assert '"map_grid_id"' not in legacy.model_dump_json()
    raw["sampled_at"] = 9.0
    with pytest.raises(DomainError):
        await ipc.read()
