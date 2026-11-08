import json
import time
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pytest

from senior_control.agent.dispatcher import StoreTaskPort
from senior_control.agent.workspace import WorkspaceReader
from senior_control.cloud.limits import CloudLimits
from senior_control.cloud.relay import Hub
from senior_control.errors import DomainError, ErrorCode
from senior_control.mapping.workspace import WorkspaceGrid, WorkspaceMap
from senior_control.models import MapRef, Pose2D
from senior_control.protocol import (
    MAX_MESSAGE_BYTES,
    ResponseEnvelope,
    ResponsePayload,
    WorkspaceEnvelope,
    serialize_envelope,
    validate_envelope,
)
from senior_control.runtime.ipc import MAX_FRAME, _encode
from senior_control.storage.database import Database
from senior_control.tasks import Schedule, TaskDefinition, TaskStore
from senior_control.waypoints.store import WaypointStore

REF = MapRef("active", 2)


class LoopbackMap:
    def __init__(self, workspace: WorkspaceMap) -> None:
        self.workspace = workspace

    async def request(self, kind: str, **fields: object) -> dict:
        return {"v": 1, **self.workspace.read(kind, fields)}


def command(kind: str, **fields: object) -> WorkspaceEnvelope:
    envelope = validate_envelope(
        json.dumps(
            {
                "v": 1,
                "type": kind,
                "robot_id": "r1",
                "session_id": "web",
                "request_id": "req",
                "seq": 1,
                "connection_epoch": 2,
                "expires_at": time.time() + 5,
                "payload": {
                    "map_ref": {"map_id": REF.map_id, "reference_revision": REF.reference_revision},
                    **fields,
                },
            }
        ).encode(),
        "r1",
    )
    assert isinstance(envelope, WorkspaceEnvelope)
    return envelope


async def test_workspace_read_is_reference_fenced(tmp_path: Path) -> None:
    pose = Pose2D(1.0, 2.0, 0.0)
    active = WorkspaceGrid(REF, "grid", 1, 1, 0.1, pose, (100,))
    grid = active
    async with Database(tmp_path / "robot.db") as database:
        tasks = TaskStore(database, robot_id="r1")
        waypoints = WaypointStore(database, robot_id="r1")
        await tasks.prepare()
        await waypoints.prepare()
        await waypoints.save_waypoint(
            waypoint_id="p1",
            pose=pose,
            map_ref=REF,
            anchor_id="anchor",
            anchor_pose=pose,
            source_robot_id="r1",
            status="confirmed",
            latitude=None,
            longitude=None,
        )
        reader = WorkspaceReader("r1", tasks, waypoints, LoopbackMap(WorkspaceMap(lambda: grid)))
        result = await reader.read(command("waypoint.list"))
        assert [point["waypoint_id"] for point in result["waypoints"]] == ["p1"]
        created_at = datetime(2026, 11, 8, tzinfo=timezone.utc)
        schedule = Schedule.interval(created_at, 90)
        task = await tasks.create(
            TaskDefinition(
                "existing", "r1", REF, ("p1",), schedule, schedule.timezone, True, 0, created_at
            )
        )
        detail = await reader.read(command("task.get", task_id=task.task_id))
        assert detail["waypoint_ids"] == ["p1"]
        assert detail["schedule"] == {"kind": "interval", "every_seconds": 90}
        assert detail["revision"] == task.revision
        assert detail["created_at"] == created_at.isoformat()

        # The source changed while the store query was in progress.
        page = waypoints.page

        async def switch(*args: Any):
            nonlocal grid
            points = await page(*args)
            grid = WorkspaceGrid(REF, "replacement", 1, 1, 0.1, pose, (0,))
            return points

        waypoints.page = switch
        with pytest.raises(DomainError) as changed:
            await reader.read(command("waypoint.list"))
        assert changed.value.code is ErrorCode.CONFLICT

        grid = active
        with pytest.raises(DomainError) as stale:
            await reader.read(
                command(
                    "map.manifest",
                    map_ref={"map_id": REF.map_id, "reference_revision": 1},
                )
            )
        assert stale.value.code is ErrorCode.CONFLICT
        reader = WorkspaceReader("r1", tasks, waypoints, LoopbackMap(WorkspaceMap(lambda: None)))
        with pytest.raises(DomainError) as missing:
            await reader.read(command("map.manifest"))
        assert missing.value.code is ErrorCode.UNAVAILABLE

        fallback = WorkspaceReader(
            "r1",
            tasks,
            waypoints,
            LoopbackMap(WorkspaceMap(lambda: None)),
            exported_grid=replace(active, display_only=True),
        )
        assert await fallback.active_map() == REF
        manifest = await fallback.read(command("map.manifest"))
        assert manifest["display_only"] is True
        tile = await fallback.read(command("map.tile", grid_id="grid", tile_x=0, tile_y=0))
        assert tile["cells"] == [100]
        with pytest.raises(DomainError) as read_only:
            await fallback.read(command("task.get", task_id=task.task_id))
        assert read_only.value.code is ErrorCode.UNAVAILABLE


def test_tile_fits_ipc_frame() -> None:
    cells = tuple(i % 102 - 1 for i in range(33 * 35))
    workspace = WorkspaceMap(
        lambda: WorkspaceGrid(REF, "grid", 33, 35, 0.1, Pose2D(0, 0, 0), cells)
    )
    combined = [-2] * len(cells)
    for ty in range(2):
        for tx in range(2):
            tile = workspace.read(
                "map.tile",
                {
                    "map_id": REF.map_id,
                    "reference_revision": REF.reference_revision,
                    "grid_id": "grid",
                    "tile_x": tx,
                    "tile_y": ty,
                },
            )
            assert len(_encode({"v": 1, **tile})) <= MAX_FRAME + 4
            assert tile["ok"]
            request = command("map.tile", grid_id="grid", tile_x=tx, tile_y=ty)
            assert len(serialize_envelope(response(request, tile))) <= MAX_MESSAGE_BYTES
            for row in range(tile["height"]):
                start = (ty * 32 + row) * 33 + tx * 32
                width = tile["width"]
                combined[start : start + width] = tile["cells"][row * width : (row + 1) * width]
    assert tuple(combined) == cells


def response(request: WorkspaceEnvelope, data: dict[str, Any]) -> ResponseEnvelope:
    return ResponseEnvelope(
        **request.model_dump(exclude={"type", "payload"}),
        type="response",
        payload=ResponsePayload(request_type=request.type, ok=True, data=data),
    )


async def test_relay_prioritizes_control_and_correlates_workspace_replies() -> None:
    hub = Hub(CloudLimits())
    agent = hub.attach_agent("r1", epoch=2)
    viewer = hub.attach_controller("r1", "owner")
    other_viewer = hub.attach_controller("r1", "other")
    assert viewer is not None and other_viewer is not None
    # Clear initial snapshots before checking who receives the response.
    while viewer.outbox.pending:
        await viewer.outbox.get()
    while other_viewer.outbox.pending:
        await other_viewer.outbox.get()

    request = command("map.manifest")
    wire = serialize_envelope(request)
    hub.handle_control_frame(viewer, wire)
    control = request.model_dump(exclude={"type", "payload"})
    stop = json.dumps({**control, "type": "drive.stop", "seq": 2, "payload": {}}).encode()
    hub.handle_control_frame(viewer, stop)
    first = await agent.outbox.get()
    assert isinstance(first, bytes)
    assert validate_envelope(first, "r1").type == "drive.stop"
    assert await agent.outbox.get() == wire

    answer = response(request, {"robot_id": "r1", "grid_id": "grid"})
    wrong = answer.model_copy(update={"seq": 0})
    hub.handle_agent_frame(agent, serialize_envelope(wrong))
    assert viewer.outbox.pending == 0
    hub.handle_agent_frame(agent, serialize_envelope(answer))
    assert await viewer.outbox.get() == serialize_envelope(answer)
    assert other_viewer.outbox.pending == 0


async def test_workspace_edit_requires_current_grid_and_saved_waypoint(tmp_path: Path) -> None:
    pose = Pose2D(1, 2, 0)
    grid = WorkspaceGrid(REF, "fresh", 1, 1, 0.1, pose, (0,))
    async with Database(tmp_path / "robot.db") as database:
        tasks = TaskStore(database, robot_id="r1")
        points = WaypointStore(database, robot_id="r1")
        await tasks.prepare()
        await points.prepare()
        reader = WorkspaceReader("r1", tasks, points, LoopbackMap(WorkspaceMap(lambda: grid)))
        port = StoreTaskPort(tasks, robot_id="r1", verify_workspace=reader.verify_task_edit)
        body = {
            "task_id": "route",
            "map_id": REF.map_id,
            "reference_revision": REF.reference_revision,
            "waypoint_ids": ["saved"],
            "schedule": {"kind": "interval", "every_seconds": 120},
            "timezone": "UTC",
            "enabled": True,
            "created_at": "2026-11-08T00:00:00+00:00",
        }

        with pytest.raises(DomainError) as old_grid:
            await port.create({**body, "grid_id": "old"})
        assert old_grid.value.code is ErrorCode.CONFLICT
        with pytest.raises(DomainError) as missing_point:
            await port.create({**body, "grid_id": "fresh"})
        assert missing_point.value.code is ErrorCode.CONFLICT
        assert await tasks.get("route") is None

        await points.save_waypoint(
            waypoint_id="saved",
            pose=pose,
            map_ref=REF,
            anchor_id="anchor",
            anchor_pose=pose,
            source_robot_id="r1",
            status="confirmed",
            latitude=None,
            longitude=None,
        )
        revision = await port.create({**body, "grid_id": "fresh"})
        grid = replace(grid, grid_id="new")
        with pytest.raises(DomainError) as stale_update:
            await port.update({**body, "grid_id": "fresh", "enabled": False}, revision)
        assert stale_update.value.code is ErrorCode.CONFLICT
        # //Compatible: the original task edit body has no grid_id.
        assert await port.update({**body, "enabled": False}, revision) == revision + 1
