import json
from dataclasses import replace

import pytest

from senior_control.agent.ports import IpcState
from senior_control.agent.snapshots import (
    RobotSnapshot,
    SnapshotCache,
    snapshot_envelope,
    snapshot_from_envelope,
)
from senior_control.models import MapRef
from senior_control.protocol import RobotPosePayload, ScanPointPayload, SnapshotPayload


def test_live_geometry_round_trips_and_stale_or_offline_copies_clear_it():
    snapshot = RobotSnapshot(
        robot_id="sim",
        connected=True,
        localized=True,
        map_ref=MapRef("map", 1),
        home=None,
        task_revision=0,
        execution_status="idle",
        collected_at=10.0,
        map_generation="generation-1",
        robot_pose=RobotPosePayload(x=1.0, y=2.0, yaw=0.5),
        scan_points=(ScanPointPayload(x=2.0, y=3.0),),
        navigation_block_reason="planner state unavailable",
    )

    def envelope(value, **kwargs):
        return snapshot_envelope(
            value,
            robot_id="sim",
            session_id="agent",
            connection_epoch=1,
            request_id="snapshot",
            seq=1,
            expires_at=20.0,
            **kwargs,
        )

    assert snapshot_from_envelope(envelope(snapshot)) == snapshot
    assert len(envelope(snapshot).model_dump_json().encode()) < 8192
    for copy in (
        envelope(snapshot, stale=True),
        envelope(replace(snapshot, connected=False)),
        envelope(replace(snapshot, localized=False)),
        envelope(replace(snapshot, map_generation=None)),
    ):
        assert copy.payload.robot_pose is None
        assert copy.payload.scan_points == []
    now = [10.0]
    cache = SnapshotCache(clock=lambda: now[0])
    cache.store(snapshot)
    now[0] = 16.0
    stale = cache.get("sim")
    assert stale.stale
    assert stale.snapshot.robot_pose is None
    assert stale.snapshot.scan_points == ()


async def test_ipc_geometry_is_hidden_on_reference_generation_mismatch():
    class Client:
        async def request(self, kind):
            return {
                "v": 1,
                "ok": True,
                "sampled_at": 10.0,
                "localized": True,
                "safety_clear": True,
                "manual_driving": False,
                "map_id": "map",
                "reference_revision": 1,
                "execution_status": "idle",
                "map_generation": "current",
                "localization_generation": "old",
                "robot_pose": {"x": 1.0, "y": 2.0, "yaw": 0.0},
                "scan_points": [{"x": 3.0, "y": 4.0}],
                "navigation_block_reason": "map or localization generation changed",
            }

    state = await IpcState(Client(), clock=lambda: 10.1).read()
    assert state.robot_pose is None
    assert state.scan_points == ()
    assert state.navigation_block_reason == "map or localization generation changed"


def test_snapshot_geometry_rejects_nonfinite_and_oversized_input():
    base = {
        "connected": True,
        "localized": True,
        "map_ref": None,
        "home": None,
        "task_revision": 0,
        "runnable_task_ids": [],
        "execution_status": "idle",
        "collected_at": 10.0,
    }
    from pydantic import ValidationError

    for geometry in (
        {"robot_pose": {"x": float("nan"), "y": 0.0, "yaw": 0.0}},
        {"scan_points": [{"x": 0.0, "y": float("inf")}]},
        {"scan_points": [{"x": 0.0, "y": 0.0}] * 97},
    ):
        with pytest.raises(ValidationError):
            SnapshotPayload.model_validate({**base, **geometry})
    assert (
        json.loads(SnapshotPayload.model_validate(base).model_dump_json()).get("robot_pose") is None
    )
