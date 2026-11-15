"""Reporting survives loss of motion evidence across IPC and authenticated relay."""

import json
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from test_cloud import cloud  # noqa: F401
from test_manual_wire import ipc_pair, node, runtime  # noqa: F401

from senior_control.agent.composition import SnapshotState
from senior_control.agent.manual_control import MotionRuntimeReceiver
from senior_control.agent.ports import IpcState
from senior_control.agent.snapshots import RobotSnapshot, snapshot_envelope
from senior_control.errors import DomainError
from senior_control.protocol import serialize_envelope


async def test_reporting_without_motion_evidence(node, cloud, monkeypatch):  # noqa: F811
    csrf = await cloud.login()
    robot, token = await cloud.pair(csrf)
    agent = await cloud.agent(token)
    browser = await cloud.control(robot)
    assert await agent.connect() and await browser.connect()
    receiver = node.value._motion_runtime = MotionRuntimeReceiver()
    tasks = SimpleNamespace(snapshot_tasks=AsyncMock(return_value=((), frozenset())))
    waypoints = SimpleNamespace(home=AsyncMock(return_value=None))

    async def state(_request):
        return {
            "v": 1,
            "ok": True,
            "sampled_at": node.clock.now,
            "localized": False,
            "safety_clear": False,
            "manual_driving": False,
            "map_id": None,
            "reference_revision": None,
            "execution_status": "idle",
            "motion_runtime": node.value.motion_runtime(),
            "runtime_observation": node.value.runtime_observation(),
            "motion_runtime_version": 1,
            "manual_protocol_version": 1,
        }

    async with ipc_pair(state, monkeypatch) as client:
        snapshots = SnapshotState(
            IpcState(client, clock=lambda: node.clock.now),
            tasks,
            waypoints,
            clock=lambda: node.clock.now,
            manual_enabled=True,
        )
        sequence = 0

        async def check(status, mode="unknown"):
            nonlocal sequence
            live = await snapshots.collect()
            ui = live.vehicle_ui
            assert ui["sequence"] > sequence
            sequence = ui["sequence"]
            assert ui["observation"]["status"] == status
            assert ui["mode"] == mode
            if mode == "unknown":
                assert ui["allowed_actions"] == []
                assert ui["confirmed_estop_status"] == "unknown"
                assert ui["wheel_linear_mps"] is None
                assert ui["body_reference"] is None
                with pytest.raises(DomainError):
                    await snapshots.recovery_revision("estop_reset", ui["estop_revision"])
            snapshot = RobotSnapshot(
                robot,
                True,
                False,
                None,
                None,
                0,
                "idle",
                time.time(),
                vehicle_ui=ui,
            )
            await agent.send(
                serialize_envelope(
                    snapshot_envelope(
                        snapshot,
                        robot_id=robot,
                        session_id="agent",
                        connection_epoch=1,
                        request_id=str(sequence),
                        seq=sequence,
                        expires_at=time.time() + 30,
                    )
                )
            )
            forwarded = json.loads(await browser.receive())
            assert forwarded["payload"]["vehicle_ui"] == ui
            return ui

        await check("missing")
        sample = runtime(10, robot_id=robot)
        assert receiver.accept(sample, b"t5", 10)
        assert (await check("current", "idle"))["allowed_actions"] == ["acquire"]
        node.clock.now = 11
        expired = await check("expired")
        assert expired["observation"]["last_sampled_steady"] == 10
        await check("expired")
        assert receiver.current(11) is None and receiver.value.valid_until == 10.25
        receiver.invalidate()
        await check("revoked")
        assert not receiver.accept(sample, b"t5", 11)
        assert receiver.accept(runtime(11, 2, robot_id=robot), b"t5", 11)
        await check("current", "idle")
        for i, mode in enumerate(("estop", "execution_fault"), start=3):
            sample = runtime(
                11,
                i,
                robot_id=robot,
                mode=mode,
                healthy=False,
                confirmed_estop_status="software_latched" if mode == "estop" else "clear",
                estop_revision=1,
                fault_revision=1,
                allowed_actions=[],
            )
            assert receiver.accept(sample, b"t5", 11)
            await check("current", mode)


async def test_ipc_failure_still_reports():
    tasks = SimpleNamespace(snapshot_tasks=AsyncMock(return_value=((), frozenset())))
    waypoints = SimpleNamespace(home=AsyncMock(return_value=None))
    from senior_control.errors import ErrorCode

    state = SimpleNamespace(read=AsyncMock(side_effect=DomainError(ErrorCode.UNAVAILABLE, "lost")))
    snapshots = SnapshotState(state, tasks, waypoints)
    first, second = await snapshots.collect(), await snapshots.collect()
    assert first.vehicle_ui["observation"]["status"] == "ipc_unavailable"
    assert second.vehicle_ui["sequence"] > first.vehicle_ui["sequence"]


def test_report_heartbeat_does_not_extend_motion_evidence():
    from senior_control.agent.manual_control import vehicle_frame

    sample = runtime(10, evidence_deadline_steady=10.05)
    frame = vehicle_frame(
        sample,
        sequence=1,
        connected=True,
        task=None,
        manual=None,
        manual_enabled=True,
        tasks_enabled=True,
        runtime_enabled=True,
        safety_revisions=(0, 0),
        now=10.1,
    )
    assert frame["observation"]["status"] == "current"
    assert frame["mode"] == "idle"
    assert frame["allowed_actions"] == []
    assert sample.evidence_deadline_steady == 10.05
    assert sample.valid_until == 10.25
