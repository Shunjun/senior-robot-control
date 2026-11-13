"""Real composition/TaskRuntime/lifecycle/ports; Cloud and ROS IPC are controlled boundaries."""

import asyncio
import json
import time
from contextlib import asynccontextmanager
from contextvars import ContextVar
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pytest
from agent_fakes import FakeConnector, FakeLink, until
from test_manual_wire import envelope, runtime

from senior_control.agent import composition
from senior_control.agent.main import Agent
from senior_control.models import MapRef, Pose2D
from senior_control.runtime.ipc import IpcClient, IpcError
from senior_control.settings.models import SettingsRequest
from senior_control.settings.service import RuntimeGate, SettingsService
from senior_control.tasks.lifecycle import Command, Event
from senior_control.tasks.models import TaskDefinition
from senior_control.tasks.schedule import Schedule


@pytest.mark.parametrize(
    "path",
    [
        "tick",
        "explicit",
        "commit-race",
        "stale-sample",
        "expired-sample",
        "failed-sample",
        "retirement-fence",
        "takeover-submit",
        "takeover-resume",
        "older-resumed",
        "older-resumed-snapshot",
    ],
)
async def test_task_port_waits_do_not_starve_fresh_manual_intents(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, path: str
) -> None:
    # Keep optional logs disabled without opening a remote socket.
    (tmp_path / "logs" / "vehicle-logs.sqlite3").mkdir(parents=True)
    link = FakeLink(1)
    agents: list[Agent] = []
    delayed = False
    manual_active = False
    port_started = asyncio.Event()
    manual_calls: list[tuple[float, float]] = []
    handoff_states: list[tuple[str, str | None, bool]] = []
    holds: list[float] = []
    snapshots: list[float] = []
    sample_waiting, sample_release = asyncio.Event(), asyncio.Event()
    slow_sample: ContextVar[bool] = ContextVar("slow_sample", default=False)
    sample_reads = 0
    original_admission = RuntimeGate.admission

    @asynccontextmanager
    async def timed_admission(self: RuntimeGate):
        async with original_admission(self):
            start = time.monotonic()
            try:
                yield
            finally:
                holds.append(time.monotonic() - start)

    original_snapshot = SettingsService.published_snapshot.fget
    assert original_snapshot is not None

    def snapshot(self: SettingsService):
        start = time.monotonic()
        value = original_snapshot(self)
        snapshots.append(time.monotonic() - start)
        return value

    monkeypatch.setattr(RuntimeGate, "admission", timed_admission)
    monkeypatch.setattr(SettingsService, "published_snapshot", property(snapshot))
    monkeypatch.setattr(composition, "WebSocketConnector", lambda *a, **k: FakeConnector(link))
    monkeypatch.setattr(Agent, "install_signal_handlers", lambda self: agents.append(self))

    async def ipc(self: IpcClient, kind: str, **fields: Any) -> dict[str, Any]:
        nonlocal delayed, manual_active, sample_reads
        if kind == "state.get":
            now = time.monotonic()
            reply = {
                "v": 1,
                "ok": True,
                "sampled_at": now,
                "localized": True,
                "safety_clear": True,
                "manual_driving": manual_active,
                "map_id": "map",
                "reference_revision": 1,
                "map_generation": "map-gen",
                "localization_generation": "map-gen",
                "navigation_ready": True,
                "execution_status": "idle",
                "robot_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                "task_lifecycle_version": 2,
                "task_evidence_ready": True,
                "motion_runtime": runtime(now, robot_id="robot").model_dump(),
                "motion_runtime_version": 1,
                "manual_protocol_version": 1,
                "manual_authority": {
                    "state": "active",
                    "authority_id": "test-authority",
                    "controller_generation": 1,
                    "connection_epoch": 1,
                }
                if manual_active
                else None,
            }
            if slow_sample.get():
                sample_reads += 1
                if sample_reads == 1:
                    sample_waiting.set()
                    await sample_release.wait()
                    if path == "failed-sample":
                        raise IpcError("controlled state read failure")
            return reply
        if kind == "manual":
            if path.startswith("takeover-") or path == "older-resumed":
                state = await service.store.load("execution")
                handoff_states.append((state.phase, state.stopped_for, state.pending is None))
            manual_calls.append((time.monotonic(), fields["deadline_steady"]))
            await asyncio.sleep(0.01)  # Explicit controlled physical IPC latency.
            if json.loads(fields["envelope"])["type"] == "control.acquire":
                manual_active = True
            return {"ok": True, "authority_id": "test-authority", "state": "prepared"}
        if kind == "drive_feedback":
            return {"ok": True, "feedback": []}
        if kind.startswith("navigation."):
            if delayed and kind in {"navigation.task_cancel", "navigation.task_status"}:
                port_started.set()
                await asyncio.sleep(0.18)
            return {
                "v": 1,
                "ok": True,
                "execution_id": fields.get("execution_id"),
                "accepted": True,
                "lifecycle_version": 2,
                "effect_id": fields.get("effect_id", fields.get("plan", {}).get("effect_id")),
                "task_epoch": fields.get("task_epoch", fields.get("plan", {}).get("task_epoch")),
                "sources_fenced_through_epoch": fields.get("task_epoch", 100),
                "controller_output_exited": True,
                "progress": None,
            }
        assert kind in {"identity", "disable_motion", "exit", "server_heartbeat"}
        return {"ok": True}

    monkeypatch.setattr(IpcClient, "request", ipc)
    config = composition.AgentConfig(
        "robot",
        "wss://cloud.test/ws/agent",
        "token",
        tmp_path / "vehicle.db",
        Path("/tmp/short-gate.sock"),
        simulation_settings=True,
    )
    running = asyncio.create_task(composition.run_agent(config))
    try:
        await until(lambda: bool(agents) and agents[0].connected and bool(link.snapshots))
        agent = agents[0]
        actions = agent._navigation
        assert actions is not None
        executor = actions.executor
        service = executor.lifecycle
        assert service is not None
        # Real durable plan/ack setup, using actual NavigationEffects/IpcNavigation.
        now = datetime.now(timezone.utc)
        task = await service.store.tasks.create(
            TaskDefinition(
                "definition",
                "robot",
                MapRef("map", 1),
                ("a",),
                Schedule.interval(now, 86400, "UTC"),
                "UTC",
                True,
                0,
                now,
            )
        )
        live = await actions.state.read()
        executor.context = live.for_tasks(True)

        async def submit():
            return await service.submit(
                Command("submit", "submit", "execution", ("0:a",)),
                task,
                now,
                execution_poses=(Pose2D(0, 0, 0),),
            )

        settings = agent._settings
        assert settings is not None
        if path in {"older-resumed", "older-resumed-snapshot"}:
            assert (await submit()).receipt.accepted
            await service.reconcile("execution")
            await executor.control_current("pause", "pause-A")
            assert (await service.store.load("execution")).phase == "paused"
            paused = await service.store.tasks.current_execution()
            assert paused is not None and paused.execution_id == "execution"
            later = datetime.now(timezone.utc)
            task_b = await service.store.tasks.create(
                TaskDefinition(
                    "definition-B",
                    "robot",
                    MapRef("map", 1),
                    ("b",),
                    Schedule.interval(later, 86400, "UTC"),
                    "UTC",
                    True,
                    0,
                    later,
                )
            )
            result = await service.submit(
                Command("submit-B", "submit", "execution-B", ("0:b",)),
                task_b,
                later,
                execution_poses=(Pose2D(1, 0, 0),),
            )
            assert result.receipt is not None and result.receipt.accepted
            await service.reconcile("execution-B")
            state_b = await service.store.load("execution-B")
            progressed = await service.event(
                "execution-B",
                Event("action_completed", state_b.epoch, "0:b"),
                expected_version=state_b.version,
            )
            completed = await service.event(
                "execution-B",
                Event("completed", state_b.epoch),
                expected_version=progressed.state.version,
            )
            assert completed.state.phase == "succeeded"
            state_a = await service.store.load("execution")
            await link.inbox.put(
                envelope(
                    "task.execution.command",
                    {
                        "request_id": "resume-A",
                        "expected_version": state_a.version,
                        "task_id": "execution",
                        "action": "resume",
                    },
                    generation=None,
                    request="resume-A",
                )
            )
            await until(lambda: any(r.request_id == "resume-A" for r in link.responses))
            assert next(r for r in link.responses if r.request_id == "resume-A").payload.ok
            assert (await service.store.load("execution")).phase == "running"
            latest = await service.store.tasks.latest_execution()
            assert latest is not None and latest.execution_id == "execution-B"
            if path == "older-resumed-snapshot":
                live = await agent._state.collect()
                assert live.vehicle_ui is not None
                frame_task = live.vehicle_ui["task"]
                state_a = await service.store.load("execution")
                assert frame_task == {
                    "task_id": "execution",
                    "version": state_a.version,
                    "phase": "running",
                    "completed": 0,
                    "total": 1,
                    "stopped_for": None,
                    "allowed_actions": ["pause", "cancel", "takeover"],
                }
                # The existing web taskCommand forwards these exact frame fields.
                await link.inbox.put(
                    envelope(
                        "task.execution.command",
                        {
                            "request_id": "frame-pause",
                            "expected_version": frame_task["version"],
                            "task_id": frame_task["task_id"],
                            "action": "pause",
                        },
                        generation=None,
                        request="frame-pause",
                    )
                )
                await until(lambda: any(r.request_id == "frame-pause" for r in link.responses))
                response = next(r for r in link.responses if r.request_id == "frame-pause")
                assert response.payload.ok
                receipt = response.payload.data
                assert receipt is not None and receipt["accepted"] is True
                assert receipt["request_id"] == "frame-pause"
                assert receipt["task_id"] == frame_task["task_id"]
                assert isinstance(receipt["version"], int)
                assert (await service.store.load("execution")).phase == "paused"
                fallback = await agent._state.collect()
                assert fallback.vehicle_ui is not None
                assert fallback.vehicle_ui["task"] == {
                    "task_id": "execution-B",
                    "version": completed.state.version,
                    "phase": "succeeded",
                    "completed": 1,
                    "total": 1,
                    "stopped_for": None,
                    "allowed_actions": [],
                }
                assert await service.store.load("execution-B") == completed.state
                return
            await link.inbox.put(envelope("control.acquire", {}, request="acquire"))
            await until(lambda: any(r.request_id == "acquire" for r in link.responses))
            acquired = next(r for r in link.responses if r.request_id == "acquire")
            assert acquired.payload.ok, acquired.payload
            assert handoff_states == [("paused", "takeover", True)]
            assert len(manual_calls) == 1
            assert await service.store.load("execution-B") == completed.state
            historical = await service.store.tasks.current_execution()
            assert historical is not None and historical.execution_id == "execution-B"
            return
        if path.startswith("takeover-"):
            if path == "takeover-resume":
                assert (await submit()).receipt.accepted
                await service.reconcile("execution")
                await executor.control_current("takeover", "original-takeover")
                assert (await service.store.load("execution")).phase == "paused"
            gap, release_gap = asyncio.Event(), asyncio.Event()
            original_control = executor.control_current
            first = True

            async def control_current(action, request_id):
                nonlocal first
                proof = await original_control(action, request_id)
                if action == "takeover" and first:
                    first = False
                    # The actual method already returned: insert a real new owner
                    # before TaskAwareMotion can acquire its physical handoff gate.
                    gap.set()
                    await release_gap.wait()
                return proof

            monkeypatch.setattr(executor, "control_current", control_current)
            await link.inbox.put(envelope("control.acquire", {}, request="acquire"))
            await asyncio.wait_for(gap.wait(), 1)
            if path == "takeover-submit":
                result = await submit()
            else:
                before = await service.store.load("execution")
                result = await service.command(
                    Command("race-resume", "resume", "execution"), expected_version=before.version
                )
            assert result.receipt is not None and result.receipt.accepted
            await service.reconcile("execution")
            assert (await service.store.load("execution")).phase == "running"
            release_gap.set()
            await until(lambda: any(r.request_id == "acquire" for r in link.responses))
            acquired = next(r for r in link.responses if r.request_id == "acquire")
            assert acquired.payload.ok
            assert handoff_states == [("paused", "takeover", True)], handoff_states
            return
        if path in {"stale-sample", "failed-sample"}:
            token = slow_sample.set(True)
            starting = asyncio.create_task(submit())
            slow_sample.reset(token)
            await asyncio.wait_for(sample_waiting.wait(), 1)
            if path == "stale-sample":
                await link.inbox.put(envelope("control.acquire", {}, request="acquire"))
                await until(lambda: any(r.request_id == "acquire" for r in link.responses))
            sample_release.set()
            result = await starting
            assert sample_reads >= (2 if path == "stale-sample" else 1)
            assert result.receipt is not None and not result.receipt.accepted
            assert await service.store.tasks.latest_execution() is None
            return
        if path in {"commit-race", "expired-sample", "retirement-fence"}:
            saving, release = asyncio.Event(), asyncio.Event()
            original_save = service.store._save

            async def save(connection, before, result):
                if before is None:
                    saving.set()
                    await release.wait()
                return await original_save(connection, before, result)

            monkeypatch.setattr(service.store, "_save", save)
            starting = asyncio.create_task(submit())
            await asyncio.wait_for(saving.wait(), 1)
            if path in {"expired-sample", "retirement-fence"}:
                from senior_control.tasks.lifecycle_store import VersionConflict

                if path == "expired-sample":
                    await asyncio.sleep(0.27)
                else:
                    await link.inbox.put(
                        envelope(
                            "control.release", {"authority_id": "test-authority"}, request="release"
                        )
                    )
                    await until(lambda: any(r.request_id == "release" for r in link.responses))
                release.set()
                with pytest.raises(VersionConflict):
                    await starting
                assert await service.store.tasks.latest_execution() is None
                assert await service.store.query("submit") is None
                return
            writing = asyncio.create_task(
                settings.apply(
                    SettingsRequest(
                        "drive-race", 0, "driving", settings.published_snapshot.driving.model_dump()
                    )
                )
            )
            await link.inbox.put(envelope("control.acquire", {}, request="acquire"))
            await asyncio.sleep(0.01)
            assert not writing.done() and not manual_calls
            release.set()
            result = await starting
            assert result.receipt is not None and result.receipt.accepted
            assert (await writing).reason == "stop_required"
            await until(lambda: any(r.request_id == "acquire" for r in link.responses))
            assert next(r for r in link.responses if r.request_id == "acquire").payload.ok
            assert settings.published_snapshot.driving_version == 0
            return
        accepted = await submit()
        # The target count must match the accepted action, including in the fixture.
        assert accepted.receipt is not None and accepted.receipt.accepted
        await service.reconcile("execution")
        assert (await service.store.load("execution")).phase == "running"
        manual_active, delayed = True, True
        if path == "explicit":
            state = await service.store.load("execution")
            await link.inbox.put(
                envelope(
                    "task.execution.command",
                    {
                        "request_id": "pause",
                        "expected_version": state.version,
                        "task_id": "execution",
                        "action": "pause",
                    },
                    generation=None,
                    request="pause",
                )
            )
        await asyncio.wait_for(port_started.wait(), 1)
        # Stream new original samples while the real task path awaits its ports.
        for seq in range(1, 6):
            await link.inbox.put(
                envelope(
                    "drive.intent",
                    {
                        "authority_id": "test-authority",
                        "gesture": 1,
                        "sequence": seq,
                        "linear": 0.2,
                        "turn": 0.0,
                        "gear": "low",
                    },
                    seq=seq,
                    request=f"intent-{seq}",
                )
            )
            await asyncio.sleep(0.05)
        await until(
            lambda: len([r for r in link.responses if r.request_id.startswith("intent-")]) == 5
        )
        replies = [r for r in link.responses if r.request_id.startswith("intent-")]
        assert all(r.payload.ok for r in replies), [
            (r.request_id, r.payload.error) for r in replies
        ]
        assert len(manual_calls) == 5
        assert all(called < deadline for called, deadline in manual_calls)
        denied = await settings.apply(
            SettingsRequest(
                "blocked", 0, "driving", settings.published_snapshot.driving.model_dump()
            )
        )
        assert denied.reason == "stop_required"
        Path(f"/private/tmp/task_b7719c254510-{path}-timing.json").write_text(
            json.dumps(
                {
                    "manual_lock_holds_seconds": holds,
                    "snapshot_read_seconds": snapshots,
                    "remaining_deadline_at_ipc_seconds": [
                        deadline - called for called, deadline in manual_calls
                    ],
                    "task_port_delays_seconds": [0.18, 0.18],
                    "manual_port_delay_seconds": 0.01,
                },
                indent=2,
            )
        )
    finally:
        if agents:
            agents[0].request_shutdown()
        await asyncio.wait_for(running, 3)
